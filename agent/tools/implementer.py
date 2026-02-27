"""
LLM-driven Implement phase — writes and modifies C++ code.

Takes a design plan → generates code changes → applies them to files.
"""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .codebase import CodebaseCollector
from .llm_client import LLMClient, Message

log = logging.getLogger("kinetra.agent.implementer")


@dataclass
class FileChange:
    """A single file creation or modification."""
    path: str            # Relative to project root
    action: str          # "create" | "modify" | "append"
    content: str         # Full file content (create) or new content (modify)
    description: str


class Implementer:
    """LLM-driven code implementation phase."""

    def __init__(
        self,
        llm: LLMClient,
        project_root: Path,
        prompts_dir: Path,
    ):
        self.llm = llm
        self.project_root = Path(project_root).resolve()
        self.codebase = CodebaseCollector(self.project_root)
        self.prompts_dir = Path(prompts_dir)

    def implement(self, plan: dict[str, Any]) -> dict[str, Any]:
        """Generate and apply code changes for the given design plan."""
        if not plan or not plan.get("improvements"):
            return {
                "success": False,
                "summary": "No improvements to implement",
                "changes": [],
            }

        # Pick the highest-priority improvement
        improvements = plan["improvements"]
        improvement = improvements[0]

        # Collect relevant file contents
        files_to_read = (
            improvement.get("files_to_modify", []) +
            self._infer_related_files(improvement)
        )
        file_contents = self.codebase.read_files(files_to_read)
        codebase_summary = self.codebase.collect_compact()

        # Load prompts
        system_prompt = self._load_prompt("system.md")
        implement_prompt = self._load_prompt("implement.md")

        # Build the implementation request
        context_parts = [
            "# Implementation Task\n\n",
            f"## Improvement: {improvement.get('title', 'unknown')}\n\n",
            f"**Description:** {improvement.get('description', improvement.get('pseudocode', ''))}\n\n",
        ]

        test_cases = improvement.get("test_cases", [])
        if test_cases:
            context_parts.append("## Required Test Cases\n")
            for tc in test_cases:
                context_parts.append(f"- {tc}\n")
            context_parts.append("\n")

        context_parts.append("## Pseudocode\n")
        context_parts.append(f"```\n{improvement.get('pseudocode', 'N/A')}\n```\n\n")

        context_parts.append("## Codebase Overview\n")
        context_parts.append(codebase_summary + "\n\n")

        if file_contents:
            context_parts.append("## Existing File Contents\n\n")
            for path, content in file_contents.items():
                context_parts.append(f"### {path}\n```cpp\n{content}\n```\n\n")

        context_parts.append("---\n\n")
        context_parts.append(implement_prompt + "\n\n")
        context_parts.append(
            "IMPORTANT: Return ONLY valid JSON with this exact structure:\n"
            "```json\n"
            "{\n"
            '  "changes": [\n'
            "    {\n"
            '      "path": "relative/path/to/file.cpp",\n'
            '      "action": "create|modify",\n'
            '      "content": "full file content as string",\n'
            '      "description": "what this change does"\n'
            "    }\n"
            "  ],\n"
            '  "reasoning": "explanation of the implementation approach"\n'
            "}\n"
            "```\n\n"
            "For 'modify' actions, provide the COMPLETE new file content.\n"
            "For 'create' actions, provide the full new file content.\n"
            "Include proper SPDX headers, includes, and namespace declarations.\n"
            "Write the test FIRST (test-driven development).\n"
        )

        context = "".join(context_parts)

        # Call LLM with higher token limit for code generation
        messages = [
            Message(role="system", content=system_prompt),
            Message(role="user", content=context),
        ]

        result = self.llm.chat_json(messages, temperature=0.2)
        if result is None:
            return {
                "success": False,
                "summary": "LLM returned invalid JSON for implementation",
                "changes": [],
            }

        # Parse and apply changes
        changes = result.get("changes", [])
        if not changes:
            return {
                "success": False,
                "summary": "LLM returned no file changes",
                "changes": [],
            }

        applied: list[dict[str, str]] = []
        errors: list[str] = []

        for change in changes:
            path = change.get("path", "")
            action = change.get("action", "create")
            content = change.get("content", "")
            desc = change.get("description", "")

            if not path or not content:
                errors.append(f"Empty path or content in change: {desc}")
                continue

            try:
                self._apply_change(path, action, content)
                applied.append({"path": path, "action": action, "description": desc})
                log.info("Applied %s: %s — %s", action, path, desc)
            except Exception as e:
                errors.append(f"Failed to apply {action} {path}: {e}")
                log.error("Failed to apply change: %s", e)

        summary_parts = [f"{len(applied)} file(s) changed"]
        if errors:
            summary_parts.append(f"{len(errors)} error(s)")

        return {
            "success": len(applied) > 0 and len(errors) == 0,
            "summary": " | ".join(summary_parts),
            "changes": applied,
            "errors": errors,
            "reasoning": result.get("reasoning", ""),
        }

    def _apply_change(self, rel_path: str, action: str, content: str) -> None:
        """Write content to a file."""
        # Sanitize path — prevent escaping project root
        rel_path = rel_path.lstrip("/")
        if ".." in rel_path:
            raise ValueError(f"Path contains '..': {rel_path}")

        full_path = self.project_root / rel_path

        if action == "create":
            full_path.parent.mkdir(parents=True, exist_ok=True)
            full_path.write_text(content)
        elif action in ("modify", "replace"):
            if not full_path.exists():
                raise FileNotFoundError(f"File not found: {rel_path}")
            # Backup original
            backup = full_path.with_suffix(full_path.suffix + ".bak")
            shutil.copy2(full_path, backup)
            full_path.write_text(content)
        elif action == "append":
            with open(full_path, "a") as f:
                f.write(content)
        else:
            raise ValueError(f"Unknown action: {action}")

    def _infer_related_files(self, improvement: dict[str, Any]) -> list[str]:
        """Infer files that might be related to the improvement."""
        related: list[str] = []
        title = improvement.get("title", "").lower()
        desc = improvement.get("description", "").lower()
        text = title + " " + desc

        # Map keywords to relevant files
        keyword_map = {
            "rrt": ["include/kinetra/planners/rrt_star.hpp",
                    "src/planners/rrt_star.cpp",
                    "tests/unit/test_rrt_star.cpp"],
            "stomp": ["include/kinetra/planners/stomp.hpp",
                      "src/planners/stomp.cpp"],
            "ilqr": ["include/kinetra/planners/ilqr.hpp",
                     "src/planners/ilqr.cpp"],
            "qp": ["include/kinetra/solvers/qp_admm.hpp",
                   "src/solvers/qp_admm.cpp",
                   "tests/unit/test_qp_solver.cpp"],
            "lqr": ["include/kinetra/solvers/lqr.hpp",
                    "src/solvers/lqr.cpp"],
            "collision": ["include/kinetra/collision/occupancy_grid.hpp",
                          "src/collision/occupancy_grid.cpp",
                          "tests/unit/test_collision.cpp"],
            "dubins": ["include/kinetra/spaces/dubins.hpp",
                       "src/spaces/dubins.cpp",
                       "tests/unit/test_dubins.cpp"],
            "se2": ["include/kinetra/spaces/se2.hpp",
                    "src/spaces/se2.cpp",
                    "tests/unit/test_se2_space.cpp"],
            "trajectory": ["include/kinetra/core/trajectory.hpp",
                          "src/core/trajectory.cpp",
                          "tests/unit/test_trajectory.cpp"],
            "robot": ["include/kinetra/robots/differential_drive.hpp",
                     "include/kinetra/robots/ackermann.hpp",
                     "tests/unit/test_robot_models.cpp"],
            "concept": ["include/kinetra/core/concepts.hpp"],
            "type": ["include/kinetra/core/types.hpp",
                    "tests/unit/test_types.cpp"],
        }

        for keyword, files in keyword_map.items():
            if keyword in text:
                related.extend(files)

        # Always include core types and concepts
        related.extend([
            "include/kinetra/core/types.hpp",
            "include/kinetra/core/concepts.hpp",
            "include/kinetra/core/result.hpp",
        ])

        # Deduplicate
        seen: set[str] = set()
        unique: list[str] = []
        for f in related:
            if f not in seen:
                seen.add(f)
                unique.append(f)
        return unique

    def rollback(self) -> int:
        """Restore all .bak files. Returns count of files restored."""
        count = 0
        for bak in self.project_root.rglob("*.bak"):
            original = bak.with_suffix("")
            shutil.move(str(bak), str(original))
            count += 1
            log.info("Rolled back: %s", original.relative_to(self.project_root))
        return count

    def clean_backups(self) -> int:
        """Remove all .bak files after successful build/test."""
        count = 0
        for bak in self.project_root.rglob("*.bak"):
            bak.unlink()
            count += 1
        return count

    def _load_prompt(self, name: str) -> str:
        path = self.prompts_dir / name
        if path.exists():
            return path.read_text()
        return f"(prompt {name} not found)"
