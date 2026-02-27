"""
LLM-driven Design phase — proposes improvements to Kinetra.

Takes research findings + codebase context → outputs a structured improvement plan.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .codebase import CodebaseCollector
from .llm_client import LLMClient, Message

log = logging.getLogger("kinetra.agent.designer")


@dataclass
class Improvement:
    title: str
    priority: int
    description: str
    test_cases: list[str]
    files_to_modify: list[str]
    files_to_create: list[str]
    pseudocode: str


@dataclass
class DesignPlan:
    improvements: list[Improvement]
    reasoning: str


class Designer:
    """LLM-driven design phase."""

    def __init__(
        self,
        llm: LLMClient,
        project_root: Path,
        prompts_dir: Path,
    ):
        self.llm = llm
        self.codebase = CodebaseCollector(project_root)
        self.prompts_dir = Path(prompts_dir)

    def design(
        self,
        focus: str,
        research_refs: list[dict[str, Any]],
        test_results: dict[str, Any] | None = None,
        benchmark_results: dict[str, Any] | None = None,
        knowledge: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Propose improvements based on research + codebase state."""
        # Collect codebase context
        codebase_summary = self.codebase.collect_compact()

        # Load prompts
        system_prompt = self._load_prompt("system.md")
        design_prompt = self._load_prompt("design.md").replace("{focus}", focus)

        # Build context message
        context_parts = [
            f"# Current Focus: {focus}\n",
            "## Codebase State\n",
            codebase_summary,
            "\n## Research Findings\n",
        ]

        if research_refs:
            for ref in research_refs[:10]:
                context_parts.append(
                    f"- [{ref.get('source', '?')}] {ref.get('title', '?')}: "
                    f"{ref.get('snippet', '')[:200]}\n"
                )
        else:
            context_parts.append("(no research references available)\n")

        if test_results:
            context_parts.append(f"\n## Test Status\n")
            context_parts.append(
                f"Passed: {test_results.get('passed', '?')}/"
                f"{test_results.get('total', '?')}\n"
            )
            failed = test_results.get("failed_tests", [])
            if failed:
                context_parts.append(f"Failed: {', '.join(failed)}\n")

        if benchmark_results and benchmark_results.get("regressions"):
            context_parts.append("\n## Benchmark Regressions\n")
            for r in benchmark_results["regressions"]:
                context_parts.append(f"- {r}\n")

        if knowledge:
            context_parts.append("\n## Previous Learnings\n")
            for k in knowledge[-5:]:
                for l in k.get("learnings", []):
                    context_parts.append(f"- {l}\n")

        context_msg = "".join(context_parts)

        # Call LLM
        messages = [
            Message(role="system", content=system_prompt),
            Message(role="user", content=f"{context_msg}\n\n---\n\n{design_prompt}\n\n"
                    "IMPORTANT: Return ONLY valid JSON matching the format above. "
                    "Choose the single most impactful improvement to implement. "
                    "Be specific about which files to modify and what tests to add."),
        ]

        result = self.llm.chat_json(messages)
        if result is None:
            return {
                "success": False,
                "summary": "LLM returned invalid JSON for design",
                "plan": None,
            }

        # Parse improvements
        improvements = result.get("improvements", [])
        return {
            "success": len(improvements) > 0,
            "summary": f"{len(improvements)} improvement(s) proposed"
                       + (f": {improvements[0].get('title', '?')}"
                          if improvements else ""),
            "plan": result,
            "improvements": improvements,
        }

    def _load_prompt(self, name: str) -> str:
        path = self.prompts_dir / name
        if path.exists():
            return path.read_text()
        return f"(prompt {name} not found)"
