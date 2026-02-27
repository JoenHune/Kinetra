"""
Codebase context collector — gathers project state for LLM prompts.

Provides a structured summary of the Kinetra codebase including:
  - File tree with sizes
  - Public API surface (headers)
  - Test names and status
  - Recent git changes
  - TODO / FIXME markers
"""

from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class FileInfo:
    path: str           # Relative to project root
    lines: int
    size_bytes: int
    language: str       # "cpp" | "hpp" | "py" | "yaml" | etc.


@dataclass
class CodebaseContext:
    """Structured snapshot of the project state."""
    files: list[FileInfo]
    public_headers: list[str]
    source_files: list[str]
    test_files: list[str]
    benchmark_files: list[str]
    todos: list[dict[str, Any]]          # {file, line, text}
    recent_commits: list[str]
    file_tree: str
    full_header_contents: dict[str, str]  # header path → content
    full_test_contents: dict[str, str]    # test path → content


class CodebaseCollector:
    """Collects codebase context for LLM prompts."""

    def __init__(self, project_root: Path):
        self.root = Path(project_root).resolve()

    def collect(self, include_contents: bool = True,
                max_file_size: int = 50000) -> CodebaseContext:
        """Gather full codebase context."""
        files = self._scan_files()
        headers = [f.path for f in files if f.path.startswith("include/")]
        sources = [f.path for f in files
                   if f.path.startswith("src/") and f.language == "cpp"]
        tests = [f.path for f in files
                 if f.path.startswith("tests/") and f.language == "cpp"]
        benchmarks = [f.path for f in files
                      if f.path.startswith("benchmarks/") and f.language == "cpp"]

        header_contents: dict[str, str] = {}
        test_contents: dict[str, str] = {}
        if include_contents:
            for h in headers:
                content = self._read_file(h, max_file_size)
                if content:
                    header_contents[h] = content
            for t in tests:
                content = self._read_file(t, max_file_size)
                if content:
                    test_contents[t] = content

        return CodebaseContext(
            files=files,
            public_headers=headers,
            source_files=sources,
            test_files=tests,
            benchmark_files=benchmarks,
            todos=self._find_todos(),
            recent_commits=self._recent_commits(),
            file_tree=self._file_tree(),
            full_header_contents=header_contents,
            full_test_contents=test_contents,
        )

    def collect_compact(self) -> str:
        """Return a compact text summary suitable for LLM context."""
        ctx = self.collect(include_contents=False)
        lines = [
            "# Kinetra Codebase Summary",
            "",
            f"## File Tree ({len(ctx.files)} source files)",
            ctx.file_tree,
            "",
            f"## Public Headers ({len(ctx.public_headers)})",
        ]
        for h in ctx.public_headers:
            lines.append(f"  - {h}")

        lines.append(f"\n## Source Files ({len(ctx.source_files)})")
        for s in ctx.source_files:
            lines.append(f"  - {s}")

        lines.append(f"\n## Tests ({len(ctx.test_files)})")
        for t in ctx.test_files:
            lines.append(f"  - {t}")

        if ctx.todos:
            lines.append(f"\n## TODOs ({len(ctx.todos)})")
            for td in ctx.todos[:20]:
                lines.append(f"  - {td['file']}:{td['line']} — {td['text']}")

        if ctx.recent_commits:
            lines.append("\n## Recent Commits")
            for c in ctx.recent_commits[:10]:
                lines.append(f"  - {c}")

        return "\n".join(lines)

    def read_files(self, paths: list[str], max_size: int = 50000) -> dict[str, str]:
        """Read specific files by relative path."""
        result: dict[str, str] = {}
        for p in paths:
            content = self._read_file(p, max_size)
            if content is not None:
                result[p] = content
        return result

    # ── Internal helpers ──────────────────────────────────────────────────

    def _scan_files(self) -> list[FileInfo]:
        """Scan project for source files."""
        files: list[FileInfo] = []
        for dirpath, _, filenames in os.walk(self.root):
            rel_dir = os.path.relpath(dirpath, self.root)
            # Skip build, .git, agent logs
            if any(skip in rel_dir for skip in
                   ["build", ".git", "node_modules", "__pycache__",
                    "agent/logs", "agent/reports"]):
                continue
            for fn in filenames:
                ext = os.path.splitext(fn)[1]
                lang_map = {
                    ".hpp": "hpp", ".h": "hpp", ".cpp": "cpp", ".cc": "cpp",
                    ".py": "py", ".yaml": "yaml", ".yml": "yaml",
                    ".md": "md", ".txt": "txt", ".cmake": "cmake",
                }
                lang = lang_map.get(ext, "")
                if not lang:
                    continue
                full = os.path.join(dirpath, fn)
                rel = os.path.relpath(full, self.root)
                try:
                    stat = os.stat(full)
                    line_count = sum(1 for _ in open(full, errors="ignore"))
                    files.append(FileInfo(
                        path=rel,
                        lines=line_count,
                        size_bytes=stat.st_size,
                        language=lang,
                    ))
                except OSError:
                    pass
        files.sort(key=lambda f: f.path)
        return files

    def _read_file(self, rel_path: str, max_size: int) -> str | None:
        """Read a file relative to project root."""
        full = self.root / rel_path
        if not full.exists() or full.stat().st_size > max_size:
            return None
        try:
            return full.read_text(errors="ignore")
        except OSError:
            return None

    def _find_todos(self) -> list[dict[str, Any]]:
        """Find TODO/FIXME/HACK markers in source files."""
        todos: list[dict[str, Any]] = []
        try:
            result = subprocess.run(
                ["grep", "-rn", "-E", r"(TODO|FIXME|HACK|XXX)\b",
                 "include/", "src/", "tests/", "benchmarks/"],
                capture_output=True, text=True, cwd=str(self.root),
                timeout=10,
            )
            for line in result.stdout.splitlines()[:50]:
                m = re.match(r"^(.+?):(\d+):(.*)$", line)
                if m:
                    todos.append({
                        "file": m.group(1),
                        "line": int(m.group(2)),
                        "text": m.group(3).strip(),
                    })
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        return todos

    def _recent_commits(self, n: int = 10) -> list[str]:
        """Get recent git commit messages."""
        try:
            result = subprocess.run(
                ["git", "log", "--oneline", f"-{n}"],
                capture_output=True, text=True, cwd=str(self.root),
                timeout=5,
            )
            return result.stdout.strip().splitlines() if result.returncode == 0 else []
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return []

    def _file_tree(self) -> str:
        """Generate a compact file tree."""
        try:
            result = subprocess.run(
                ["find", "include", "src", "tests", "benchmarks",
                 "-type", "f", "-name", "*.hpp", "-o", "-name", "*.cpp",
                 "-o", "-name", "*.h"],
                capture_output=True, text=True, cwd=str(self.root),
                timeout=5,
            )
            if result.returncode == 0:
                lines = sorted(result.stdout.strip().splitlines())
                return "\n".join(f"  {l}" for l in lines)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        return "(unavailable)"
