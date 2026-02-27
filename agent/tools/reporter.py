"""
Markdown & HTML report generation from iteration history.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Reporter:
    report_dir: Path

    def __post_init__(self):
        self.report_dir = Path(self.report_dir)
        self.report_dir.mkdir(parents=True, exist_ok=True)

    def generate(self, history: list) -> None:
        """Generate cumulative Markdown report and a JSON summary."""
        self._generate_markdown(history)
        self._generate_json(history)

    def _generate_markdown(self, history: list) -> None:
        lines = [
            "# Kinetra Agent — Development Report",
            "",
            f"**Total iterations:** {len(history)}",
            "",
        ]

        # Summary table
        lines.append("| Iter | Focus | Build | Test | Bench | Time (s) |")
        lines.append("|------|-------|-------|------|-------|----------|")

        for rec in history:
            phases = {p["phase"]: p for p in rec.get("phases", []) if isinstance(rec, dict)}
            if not isinstance(rec, dict):
                # dataclass
                phases = {p.phase: p for p in rec.phases}

            def _status(phase_name: str) -> str:
                p = phases.get(phase_name)
                if p is None:
                    return "—"
                ok = p["success"] if isinstance(p, dict) else p.success
                return "✓" if ok else "✗"

            def _total_time() -> float:
                if isinstance(rec, dict):
                    return sum(p.get("duration_s", 0) for p in rec.get("phases", []))
                return sum(p.duration_s for p in rec.phases)

            it = rec["iteration"] if isinstance(rec, dict) else rec.iteration
            focus = rec["focus"] if isinstance(rec, dict) else rec.focus

            lines.append(
                f"| {it} | {focus} | {_status('build')} | {_status('test')} "
                f"| {_status('benchmark')} | {_total_time():.1f} |"
            )

        lines.append("")

        # Detail sections
        for rec in history:
            it = rec["iteration"] if isinstance(rec, dict) else rec.iteration
            focus = rec["focus"] if isinstance(rec, dict) else rec.focus
            learnings = rec.get("learnings", []) if isinstance(rec, dict) else rec.learnings

            lines.append(f"## Iteration {it} — {focus}")
            lines.append("")
            if learnings:
                for l in learnings:
                    lines.append(f"- {l}")
                lines.append("")

        report_path = self.report_dir / "report.md"
        report_path.write_text("\n".join(lines))

    def _generate_json(self, history: list) -> None:
        """Write raw JSON for programmatic consumption."""
        data = []
        for rec in history:
            if isinstance(rec, dict):
                data.append(rec)
            else:
                data.append(rec.to_dict())

        path = self.report_dir / "report.json"
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
