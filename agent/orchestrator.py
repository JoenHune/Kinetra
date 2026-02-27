#!/usr/bin/env python3
"""
Kinetra Development Agent — Orchestrator
=========================================
Drives the iterative development cycle:
  Research → Design → Implement → Test → Benchmark → Reflect

Each iteration is logged as a JSON record under logs/ and a human-readable
Markdown report is generated under reports/.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

# ── Local tool imports ────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.builder import Builder
from tools.tester import Tester
from tools.benchmarker import Benchmarker
from tools.reporter import Reporter
from tools.researcher import Researcher

console = Console()

# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class PhaseResult:
    """Result of a single phase within an iteration."""
    phase: str
    success: bool
    duration_s: float = 0.0
    summary: str = ""
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class IterationRecord:
    """Complete record of one development iteration."""
    iteration: int
    timestamp: str
    phases: list[PhaseResult] = field(default_factory=list)
    focus: str = ""
    learnings: list[str] = field(default_factory=list)
    overall_success: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


# ── Orchestrator ──────────────────────────────────────────────────────────────

class Orchestrator:
    """Main agent loop."""

    def __init__(self, config_path: str):
        with open(config_path) as f:
            self.cfg = yaml.safe_load(f)

        self.agent_dir = Path(__file__).resolve().parent
        self.project_root = (self.agent_dir / self.cfg["project"]["root"]).resolve()
        self.build_dir = (self.agent_dir / self.cfg["project"]["build_dir"]).resolve()
        self.log_dir = self.agent_dir / self.cfg["agent"]["log_dir"]
        self.report_dir = self.agent_dir / self.cfg["agent"]["report_dir"]

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.report_dir.mkdir(parents=True, exist_ok=True)

        # Tool instances
        self.builder = Builder(
            source_dir=self.project_root,
            build_dir=self.build_dir,
            cmake_options=self.cfg["project"].get("cmake_options", []),
            parallel_jobs=self.cfg["build"].get("parallel_jobs", 0),
            timeout=self.cfg["build"]["timeout_seconds"],
        )
        self.tester = Tester(
            build_dir=self.build_dir,
            timeout=self.cfg["test"]["timeout_seconds"],
        )
        self.benchmarker = Benchmarker(
            build_dir=self.build_dir,
            timeout=self.cfg["benchmark"]["timeout_seconds"],
            repetitions=self.cfg["benchmark"]["repetitions"],
            baseline_file=str(self.agent_dir / self.cfg["benchmark"]["baseline_file"]),
            regression_threshold=self.cfg["benchmark"]["regression_threshold_pct"],
        )
        self.researcher = Researcher(
            cache_dir=str(self.agent_dir / self.cfg["research"]["cache_dir"]),
            enabled=self.cfg["research"]["enabled"],
        )
        self.reporter = Reporter(report_dir=self.report_dir)

        # History
        self.history: list[IterationRecord] = []

    # ── Phase runners ─────────────────────────────────────────────────────

    def _run_phase(self, name: str, fn, **kwargs) -> PhaseResult:
        """Execute a phase, timing it and catching errors."""
        console.rule(f"[bold cyan]Phase: {name}")
        t0 = time.monotonic()
        try:
            result = fn(**kwargs)
            elapsed = time.monotonic() - t0
            pr = PhaseResult(
                phase=name,
                success=result.get("success", True),
                duration_s=round(elapsed, 2),
                summary=result.get("summary", ""),
                details=result,
            )
        except Exception as exc:
            elapsed = time.monotonic() - t0
            pr = PhaseResult(
                phase=name,
                success=False,
                duration_s=round(elapsed, 2),
                summary=f"Exception: {exc}",
                details={"error": str(exc)},
            )
        status = "[green]OK[/]" if pr.success else "[red]FAIL[/]"
        console.print(f"  {status}  {pr.summary}  ({pr.duration_s:.1f}s)")
        return pr

    def phase_research(self, focus: str) -> dict[str, Any]:
        """Search for relevant papers and repos."""
        results = self.researcher.search(focus)
        return {
            "success": True,
            "summary": f"Found {len(results)} references for '{focus}'",
            "references": results,
        }

    def phase_build(self) -> dict[str, Any]:
        """Configure and build the project."""
        return self.builder.build()

    def phase_test(self) -> dict[str, Any]:
        """Run the test suite via CTest."""
        return self.tester.run()

    def phase_benchmark(self, iteration: int) -> dict[str, Any]:
        """Run benchmarks and compare with baseline."""
        return self.benchmarker.run(iteration)

    def phase_reflect(self, record: IterationRecord) -> dict[str, Any]:
        """Produce learnings and persist to knowledge base."""
        learnings: list[str] = []

        for p in record.phases:
            if not p.success:
                learnings.append(f"[FIX NEEDED] {p.phase}: {p.summary}")

        # Persist to append-only knowledge base
        kb_path = self.agent_dir / self.cfg["reflect"]["knowledge_base"]
        kb_path.parent.mkdir(parents=True, exist_ok=True)
        entry = {
            "iteration": record.iteration,
            "timestamp": record.timestamp,
            "focus": record.focus,
            "learnings": learnings,
        }
        with open(kb_path, "a") as f:
            f.write(json.dumps(entry) + "\n")

        record.learnings = learnings
        return {
            "success": True,
            "summary": f"{len(learnings)} learnings recorded",
            "learnings": learnings,
        }

    # ── Main loop ─────────────────────────────────────────────────────────

    def run_iteration(self, iteration: int, focus: str) -> IterationRecord:
        """Execute one full development cycle."""
        record = IterationRecord(
            iteration=iteration,
            timestamp=datetime.now(timezone.utc).isoformat(),
            focus=focus,
        )

        console.print(Panel(
            f"[bold]Iteration {iteration}[/]  —  Focus: {focus}",
            style="bold blue",
        ))

        # 1. Research
        pr = self._run_phase("research", self.phase_research, focus=focus)
        record.phases.append(pr)

        # 2. Build
        pr = self._run_phase("build", self.phase_build)
        record.phases.append(pr)
        if not pr.success:
            console.print("[yellow]Build failed — skipping test & benchmark.")
            record.overall_success = False
            self._save_record(record)
            return record

        # 3. Test
        pr = self._run_phase("test", self.phase_test)
        record.phases.append(pr)

        # 4. Benchmark (only if tests pass)
        if pr.success:
            pr = self._run_phase("benchmark", self.phase_benchmark, iteration=iteration)
            record.phases.append(pr)

        # 5. Reflect
        pr = self._run_phase("reflect", self.phase_reflect, record=record)
        record.phases.append(pr)

        record.overall_success = all(p.success for p in record.phases)
        self._save_record(record)

        # Print summary table
        self._print_summary(record)
        return record

    def _save_record(self, record: IterationRecord) -> None:
        """Persist iteration record as JSON."""
        path = self.log_dir / f"iteration_{record.iteration:04d}.json"
        with open(path, "w") as f:
            json.dump(record.to_dict(), f, indent=2)

    def _print_summary(self, record: IterationRecord) -> None:
        """Print a rich table summarising the iteration."""
        table = Table(title=f"Iteration {record.iteration} Summary")
        table.add_column("Phase", style="cyan")
        table.add_column("Status")
        table.add_column("Time (s)", justify="right")
        table.add_column("Summary")
        for p in record.phases:
            status = "[green]✓[/]" if p.success else "[red]✗[/]"
            table.add_row(p.phase, status, f"{p.duration_s:.1f}", p.summary)
        console.print(table)

    def run(self, max_iterations: int | None = None) -> None:
        """Run the full agent loop for N iterations."""
        n = max_iterations or self.cfg["agent"]["max_iterations"]
        focus_areas = self.cfg["reflect"]["focus_areas"]

        for i in range(1, n + 1):
            focus = focus_areas[(i - 1) % len(focus_areas)]
            record = self.run_iteration(i, focus)
            self.history.append(record)

            # Generate cumulative report
            self.reporter.generate(self.history)

        console.print(Panel("[bold green]Agent loop complete[/]"))


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Kinetra Development Agent")
    parser.add_argument(
        "--iterations", "-n", type=int, default=None,
        help="Number of iterations (overrides config)",
    )
    parser.add_argument(
        "--config", "-c", type=str,
        default=str(Path(__file__).resolve().parent / "config.yaml"),
        help="Path to config.yaml",
    )
    args = parser.parse_args()

    orchestrator = Orchestrator(args.config)
    orchestrator.run(max_iterations=args.iterations)


if __name__ == "__main__":
    main()
