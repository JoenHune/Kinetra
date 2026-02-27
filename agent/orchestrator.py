#!/usr/bin/env python3
"""
Kinetra Development Agent — Orchestrator
=========================================
Drives the iterative AI-powered development cycle:
  Research → Design → Implement → Build → Test → Benchmark → Reflect

The Design and Implement phases call an LLM (OpenAI-compatible API) to
propose improvements and generate code.  If Build or Test fails after
implementation, the agent rolls back the changes and logs the failure.

Configuration: agent/config.yaml
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
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
from tools.designer import Designer
from tools.implementer import Implementer
from tools.backlog import Backlog
from tools.llm_client import LLMClient, LLMConfig, Message

console = Console()
log = logging.getLogger("kinetra.agent")

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
    task_title: str = ""
    learnings: list[str] = field(default_factory=list)
    files_changed: list[str] = field(default_factory=list)
    overall_success: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


# ── Orchestrator ──────────────────────────────────────────────────────────────

class Orchestrator:
    """Main agent loop — now with LLM-driven code generation."""

    def __init__(self, config_path: str):
        with open(config_path) as f:
            self.cfg = yaml.safe_load(f)

        self.agent_dir = Path(__file__).resolve().parent
        self.project_root = (self.agent_dir / self.cfg["project"]["root"]).resolve()
        self.build_dir = (self.agent_dir / self.cfg["project"]["build_dir"]).resolve()
        self.log_dir = self.agent_dir / self.cfg["agent"]["log_dir"]
        self.report_dir = self.agent_dir / self.cfg["agent"]["report_dir"]
        self.prompts_dir = self.agent_dir / "prompts"

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.report_dir.mkdir(parents=True, exist_ok=True)

        # ── LLM client ───────────────────────────────────────────────────
        llm_cfg = self.cfg.get("llm", {})
        self.llm_enabled = llm_cfg.get("enabled", True)
        self.llm: LLMClient | None = None
        if self.llm_enabled:
            try:
                self.llm = LLMClient.from_config(llm_cfg)
                console.print(
                    f"[dim]LLM: {llm_cfg.get('provider', 'openai')}/"
                    f"{llm_cfg.get('model', 'gpt-4o')}[/]"
                )
            except Exception as e:
                console.print(f"[yellow]LLM init failed: {e} — running in CI-only mode[/]")
                self.llm_enabled = False

        # ── Tool instances ────────────────────────────────────────────────
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
        self.backlog = Backlog(self.log_dir / "backlog.json")

        # LLM-powered tools (only if LLM available)
        self.designer: Designer | None = None
        self.implementer: Implementer | None = None
        if self.llm_enabled and self.llm:
            self.designer = Designer(
                llm=self.llm,
                project_root=self.project_root,
                prompts_dir=self.prompts_dir,
            )
            self.implementer = Implementer(
                llm=self.llm,
                project_root=self.project_root,
                prompts_dir=self.prompts_dir,
            )

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

    # ── Individual phases ─────────────────────────────────────────────────

    def phase_research(self, focus: str) -> dict[str, Any]:
        """Search for relevant papers and repos."""
        results = self.researcher.search(focus)
        return {
            "success": True,
            "summary": f"Found {len(results)} references for '{focus}'",
            "references": results,
        }

    def phase_design(self, focus: str, research_refs: list,
                     test_results: dict | None = None) -> dict[str, Any]:
        """LLM proposes improvements based on research + codebase state."""
        if not self.designer:
            return {"success": True, "summary": "LLM not configured — skipping design",
                    "plan": None}

        # Load recent knowledge
        knowledge = self._load_knowledge()

        return self.designer.design(
            focus=focus,
            research_refs=research_refs,
            test_results=test_results,
            knowledge=knowledge,
        )

    def phase_implement(self, plan: dict | None) -> dict[str, Any]:
        """LLM generates and applies code changes."""
        if not self.implementer or not plan:
            return {"success": True, "summary": "No implementation needed",
                    "changes": []}
        return self.implementer.implement(plan)

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
            elif p.phase == "implement" and p.details.get("changes"):
                changes = p.details["changes"]
                learnings.append(
                    f"[IMPLEMENTED] {record.task_title}: "
                    f"{len(changes)} file(s) changed"
                )

        # LLM reflection (if available)
        if self.llm_enabled and self.llm and record.phases:
            llm_reflection = self._llm_reflect(record)
            if llm_reflection:
                learnings.extend(llm_reflection)

        # Persist to append-only knowledge base
        kb_path = self.agent_dir / self.cfg["reflect"]["knowledge_base"]
        kb_path.parent.mkdir(parents=True, exist_ok=True)
        entry = {
            "iteration": record.iteration,
            "timestamp": record.timestamp,
            "focus": record.focus,
            "task": record.task_title,
            "learnings": learnings,
            "files_changed": record.files_changed,
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
        """Execute one full development cycle with LLM-driven code generation."""
        record = IterationRecord(
            iteration=iteration,
            timestamp=datetime.now(timezone.utc).isoformat(),
            focus=focus,
        )

        console.print(Panel(
            f"[bold]Iteration {iteration}[/]  —  Focus: {focus}"
            + (f"\n{self.backlog.summary()}" if self.backlog.tasks else ""),
            style="bold blue",
        ))

        # ── 1. Research ───────────────────────────────────────────────────
        pr = self._run_phase("research", self.phase_research, focus=focus)
        record.phases.append(pr)
        research_refs = pr.details.get("references", [])

        # ── 2. Design (LLM) ──────────────────────────────────────────────
        pr = self._run_phase("design", self.phase_design,
                             focus=focus, research_refs=research_refs)
        record.phases.append(pr)
        plan = pr.details.get("plan")

        # Add proposed tasks to backlog
        if plan:
            added = self.backlog.add_from_plan(plan, iteration=iteration)
            if added:
                console.print(f"  [dim]Backlog: +{len(added)} task(s)[/]")

        # ── 3. Implement (LLM) ───────────────────────────────────────────
        current_task = self.backlog.next_task()
        impl_plan = plan  # Use design plan for implementation

        if current_task:
            record.task_title = current_task.title
            self.backlog.start(current_task.id)
            console.print(
                f"  [dim]Working on: #{current_task.id} {current_task.title}[/]"
            )

        pr = self._run_phase("implement", self.phase_implement, plan=impl_plan)
        record.phases.append(pr)
        files_changed = [c["path"] for c in pr.details.get("changes", [])]
        record.files_changed = files_changed
        code_was_changed = pr.success and len(files_changed) > 0

        # ── 4. Build ─────────────────────────────────────────────────────
        pr = self._run_phase("build", self.phase_build)
        record.phases.append(pr)

        if not pr.success and code_was_changed:
            # Rollback on build failure
            console.print("[yellow]Build failed after LLM changes — rolling back[/]")
            self._rollback(current_task)
            # Rebuild to restore working state
            self._run_phase("build (restore)", self.phase_build)
            record.overall_success = False
            self._finalize(record, iteration)
            return record

        if not pr.success:
            console.print("[yellow]Build failed — skipping test & benchmark[/]")
            record.overall_success = False
            self._finalize(record, iteration)
            return record

        # ── 5. Test ──────────────────────────────────────────────────────
        pr = self._run_phase("test", self.phase_test)
        record.phases.append(pr)

        if not pr.success and code_was_changed:
            # Rollback on test failure
            console.print("[yellow]Tests failed after LLM changes — rolling back[/]")
            self._rollback(current_task)
            # Rebuild & re-test to confirm clean state
            self._run_phase("build (restore)", self.phase_build)
            record.overall_success = False
            self._finalize(record, iteration)
            return record

        # Mark task as done if tests pass
        if current_task and code_was_changed and pr.success:
            self.backlog.complete(current_task.id,
                                 files_changed=files_changed,
                                 iteration=iteration)
            # Clean up backup files
            if self.implementer:
                self.implementer.clean_backups()

            # Auto-commit if configured
            if self.cfg["agent"].get("auto_commit", False):
                self._git_commit(record)

        # ── 6. Benchmark (only if tests pass) ────────────────────────────
        if pr.success:
            pr = self._run_phase("benchmark", self.phase_benchmark,
                                 iteration=iteration)
            record.phases.append(pr)

        # ── 7. Reflect ───────────────────────────────────────────────────
        pr = self._run_phase("reflect", self.phase_reflect, record=record)
        record.phases.append(pr)

        record.overall_success = all(
            p.success for p in record.phases
            if p.phase not in ("build (restore)",)
        )
        self._finalize(record, iteration)
        return record

    # ── Helpers ───────────────────────────────────────────────────────────

    def _rollback(self, task=None) -> None:
        """Rollback LLM changes and mark task as failed."""
        if self.implementer:
            n = self.implementer.rollback()
            console.print(f"  [yellow]Rolled back {n} file(s)[/]")
        if task:
            self.backlog.fail(task.id, notes="Build/test failed after implementation")

    def _git_commit(self, record: IterationRecord) -> None:
        """Auto-commit changes if enabled."""
        try:
            subprocess.run(
                ["git", "add", "-A"],
                cwd=str(self.project_root), capture_output=True, timeout=10,
            )
            msg = (
                f"agent(iter-{record.iteration}): {record.task_title}\n\n"
                f"Focus: {record.focus}\n"
                f"Files: {', '.join(record.files_changed)}"
            )
            subprocess.run(
                ["git", "commit", "-m", msg],
                cwd=str(self.project_root), capture_output=True, timeout=10,
            )
        except Exception as e:
            log.warning("Git commit failed: %s", e)

    def _finalize(self, record: IterationRecord, iteration: int) -> None:
        """Save record and print summary."""
        self._save_record(record)
        self._print_summary(record)

    def _save_record(self, record: IterationRecord) -> None:
        """Persist iteration record as JSON."""
        path = self.log_dir / f"iteration_{record.iteration:04d}.json"
        with open(path, "w") as f:
            json.dump(record.to_dict(), f, indent=2)

    def _print_summary(self, record: IterationRecord) -> None:
        """Print a rich table summarising the iteration."""
        table = Table(title=f"Iteration {record.iteration} Summary"
                      + (f" — {record.task_title}" if record.task_title else ""))
        table.add_column("Phase", style="cyan")
        table.add_column("Status")
        table.add_column("Time (s)", justify="right")
        table.add_column("Summary")
        for p in record.phases:
            status = "[green]✓[/]" if p.success else "[red]✗[/]"
            table.add_row(p.phase, status, f"{p.duration_s:.1f}", p.summary)
        console.print(table)
        if record.files_changed:
            console.print(f"  [dim]Files changed: {', '.join(record.files_changed)}[/]")

    def _load_knowledge(self) -> list[dict[str, Any]]:
        """Load knowledge base entries."""
        kb_path = self.agent_dir / self.cfg["reflect"]["knowledge_base"]
        if not kb_path.exists():
            return []
        entries = []
        for line in kb_path.read_text().strip().splitlines():
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                pass
        return entries

    def _llm_reflect(self, record: IterationRecord) -> list[str]:
        """Use LLM to generate deeper reflections."""
        if not self.llm:
            return []
        try:
            phase_summary = "\n".join(
                f"- {p.phase}: {'OK' if p.success else 'FAIL'} — {p.summary}"
                for p in record.phases
            )
            prompt = (
                f"You are reflecting on iteration {record.iteration} of the "
                f"Kinetra development agent.\n\n"
                f"Focus: {record.focus}\n"
                f"Task: {record.task_title or 'none'}\n\n"
                f"Phase results:\n{phase_summary}\n\n"
                f"Files changed: {', '.join(record.files_changed) or 'none'}\n\n"
                "Provide 1-3 concise technical learnings (one sentence each). "
                "Return JSON: {\"learnings\": [\"...\"]}"
            )
            result = self.llm.chat_json([
                Message(role="system", content="You are a robotics software engineer."),
                Message(role="user", content=prompt),
            ])
            return result.get("learnings", []) if result else []
        except Exception:
            return []

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
    parser.add_argument(
        "--no-llm", action="store_true",
        help="Disable LLM (CI-only mode: build/test/benchmark only)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable debug logging",
    )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format="%(name)s %(message)s")

    if args.no_llm:
        # Override config
        orchestrator = Orchestrator(args.config)
        orchestrator.llm_enabled = False
        orchestrator.designer = None
        orchestrator.implementer = None
    else:
        orchestrator = Orchestrator(args.config)

    orchestrator.run(max_iterations=args.iterations)


if __name__ == "__main__":
    main()
