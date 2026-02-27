"""
Google Benchmark runner & regression detector.
"""

from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Benchmarker:
    build_dir: Path
    timeout: int = 600
    repetitions: int = 3
    baseline_file: str = ""
    regression_threshold: float = 5.0  # percent

    def __post_init__(self):
        self.build_dir = Path(self.build_dir)

    def _find_benchmarks(self) -> list[Path]:
        """Locate benchmark executables in the build tree."""
        candidates = []
        for pattern in ["bench_*", "benchmarks/bench_*", "**/bench_*"]:
            candidates.extend(self.build_dir.glob(pattern))
        return [p for p in candidates if p.is_file() and os.access(p, os.X_OK)]

    def run(self, iteration: int) -> dict[str, Any]:
        """Run all benchmarks, output JSON, compare with baseline."""
        import os  # required for os.access

        benchmarks = self._find_benchmarks()
        if not benchmarks:
            return {
                "success": True,
                "summary": "No benchmark executables found",
                "results": [],
            }

        all_results: list[dict] = []
        regressions: list[str] = []

        for bench_exe in benchmarks:
            out_file = self.build_dir / f"{bench_exe.stem}_iter{iteration:04d}.json"
            cmd = [
                str(bench_exe),
                f"--benchmark_repetitions={self.repetitions}",
                "--benchmark_format=json",
                f"--benchmark_out={out_file}",
            ]
            try:
                subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout,
                )
            except subprocess.TimeoutExpired:
                all_results.append({
                    "name": bench_exe.stem,
                    "error": "timeout",
                })
                continue

            if out_file.exists():
                with open(out_file) as f:
                    data = json.load(f)
                    all_results.append({
                        "name": bench_exe.stem,
                        "benchmarks": data.get("benchmarks", []),
                    })

        # ── Baseline comparison ─────────────────────────────────────────────
        baseline = self._load_baseline()
        if baseline:
            regressions = self._detect_regressions(baseline, all_results)

        # Save current as baseline if first run or no regressions
        if not baseline or not regressions:
            self._save_baseline(all_results)

        summary_parts = [f"{len(all_results)} benchmark suites"]
        if regressions:
            summary_parts.append(f"{len(regressions)} regressions")

        return {
            "success": len(regressions) == 0,
            "summary": " | ".join(summary_parts),
            "results": all_results,
            "regressions": regressions,
        }

    def _load_baseline(self) -> list[dict] | None:
        p = Path(self.baseline_file)
        if p.exists():
            with open(p) as f:
                return json.load(f)
        return None

    def _save_baseline(self, results: list[dict]) -> None:
        if not self.baseline_file:
            return
        p = Path(self.baseline_file)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w") as f:
            json.dump(results, f, indent=2)

    def _detect_regressions(
        self,
        baseline: list[dict],
        current: list[dict],
    ) -> list[str]:
        """Compare mean times; flag regressions above threshold."""
        regressions = []

        # Build lookup: benchmark_name -> mean real_time
        def _build_map(results: list[dict]) -> dict[str, float]:
            m: dict[str, float] = {}
            for suite in results:
                for b in suite.get("benchmarks", []):
                    name = b.get("name", "")
                    if name.endswith("_mean"):
                        m[name] = b.get("real_time", 0.0)
            return m

        base_map = _build_map(baseline)
        curr_map = _build_map(current)

        for name, curr_time in curr_map.items():
            base_time = base_map.get(name)
            if base_time and base_time > 0:
                pct_change = ((curr_time - base_time) / base_time) * 100
                if pct_change > self.regression_threshold:
                    regressions.append(
                        f"{name}: {pct_change:+.1f}% "
                        f"({base_time:.1f}→{curr_time:.1f})"
                    )

        return regressions


# Need os for _find_benchmarks
import os
