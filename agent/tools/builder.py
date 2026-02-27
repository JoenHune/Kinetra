"""
CMake configure + build wrapper.
"""

from __future__ import annotations

import multiprocessing
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Builder:
    source_dir: Path
    build_dir: Path
    cmake_options: list[str]
    parallel_jobs: int = 0
    timeout: int = 120

    def __post_init__(self):
        self.source_dir = Path(self.source_dir)
        self.build_dir = Path(self.build_dir)
        if self.parallel_jobs <= 0:
            self.parallel_jobs = max(1, multiprocessing.cpu_count())

    def _run(self, cmd: list[str], label: str) -> dict[str, Any]:
        """Run a subprocess, return result dict."""
        try:
            result = subprocess.run(
                cmd,
                cwd=str(self.build_dir),
                capture_output=True,
                text=True,
                timeout=self.timeout,
            )
            success = result.returncode == 0
            return {
                "success": success,
                "summary": f"{label}: {'OK' if success else 'FAILED'}",
                "returncode": result.returncode,
                "stdout": result.stdout[-2000:] if result.stdout else "",
                "stderr": result.stderr[-2000:] if result.stderr else "",
            }
        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "summary": f"{label}: TIMEOUT ({self.timeout}s)",
                "returncode": -1,
                "stdout": "",
                "stderr": "Timeout",
            }

    def configure(self) -> dict[str, Any]:
        """Run cmake configure step."""
        self.build_dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            "cmake",
            str(self.source_dir),
            *self.cmake_options,
        ]
        return self._run(cmd, "Configure")

    def compile(self) -> dict[str, Any]:
        """Run cmake --build."""
        cmd = [
            "cmake", "--build", ".",
            "--parallel", str(self.parallel_jobs),
        ]
        return self._run(cmd, "Build")

    def build(self) -> dict[str, Any]:
        """Full configure + build pipeline."""
        cfg = self.configure()
        if not cfg["success"]:
            return cfg
        bld = self.compile()
        # Merge outputs
        bld["configure_output"] = cfg.get("stdout", "")
        return bld
