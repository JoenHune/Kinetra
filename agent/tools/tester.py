"""
CTest runner & result parser.
"""

from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Tester:
    build_dir: Path
    timeout: int = 300

    def __post_init__(self):
        self.build_dir = Path(self.build_dir)

    def run(self) -> dict[str, Any]:
        """Run ctest and parse results."""
        try:
            result = subprocess.run(
                ["ctest", "--test-dir", str(self.build_dir),
                 "--output-on-failure", "--timeout", str(self.timeout)],
                capture_output=True,
                text=True,
                timeout=self.timeout + 30,
            )
        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "summary": f"CTest timeout ({self.timeout}s)",
                "passed": 0,
                "failed": 0,
                "total": 0,
            }

        # Parse summary line like "100% tests passed, 0 tests failed out of 12"
        passed = failed = total = 0
        m = re.search(
            r"(\d+)% tests passed,\s*(\d+) tests? failed out of (\d+)",
            result.stdout + result.stderr,
        )
        if m:
            failed = int(m.group(2))
            total = int(m.group(3))
            passed = total - failed

        # Collect individual failed test names
        failed_tests: list[str] = []
        for line in (result.stdout + result.stderr).splitlines():
            # CTest failure lines look like "  3 - test_name (Failed)"
            fm = re.match(r"\s*\d+\s*-\s*(\S+)\s*\(Failed\)", line)
            if fm:
                failed_tests.append(fm.group(1))

        success = result.returncode == 0 and failed == 0

        return {
            "success": success,
            "summary": f"{passed}/{total} passed" + (
                f" — FAILURES: {', '.join(failed_tests)}" if failed_tests else ""
            ),
            "passed": passed,
            "failed": failed,
            "total": total,
            "failed_tests": failed_tests,
            "stdout": result.stdout[-3000:] if result.stdout else "",
            "stderr": result.stderr[-1000:] if result.stderr else "",
        }
