"""
Task backlog — tracks improvement ideas across iterations.

Persists as a JSON file; each task has a status lifecycle:
  proposed → accepted → in_progress → done | failed | deferred
"""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

log = logging.getLogger("kinetra.agent.backlog")


@dataclass
class Task:
    id: int
    title: str
    description: str = ""
    priority: int = 5                 # 1 = highest
    status: str = "proposed"          # proposed|accepted|in_progress|done|failed|deferred
    focus_area: str = ""
    proposed_at: str = ""
    completed_at: str = ""
    iteration_proposed: int = 0
    iteration_completed: int = 0
    source: str = ""                  # "llm" | "research" | "test_failure" | "manual"
    files_changed: list[str] = field(default_factory=list)
    notes: str = ""


class Backlog:
    """Persistent task backlog."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.tasks: list[Task] = []
        self._load()

    def _load(self) -> None:
        if self.path.exists():
            try:
                data = json.loads(self.path.read_text())
                self.tasks = [Task(**t) for t in data]
            except (json.JSONDecodeError, TypeError) as e:
                log.warning("Failed to load backlog: %s", e)
                self.tasks = []

    def _save(self) -> None:
        self.path.write_text(json.dumps(
            [asdict(t) for t in self.tasks], indent=2
        ))

    def _next_id(self) -> int:
        return max((t.id for t in self.tasks), default=0) + 1

    def add(self, title: str, description: str = "", priority: int = 5,
            focus_area: str = "", source: str = "llm",
            iteration: int = 0) -> Task:
        """Add a new task to the backlog."""
        # Check for duplicates (by title similarity)
        for t in self.tasks:
            if t.title.lower().strip() == title.lower().strip():
                log.info("Duplicate task skipped: %s", title)
                return t

        task = Task(
            id=self._next_id(),
            title=title,
            description=description,
            priority=priority,
            status="proposed",
            focus_area=focus_area,
            source=source,
            proposed_at=datetime.now(timezone.utc).isoformat(),
            iteration_proposed=iteration,
        )
        self.tasks.append(task)
        self._save()
        log.info("Added task #%d: %s (priority=%d)", task.id, title, priority)
        return task

    def add_from_plan(self, plan: dict[str, Any], iteration: int = 0) -> list[Task]:
        """Add tasks from a design plan."""
        added: list[Task] = []
        for imp in plan.get("improvements", []):
            task = self.add(
                title=imp.get("title", "untitled"),
                description=imp.get("description", imp.get("pseudocode", "")),
                priority=imp.get("priority", 5),
                focus_area=imp.get("focus_area", ""),
                source="llm",
                iteration=iteration,
            )
            if task.status == "proposed":
                added.append(task)
        return added

    def next_task(self) -> Task | None:
        """Get the highest-priority pending task."""
        candidates = [t for t in self.tasks
                      if t.status in ("proposed", "accepted")]
        if not candidates:
            return None
        candidates.sort(key=lambda t: (t.priority, t.id))
        return candidates[0]

    def start(self, task_id: int) -> Task | None:
        """Mark a task as in-progress."""
        task = self.get(task_id)
        if task:
            task.status = "in_progress"
            self._save()
        return task

    def complete(self, task_id: int, files_changed: list[str] | None = None,
                 iteration: int = 0) -> Task | None:
        """Mark a task as done."""
        task = self.get(task_id)
        if task:
            task.status = "done"
            task.completed_at = datetime.now(timezone.utc).isoformat()
            task.iteration_completed = iteration
            if files_changed:
                task.files_changed = files_changed
            self._save()
        return task

    def fail(self, task_id: int, notes: str = "") -> Task | None:
        """Mark a task as failed."""
        task = self.get(task_id)
        if task:
            task.status = "failed"
            task.notes = notes
            self._save()
        return task

    def defer(self, task_id: int) -> Task | None:
        """Defer a task for later."""
        task = self.get(task_id)
        if task:
            task.status = "deferred"
            self._save()
        return task

    def get(self, task_id: int) -> Task | None:
        for t in self.tasks:
            if t.id == task_id:
                return t
        return None

    def pending_count(self) -> int:
        return sum(1 for t in self.tasks if t.status in ("proposed", "accepted"))

    def summary(self) -> str:
        """Human-readable summary."""
        by_status: dict[str, int] = {}
        for t in self.tasks:
            by_status[t.status] = by_status.get(t.status, 0) + 1
        parts = [f"{v} {k}" for k, v in sorted(by_status.items())]
        return f"Backlog: {len(self.tasks)} tasks ({', '.join(parts)})"
