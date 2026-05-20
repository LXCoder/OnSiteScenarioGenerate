from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Empty

from ..docker.runner import build_runner
from ..extensions import task_queue


@dataclass(slots=True)
class ActiveTask:
    task_id: str
    cancel_event: threading.Event


class TaskWorker:
    def __init__(self, config, task_service):
        self.config = config
        self.task_service = task_service
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._active_task: ActiveTask | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        for task in self.task_service.repository.list_pending_tasks():
            task_queue.put(task["task_id"])

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def enqueue(self, task_id: str) -> None:
        task_queue.put(task_id)

    def cancel(self, task_id: str) -> bool:
        with self._lock:
            if self._active_task and self._active_task.task_id == task_id:
                self._active_task.cancel_event.set()
                return True

        task = self.task_service.get_task(task_id)
        if not task:
            return False
        self.task_service.cancel_task(task_id)
        return True

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                task_id = task_queue.get(timeout=0.5)
            except Empty:
                continue

            task = self.task_service.get_task(task_id)
            if not task:
                continue

            if task["status"] == "CANCELLED":
                continue

            self.task_service.mark_running(task_id)

            cancel_event = threading.Event()
            with self._lock:
                self._active_task = ActiveTask(task_id=task_id, cancel_event=cancel_event)

            try:
                runner = build_runner(self.config)
                result = runner.run(
                    workspace_root=Path(self.config["PROJECT_ROOT"]),
                    batch_config_path=Path(task["batch_config_path"]),
                    stdout_path=Path(task["log_dir"]) / "stdout.log",
                    stderr_path=Path(task["log_dir"]) / "stderr.log",
                    timeout_seconds=int(self.config["DEFAULT_TIMEOUT_SECONDS"]),
                    cancel_event=cancel_event,
                )
                self.task_service.mark_finished(
                    task_id,
                    result.status,
                    result.exit_code,
                    result.message,
                )
            except Exception as exc:
                self.task_service.mark_finished(
                    task_id,
                    "FAILED",
                    None,
                    f"Worker failed: {exc}",
                )
            finally:
                with self._lock:
                    self._active_task = None
                time.sleep(0.1)
