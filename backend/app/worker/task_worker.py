from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Empty

from ..docker.runner import build_runner
from ..extensions import task_queue
from ..services.task_service import AccessContext

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class ActiveTask:
    task_id: str
    cancel_event: threading.Event


class TaskWorker:
    def __init__(self, app, config, task_service, submit_service=None):
        self.app = app
        self.config = config
        self.task_service = task_service
        self.submit_service = submit_service
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._active_task: ActiveTask | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        with self.app.app_context():
            for task in self.task_service.repository.list_pending_tasks():
                task_queue.put(task["task_id"])
            logger.info("reloaded pending tasks into worker queue")

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("task worker thread started")

    def enqueue(self, task_id: str) -> None:
        task_queue.put(task_id)
        logger.info("task enqueued: %s", task_id)

    def cancel(self, task_id: str, *, access: AccessContext) -> bool:
        task = self.task_service.get_task(task_id, access=access)
        if not task:
            return False

        with self._lock:
            if self._active_task and self._active_task.task_id == task_id:
                self._active_task.cancel_event.set()
                logger.info("cancel requested for running task: %s", task_id)
                return True

        self.task_service.cancel_task(task_id, access=access)
        logger.info("task cancelled before start: %s", task_id)
        return True

    def _loop(self) -> None:
        with self.app.app_context():
            while not self._stop_event.is_set():
                try:
                    task_id = task_queue.get(timeout=0.5)
                except Empty:
                    continue

                task = self.task_service.get_task(task_id)
                if not task:
                    logger.warning("task not found when worker consumed queue item: %s", task_id)
                    continue

                if task["status"] == "CANCELLED":
                    logger.info("skip cancelled task: %s", task_id)
                    continue

                self.task_service.mark_running(task_id)
                if self.submit_service is not None:
                    self.submit_service.mark_testing(task_id)
                logger.info("task started: %s", task_id)

                cancel_event = threading.Event()
                with self._lock:
                    self._active_task = ActiveTask(task_id=task_id, cancel_event=cancel_event)

                try:
                    upload_info = self._get_upload_info(task["request_json"])
                    runner = build_runner(self.config, upload_info)
                    result = runner.run(
                        workspace_root=Path(self.config["PROJECT_ROOT"]),
                        batch_config_path=Path(task["batch_config_path"]),
                        stdout_path=Path(task["log_dir"]) / "stdout.log",
                        stderr_path=Path(task["log_dir"]) / "stderr.log",
                        output_dir=Path(task["output_dir"]),
                        timeout_seconds=int(self.config["DEFAULT_TIMEOUT_SECONDS"]),
                        cancel_event=cancel_event,
                    )
                    self.task_service.mark_finished(
                        task_id,
                        result.status,
                        result.exit_code,
                        result.message,
                    )
                    if self.submit_service is not None:
                        self._finalize_submit(task_id, result)
                    logger.info(
                        "task finished: %s status=%s exit_code=%s",
                        task_id,
                        result.status,
                        result.exit_code,
                    )
                except Exception as exc:
                    logger.exception("worker execution failed for task: %s", task_id)
                    self.task_service.mark_finished(
                        task_id,
                        "FAILED",
                        None,
                        f"Worker failed: {exc}",
                    )
                    if self.submit_service is not None:
                        self.submit_service.mark_finished(
                            task_id,
                            status="FAILED",
                            score=None,
                        )
                finally:
                    with self._lock:
                        self._active_task = None
                    time.sleep(0.1)

    def _get_upload_info(self, request_json):
        with open(request_json, "r") as f:
            req_data = json.load(f)
            return req_data.get("uploads", [])

    def _finalize_submit(self, task_id: str, result) -> None:
        submit = self.submit_service.get_submit(task_id)
        if not submit:
            return

        if result.status != "SUCCESS":
            self.submit_service.mark_finished(
                task_id,
                status="FAILED",
                score=None,
            )
            return

        access = AccessContext(
            user_id=submit["submitterId"],
            username=submit["submitterId"],
            is_admin=False,
        )
        try:
            evaluation = self.task_service.get_evaluation_result(task_id, access=access)
            score = None
            if evaluation:
                score = evaluation.get("summary", {}).get("total_100_avg")
                if score is not None:
                    score = float(score)

            self.submit_service.mark_finished(
                task_id,
                status="SUCCESS",
                score=score,
            )
        except Exception as exc:
            logger.exception("failed to read evaluation result for submit: %s", task_id)
            self.submit_service.mark_finished(
                task_id,
                status="FAILED",
                score=None,
            )
