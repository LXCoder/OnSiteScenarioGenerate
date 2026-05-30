from __future__ import annotations

import logging
import os
import shutil
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
from typing import Any
from urllib.parse import urlparse
from urllib.request import urlopen

from ..services.submit_service import SubmitService
from ..services.task_service import AccessContext

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class DownloadedArchive:
    filename: str
    source_path: Path

    def save(self, target: Path) -> None:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(self.source_path, target)


class SubmitWorker:
    def __init__(
        self, app, config, task_service, task_worker, submit_service: SubmitService
    ):
        self.app = app
        self.config = config
        self.task_service = task_service
        self.task_worker = task_worker
        self.submit_service = submit_service
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._queue: Queue[str] = Queue()
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        with self.app.app_context():
            while True:
                submit = self.submit_service.claim_next_waitscore()
                if not submit:
                    break
                self._queue.put(submit["submitId"])
            for submit in self.submit_service.list_queuing_submits():
                if self.task_service.get_task(submit["submitId"]):
                    continue
                self._queue.put(submit["submitId"])
            logger.info("reloaded pending submits into submit queue")

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("submit worker thread started")

    def stop(self) -> None:
        self._stop_event.set()

    def _loop(self) -> None:
        with self.app.app_context():
            while not self._stop_event.is_set():
                try:
                    submit_id = self._queue.get(timeout=0.5)
                except Empty:
                    self._poll_new_submits()
                    continue

                try:
                    self._process_submit(submit_id)
                except Exception:
                    logger.exception("failed to process submit: %s", submit_id)
                finally:
                    time.sleep(0.1)

    def _poll_new_submits(self) -> None:
        while True:
            submit = self.submit_service.claim_next_waitscore()
            if not submit:
                break
            self._queue.put(submit["submitId"])
        for submit in self.submit_service.list_queuing_submits():
            submit_id = submit["submitId"]
            if self.task_service.get_task(submit_id):
                continue
            self._queue.put(submit_id)

    def _process_submit(self, submit_id: str) -> None:
        if self.task_service.get_task(submit_id):
            logger.info("task already exists for submit: %s", submit_id)
            return

        submit = self.submit_service.get_submit(submit_id)
        if not submit:
            logger.warning("submit not found: %s", submit_id)
            return

        if submit["status"] != "QUEUING":
            logger.info("skip submit in status=%s: %s", submit["status"], submit_id)
            return

        self.submit_service.mark_pulling(submit_id)

        archive_path = None
        try:
            archive_path = self._download_archive(submit["resultLink"])
            self.submit_service.mark_testing(submit_id)

            access = AccessContext(
                user_id=submit["submitterId"],
                username=submit["submitterId"],
                is_admin=False,
            )
            payload = self._build_payload(submit)
            archive = DownloadedArchive(
                filename=os.path.basename(urlparse(submit["resultLink"]).path)
                or f"{submit_id}.zip",
                source_path=archive_path,
            )
            task = self.task_service.create_task(payload, [archive], access=access)
            self.task_worker.enqueue(task["task_id"])
            logger.info("submit converted to task: %s", submit_id)
        except Exception as exc:
            logger.exception("failed to create task from submit: %s", submit_id)
            self.submit_service.mark_finished(
                submit_id, self._map_failed_status(str(exc))
            )
        finally:
            if archive_path and archive_path.exists():
                try:
                    archive_path.unlink()
                except Exception:
                    pass

    def _download_archive(self, url: str) -> Path:
        if not url:
            raise ValueError("resultLink is empty")

        parsed = urlparse(url)
        suffix = Path(parsed.path).suffix or ".zip"
        with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp_file:
            temp_path = Path(tmp_file.name)

        try:
            with urlopen(
                url, timeout=int(self.config.get("SUBMIT_DOWNLOAD_TIMEOUT_SECONDS", 60))
            ) as response:
                with open(temp_path, "wb") as output:
                    shutil.copyfileobj(response, output)
            return temp_path
        except Exception as exc:
            if temp_path.exists():
                temp_path.unlink(missing_ok=True)
            raise ValueError(f"failed to download submission archive: {exc}") from exc

    def _build_payload(self, submit: dict[str, Any]) -> dict[str, Any]:
        paper_type = (submit.get("paperType") or "").strip().upper()
        if paper_type not in {"A", "B", "C"}:
            raise ValueError(f"Unsupported paperType: {paper_type}")

        data_dir = f"Data/prod/{paper_type}"
        if "A" == paper_type:
            data_dir = f"{data_dir}/test"

        return {
            "name": submit["submitId"],
            "batch_items": [
                {
                    "name": submit["submitId"],
                    "BG_MODEL_FILENAME": "model.zip",
                    "DATA_DIR": data_dir,
                }
            ],
        }

    @staticmethod
    def _map_failed_status(message: str) -> str:
        text = message.lower()
        if "download" in text or "url" in text or "resultlink" in text:
            return "PULL ERROR"
        if "docker" in text or "container" in text:
            return "CONTAINER ERROR"
        return "EVALUATE ERROR"
