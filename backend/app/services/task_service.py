from __future__ import annotations

import json
import shutil
import sqlite3
import tempfile
import uuid
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from werkzeug.utils import secure_filename


TASK_STATUSES = {
    "PENDING",
    "RUNNING",
    "SUCCESS",
    "FAILED",
    "TIMEOUT",
    "CANCELLED",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_json_maybe(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        return json.loads(text)
    return value


@dataclass(slots=True)
class TaskPaths:
    task_dir: Path
    upload_dir: Path
    output_dir: Path
    log_dir: Path
    batch_config_path: Path
    request_path: Path
    archive_dir: Path


class TaskRepository:
    def __init__(self, database_path: Path):
        self.database_path = Path(database_path)

    def _connect(self) -> sqlite3.Connection:
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(self.database_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL;")
        conn.execute("PRAGMA foreign_keys=ON;")
        return conn

    def init_db(self) -> None:
        schema = """
        CREATE TABLE IF NOT EXISTS tasks (
            task_id TEXT PRIMARY KEY,
            name TEXT,
            status TEXT NOT NULL,
            create_time TEXT NOT NULL,
            start_time TEXT,
            finish_time TEXT,
            batch_config_path TEXT NOT NULL,
            request_json TEXT NOT NULL,
            log_dir TEXT NOT NULL,
            output_dir TEXT NOT NULL,
            message TEXT DEFAULT '',
            exit_code INTEGER,
            cancel_requested INTEGER NOT NULL DEFAULT 0
        )
        """
        with self._connect() as conn:
            conn.execute(schema)
            conn.commit()

    def create_task(self, record: dict[str, Any]) -> dict[str, Any]:
        with self._connect() as conn:
            conn.execute(
                """
                INSERT INTO tasks (
                    task_id, name, status, create_time, batch_config_path,
                    request_json, log_dir, output_dir, message, cancel_requested
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    record["task_id"],
                    record.get("name"),
                    record["status"],
                    record["create_time"],
                    record["batch_config_path"],
                    record["request_json"],
                    record["log_dir"],
                    record["output_dir"],
                    record.get("message", ""),
                    int(record.get("cancel_requested", False)),
                ),
            )
            conn.commit()
        return self.get_task(record["task_id"])

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        with self._connect() as conn:
            row = conn.execute(
                "SELECT * FROM tasks WHERE task_id = ?",
                (task_id,),
            ).fetchone()
            return self._row_to_dict(row) if row else None

    def list_tasks(self) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT * FROM tasks ORDER BY create_time DESC"
            ).fetchall()
            return [self._row_to_dict(row) for row in rows]

    def list_pending_tasks(self) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT * FROM tasks WHERE status = 'PENDING' ORDER BY create_time ASC"
            ).fetchall()
            return [self._row_to_dict(row) for row in rows]

    def update_task(self, task_id: str, **fields: Any) -> dict[str, Any] | None:
        if not fields:
            return self.get_task(task_id)

        allowed = {
            "name",
            "status",
            "start_time",
            "finish_time",
            "batch_config_path",
            "request_json",
            "log_dir",
            "output_dir",
            "message",
            "exit_code",
            "cancel_requested",
        }
        updates = {key: fields[key] for key in fields if key in allowed}
        if not updates:
            return self.get_task(task_id)

        columns = ", ".join(f"{key} = ?" for key in updates)
        values = list(updates.values()) + [task_id]
        with self._connect() as conn:
            conn.execute(
                f"UPDATE tasks SET {columns} WHERE task_id = ?",
                values,
            )
            conn.commit()
        return self.get_task(task_id)

    @staticmethod
    def _row_to_dict(row: sqlite3.Row | None) -> dict[str, Any] | None:
        if row is None:
            return None
        return dict(row)


class TaskService:
    def __init__(self, config: Any, repository: TaskRepository):
        self.config = config
        self.repository = repository

    def build_task_paths(self, task_id: str) -> TaskPaths:
        task_dir = self.config["TASK_DATA_ROOT"] / task_id
        upload_dir = task_dir / "upload"
        output_dir = task_dir / "output"
        log_dir = task_dir / "logs"
        archive_dir = task_dir / "archive"
        batch_config_path = task_dir / "batch_config.json"
        request_path = task_dir / "request.json"

        for path in (upload_dir, output_dir, log_dir, archive_dir):
            path.mkdir(parents=True, exist_ok=True)

        return TaskPaths(
            task_dir=task_dir,
            upload_dir=upload_dir,
            output_dir=output_dir,
            log_dir=log_dir,
            batch_config_path=batch_config_path,
            request_path=request_path,
            archive_dir=archive_dir,
        )

    def create_task(self, payload: dict[str, Any], uploads: Iterable[Any] | None = None) -> dict[str, Any]:
        task_id = self._generate_task_id()
        paths = self.build_task_paths(task_id)

        request_snapshot = {
            "payload": payload,
            "uploads": [],
        }
        if uploads:
            request_snapshot["uploads"] = self._save_uploads(paths.upload_dir, uploads)

        batch_items = self._extract_batch_items(payload)
        self._write_json(paths.batch_config_path, batch_items)
        self._write_json(paths.request_path, request_snapshot)

        record = {
            "task_id": task_id,
            "name": payload.get("name") or task_id,
            "status": "PENDING",
            "create_time": _utc_now(),
            "batch_config_path": str(paths.batch_config_path),
            "request_json": str(paths.request_path),
            "log_dir": str(paths.log_dir),
            "output_dir": str(paths.output_dir),
            "message": "Task created",
            "cancel_requested": False,
        }
        task = self.repository.create_task(record)
        return self._attach_runtime_fields(task)

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        task = self.repository.get_task(task_id)
        return self._attach_runtime_fields(task) if task else None

    def list_tasks(self) -> list[dict[str, Any]]:
        return [self._attach_runtime_fields(task) for task in self.repository.list_tasks()]

    def cancel_task(self, task_id: str) -> dict[str, Any] | None:
        task = self.repository.get_task(task_id)
        if not task:
            return None
        if task["status"] == "PENDING":
            return self.repository.update_task(
                task_id,
                status="CANCELLED",
                finish_time=_utc_now(),
                message="Cancelled before execution",
                cancel_requested=1,
            )
        return self.repository.update_task(task_id, cancel_requested=1, message="Cancel requested")

    def mark_running(self, task_id: str) -> dict[str, Any] | None:
        return self.repository.update_task(
            task_id,
            status="RUNNING",
            start_time=_utc_now(),
            message="Running",
        )

    def mark_finished(
        self,
        task_id: str,
        status: str,
        exit_code: int | None,
        message: str,
    ) -> dict[str, Any] | None:
        if status not in TASK_STATUSES:
            raise ValueError(f"Unsupported status: {status}")
        return self.repository.update_task(
            task_id,
            status=status,
            finish_time=_utc_now(),
            exit_code=exit_code,
            message=message,
        )

    def load_batch_config(self, task_id: str) -> list[dict[str, Any]] | None:
        task = self.repository.get_task(task_id)
        if not task:
            return None
        with open(task["batch_config_path"], "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return data
        raise ValueError("batch_config.json must contain a JSON array")

    def build_download_archive(self, task_id: str) -> Path | None:
        task = self.repository.get_task(task_id)
        if not task:
            return None

        task_dir = Path(task["log_dir"]).parent
        archive_path = task_dir / "archive" / f"{task_id}.zip"
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        if archive_path.exists():
            archive_path.unlink()

        with tempfile.NamedTemporaryFile(delete=False, suffix=".zip") as tmp_file:
            temp_path = Path(tmp_file.name)

        try:
            with zipfile.ZipFile(temp_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for rel_name in ("batch_config.json", "request.json"):
                    src = task_dir / rel_name
                    if src.exists():
                        zf.write(src, arcname=f"{task_id}/{rel_name}")

                for folder_name in ("logs", "output", "upload"):
                    folder = task_dir / folder_name
                    if not folder.exists():
                        continue
                    for file_path in folder.rglob("*"):
                        if file_path.is_file():
                            zf.write(
                                file_path,
                                arcname=str(Path(task_id) / folder_name / file_path.relative_to(folder)),
                            )

            shutil.move(str(temp_path), archive_path)
            return archive_path
        finally:
            if temp_path.exists():
                temp_path.unlink(missing_ok=True)

    def _extract_batch_items(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        source = payload.get("batch_items")
        if source is None:
            source = payload.get("batch_config")
        if source is None:
            source = payload.get("task")
        if source is None:
            source = {
                key: value
                for key, value in payload.items()
                if key not in {"name", "batch_items", "batch_config", "task"}
            }

        source = _load_json_maybe(source)
        if isinstance(source, dict):
            source = [source]
        if not isinstance(source, list) or not source:
            raise ValueError("Request must provide batch_items or batch_config")

        normalized: list[dict[str, Any]] = []
        for index, item in enumerate(source):
            if not isinstance(item, dict):
                raise ValueError(f"Batch item {index} is not a JSON object")
            normalized.append(dict(item))

        return normalized

    def _save_uploads(self, upload_dir: Path, uploads: Iterable[Any]) -> list[dict[str, Any]]:
        saved: list[dict[str, Any]] = []
        for upload in uploads:
            filename = secure_filename(upload.filename or "")
            if not filename:
                continue
            target = upload_dir / filename
            if target.exists():
                stem = target.stem
                suffix = target.suffix
                target = upload_dir / f"{stem}_{uuid.uuid4().hex[:6]}{suffix}"
            upload.save(target)
            saved.append(
                {
                    "field_name": getattr(upload, "name", ""),
                    "original_filename": upload.filename,
                    "stored_filename": target.name,
                    "relative_path": str(target.relative_to(self.config["TASK_DATA_ROOT"])),
                }
            )
        return saved

    @staticmethod
    def _write_json(path: Path, data: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    @staticmethod
    def _generate_task_id() -> str:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
        suffix = uuid.uuid4().hex[:6]
        return f"task_{stamp}_{suffix}"

    def _attach_runtime_fields(self, task: dict[str, Any] | None) -> dict[str, Any] | None:
        if task is None:
            return None
        result = dict(task)
        result["batch_config"] = self._read_json_file(result["batch_config_path"])
        result["request"] = self._read_json_file(result["request_json"])
        return result

    @staticmethod
    def _read_json_file(path: str) -> Any:
        file_path = Path(path)
        if not file_path.exists():
            return None
        with open(file_path, "r", encoding="utf-8") as f:
            return json.load(f)
