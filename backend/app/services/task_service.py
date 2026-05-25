from __future__ import annotations

import csv
import json
import os
import shutil
import tempfile
import uuid
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import pymysql
from pymysql.cursors import DictCursor
from werkzeug.utils import secure_filename

from ..extensions import db
from ..models.task import Task


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
class AccessContext:
    user_id: str | None
    username: str | None = None
    is_admin: bool = False


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
    def __init__(self, config: Any):
        self.host = str(config["MYSQL_HOST"])
        self.port = int(config["MYSQL_PORT"])
        self.user = str(config["MYSQL_USER"])
        self.password = str(config["MYSQL_PASSWORD"])
        self.database = str(config["MYSQL_DATABASE"])
        self.charset = str(config.get("MYSQL_CHARSET", "utf8mb4"))

    def _connect(self, *, with_database: bool = True):
        kwargs = {
            "host": self.host,
            "port": self.port,
            "user": self.user,
            "password": self.password,
            "charset": self.charset,
            "cursorclass": DictCursor,
            "autocommit": False,
        }
        if with_database:
            kwargs["database"] = self.database
        return pymysql.connect(**kwargs)

    def init_db(self) -> None:
        # 如数据库不存在，创建数据库
        with self._connect(with_database=False) as conn:
            with conn.cursor() as cursor:
                sql_create_database = f"CREATE DATABASE IF NOT EXISTS `{self.database}` CHARACTER SET {self.charset}"
                cursor.execute(sql_create_database)
            conn.commit()

        db.create_all()

    def create_task(self, record: dict[str, Any]) -> dict[str, Any]:
        task = Task(
            task_id=record["task_id"],
            user_id=record["user_id"],
            name=record.get("name"),
            status=record["status"],
            create_time=record["create_time"],
            batch_config_path=record["batch_config_path"],
            request_json=record["request_json"],
            log_dir=record["log_dir"],
            output_dir=record["output_dir"],
            message=record.get("message", ""),
            cancel_requested=bool(record.get("cancel_requested", False)),
        )
        db.session.add(task)
        db.session.commit()
        return self.get_task(record["task_id"])

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        task = db.session.get(Task, task_id)
        return task.to_dict() if task else None

    def list_tasks(self) -> list[dict[str, Any]]:
        tasks = db.session.query(Task).order_by(Task.create_time.desc()).all()
        return [task.to_dict() for task in tasks]

    def list_tasks_by_user(self, user_id: str) -> list[dict[str, Any]]:
        tasks = (
            db.session.query(Task)
            .filter(Task.user_id == user_id)
            .order_by(Task.create_time.desc())
            .all()
        )
        return [task.to_dict() for task in tasks]

    def list_pending_tasks(self) -> list[dict[str, Any]]:
        tasks = (
            db.session.query(Task)
            .filter(Task.status == "PENDING")
            .order_by(Task.create_time.asc())
            .all()
        )
        return [task.to_dict() for task in tasks]

    def update_task(self, task_id: str, **fields: Any) -> dict[str, Any] | None:
        if not fields:
            return self.get_task(task_id)

        allowed = {
            "user_id",
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

        task = db.session.get(Task, task_id)
        if not task:
            return None

        for key, value in updates.items():
            setattr(task, key, value)
        db.session.commit()
        return self.get_task(task_id)


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

    def create_task(
        self,
        payload: dict[str, Any],
        uploads: Iterable[Any] | None = None,
        *,
        access: AccessContext,
    ) -> dict[str, Any]:
        if not access.user_id:
            raise ValueError("user_id is required")

        task_id = self._generate_task_id()
        paths = self.build_task_paths(task_id)

        request_snapshot = {
            "payload": payload,
            "uploads": [],
            "user_id": access.user_id,
        }

        batch_items = self._extract_batch_items(payload)

        if uploads:
            upload_infos = self._save_uploads(paths.upload_dir, uploads)
            relative_path = self._get_model_relative_path(upload_infos)
            if relative_path:
                bg_model_fullpath = os.path.join(
                    str(self.config.get("DOCKER_WORKDIR", "/workspace")),
                    relative_path,
                )
                batch_items[0]["BG_MODEL_FULL_PATH"] = bg_model_fullpath

            request_snapshot["uploads"] = upload_infos
            

        self._write_json(paths.batch_config_path, batch_items)
        self._write_json(paths.request_path, request_snapshot)

        record = {
            "task_id": task_id,
            "user_id": access.user_id,
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

    def get_task(
        self, task_id: str, *, access: AccessContext | None = None
    ) -> dict[str, Any] | None:
        task = self.repository.get_task(task_id)
        if not task:
            return None
        if access and not self._can_access_task(task, access):
            return None
        return self._attach_runtime_fields(task)

    def list_tasks(self, *, access: AccessContext) -> list[dict[str, Any]]:
        if access.is_admin:
            tasks = self.repository.list_tasks()
        else:
            if not access.user_id:
                return []
            tasks = self.repository.list_tasks_by_user(access.user_id)
        return [self._attach_runtime_fields(task) for task in tasks]

    def cancel_task(
        self, task_id: str, *, access: AccessContext
    ) -> dict[str, Any] | None:
        task = self.repository.get_task(task_id)
        if not task or not self._can_access_task(task, access):
            return None

        if task["status"] == "PENDING":
            return self.repository.update_task(
                task_id,
                status="CANCELLED",
                finish_time=_utc_now(),
                message="Cancelled before execution",
                cancel_requested=1,
            )
        return self.repository.update_task(
            task_id,
            cancel_requested=1,
            message="Cancel requested",
        )

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

    def build_download_archive(
        self,
        task_id: str,
        *,
        access: AccessContext,
    ) -> Path | None:
        task = self.repository.get_task(task_id)
        if not task or not self._can_access_task(task, access):
            return None

        task_dir = Path(task["log_dir"]).parent
        archive_path = task_dir / "archive" / f"{task_id}.zip"
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        if archive_path.exists():
            archive_path.unlink()

        with tempfile.NamedTemporaryFile(delete=False, suffix=".zip") as tmp_file:
            temp_path = Path(tmp_file.name)

        try:
            with zipfile.ZipFile(
                temp_path, "w", compression=zipfile.ZIP_DEFLATED
            ) as zf:
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
                                arcname=str(
                                    Path(task_id)
                                    / folder_name
                                    / file_path.relative_to(folder)
                                ),
                            )

            shutil.move(str(temp_path), archive_path)
            return archive_path
        finally:
            if temp_path.exists():
                temp_path.unlink(missing_ok=True)

    def get_evaluation_result(
        self,
        task_id: str,
        *,
        access: AccessContext,
    ) -> dict[str, Any] | None:
        task = self.repository.get_task(task_id)
        if not task or not self._can_access_task(task, access):
            return None

        output_dir = Path(task["output_dir"])
        evaluation_root = output_dir / "evaluation"
        if not evaluation_root.exists():
            raise FileNotFoundError("evaluation directory not found")

        evaluation_dirs = [path for path in evaluation_root.iterdir() if path.is_dir()]
        if not evaluation_dirs:
            raise FileNotFoundError("evaluation result directory not found")

        latest_dir = max(evaluation_dirs, key=lambda path: path.stat().st_mtime)
        summary_path = latest_dir / "summary_averages_100.csv"
        per_scene_path = latest_dir / "per_scene_detailed_100.csv"

        if not summary_path.exists():
            raise FileNotFoundError("summary_averages_100.csv not found")

        summary_rows = self._read_csv_rows(summary_path)
        if not summary_rows:
            raise ValueError("summary_averages_100.csv is empty")

        per_scene_rows = self._read_csv_rows(per_scene_path) if per_scene_path.exists() else []

        return {
            "task_id": task_id,
            "summary": summary_rows[0], # 场景总结结果
            # "per_scene": per_scene_rows,# 场景详情，先注释，后续需要再打开
        }

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

    def _save_uploads(
        self, upload_dir: Path, uploads: Iterable[Any]
    ) -> list[dict[str, Any]]:
        saved: list[dict[str, Any]] = []
        for upload in uploads:
            filename = secure_filename(upload.filename or "")
            if not filename:
                continue

            # 1. 识别文件类型并动态决定存储子目录
            # suffix 包含点号，例如 '.xosc'
            is_xosc = filename.lower().endswith(".xosc")

            scenario_upload_dir = upload_dir / "scenario"

            if is_xosc:
                current_upload_dir = scenario_upload_dir
                file_type = "scenario"
            else:
                current_upload_dir = upload_dir
                file_type = "model"

            # 2. 确保目标目录存在（特别是新建的 scenario 子目录）
            current_upload_dir.mkdir(parents=True, exist_ok=True)

            # 3. 处理重名冲突
            target = current_upload_dir / filename
            if target.exists():
                stem = target.stem
                suffix = target.suffix
                target = current_upload_dir / f"{stem}_{uuid.uuid4().hex[:6]}{suffix}"

            # 4. 执行保存
            upload.save(target)

            # 5. 组装返回的 JSON 信息，添加 file_type 标识符
            saved.append(
                {
                    "field_name": getattr(upload, "name", ""),
                    "original_filename": upload.filename,
                    "stored_filename": target.name,
                    "file_type": file_type,  # 文件标识符: 'scenario' 或 'model'
                    "relative_path": str(
                        target.relative_to(self.config["TASK_DATA_ROOT"])
                    ),
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

    def _attach_runtime_fields(
        self, task: dict[str, Any] | None
    ) -> dict[str, Any] | None:
        if task is None:
            return None
        result = dict(task)
        result["batch_config"] = self._read_json_file(result["batch_config_path"])
        result["request"] = self._read_json_file(result["request_json"])
        result["cancel_requested"] = bool(result.get("cancel_requested", 0))
        return result

    def _can_access_task(self, task: dict[str, Any], access: AccessContext) -> bool:
        if access.is_admin:
            return True
        return bool(access.user_id) and task.get("user_id") == access.user_id

    @staticmethod
    def _read_json_file(path: str) -> Any:
        file_path = Path(path)
        if not file_path.exists():
            return None
        with open(file_path, "r", encoding="utf-8") as f:
            return json.load(f)

    @staticmethod
    def _read_csv_rows(path: Path) -> list[dict[str, Any]]:
        with open(path, "r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            return [
                {key: TaskService._coerce_csv_value(value) for key, value in row.items()}
                for row in reader
            ]

    @staticmethod
    def _coerce_csv_value(value: str | None) -> Any:
        if value is None:
            return None
        text = value.strip()
        if text == "":
            return None
        try:
            if any(ch in text for ch in (".", "e", "E")):
                return float(text)
            return int(text)
        except ValueError:
            return text

    @staticmethod
    def _get_model_relative_path(upload_infos):
        for info in upload_infos:
            file_type = info["file_type"]
            if file_type != "model":
                continue
            return info["relative_path"]

        return ""