from __future__ import annotations

import logging
from pathlib import Path

from flask import Blueprint, Response, current_app, g, jsonify, request, send_file

from ..utils.verification import require_access


bp = Blueprint("tasks", __name__)
logger = logging.getLogger(__name__)


# 赛题类型映射字典
TYPE_MAP = {
    0: "A",
    1: "B",
    2: "C"
}

def _service():
    return current_app.extensions["task_service"]


def _worker():
    return current_app.extensions["task_worker"]


@bp.post("/create")
@require_access()
def create_task():
    access = g.access_context
    req_data = request.get_json(silent=True) or {}
    if not req_data:
        req_data = {key: value for key, value in request.form.items()}

    uploads = [file_storage for _, file_storage in request.files.items(multi=True)]

    qtype = int(req_data.get("qtype", -1))  # 0 -> A, 1 -> B, 2 ->C
    qname = req_data.get("name") or ""
    bg_model = req_data.get("bg_model", "model.zip.v6")
    is_train = req_data.get("is_train", False)

    if not isinstance(qtype, int) or qtype not in TYPE_MAP:
        return jsonify({"error": "不存在对应的类型的赛题"}), 400

    payload = {
        "name": qname,
        "batch_items": [
            {
                "name": qname,
                "BG_MODEL_FILENAME": bg_model,
                "DATA_DIR": f"Data/prod/{TYPE_MAP[qtype]}/{'train' if is_train else 'test'}",
            }
        ],
    }

    try:
        task = _service().create_task(payload, uploads, access=access)
    except Exception as exc:
        logger.exception("failed to create task")
        return jsonify({"error": "An internal error has occurred."}), 400

    _worker().enqueue(task["task_id"])
    logger.info("task created: %s by user=%s", task["task_id"], access.user_id)
    return jsonify(task), 201


@bp.get("")
@require_access()
def list_tasks():
    access = g.access_context
    tasks = _service().list_tasks(access=access)
    logger.info("list tasks for user=%s count=%s", access.user_id, len(tasks))
    return jsonify({"items": tasks, "count": len(tasks)})


@bp.get("/<task_id>")
@require_access()
def get_task(task_id: str):
    access = g.access_context
    task = _service().get_task(task_id, access=access)
    if not task:
        return jsonify({"error": "Task not found"}), 404
    return jsonify(task)


@bp.get("/<task_id>/logs")
@require_access()
def get_logs(task_id: str):
    access = g.access_context
    task = _service().get_task(task_id, access=access)
    if not task:
        return jsonify({"error": "Task not found"}), 404

    log_dir = Path(task["log_dir"])
    stream = (request.args.get("stream") or "combined").strip().lower()
    tail = int(request.args.get("tail", "200"))

    if stream == "stdout":
        content = _read_tail(log_dir / "stdout.log", tail)
    elif stream == "stderr":
        content = _read_tail(log_dir / "stderr.log", tail)
    else:
        stdout_text = _read_tail(log_dir / "stdout.log", tail)
        stderr_text = _read_tail(log_dir / "stderr.log", tail)
        content = f"[stdout]\n{stdout_text}\n[stderr]\n{stderr_text}"

    logger.info("task logs fetched: %s stream=%s tail=%s", task_id, stream, tail)
    return Response(content, mimetype="text/plain; charset=utf-8")


@bp.get("/<task_id>/download")
@require_access()
def download_task(task_id: str):
    access = g.access_context
    task = _service().get_task(task_id, access=access)
    if not task:
        return jsonify({"error": "Task not found"}), 404

    archive_path = _service().build_download_archive(task_id, access=access)
    if not archive_path or not archive_path.exists():
        return jsonify({"error": "Archive not available"}), 404

    logger.info("task archive downloaded: %s", task_id)
    return send_file(
        archive_path,
        as_attachment=True,
        download_name=f"{task_id}.zip",
        mimetype="application/zip",
    )


@bp.post("/<task_id>/cancel")
@require_access()
def cancel_task(task_id: str):
    access = g.access_context
    task = _service().get_task(task_id, access=access)
    if not task:
        return jsonify({"error": "Task not found"}), 404

    ok = _worker().cancel(task_id, access=access)
    if not ok:
        return jsonify({"error": "Unable to cancel task"}), 400

    logger.info("task cancel requested: %s by user=%s", task_id, access.user_id)
    return jsonify(_service().get_task(task_id, access=access))


def _read_tail(path: Path, lines: int) -> str:
    if not path.exists():
        return ""
    if lines <= 0:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()

    from collections import deque

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return "".join(deque(f, maxlen=lines))
