from __future__ import annotations

from pathlib import Path

from flask import Blueprint, Response, current_app, g, jsonify, request, send_file

from ..utils.verification import require_access


bp = Blueprint("tasks", __name__)


def _service():
    return current_app.extensions["task_service"]


def _worker():
    return current_app.extensions["task_worker"]


@bp.post("/create")
@require_access()
def create_task():
    access = g.access_context
    payload = request.get_json(silent=True) or {}
    if not payload:
        payload = {key: value for key, value in request.form.items()}

    uploads = [file_storage for _, file_storage in request.files.items(multi=True)]
    try:
        task = _service().create_task(payload, uploads, access=access)
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400

    _worker().enqueue(task["task_id"])
    return jsonify(task), 201


@bp.get("")
@require_access()
def list_tasks():
    access = g.access_context
    tasks = _service().list_tasks(access=access)
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
