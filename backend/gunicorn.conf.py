from __future__ import annotations

import os
from pathlib import Path

from app.logging_setup import build_logging_config


bind = "0.0.0.0:5000"
workers = 8
timeout = 120
graceful_timeout = 30
keepalive = 5

pidfile = "gunicorn.pid"

# log
log_level = os.getenv("BACKEND_LOG_LEVEL", "INFO").strip().lower() or "info"
backup_count = int(os.getenv("BACKEND_LOG_BACKUP_COUNT", "14"))
log_dir = Path(
    os.getenv("BACKEND_LOG_DIR", str(Path(__file__).resolve().parent / "logs"))
).resolve()

accesslog = "-"
errorlog = "-"
capture_output = True

logconfig_dict = build_logging_config(
    log_dir=log_dir,
    level=log_level.upper(),
    backup_count=backup_count,
    include_gunicorn=True,
)

