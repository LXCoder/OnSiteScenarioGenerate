from __future__ import annotations

import os
from pathlib import Path
from dotenv import load_dotenv

def _as_bool(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}

# 加载 .env 文件中的环境变量
load_dotenv()
BACKEND_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = BACKEND_ROOT.parent
TASK_DATA_ROOT = BACKEND_ROOT / "task_data"


class Config:
    JSON_AS_ASCII = False
    BACKEND_ROOT = BACKEND_ROOT
    PROJECT_ROOT = PROJECT_ROOT
    TASK_DATA_ROOT = TASK_DATA_ROOT
    DATABASE_PATH = TASK_DATA_ROOT / "backend.sqlite3"
    DEFAULT_TIMEOUT_SECONDS = int(os.getenv("BACKEND_TASK_TIMEOUT", "3600"))
    ENABLE_WORKER = _as_bool(os.getenv("BACKEND_ENABLE_WORKER"), True)
    USE_DOCKER = _as_bool(os.getenv("BACKEND_USE_DOCKER"), True)
    DOCKER_IMAGE = os.getenv("BACKEND_DOCKER_IMAGE", "").strip()
    DOCKER_WORKDIR = os.getenv("BACKEND_DOCKER_WORKDIR", "/app").strip() or "/app"
    DOCKER_NETWORK = os.getenv("BACKEND_DOCKER_NETWORK", "host").strip()
    DOCKER_MEM_LIMIT = os.getenv("BACKEND_DOCKER_MEM_LIMIT", "").strip()
    DOCKER_NANO_CPUS = int(os.getenv("BACKEND_DOCKER_NANO_CPUS", "0") or 0)
    TESSNG_CERT_DIR = os.getenv("BACKEND_CERT_DIR","").strip()

