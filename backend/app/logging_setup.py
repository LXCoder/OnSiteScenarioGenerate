from __future__ import annotations

import logging
from logging.config import dictConfig
from pathlib import Path

BACKEND_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_LOG_FORMAT = (
    "%(asctime)s [%(process)d] [%(levelname)s] %(name)s "
    "[%(relativepath)s:%(lineno)d]: %(message)s"
)
DEFAULT_ACCESS_LOG_FORMAT = (
    "%(asctime)s [%(process)d] [%(levelname)s] %(name)s "
    "[%(relativepath)s:%(lineno)d]: %(message)s"
)


class RelativePathFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        pathname = Path(record.pathname).resolve()
        try:
            record.relativepath = str(pathname.relative_to(BACKEND_ROOT))
        except ValueError:
            record.relativepath = record.pathname
        return super().format(record)


def build_logging_config(
    *,
    log_dir: str | Path,
    level: str = "INFO",
    backup_count: int = 14,
    include_gunicorn: bool = False,
) -> dict:
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    handlers = {
        "console": {
            "class": "logging.StreamHandler",
            "level": level,
            "formatter": "default",
            "stream": "ext://sys.stdout",
        },
        "app_file": {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": level,
            "formatter": "default",
            "filename": str(log_path / "backend.log"),
            "when": "midnight",
            "interval": 1,
            "backupCount": backup_count,
            "encoding": "utf-8",
        },
        "error_file": {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": "WARNING",
            "formatter": "default",
            "filename": str(log_path / "backend.error.log"),
            "when": "midnight",
            "interval": 1,
            "backupCount": backup_count,
            "encoding": "utf-8",
        },
    }

    root_handlers = ["console", "app_file", "error_file"]
    loggers = {
        "werkzeug": {
            "level": level,
            "propagate": True,
        },
    }

    if include_gunicorn:
        handlers["gunicorn_access_file"] = {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": level,
            "formatter": "access",
            "filename": str(log_path / "gunicorn.access.log"),
            "when": "midnight",
            "interval": 1,
            "backupCount": backup_count,
            "encoding": "utf-8",
        }
        handlers["gunicorn_error_file"] = {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": level,
            "formatter": "default",
            "filename": str(log_path / "gunicorn.error.log"),
            "when": "midnight",
            "interval": 1,
            "backupCount": backup_count,
            "encoding": "utf-8",
        }
        loggers["gunicorn.error"] = {
            "handlers": ["console", "gunicorn_error_file", "error_file"],
            "level": level,
            "propagate": False,
        }
        loggers["gunicorn.access"] = {
            "handlers": ["console", "gunicorn_access_file"],
            "level": level,
            "propagate": False,
        }

    return {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "()": "app.logging_setup.RelativePathFormatter",
                "format": DEFAULT_LOG_FORMAT,
            },
            "access": {
                "()": "app.logging_setup.RelativePathFormatter",
                "format": DEFAULT_ACCESS_LOG_FORMAT,
            },
        },
        "handlers": handlers,
        "root": {
            "handlers": root_handlers,
            "level": level,
        },
        "loggers": loggers,
    }


def configure_logging(app) -> None:
    if app.config.get("_LOGGING_READY"):
        return

    dictConfig(
        build_logging_config(
            log_dir=app.config["LOG_DIR"],
            level=app.config["LOG_LEVEL"],
            backup_count=app.config["LOG_BACKUP_COUNT"],
            include_gunicorn=False,
        )
    )
    app.config["_LOGGING_READY"] = True

    gunicorn_error_logger = logging.getLogger("gunicorn.error")
    if gunicorn_error_logger.handlers:
        app.logger.handlers = gunicorn_error_logger.handlers
        app.logger.setLevel(gunicorn_error_logger.level or logging.INFO)
    else:
        app.logger.setLevel(getattr(logging, app.config["LOG_LEVEL"], logging.INFO))
