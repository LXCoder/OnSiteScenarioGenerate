from __future__ import annotations

from flask import Flask

from .api.health import bp as health_bp
from .api.tasks import bp as tasks_bp
from .config import Config
from .extensions import db
from .logging_setup import configure_logging
from .models.submit import Submit
from .services.task_service import TaskRepository, TaskService
from .services.submit_service import SubmitRepository, SubmitService
from .worker.task_worker import TaskWorker
from .worker.submit_worker import SubmitWorker


def create_app(config_object: type[Config] | None = None) -> Flask:
    app = Flask(__name__)
    app.config.from_object(config_object or Config)
    configure_logging(app)
    db.init_app(app)

    repository = TaskRepository(app.config)
    submit_repository = SubmitRepository()
    with app.app_context():
        repository.init_db()
        task_service = TaskService(app.config, repository)
        submit_service = SubmitService(submit_repository)
        app.logger.info("database initialized")

    app.extensions["task_repository"] = repository
    app.extensions["task_service"] = task_service
    app.extensions["submit_repository"] = submit_repository
    app.extensions["submit_service"] = submit_service

    worker = TaskWorker(app, app.config, task_service, submit_service=submit_service)
    app.extensions["task_worker"] = worker
    submit_worker = SubmitWorker(app, app.config, task_service, worker, submit_service)
    app.extensions["submit_worker"] = submit_worker
    if app.config["ENABLE_WORKER"]:
        worker.start()
        submit_worker.start()
        app.logger.info("task worker started")
        app.logger.info("submit worker started")
    else:
        app.logger.info("task worker disabled")

    app.register_blueprint(health_bp)
    app.register_blueprint(tasks_bp, url_prefix="/tasks")
    app.logger.info("backend app initialized")

    return app
