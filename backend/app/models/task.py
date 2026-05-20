from __future__ import annotations

from ..extensions import db


class Task(db.Model):
    __tablename__ = "tasks"

    task_id = db.Column(db.String(64), primary_key=True)
    user_id = db.Column(db.String(20), nullable=False, index=True)
    name = db.Column(db.String(255))
    status = db.Column(db.String(32), nullable=False, index=True)
    create_time = db.Column(db.Text, nullable=False)
    start_time = db.Column(db.Text)
    finish_time = db.Column(db.Text)
    batch_config_path = db.Column(db.Text, nullable=False)
    request_json = db.Column(db.Text, nullable=False)
    log_dir = db.Column(db.Text, nullable=False)
    output_dir = db.Column(db.Text, nullable=False)
    message = db.Column(db.Text)
    exit_code = db.Column(db.Integer)
    cancel_requested = db.Column(db.Boolean, nullable=False, default=False)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "user_id": self.user_id,
            "name": self.name,
            "status": self.status,
            "create_time": self.create_time,
            "start_time": self.start_time,
            "finish_time": self.finish_time,
            "batch_config_path": self.batch_config_path,
            "request_json": self.request_json,
            "log_dir": self.log_dir,
            "output_dir": self.output_dir,
            "message": self.message,
            "exit_code": self.exit_code,
            "cancel_requested": bool(self.cancel_requested),
        }
