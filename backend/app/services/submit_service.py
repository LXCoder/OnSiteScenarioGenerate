from __future__ import annotations

from datetime import datetime
from typing import Any

from ..extensions import db
from ..models.submit import Submit


class SubmitRepository:
    def get_submit(self, submit_id: str) -> dict[str, Any] | None:
        submit = db.session.get(Submit, submit_id)
        return submit.to_dict() if submit else None

    def list_waitscore_submits(self) -> list[dict[str, Any]]:
        items = (
            db.session.query(Submit)
            .filter(Submit.status == "WAITSCORE")
            .order_by(Submit.submitTime.asc(), Submit.submitId.asc())
            .all()
        )
        return [item.to_dict() for item in items]

    def claim_next_waitscore(self) -> dict[str, Any] | None:
        with db.session.begin():
            submit = (
                db.session.query(Submit)
                .filter(Submit.status == "WAITSCORE")
                .order_by(Submit.submitTime.asc(), Submit.submitId.asc())
                .with_for_update(skip_locked=True)
                .first()
            )
            if not submit:
                return None
            submit.status = "QUEUING"
            if submit.submitTime is None:
                submit.submitTime = datetime.now()
            db.session.flush()
            return submit.to_dict()

    def list_queuing_submits(self) -> list[dict[str, Any]]:
        items = (
            db.session.query(Submit)
            .filter(Submit.status == "QUEUING")
            .order_by(Submit.submitTime.asc(), Submit.submitId.asc())
            .all()
        )
        return [item.to_dict() for item in items]

    def update_submit(self, submit_id: str, **fields: Any) -> dict[str, Any] | None:
        allowed = {
            "submitTime",
            "testTime",
            "score",
            "dockerId",
            "paperType",
            "resultLink",
            "status",
        }
        updates = {key: fields[key] for key in fields if key in allowed}
        if not updates:
            return self.get_submit(submit_id)

        submit = db.session.get(Submit, submit_id)
        if not submit:
            return None

        for key, value in updates.items():
            setattr(submit, key, value)
        db.session.commit()
        return self.get_submit(submit_id)

    def mark_queuing(self, submit_id: str) -> dict[str, Any] | None:
        submit = db.session.get(Submit, submit_id)
        if not submit or submit.status != "WAITSCORE":
            return None
        submit.status = "QUEUING"
        if submit.submitTime is None:
            submit.submitTime = datetime.now()
        db.session.commit()
        return self.get_submit(submit_id)

    def mark_pulling(self, submit_id: str) -> dict[str, Any] | None:
        return self.update_submit(submit_id, status="PULLING")

    def mark_testing(self, submit_id: str) -> dict[str, Any] | None:
        return self.update_submit(submit_id, status="TESTING", testTime=datetime.now())

    def mark_finished(
        self,
        submit_id: str,
        *,
        status: str,
        score: float | None = None,
    ) -> dict[str, Any] | None:
        return self.update_submit(
            submit_id,
            status=status,
            score=score,
            testTime=datetime.now(),
        )


class SubmitService:
    def __init__(self, repository: SubmitRepository):
        self.repository = repository

    def get_submit(self, submit_id: str) -> dict[str, Any] | None:
        return self.repository.get_submit(submit_id)

    def list_waitscore_submits(self) -> list[dict[str, Any]]:
        return self.repository.list_waitscore_submits()

    def claim_waitscore(self, submit_id: str) -> dict[str, Any] | None:
        with db.session.begin():
            submit = (
                db.session.query(Submit)
                .filter(
                    Submit.submitId == submit_id,
                    Submit.status == "WAITSCORE",
                )
                .with_for_update(skip_locked=True)
                .one_or_none()
            )
            if not submit:
                return None
            submit.status = "QUEUING"
            if submit.submitTime is None:
                submit.submitTime = datetime.now()
            db.session.flush()
            return submit.to_dict()

    def claim_next_waitscore(self) -> dict[str, Any] | None:
        return self.repository.claim_next_waitscore()

    def mark_pulling(self, submit_id: str) -> dict[str, Any] | None:
        return self.repository.mark_pulling(submit_id)

    def mark_testing(self, submit_id: str) -> dict[str, Any] | None:
        return self.repository.mark_testing(submit_id)

    def mark_finished(
        self,
        submit_id: str,
        *,
        status: str,
        score: float | None = None,
    ) -> dict[str, Any] | None:
        return self.repository.mark_finished(
            submit_id,
            status=status,
            score=score,
        )
