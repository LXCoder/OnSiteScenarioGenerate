from __future__ import annotations

from ..extensions import db


class Submit(db.Model):
    __tablename__ = "submit"

    submitId = db.Column(db.String(64), primary_key=True)
    submitterId = db.Column(db.String(32), nullable=False, index=True)
    competitionId = db.Column(db.String(256), nullable=False)
    submitTime = db.Column(db.DateTime)
    testTime = db.Column(db.DateTime)
    score = db.Column(db.Float)
    dockerId = db.Column(db.String(256))
    paperType = db.Column(db.String(16))
    resultLink = db.Column(db.String(255))
    status = db.Column(db.String(64), index=True)
    testDetail = db.Column(db.JSON)

    def to_dict(self) -> dict:
        return {
            "submitId": self.submitId,
            "submitterId": self.submitterId,
            "competitionId": self.competitionId,
            "submitTime": self.submitTime,
            "testTime": self.testTime,
            "score": self.score,
            "dockerId": self.dockerId,
            "paperType": self.paperType,
            "resultLink": self.resultLink,
            "status": self.status,
        }
