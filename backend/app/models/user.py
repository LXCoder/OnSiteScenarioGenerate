from __future__ import annotations

from ..extensions import db


class User(db.Model):
    __tablename__ = "user"

    userId = db.Column(db.String(20), primary_key=True)
    userName = db.Column(db.String(20), nullable=False)
    mobile = db.Column(db.String(20))
    email = db.Column(db.String(50))
    age = db.Column(db.Integer)
    sex = db.Column(db.String(20))
    enp_password = db.Column(db.String(255))
    school = db.Column(db.String(50))
    department = db.Column(db.String(50))
    supervisorName = db.Column(db.String(20))
    supervisorRank = db.Column(db.String(50))
    researchField = db.Column(db.String(50))
    works = db.Column(db.String(255))
    name = db.Column(db.String(50))
    identity = db.Column(db.String(20))
    cjzxIdentity = db.Column(db.String(255), default="common")

    @property
    def is_admin(self) -> bool:
        return (self.identity or "").strip() == "Administrator" or (
            self.cjzxIdentity or ""
        ).strip().lower() == "admin"
