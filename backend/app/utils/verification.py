from __future__ import annotations

from functools import wraps
from typing import Callable

import jwt
from flask import current_app, g, jsonify, request

from ..models.user import User
from ..services.task_service import AccessContext


def build_access_context() -> AccessContext:
    token = (request.headers.get("Token") or "").strip()
    if not token:
        raise AuthError("Token header is required", 401)

    secret = current_app.config.get("JWT_SECRET_KEY", "")
    if not secret:
        raise AuthError("JWT secret is not configured", 500)

    algorithms = current_app.config.get("JWT_ALGORITHMS", ["HS256"])
    issuer = current_app.config.get("JWT_ISSUER", "").strip()

    decode_kwargs = {"algorithms": algorithms}
    if issuer:
        decode_kwargs["issuer"] = issuer

    try:
        payload = jwt.decode(token, secret, **decode_kwargs)
    except jwt.ExpiredSignatureError as exc:
        raise AuthError("Token expired", 401) from exc
    except jwt.InvalidTokenError as exc:
        raise AuthError("Invalid token", 401) from exc

    username = str(payload.get("username") or "").strip()
    if not username:
        raise AuthError("Token payload missing username", 401)

    user = User.query.filter_by(userName=username).first()
    if user is None:
        raise AuthError("User not found", 401)

    g.current_user = user
    g.jwt_payload = payload
    return AccessContext(
        user_id=user.userId,
        username=user.userName,
        is_admin=user.is_admin,
    )


class AuthError(Exception):
    def __init__(self, message: str, status_code: int):
        super().__init__(message)
        self.message = message
        self.status_code = status_code


def _check_access_headers(
    access: AccessContext, require_admin: bool
) -> tuple[bool, tuple | None]:
    if require_admin and not access.is_admin:
        return False, (jsonify({"error": "Admin role required"}), 403)

    if not access.user_id:
        return False, (jsonify({"error": "Authenticated user missing user_id"}), 401)

    return True, None


def require_access(*, admin_only: bool = False) -> Callable:
    """
    统一认证包装器。

    默认规则：
    - 从 `Token` 请求头读取 JWT
    - 使用 JWT 中的 `username` 查询 `user` 表
    - 用户角色以数据库中的 `identity/cjzxIdentity` 为准
    - 认证结果写入 `g.access_context`
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                access = build_access_context()
            except AuthError as exc:
                return jsonify({"error": exc.message}), exc.status_code
            ok, error_response = _check_access_headers(access, admin_only)
            if not ok:
                return error_response

            g.access_context = access
            return func(*args, **kwargs)

        return wrapper

    return decorator
