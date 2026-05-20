from __future__ import annotations

import os

from app import create_app


app = create_app()


if __name__ == "__main__":
    host = os.getenv("BACKEND_HOST", "0.0.0.0")
    port = int(os.getenv("BACKEND_PORT", "5000"))
    debug = os.getenv("BACKEND_DEBUG", "false").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    app.run(host=host, port=port, debug=debug, use_reloader=False)
