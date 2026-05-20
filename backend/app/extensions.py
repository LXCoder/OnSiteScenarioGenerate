from __future__ import annotations

from queue import Queue
from threading import Event, Lock


task_queue: Queue[str] = Queue()
queue_lock = Lock()
stop_event = Event()
