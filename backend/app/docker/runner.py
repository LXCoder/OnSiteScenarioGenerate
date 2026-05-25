from __future__ import annotations

import logging
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from ..config import TASK_DATA_ROOT

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class RunResult:
    status: str
    exit_code: int | None
    message: str


class BaseRunner:
    def run(
        self,
        *,
        workspace_root: Path,
        batch_config_path: Path,
        stdout_path: Path,
        stderr_path: Path,
        output_dir: Path,
        timeout_seconds: int | None,
        cancel_event: threading.Event,
    ) -> RunResult:
        raise NotImplementedError


class LocalRunner(BaseRunner):
    def run(
        self,
        *,
        workspace_root: Path,
        batch_config_path: Path,
        stdout_path: Path,
        stderr_path: Path,
        output_dir: Path,
        timeout_seconds: int | None,
        cancel_event: threading.Event,
    ) -> RunResult:
        rel_config = batch_config_path.relative_to(workspace_root)
        command = [sys.executable, "main.py", "--batch-config", str(rel_config)]
        logger.info("starting local runner command: %s", " ".join(command))
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stderr_path.parent.mkdir(parents=True, exist_ok=True)

        with (
            open(stdout_path, "a", encoding="utf-8") as stdout_file,
            open(stderr_path, "a", encoding="utf-8") as stderr_file,
        ):
            process = subprocess.Popen(
                command,
                cwd=str(workspace_root),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )

            def pump(stream, output_file):
                try:
                    for line in iter(stream.readline, ""):
                        output_file.write(line)
                        output_file.flush()
                finally:
                    stream.close()

            stdout_thread = threading.Thread(
                target=pump, args=(process.stdout, stdout_file), daemon=True
            )
            stderr_thread = threading.Thread(
                target=pump, args=(process.stderr, stderr_file), daemon=True
            )
            stdout_thread.start()
            stderr_thread.start()

            start = time.monotonic()
            status = "FAILED"
            message = "Process exited with a non-zero status"

            while True:
                if cancel_event.is_set():
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    status = "CANCELLED"
                    message = "Task cancelled"
                    logger.warning("local runner cancelled")
                    break

                return_code = process.poll()
                if return_code is not None:
                    status = "SUCCESS" if return_code == 0 else "FAILED"
                    message = f"Process exited with code {return_code}"
                    logger.info("local runner exited with code %s", return_code)
                    break

                if timeout_seconds is not None and timeout_seconds > 0:
                    elapsed = time.monotonic() - start
                    if elapsed > timeout_seconds:
                        process.kill()
                        status = "TIMEOUT"
                        message = f"Task timed out after {timeout_seconds} seconds"
                        logger.warning(
                            "local runner timed out after %s seconds",
                            timeout_seconds,
                        )
                        break

                time.sleep(0.5)

            stdout_thread.join(timeout=5)
            stderr_thread.join(timeout=5)
            return RunResult(
                status=status, exit_code=process.returncode, message=message
            )


class DockerRunner(BaseRunner):
    def __init__(
        self,
        image: str,
        network_mode: str,
        cert_dir: str,
        scenario_dir: str,
        model_path: str,
        bv_scenario_dir: str,
        workdir: str = "/workspace",
    ):
        self.image = image
        self.workdir = workdir
        self.network_mode = network_mode
        self.cert_dir = cert_dir
        self.scenario_dir = scenario_dir
        self.model_path = model_path
        self.bv_scenario_dir = bv_scenario_dir

    def run(
        self,
        *,
        workspace_root: Path,
        batch_config_path: Path,
        stdout_path: Path,
        stderr_path: Path,
        output_dir: Path,
        timeout_seconds: int | None,
        cancel_event: threading.Event,
    ) -> RunResult:
        try:
            import docker

            client = docker.from_env()
            rel_config = batch_config_path.relative_to(workspace_root)
            # command = ["python", "main.py", "--batch-config", "batch_config.json"]

            container_cert_dir = os.path.join(self.workdir, "Cert")
            container_bv_scenarion_dir = os.path.join(
                self.workdir, "Data", "bv_scenario"
            )
            container_scenarion_dir = os.path.join(self.workdir, "Data", "prod")

            command = [
                "bash",
                "-c",
                "python main.py --batch-config batch_config.json && "
                "python evaluation/main.py /tmp/output/traj "
                f"--bv-scene-root {container_bv_scenarion_dir} "
                f"--gt-scene-root {container_scenarion_dir} "
                "--output-root /tmp/output/evaluation "
                "--num-workers-bv 4 --num-workers-av 1"
            ]

            volumes = self._volumes(
                output_dir=output_dir,
                batch_config_path=batch_config_path,
                cert_dir=container_cert_dir,
                scenarion_dir=container_scenarion_dir,
                bv_scenarion_dir=container_bv_scenarion_dir
            )

            container_kwargs: dict[str, Any] = {
                "image": self.image,
                "command": command,
                "working_dir": self.workdir,
                "volumes": volumes,
                "detach": True,
                "stdout": True,
                "stderr": True,
                "tty": False,
                "remove": False,
                "network_mode": self.network_mode,
                "environment": {
                    "PYTHONUNBUFFERED": "1",
                    "QT_QPA_PLATFORM": "offscreen",
                    "QT_LOGGING_RULES": "*.debug=false;*.info=true;qt.widgets.painting=false",
                },
            }

            container = client.containers.run(**container_kwargs)
            logger.info("docker runner started container: %s", container.id)

            stdout_path.parent.mkdir(parents=True, exist_ok=True)
            stderr_path.parent.mkdir(parents=True, exist_ok=True)
            output_dir.mkdir(parents=True, exist_ok=True)

            stdout_file = open(stdout_path, "a", encoding="utf-8")
            stderr_file = open(stderr_path, "a", encoding="utf-8")
            log_thread: threading.Thread | None = None

            def collect_logs() -> None:
                try:
                    try:
                        stream = container.logs(
                            stream=True,
                            follow=True,
                            stdout=True,
                            stderr=True,
                            demux=True,
                        )
                        for stdout_chunk, stderr_chunk in stream:
                            if stdout_chunk:
                                stdout_file.write(
                                    stdout_chunk.decode("utf-8", errors="replace")
                                )
                                stdout_file.flush()
                            if stderr_chunk:
                                stderr_file.write(
                                    stderr_chunk.decode("utf-8", errors="replace")
                                )
                                stderr_file.flush()
                    except TypeError:
                        stream = container.logs(stream=True, follow=True)
                        for chunk in stream:
                            stdout_file.write(chunk.decode("utf-8", errors="replace"))
                            stdout_file.flush()
                finally:
                    stdout_file.close()
                    stderr_file.close()

            log_thread = threading.Thread(target=collect_logs, daemon=True)
            log_thread.start()

            start = time.monotonic()
            while True:
                container.reload()
                state = container.attrs.get("State", {})
                status = str(state.get("Status", "")).lower()
                if status in {"exited", "dead"}:
                    exit_code = state.get("ExitCode")
                    if exit_code == 0:
                        logger.info(
                            "docker runner container exited successfully: %s",
                            container.id,
                        )
                        return RunResult(
                            status="SUCCESS",
                            exit_code=0,
                            message="Container exited successfully",
                        )
                    logger.warning(
                        "docker runner container exited with code %s: %s",
                        exit_code,
                        container.id,
                    )
                    return RunResult(
                        status="FAILED",
                        exit_code=exit_code,
                        message=f"Container exited with code {exit_code}",
                    )

                if cancel_event.is_set():
                    container.kill()
                    logger.warning(
                        "docker runner container cancelled and killed: %s",
                        container.id,
                    )
                    return RunResult(
                        status="CANCELLED",
                        exit_code=None,
                        message="Task cancelled",
                    )

                if timeout_seconds is not None and timeout_seconds > 0:
                    elapsed = time.monotonic() - start
                    if elapsed > timeout_seconds:
                        container.kill()
                        logger.warning(
                            "docker runner container timed out after %s seconds: %s",
                            timeout_seconds,
                            container.id,
                        )
                        return RunResult(
                            status="TIMEOUT",
                            exit_code=None,
                            message=f"Task timed out after {timeout_seconds} seconds",
                        )

                time.sleep(1.0)
        except Exception as exc:
            logger.exception("docker runner failed")
            return RunResult(
                status="FAILED", exit_code=None, message=f"Docker error: {exc}"
            )
        finally:
            try:
                if "log_thread" in locals() and log_thread is not None:
                    log_thread.join(timeout=10)
            except Exception:
                pass
            try:
                container.remove(force=True)  # type: ignore[name-defined]
            except Exception:
                pass

    def _volumes(
        self,
        batch_config_path: Path,
        output_dir: Path,
        cert_dir: str,
        scenarion_dir: str,
        bv_scenarion_dir:str
    ):
        volumes = {
            "/usr/share/fonts": {"bind": "/usr/share/fonts", "mode": "rw"},
            "/etc/fonts": {"bind": "/etc/fonts", "mode": "rw"},
            "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
            output_dir: {"bind": "/tmp/output", "mode": "rw"},
            self.cert_dir: {
                "bind": cert_dir,
                "mode": "rw",
            },
            self.scenario_dir: {
                "bind": scenarion_dir,
                "mode": "ro",
            },
            batch_config_path: {
                "bind": os.path.join(self.workdir, batch_config_path.name),
                "mode": "rw",
            },
        }

        if self.model_path:
            full_path = os.path.join(TASK_DATA_ROOT, self.model_path)
            volumes[full_path] = {
                "bind": os.path.join(self.workdir, self.model_path),
                "mode": "ro",
            }

        if self.bv_scenario_dir:
            full_path = os.path.join(TASK_DATA_ROOT, self.bv_scenario_dir)
            volumes[full_path] = {
                "bind": bv_scenarion_dir,
                "mode": "ro",
            }

        return volumes


def build_runner(config: Any, upload_info: Any = None) -> BaseRunner:
    use_docker = bool(config.get("USE_DOCKER", False))
    image = str(config.get("DOCKER_IMAGE", "")).strip()

    model_path, bv_scenario_dir = _get_model_and_bv_scenario_path(
        upload_info=upload_info
    )
    logger.error(f"model: {model_path}")
    logger.error(f"bv_scenario_dir: {bv_scenario_dir}")

    if use_docker and image:
        try:    
            import docker  # noqa: F401

            return DockerRunner(
                image=image,
                network_mode=str(config.get("BACKEND_DOCKER_NETWORK", "host")),
                workdir=str(config.get("DOCKER_WORKDIR", "/workspace")),
                cert_dir=str(config.get("TESSNG_CERT_DIR", "")),
                scenario_dir=str(config.get("DOCKER_SCENARIO_DIR", "")),
                model_path=model_path,
                bv_scenario_dir=bv_scenario_dir,
            )
        except Exception:
            pass

    return LocalRunner()


def _get_model_and_bv_scenario_path(upload_info):
    model_path = ""
    bv_scenario_dir = ""

    if not upload_info:
        return model_path, bv_scenario_dir

    for info in upload_info:
        file_type = info["file_type"]
        relative_path = info.get("relative_path", "")

        if not relative_path:
            continue

        if file_type == "model":
            model_path = info.get("relative_path", "")
        elif file_type == "scenario":
            bv_scenario_dir = os.path.dirname(relative_path)

        if model_path and bv_scenario_dir:
            break
    return model_path, bv_scenario_dir
