#!/usr/bin/python3

import argparse
import json
import os
import subprocess
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List

from ament_index_python.packages import get_package_prefix


COLLISION_THRESHOLD_SPECS = {
    "lower_torque_thresholds_nominal": (7, [40.0, 40.0, 35.0, 35.0, 30.0, 25.0, 20.0]),
    "upper_torque_thresholds_nominal": (7, [55.0, 55.0, 50.0, 50.0, 45.0, 40.0, 35.0]),
    "lower_torque_thresholds_acceleration": (7, [40.0, 40.0, 35.0, 35.0, 30.0, 25.0, 20.0]),
    "upper_torque_thresholds_acceleration": (7, [55.0, 55.0, 50.0, 50.0, 45.0, 40.0, 35.0]),
    "lower_force_thresholds_nominal": (6, [45.0, 45.0, 45.0, 35.0, 35.0, 35.0]),
    "upper_force_thresholds_nominal": (6, [60.0, 60.0, 60.0, 50.0, 50.0, 50.0]),
    "lower_force_thresholds_acceleration": (6, [45.0, 45.0, 45.0, 35.0, 35.0, 35.0]),
    "upper_force_thresholds_acceleration": (6, [60.0, 60.0, 60.0, 50.0, 50.0, 50.0]),
}

COLLISION_ENV_NAMES = {
    "lower_torque_thresholds_nominal": "LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_NOMINAL",
    "upper_torque_thresholds_nominal": "LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_NOMINAL",
    "lower_torque_thresholds_acceleration": "LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_ACCELERATION",
    "upper_torque_thresholds_acceleration": "LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_ACCELERATION",
    "lower_force_thresholds_nominal": "LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_NOMINAL",
    "upper_force_thresholds_nominal": "LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_NOMINAL",
    "lower_force_thresholds_acceleration": "LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_ACCELERATION",
    "upper_force_thresholds_acceleration": "LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_ACCELERATION",
}


class LibfrankaHTTPServer:
    def __init__(self, robot_ip: str, host: str, port: int, helper_path: str) -> None:
        self.robot_ip = robot_ip
        self.host = host
        self.port = port
        self.helper_path = helper_path
        self.http_server: ThreadingHTTPServer | None = None
        self.helper_lock = threading.Lock()
        self.collision_thresholds = {
            name: list(default_values) for name, (_, default_values) in COLLISION_THRESHOLD_SPECS.items()
        }

    def run(self) -> None:
        server = self

        class RequestHandler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                server.handle_request(self)

            def do_POST(self):  # noqa: N802
                server.handle_request(self)

            def log_message(self, fmt: str, *args) -> None:
                print(f'HTTP {self.command} - {fmt % args}', flush=True)

        self.http_server = ThreadingHTTPServer((self.host, self.port), RequestHandler)
        print(f"libfranka HTTP server listening on http://{self.host}:{self.port}", flush=True)
        self.http_server.serve_forever()

    def handle_request(self, handler: BaseHTTPRequestHandler) -> None:
        try:
            payload = self._read_json(handler)
            response = self._dispatch(handler.path, payload)
            self._send_json(handler, HTTPStatus.OK, response)
        except ValueError as exc:
            print(f"HTTP {handler.path} bad request: {exc}", flush=True)
            self._send_json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except Exception as exc:  # pylint: disable=broad-except
            print(f"HTTP {handler.path} failed: {exc}", flush=True)
            self._send_json(handler, HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})

    @staticmethod
    def _read_json(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
        length = int(handler.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        body = handler.rfile.read(length)
        if not body:
            return {}
        return json.loads(body.decode("utf-8"))

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        handler.send_response(status.value)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Content-Length", str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _dispatch(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        routes = {
            "/health": self.health,
            "/getstate": self.get_state,
            "/getpos": self.get_pose,
            "/pose": lambda: self.move_pose(payload),
            "/clearerr": self.clear_error,
            "/get_collision": self.get_collision,
            "/set_collision": lambda: self.set_collision(payload),
        }
        if path not in routes:
            raise ValueError(f"Unknown endpoint: {path}")
        return routes[path]()

    def health(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "robot_ip": self.robot_ip,
            "helper_path": self.helper_path,
            "collision_thresholds": self._collision_thresholds_response(),
        }

    def get_pose(self) -> Dict[str, Any]:
        return self._run_helper(["get_pose"])

    def get_state(self) -> Dict[str, Any]:
        return self._run_helper(["get_state"])

    def clear_error(self) -> Dict[str, Any]:
        return self._run_helper(["clear_error"])

    def get_collision(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "collision_thresholds": self._collision_thresholds_response(),
        }

    def set_collision(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(payload, dict) or not payload:
            raise ValueError("set_collision expects a JSON object with collision threshold arrays")

        unknown = sorted(set(payload.keys()) - set(COLLISION_THRESHOLD_SPECS.keys()))
        if unknown:
            raise ValueError(f"Unknown collision threshold fields: {unknown}")

        updates: Dict[str, List[float]] = {}
        for name, values in payload.items():
            expected_length, _ = COLLISION_THRESHOLD_SPECS[name]
            if not isinstance(values, list) or len(values) != expected_length:
                raise ValueError(f"{name} must contain {expected_length} numeric values")
            updates[name] = [float(value) for value in values]

        self.collision_thresholds.update(updates)
        response = self._run_helper(["set_collision"])
        response["collision_thresholds"] = self._collision_thresholds_response()
        response["updated"] = list(updates.keys())
        return response

    def move_pose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose = payload.get("arr")
        if not isinstance(pose, list) or len(pose) != 7:
            raise ValueError("arr must contain [x, y, z, qx, qy, qz, qw]")

        command: List[str] = ["move_pose"] + [str(float(value)) for value in pose]
        if "duration_sec" in payload:
            command.append(str(float(payload["duration_sec"])))
        elif "controller_mode" in payload:
            command.append("0.0")

        if "controller_mode" in payload:
            controller_mode = str(payload["controller_mode"]).strip().lower()
            if controller_mode not in ("joint", "cartesian"):
                raise ValueError("controller_mode must be 'joint' or 'cartesian'")
            if len(command) == 8:
                command.append("0.0")
            command.append(controller_mode)

        response = self._run_helper(command)
        response["target_pose"] = [float(value) for value in pose]
        return response

    def _run_helper(self, arguments: List[str]) -> Dict[str, Any]:
        if not arguments:
            raise RuntimeError("Helper command is empty")

        command_name = arguments[0]
        if command_name == "move_pose":
            command = [self.helper_path, "move_pose", self.robot_ip] + arguments[1:]
        else:
            command = [self.helper_path, command_name, self.robot_ip]

        with self.helper_lock:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                env=self._helper_env(),
            )
        stdout = completed.stdout.strip()
        stderr = completed.stderr.strip()
        raw_output = stdout or stderr
        if not raw_output:
            raise RuntimeError(f"Helper returned no output (code {completed.returncode})")
        try:
            response = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Failed to parse helper output: {raw_output}") from exc

        if completed.returncode != 0:
            error = response.get("error") if isinstance(response, dict) else raw_output
            raise RuntimeError(str(error))
        if not isinstance(response, dict):
            raise RuntimeError("Helper output was not a JSON object")
        return response

    def _helper_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        for name, values in self.collision_thresholds.items():
            env[COLLISION_ENV_NAMES[name]] = ",".join(str(float(value)) for value in values)
        return env

    def _collision_thresholds_response(self) -> Dict[str, List[float]]:
        return {name: list(values) for name, values in self.collision_thresholds.items()}


def default_helper_path() -> str:
    prefix = Path(get_package_prefix("serl_franka_controllers_ros2"))
    return str(prefix / "lib" / "serl_franka_controllers_ros2" / "libfranka_http_tool")


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone pure-libfranka HTTP server")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--helper-path", default=default_helper_path())
    args = parser.parse_args()

    server = LibfrankaHTTPServer(
        robot_ip=args.robot_ip,
        host=args.host,
        port=args.port,
        helper_path=args.helper_path,
    )
    server.run()


if __name__ == "__main__":
    main()
