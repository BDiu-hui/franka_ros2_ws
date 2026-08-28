#!/usr/bin/python3

import argparse
import json
import math
import os
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

from ament_index_python.packages import get_package_prefix


COLLISION_THRESHOLD_SPECS = {
    "lower_torque_thresholds_nominal": (7, [50.0, 50.0, 45.0, 45.0, 40.0, 35.0, 30.0]),
    "upper_torque_thresholds_nominal": (7, [70.0, 70.0, 60.0, 60.0, 55.0, 50.0, 45.0]),
    "lower_torque_thresholds_acceleration": (7, [50.0, 50.0, 45.0, 45.0, 40.0, 35.0, 30.0]),
    "upper_torque_thresholds_acceleration": (7, [70.0, 70.0, 60.0, 60.0, 55.0, 50.0, 45.0]),
    "lower_force_thresholds_nominal": (6, [55.0, 55.0, 55.0, 45.0, 45.0, 45.0]),
    "upper_force_thresholds_nominal": (6, [75.0, 75.0, 75.0, 60.0, 60.0, 60.0]),
    "lower_force_thresholds_acceleration": (6, [55.0, 55.0, 55.0, 45.0, 45.0, 45.0]),
    "upper_force_thresholds_acceleration": (6, [75.0, 75.0, 75.0, 60.0, 60.0, 60.0]),
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


def normalize_pose_payload(payload: Dict[str, Any]) -> Dict[str, List[float]]:
    if not isinstance(payload, dict):
        raise ValueError("pose payload must be a JSON object")
    pose = payload.get("arr")
    if not isinstance(pose, list) or len(pose) != 7:
        raise ValueError("arr must contain [x, y, z, qx, qy, qz, qw]")
    normalized: Dict[str, List[float]] = {"arr": [float(value) for value in pose]}
    if not all(math.isfinite(value) for value in normalized["arr"]):
        raise ValueError("arr must contain finite values")
    if sum(value * value for value in normalized["arr"][3:]) < 1e-18:
        raise ValueError("pose quaternion must be non-zero")

    master_q = payload.get("q")
    if master_q is not None:
        if not isinstance(master_q, list) or len(master_q) != 7:
            raise ValueError("q must contain 7 joint angles")
        normalized["q"] = [float(value) for value in master_q]
        if not all(math.isfinite(value) for value in normalized["q"]):
            raise ValueError("q must contain finite values")
    return normalized


def quaternion_transition(reference: List[float], current: List[float]) -> Dict[str, Any]:
    reference_norm = math.sqrt(sum(value * value for value in reference))
    current_norm = math.sqrt(sum(value * value for value in current))
    reference_q = [value / reference_norm for value in reference]
    current_q = [value / current_norm for value in current]
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(reference_q, current_q))))
    return {
        "reference_quaternion": reference_q,
        "target_quaternion": current_q,
        "quaternion_dot": dot,
        "hemisphere_flip": dot < 0.0,
        "representation_step_deg": math.degrees(2.0 * math.acos(dot)),
        "physical_step_deg": math.degrees(2.0 * math.acos(abs(dot))),
    }


class LibfrankaHTTPServer:
    def __init__(
        self,
        robot_ip: str,
        host: str,
        port: int,
        helper_path: str,
        helper_cpu: str = "",
        arm_name: str = "single",
    ) -> None:
        self.robot_ip = robot_ip
        self.host = host
        self.port = port
        self.helper_path = helper_path
        self.helper_cpu = helper_cpu
        self.http_server: ThreadingHTTPServer | None = None
        self.helper_lock = threading.Lock()
        self.helper_process: Optional[subprocess.Popen] = None
        self.arm_name = arm_name
        self.pose_diagnostic_lock = threading.Lock()
        self.last_measured_quaternion: Optional[List[float]] = None
        self.previous_target_quaternion: Optional[List[float]] = None
        self.pose_diagnostic_sequence = 0
        self.pose_diagnostic_file = None
        diagnostic_dir = Path(
            os.environ.get("LIBFRANKA_HTTP_POSE_LOG_DIR", "/tmp/franka_pose_diagnostics")
        )
        try:
            diagnostic_dir.mkdir(parents=True, exist_ok=True)
            diagnostic_path = diagnostic_dir / (
                f"{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}_{arm_name}.jsonl"
            )
            self.pose_diagnostic_file = diagnostic_path.open("w", encoding="utf-8", buffering=1)
            print(f"{arm_name} pose diagnostics: {diagnostic_path}", flush=True)
        except OSError as exc:
            print(f"Failed to open {arm_name} pose diagnostics: {exc}", flush=True)
        self.collision_thresholds = {
            name: list(default_values)
            for name, (_, default_values) in COLLISION_THRESHOLD_SPECS.items()
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

        self.start_control()
        try:
            self.http_server = ThreadingHTTPServer((self.host, self.port), RequestHandler)
            print(f"libfranka HTTP server listening on http://{self.host}:{self.port}", flush=True)
            self.http_server.serve_forever()
        finally:
            self.stop_control()
            self.close_pose_diagnostics()

    def start_control(self) -> None:
        with self.helper_lock:
            self._start_helper_locked()

    def stop_control(self) -> None:
        with self.helper_lock:
            self._stop_helper_locked()

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
            self._send_json(
                handler, HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)}
            )

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
    def _send_json(
        handler: BaseHTTPRequestHandler, status: HTTPStatus, payload: Dict[str, Any]
    ) -> None:
        data = json.dumps(payload).encode("utf-8")
        handler.send_response(status.value)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Content-Length", str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _dispatch(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        routes = {
            "/health": self.health,
            "/startimp": self.start_impedance,
            "/stopimp": self.stop_impedance,
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

    def start_impedance(self) -> Dict[str, Any]:
        self.start_control()
        return {"ok": True, "message": "Impedance control active"}

    def stop_impedance(self) -> Dict[str, Any]:
        self.stop_control()
        return {"ok": True, "message": "Impedance control stopped"}

    def health(self) -> Dict[str, Any]:
        response = self._run_helper(["health"])
        response.update({
            "ok": True,
            "robot_ip": self.robot_ip,
            "helper_path": self.helper_path,
            "collision_thresholds": self._collision_thresholds_response(),
            "control_mode": "cartesian_impedance",
        })
        return response

    def get_pose(self) -> Dict[str, Any]:
        return self._run_helper(["get_pose"])

    def get_state(self) -> Dict[str, Any]:
        response = self._run_helper(["get_state"])
        pose = response.get("pose")
        if isinstance(pose, list) and len(pose) == 7:
            with self.pose_diagnostic_lock:
                self.last_measured_quaternion = [float(value) for value in pose[3:]]
        return response

    def clear_error(self) -> Dict[str, Any]:
        try:
            self._run_helper(["health"])
            return {"ok": True, "message": "No active robot error"}
        except RuntimeError:
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
        normalized = normalize_pose_payload(payload)
        pose = normalized["arr"]
        command: List[str] = ["pose"] + [str(value) for value in pose]
        master_q = normalized.get("q")
        if master_q is not None:
            command.extend(str(value) for value in master_q)

        response = self._run_helper(command)
        self._write_pose_diagnostic(pose)
        response["target_pose"] = [float(value) for value in pose]
        response["method"] = "cartesian_impedance"
        return response

    def _write_pose_diagnostic(self, pose: List[float]) -> None:
        with self.pose_diagnostic_lock:
            reference = self.previous_target_quaternion or self.last_measured_quaternion
            reference_kind = (
                "previous_target"
                if self.previous_target_quaternion is not None
                else "measured_pose"
            )
            self.pose_diagnostic_sequence += 1
            record: Dict[str, Any] = {
                "time_ns": time.time_ns(),
                "arm": self.arm_name,
                "sequence": self.pose_diagnostic_sequence,
                "target_pose": pose,
                "reference_kind": reference_kind if reference is not None else None,
            }
            if reference is not None:
                record.update(quaternion_transition(reference, pose[3:]))
            self.previous_target_quaternion = pose[3:].copy()
            if self.pose_diagnostic_file is not None:
                try:
                    line = json.dumps(record, separators=(",", ":"))
                    self.pose_diagnostic_file.write(line + "\n")
                except OSError as exc:
                    print(f"Failed to write {self.arm_name} pose diagnostics: {exc}", flush=True)
                    self.close_pose_diagnostics()

    def close_pose_diagnostics(self) -> None:
        if self.pose_diagnostic_file is not None:
            self.pose_diagnostic_file.close()
            self.pose_diagnostic_file = None

    def _run_helper(self, arguments: List[str]) -> Dict[str, Any]:
        if not arguments:
            raise RuntimeError("Helper command is empty")

        with self.helper_lock:
            if arguments[0] in ("clear_error", "set_collision"):
                self._stop_helper_locked()
                try:
                    return self._run_one_shot_locked(arguments[0])
                finally:
                    self._start_helper_locked()
            return self._request_helper_locked(arguments)

    def _start_helper_locked(self) -> None:
        if self.helper_process is not None and self.helper_process.poll() is None:
            return
        command = [self.helper_path, "impedance", self.robot_ip]
        if self.helper_cpu:
            command = ["taskset", "-c", self.helper_cpu] + command
        self.helper_process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=self._helper_env(),
        )
        try:
            response = self._read_helper_response_locked()
        except Exception:
            self._stop_helper_locked(force=True)
            raise
        if not response.get("ok", False):
            self._stop_helper_locked(force=True)
            raise RuntimeError(str(response.get("error", "Failed to start impedance control")))

    def _stop_helper_locked(self, force: bool = False) -> None:
        process = self.helper_process
        self.helper_process = None
        if process is None or process.poll() is not None:
            return
        if not force:
            try:
                if process.stdin is not None:
                    process.stdin.write("shutdown\n")
                    process.stdin.flush()
                if process.stdout is not None:
                    process.stdout.readline()
                process.wait(timeout=5.0)
                return
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                pass
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)

    def _request_helper_locked(self, arguments: List[str]) -> Dict[str, Any]:
        self._start_helper_locked()
        process = self.helper_process
        if process is None or process.stdin is None:
            raise RuntimeError("Impedance helper is unavailable")
        process.stdin.write(" ".join(arguments) + "\n")
        process.stdin.flush()
        response = self._read_helper_response_locked()
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error", "Impedance helper request failed")))
        return response

    def _read_helper_response_locked(self) -> Dict[str, Any]:
        process = self.helper_process
        if process is None or process.stdout is None:
            raise RuntimeError("Impedance helper is unavailable")
        raw_output = process.stdout.readline().strip()
        if not raw_output:
            return_code = process.poll()
            raise RuntimeError(f"Impedance helper stopped unexpectedly (code {return_code})")
        try:
            response = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Failed to parse helper output: {raw_output}") from exc
        if not isinstance(response, dict):
            raise RuntimeError("Helper output was not a JSON object")
        return response

    def _run_one_shot_locked(self, command_name: str) -> Dict[str, Any]:
        completed = subprocess.run(
            [self.helper_path, command_name, self.robot_ip],
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


class DualLibfrankaHTTPServer:
    def __init__(
        self,
        left_robot_ip: str,
        right_robot_ip: str,
        host: str,
        left_port: int,
        right_port: int,
        helper_path: str,
        left_helper_cpu: str = "",
        right_helper_cpu: str = "",
    ) -> None:
        self.host = host
        self.ports = {"left": left_port, "right": right_port}
        self.http_servers: List[ThreadingHTTPServer] = []
        self.arms = {
            "left": LibfrankaHTTPServer(
                left_robot_ip, host, left_port, helper_path, left_helper_cpu, "left"
            ),
            "right": LibfrankaHTTPServer(
                right_robot_ip, host, right_port, helper_path, right_helper_cpu, "right"
            ),
        }

    def run(self) -> None:
        server = self

        def request_handler(arm_name: str):
            class RequestHandler(BaseHTTPRequestHandler):
                def do_GET(self):  # noqa: N802
                    server.handle_request(self, arm_name)

                def do_POST(self):  # noqa: N802
                    server.handle_request(self, arm_name)

                def log_message(self, fmt: str, *args) -> None:
                    print(f'HTTP {arm_name} {self.command} - {fmt % args}', flush=True)

            return RequestHandler

        threads: List[threading.Thread] = []
        try:
            self._parallel({name: arm.start_control for name, arm in self.arms.items()})
            for name, port in self.ports.items():
                http_server = ThreadingHTTPServer((self.host, port), request_handler(name))
                self.http_servers.append(http_server)
                thread = threading.Thread(target=http_server.serve_forever, daemon=True)
                threads.append(thread)
                thread.start()
                print(
                    f"dual libfranka {name} HTTP listening on http://{self.host}:{port}",
                    flush=True,
                )
            while all(thread.is_alive() for thread in threads):
                for thread in threads:
                    thread.join(timeout=0.5)
        finally:
            for http_server in self.http_servers:
                http_server.shutdown()
                http_server.server_close()
            for thread in threads:
                thread.join(timeout=2.0)
            self._parallel({name: arm.stop_control for name, arm in self.arms.items()})
            for arm in self.arms.values():
                arm.close_pose_diagnostics()

    def handle_request(self, handler: BaseHTTPRequestHandler, arm_name: str) -> None:
        try:
            payload = LibfrankaHTTPServer._read_json(handler)
            if handler.path.startswith("/dual/"):
                response = self._dispatch(handler.path[len("/dual"):], payload)
            else:
                response = self.arms[arm_name]._dispatch(handler.path, payload)
            LibfrankaHTTPServer._send_json(handler, HTTPStatus.OK, response)
        except ValueError as exc:
            print(f"HTTP {handler.path} bad request: {exc}", flush=True)
            LibfrankaHTTPServer._send_json(
                handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)}
            )
        except Exception as exc:  # pylint: disable=broad-except
            print(f"HTTP {handler.path} failed: {exc}", flush=True)
            LibfrankaHTTPServer._send_json(
                handler, HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)}
            )

    def _dispatch(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        for name, arm in self.arms.items():
            prefix = f"/{name}"
            if path.startswith(prefix + "/"):
                return arm._dispatch(path[len(prefix):], payload)

        if path in ("/health", "/getstate", "/getpos", "/clearerr", "/get_collision"):
            return self._dual_response(
                self._parallel(
                    {
                        name: lambda selected=arm: selected._dispatch(path, {})
                        for name, arm in self.arms.items()
                    }
                )
            )
        if path in ("/pose", "/set_collision"):
            arm_payloads = self._dual_payload(payload, path == "/pose")
            return self._dual_response(
                self._parallel(
                    {
                        name: lambda selected=arm, selected_payload=arm_payloads[name]:
                            selected._dispatch(path, selected_payload)
                        for name, arm in self.arms.items()
                    }
                )
            )
        raise ValueError(f"Unknown endpoint: {path}")

    @staticmethod
    def _dual_payload(payload: Dict[str, Any], pose: bool) -> Dict[str, Dict[str, Any]]:
        if not isinstance(payload, dict) or "left" not in payload or "right" not in payload:
            raise ValueError("dual request requires left and right objects")
        arm_payloads: Dict[str, Dict[str, Any]] = {}
        for name in ("left", "right"):
            value = payload[name]
            if pose and isinstance(value, list):
                value = {"arr": value}
            if not isinstance(value, dict):
                raise ValueError(f"{name} must be a JSON object")
            arm_payloads[name] = normalize_pose_payload(value) if pose else value
        return arm_payloads

    @staticmethod
    def _parallel(callbacks: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = {name: executor.submit(callback) for name, callback in callbacks.items()}
            results = {}
            for name, future in futures.items():
                try:
                    results[name] = future.result()
                except Exception as exc:
                    raise RuntimeError(f"{name} arm: {exc}") from exc
            return results

    @staticmethod
    def _dual_response(results: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
        return {"ok": True, "left": results["left"], "right": results["right"]}


def default_helper_path() -> str:
    prefix = Path(get_package_prefix("serl_franka_controllers_ros2"))
    return str(prefix / "lib" / "serl_franka_controllers_ros2" / "libfranka_http_tool")


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone pure-libfranka HTTP server")
    parser.add_argument("--robot-ip")
    parser.add_argument("--left-robot-ip")
    parser.add_argument("--right-robot-ip")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--left-port", type=int)
    parser.add_argument("--right-port", type=int)
    parser.add_argument("--helper-path", default=default_helper_path())
    parser.add_argument("--helper-cpu", default="")
    parser.add_argument("--left-helper-cpu", default="")
    parser.add_argument("--right-helper-cpu", default="")
    args = parser.parse_args()

    dual_mode = bool(args.left_robot_ip or args.right_robot_ip)
    if dual_mode:
        if args.robot_ip or not args.left_robot_ip or not args.right_robot_ip:
            parser.error("dual mode requires both --left-robot-ip and --right-robot-ip")
        server = DualLibfrankaHTTPServer(
            left_robot_ip=args.left_robot_ip,
            right_robot_ip=args.right_robot_ip,
            host=args.host,
            left_port=args.left_port if args.left_port is not None else args.port,
            right_port=args.right_port if args.right_port is not None else args.port + 1,
            helper_path=args.helper_path,
            left_helper_cpu=args.left_helper_cpu,
            right_helper_cpu=args.right_helper_cpu,
        )
    else:
        if not args.robot_ip:
            parser.error("single-arm mode requires --robot-ip")
        server = LibfrankaHTTPServer(
            robot_ip=args.robot_ip,
            host=args.host,
            port=args.port,
            helper_path=args.helper_path,
            helper_cpu=args.helper_cpu,
        )
    server.run()


if __name__ == "__main__":
    main()
