from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Sequence
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import unquote, urlparse


CommandCallback = Callable[[str, list[float]], None]
HandsCallback = Callable[[], Sequence[str]]
ActualStateCallback = Callable[[str], tuple[Sequence[float], int] | None]
MAX_REQUEST_BYTES = 64 * 1024
LOOPBACK_HOSTS = {"127.0.0.1", "localhost"}


class _WujiHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        command_callback: CommandCallback,
        hands_callback: HandsCallback,
        actual_state_callback: ActualStateCallback | None,
    ) -> None:
        self.command_callback = command_callback
        self.hands_callback = hands_callback
        self.actual_state_callback = actual_state_callback
        super().__init__(server_address, _WujiRequestHandler)


class _WujiRequestHandler(BaseHTTPRequestHandler):
    server: _WujiHTTPServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/health":
            self._write_json(
                200,
                {
                    "ok": True,
                    "hands": sorted(
                        str(side) for side in self.server.hands_callback()
                    ),
                },
            )
            return

        parts = [unquote(part) for part in path.split("/") if part]
        if (
            len(parts) != 3
            or parts[0] != "hands"
            or parts[2] != "actual_joint_positions"
        ):
            self._write_json(404, {"ok": False, "error": "not found"})
            return

        side = parts[1]
        if side not in self.server.hands_callback():
            self._write_json(404, {"ok": False, "error": f"unknown hand {side!r}"})
            return
        callback = self.server.actual_state_callback
        if callback is None:
            self._write_json(503, {"ok": False, "error": "actual state unavailable"})
            return
        try:
            state = callback(side)
            if state is None:
                raise RuntimeError(f"no actual state received for hand {side!r}")
            raw_positions, timestamp_ns = state
            positions = [float(value) for value in raw_positions]
            timestamp_ns = int(timestamp_ns)
            if len(positions) != 20:
                raise ValueError("actual positions must contain exactly 20 values")
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("actual positions contains a non-finite value")
            if timestamp_ns <= 0:
                raise ValueError("actual state timestamp must be positive")
            age_ms = max(0.0, (time.monotonic_ns() - timestamp_ns) / 1_000_000.0)
        except KeyError as exc:
            self._write_json(404, {"ok": False, "error": str(exc)})
            return
        except (RuntimeError, TypeError, ValueError) as exc:
            self._write_json(503, {"ok": False, "error": str(exc)})
            return

        self._write_json(
            200,
            {
                "ok": True,
                "side": side,
                "positions": positions,
                "timestamp_monotonic_ns": timestamp_ns,
                "age_ms": age_ms,
            },
        )

    def do_POST(self) -> None:
        parts = [unquote(part) for part in urlparse(self.path).path.split("/") if part]
        if len(parts) != 3 or parts[0] != "hands" or parts[2] != "joint_targets":
            self._write_json(404, {"ok": False, "error": "not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            content_length = 0
        if not 0 < content_length <= MAX_REQUEST_BYTES:
            self._write_json(400, {"ok": False, "error": "invalid request body size"})
            return

        try:
            payload = json.loads(self.rfile.read(content_length))
            if not isinstance(payload, dict) or not isinstance(payload.get("positions"), list):
                raise ValueError("request JSON must contain a positions list")
            positions = [float(value) for value in payload["positions"]]
            if len(positions) != 20:
                raise ValueError("positions must contain exactly 20 values")
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("positions contains a non-finite value")
            side = parts[1]
            self.server.command_callback(side, positions)
        except KeyError as exc:
            self._write_json(404, {"ok": False, "error": str(exc)})
            return
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._write_json(400, {"ok": False, "error": str(exc)})
            return
        except Exception as exc:  # noqa: BLE001
            self._write_json(500, {"ok": False, "error": repr(exc)})
            return

        self._write_json(200, {"ok": True, "side": side})

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _write_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class WujiCommandServer:
    """Loopback HTTP bridge that keeps Wuji USB ownership in the ROS process."""

    def __init__(
        self,
        host: str,
        port: int,
        command_callback: CommandCallback,
        hands_callback: HandsCallback,
        actual_state_callback: ActualStateCallback | None = None,
    ) -> None:
        host = str(host).strip().lower()
        if host not in LOOPBACK_HOSTS:
            raise ValueError(
                f"Wuji command server must bind to a loopback host, got {host!r}."
            )
        if not 0 <= int(port) <= 65535:
            raise ValueError(f"Wuji command server port must be in [0, 65535], got {port}.")
        self._server = _WujiHTTPServer(
            (host, int(port)),
            command_callback,
            hands_callback,
            actual_state_callback,
        )
        self._thread: threading.Thread | None = None

    @property
    def host(self) -> str:
        return str(self._server.server_address[0])

    @property
    def port(self) -> int:
        return int(self._server.server_address[1])

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="wuji_command_server",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        thread = self._thread
        if thread is None:
            return
        self._server.shutdown()
        self._server.server_close()
        thread.join(timeout=2.0)
        self._thread = None


__all__ = ["WujiCommandServer"]
