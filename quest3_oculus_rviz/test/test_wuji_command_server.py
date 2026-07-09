from __future__ import annotations

import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from quest3_oculus_rviz.wuji_command_server import WujiCommandServer


def _request_json(
    url: str,
    *,
    method: str = "GET",
    payload: dict | None = None,
) -> tuple[int, dict]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = Request(
        url,
        data=body,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        response = urlopen(request, timeout=1.0)
    except HTTPError as exc:
        return exc.code, json.loads(exc.read())
    with response:
        return response.status, json.loads(response.read())


def test_wuji_command_server_health_and_joint_targets() -> None:
    received = []
    server = WujiCommandServer(
        "127.0.0.1",
        0,
        lambda side, positions: received.append((side, positions)),
        lambda: ("left", "right"),
    )
    server.start()
    try:
        status, health = _request_json(f"{server.url}/health")
        assert status == 200
        assert health == {"ok": True, "hands": ["left", "right"]}

        positions = [float(index) for index in range(20)]
        status, result = _request_json(
            f"{server.url}/hands/left/joint_targets",
            method="POST",
            payload={"positions": positions},
        )
        assert status == 200
        assert result == {"ok": True, "side": "left"}
        assert received == [("left", positions)]
    finally:
        server.stop()


def test_wuji_command_server_rejects_invalid_joint_targets() -> None:
    server = WujiCommandServer(
        "127.0.0.1",
        0,
        lambda _side, _positions: None,
        lambda: ("right",),
    )
    server.start()
    try:
        status, result = _request_json(
            f"{server.url}/hands/right/joint_targets",
            method="POST",
            payload={"positions": [0.0] * 19},
        )
        assert status == 400
        assert "exactly 20" in result["error"]
    finally:
        server.stop()


def test_wuji_command_server_rejects_non_loopback_bind() -> None:
    try:
        WujiCommandServer("0.0.0.0", 8765, lambda *_args: None, lambda: ())
    except ValueError as exc:
        assert "loopback" in str(exc)
    else:
        raise AssertionError("non-loopback Wuji command server bind was accepted")
