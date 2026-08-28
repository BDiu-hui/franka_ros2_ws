#!/usr/bin/env python3

import importlib.util
import sys


spec = importlib.util.spec_from_file_location("libfranka_http_server", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeArm:
    def __init__(self, name):
        self.name = name
        self.calls = []

    def _dispatch(self, path, payload):
        self.calls.append((path, payload))
        return {"ok": True, "arm": self.name}


server = module.DualLibfrankaHTTPServer.__new__(module.DualLibfrankaHTTPServer)
server.arms = {"left": FakeArm("left"), "right": FakeArm("right")}
pose = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
response = server._dispatch("/pose", {"left": pose, "right": {"arr": pose}})
if not response["ok"] or response["left"]["arm"] != "left" or response["right"]["arm"] != "right":
    raise SystemExit(1)
if server._dispatch("/left/getpos", {})["arm"] != "left":
    raise SystemExit(1)

try:
    server._dispatch("/pose", {"left": pose})
except ValueError:
    pass
else:
    raise SystemExit(1)

single = module.LibfrankaHTTPServer.__new__(module.LibfrankaHTTPServer)
single.start_control = lambda: None
single.stop_control = lambda: None
single._run_helper = lambda command: {"ok": command == ["health"]}
if not single._dispatch("/startimp", {})["ok"]:
    raise SystemExit(1)
if not single._dispatch("/stopimp", {})["ok"]:
    raise SystemExit(1)
if not single._dispatch("/clearerr", {})["ok"]:
    raise SystemExit(1)

calls_before = sum(len(arm.calls) for arm in server.arms.values())
zero_quaternion = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
try:
    server._dispatch("/pose", {"left": pose, "right": zero_quaternion})
except ValueError:
    pass
else:
    raise SystemExit(1)
if sum(len(arm.calls) for arm in server.arms.values()) != calls_before:
    raise SystemExit(1)


def fail():
    raise ValueError("busy")


try:
    server._parallel({"left": fail, "right": lambda: {"ok": True}})
except RuntimeError as exc:
    if "left arm" not in str(exc):
        raise SystemExit(1)
else:
    raise SystemExit(1)
