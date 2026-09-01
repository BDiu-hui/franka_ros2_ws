#!/usr/bin/env python3
"""FKViewer local control console.

The server intentionally defaults to dry-run mode. Status probes read local
HTTP endpoints and process lists, but actions that would move hardware or start
robot processes only return the command/payload unless live_control is enabled.
"""

from __future__ import annotations

import argparse
import ast
import errno
import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen


APP_ROOT = Path(__file__).resolve().parent
STATIC_ROOT = APP_ROOT / "static"
DEFAULT_WORKSPACE = Path(os.environ.get("FKVIEWER_WORKSPACE", "/home/lumos/franka_ros2_ws"))
DEFAULT_EASYDP = Path(os.environ.get("FKVIEWER_EASYDP_ROOT", "/home/lumos/luolei/easydp"))
DEFAULT_EASYDP_PYTHON = Path(os.environ.get("FKVIEWER_EASYDP_PYTHON", "/home/lumos/miniconda3/envs/easydp/bin/python"))
DEFAULT_PROFILE_FILE = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml"
DEFAULT_WUJI_FILE = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz/config/wuji_trigger_hand.yaml"
DEFAULT_WUJI_POLICY_FILE = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz/config/wuji_policy_bridge.yaml"
DEFAULT_DATA_RECORDER_FILE = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz/config/data_recorder.yaml"
DEFAULT_SINGLE_TELEOP_FILE = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz/config/simple_impedance_teleop.yaml"
DEFAULT_SERL_ROOT = DEFAULT_WORKSPACE / "src/serl_franka_controllers_ros2"
DEFAULT_WUJI_ROS2_ROOT = DEFAULT_WORKSPACE / "src/wujihandros2"
DEFAULT_WUJI_PY_ROOT = DEFAULT_WORKSPACE / "src/wujihandpy"
DEFAULT_QUEST_ROOT = DEFAULT_WORKSPACE / "src/quest3_oculus_rviz"
DEFAULT_UNIFIED_ROOT = DEFAULT_WORKSPACE / "src/unified_impedance_control"
EASYDP_INFERENCE_CLIENT_REL = Path("projects/task_insertion_stage2/client/client_dual.py")
DEFAULT_RECORDING_DIRS = [
    str(Path.home() / "quest3_recordings"),
    "/tmp/quest3_recordings",
    "/data/quest3_recordings",
]


def _env_flag(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"", "0", "false", "no", "off"}


DEFAULT_CONFIG: dict[str, Any] = {
    "workspace": str(DEFAULT_WORKSPACE),
    "easydp_root": str(DEFAULT_EASYDP),
    "easydp_python": str(DEFAULT_EASYDP_PYTHON),
    "profile_file": str(DEFAULT_PROFILE_FILE),
    "wuji_config_file": str(DEFAULT_WUJI_FILE),
    "wuji_policy_file": str(DEFAULT_WUJI_POLICY_FILE),
    "data_recorder_config_file": str(DEFAULT_DATA_RECORDER_FILE),
    "single_teleop_config_file": str(DEFAULT_SINGLE_TELEOP_FILE),
    "code_roots": {
        "impedance": str(DEFAULT_SERL_ROOT),
        "wujihandros2": str(DEFAULT_WUJI_ROS2_ROOT),
        "wujihandpy": str(DEFAULT_WUJI_PY_ROOT),
        "teleop": str(DEFAULT_QUEST_ROOT),
        "inference": str(DEFAULT_EASYDP),
        "unified": str(DEFAULT_UNIFIED_ROOT),
    },
    "recording_dirs": DEFAULT_RECORDING_DIRS,
    "live_control": _env_flag("FKVIEWER_LIVE_CONTROL", False),
    "request_timeout_sec": 0.7,
    "arms": {
        "left": {"url": "http://127.0.0.1:5000"},
        "right": {"url": "http://127.0.0.1:5001"},
    },
    "wuji": {"url": "http://127.0.0.1:8765"},
    "easydp": {"url": "http://127.0.0.1:8090", "config": "task_insertion_stage2/dual_arm_predict"},
}


def deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_update(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: Path | None = None) -> dict[str, Any]:
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    config_path = path or (APP_ROOT / "config.json")
    if config_path.exists():
        with config_path.open("r", encoding="utf-8") as handle:
            deep_update(config, json.load(handle))
    if os.environ.get("FKVIEWER_LIVE_CONTROL") is not None:
        config["live_control"] = _env_flag("FKVIEWER_LIVE_CONTROL")
    return config


def json_response(handler: BaseHTTPRequestHandler, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
    try:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    except ValueError as exc:
        status = HTTPStatus.INTERNAL_SERVER_ERROR
        body = json.dumps(
            {"ok": False, "error": f"invalid JSON payload: {exc}"},
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    try:
        handler.send_response(status.value)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
    except (BrokenPipeError, ConnectionResetError):
        return


def read_json(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0"))
    if length <= 0:
        return {}
    raw = handler.rfile.read(length)
    if not raw:
        return {}
    data = json.loads(raw.decode("utf-8"))
    if not isinstance(data, dict):
        raise ValueError("JSON body must be an object")
    return data


def request_json(method: str, url: str, payload: dict[str, Any] | None = None, timeout: float = 0.7) -> dict[str, Any]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    req = Request(url, data=body, method=method, headers={"Content-Type": "application/json"})
    try:
        with urlopen(req, timeout=timeout) as response:
            data = response.read()
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        return {"ok": False, "status": exc.code, "error": detail}
    except (URLError, TimeoutError, OSError) as exc:
        return {"ok": False, "error": str(exc)}
    try:
        parsed = json.loads(data.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return {"ok": False, "error": "invalid JSON response"}
    if isinstance(parsed, dict):
        return parsed
    return {"ok": False, "error": "JSON response was not an object", "data": parsed}


def parse_profile_file(path: Path) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    profiles: dict[str, dict[str, str]] = {}
    current: str | None = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        text = line.strip()
        if indent == 0 and text.endswith(":"):
            current = text[:-1].strip()
            profiles[current] = {}
            continue
        if current is None or ":" not in text:
            continue
        key, value = text.split(":", 1)
        profiles[current][key.strip()] = value.strip().strip("\"'")
    return profiles


def profile_file_payload(path: Path) -> dict[str, Any]:
    profiles = parse_profile_file(path)
    return {
        "ok": True,
        "path": str(path),
        "profiles": [
            {
                "name": name,
                "params": [{"key": key, "value": value} for key, value in params.items()],
            }
            for name, params in profiles.items()
        ],
    }


def format_yaml_scalar(value: Any) -> str:
    text = str(value).strip()
    if text == "":
        return '""'
    lowered = text.lower()
    if lowered in {"true", "false"}:
        return lowered
    if any(char in text for char in ["#", ":"]) or text != str(value):
        return json.dumps(text, ensure_ascii=False)
    return text


def update_profile_file(path: Path, profiles: dict[str, dict[str, Any]]) -> dict[str, Any]:
    if not path.exists():
        raise ValueError(f"YAML file does not exist: {path}")
    lines = path.read_text(encoding="utf-8").splitlines()
    current: str | None = None
    changed = 0
    out: list[str] = []
    for raw in lines:
        line_no_comment = raw.split("#", 1)[0].rstrip()
        stripped = line_no_comment.strip()
        indent = len(line_no_comment) - len(line_no_comment.lstrip())
        if indent == 0 and stripped.endswith(":"):
            current = stripped[:-1].strip()
            out.append(raw)
            continue
        if current in profiles and ":" in stripped:
            key = stripped.split(":", 1)[0].strip()
            if key in profiles[current]:
                colon_index = raw.find(":")
                head = raw[: colon_index + 1]
                comment = ""
                hash_index = raw.find("#", colon_index + 1)
                if hash_index >= 0:
                    comment = "  " + raw[hash_index:]
                out.append(f"{head} {format_yaml_scalar(profiles[current][key])}{comment}")
                changed += 1
                continue
        out.append(raw)
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return {"ok": True, "path": str(path), "changed": changed}


def profile_file_for_target(config: dict[str, Any], target: str) -> Path:
    if target not in {"teleop_split", "impedance_profiles"}:
        raise ValueError(f"unknown yaml target {target!r}")
    return Path(config["profile_file"])


def _is_under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def _rel(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def list_safe_files(root: Path, suffixes: tuple[str, ...], *, max_depth: int = 5) -> list[dict[str, str]]:
    if not root.exists():
        return []
    out: list[dict[str, str]] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if len(path.relative_to(root).parts) > max_depth:
            continue
        if any(part in {"build", "install", "__pycache__", ".git"} for part in path.relative_to(root).parts):
            continue
        if path.name.endswith(suffixes):
            out.append({"label": _rel(path, root), "path": str(path)})
    return out


def dedupe_options(items: list[dict[str, str]]) -> list[dict[str, str]]:
    seen: set[str] = set()
    out: list[dict[str, str]] = []
    for item in items:
        value = item.get("path") or item.get("value") or item.get("label")
        if not value or value in seen:
            continue
        seen.add(value)
        out.append(item)
    return out


def recording_dir_options(config: dict[str, Any]) -> list[dict[str, str]]:
    candidates = list(config.get("recording_dirs", []))
    for root in [Path.home(), Path("/data"), Path("/tmp")]:
        for name in ("quest3_recordings", "recordings"):
            candidates.append(str(root / name))
    items = [{"label": path, "path": path} for path in candidates]
    return dedupe_options(items)


def shell_arg(value: Any) -> str:
    return shlex.quote(str(value))


def ros_arg(name: str, value: Any) -> str:
    return shell_arg(f"{name}:={value}")


def choice(selections: dict[str, Any], name: str, default: Any) -> str:
    value = selections.get(name)
    if value is None or str(value).strip() == "":
        return str(default)
    return str(value)


def bool_choice(selections: dict[str, Any], name: str, default: bool = False) -> str:
    raw = str(selections.get(name, "true" if default else "false")).strip().lower()
    return "true" if raw in {"1", "true", "yes", "on"} else "false"


def extract_wuji_pose(path: Path, side: str, pose: str) -> list[float] | None:
    key = f"{side}_{pose}_pose:"
    if not path.exists():
        return None
    lines = path.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if not line.strip().startswith(key):
            continue
        collected: list[str] = []
        suffix = line.split(":", 1)[1].strip()
        if suffix:
            collected.append(suffix)
        for follow in lines[index + 1:]:
            collected.append(follow.strip())
            if "]" in follow:
                break
        try:
            values = ast.literal_eval(" ".join(collected))
        except (ValueError, SyntaxError):
            return None
        if isinstance(values, list) and len(values) == 20:
            return [float(value) for value in values]
    return None


def _parse_inline_value(text: str) -> Any:
    stripped = text.strip()
    if stripped == "":
        return ""
    lowered = stripped.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    if re.fullmatch(r"[-+]?\d+", stripped):
        try:
            return int(stripped)
        except ValueError:
            return stripped.strip("\"'")
    if re.fullmatch(r"[-+]?(?:\d+\.\d*|\.\d+)(?:[eE][-+]?\d+)?", stripped):
        try:
            return float(stripped)
        except ValueError:
            return stripped.strip("\"'")
    return stripped.strip("\"'")


def extract_wuji_hand_config(path: Path) -> dict[str, Any]:
    data: dict[str, Any] = {"poses": {}, "params": {}}
    if not path.exists():
        return data
    lines = path.read_text(encoding="utf-8").splitlines()
    index = 0
    while index < len(lines):
        raw = lines[index]
        text = raw.split("#", 1)[0].strip()
        index += 1
        if not text or ":" not in text:
            continue
        key, suffix = text.split(":", 1)
        key = key.strip()
        suffix = suffix.strip()
        if key in {"wuji_trigger_hand", "ros__parameters"}:
            continue
        if key.endswith("_pose"):
            collected: list[str] = []
            if suffix:
                collected.append(suffix)
            while "]" not in suffix and index < len(lines):
                follow = lines[index].split("#", 1)[0].strip()
                index += 1
                if follow:
                    collected.append(follow)
                if "]" in follow:
                    break
            try:
                values = ast.literal_eval(" ".join(collected))
            except (ValueError, SyntaxError):
                continue
            if isinstance(values, list) and len(values) == 20:
                data["poses"][key] = [float(value) for value in values]
            continue
        data["params"][key] = _parse_inline_value(suffix)
    return data


def validate_joint_positions(values: Any) -> list[float]:
    if not isinstance(values, list) or len(values) != 20:
        raise ValueError("joint positions must contain exactly 20 values")
    out: list[float] = []
    for index, value in enumerate(values):
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"joint {index + 1} is not a number") from exc
        out.append(numeric)
    return out


def hand_config_path(config: dict[str, Any], raw_path: Any = "") -> Path:
    base = DEFAULT_QUEST_ROOT / "config"
    candidate = Path(str(raw_path or config["wuji_config_file"]))
    if not candidate.is_absolute():
        candidate = base / candidate
    if not _is_under(candidate, base):
        raise ValueError(f"hand config must be under {base}")
    return candidate


def ros2_joint_command(setup: str, topic: str, positions: list[float]) -> str:
    payload = {"position": [round(value, 6) for value in positions]}
    return (
        f"{setup} && ros2 topic pub --times 3 --rate 20 --wait-matching-subscriptions 0 "
        f"--qos-profile sensor_data "
        f"{shell_arg(topic)} sensor_msgs/msg/JointState {shell_arg(json.dumps(payload))}"
    )


def ros2_service_command(setup: str, service: str, srv_type: str, payload: dict[str, Any]) -> str:
    return f"{setup} && ros2 service call {shell_arg(service)} {shell_arg(srv_type)} {shell_arg(json.dumps(payload))}"


def command_catalog(config: dict[str, Any], selections: dict[str, Any] | None = None) -> dict[str, dict[str, str]]:
    selections = selections or {}
    workspace = Path(config["workspace"])
    easydp = Path(config["easydp_root"])
    easydp_python = str(config.get("easydp_python") or DEFAULT_EASYDP_PYTHON)
    easydp_env = f"export PATH={shell_arg(Path(easydp_python).parent)}:$PATH"
    setup = "source setup_env.bash"
    wuji_config = config["wuji_config_file"]
    wuji_policy = config["wuji_policy_file"]
    easydp_cfg = config.get("easydp", {}).get("config", "task_insertion_stage2/dual_arm_predict")
    if selections.get("wuji_config"):
        wuji_config = str(selections["wuji_config"])
    if selections.get("wuji_policy"):
        wuji_policy = str(selections["wuji_policy"])
    teleop_profile_file = str(selections.get("profile_file") or config["profile_file"])
    teleop_profile = str(selections.get("profile") or "quest_teleop")
    teleop_launch = Path(str(selections.get("teleop_launch") or "simple_dual_profile.launch.py")).name
    keep_close = bool_choice(selections, "keep_close", False)
    impedance_launch = Path(str(selections.get("impedance_launch") or "http_control.launch.py")).name
    hand_launch = Path(str(selections.get("hand_launch") or "wujihand_dual.launch.py")).name
    hand_launch_args: list[str] = []
    if hand_launch == "wujihand_dual.launch.py":
        hand_launch_args = [
            ros_arg("rviz", bool_choice(selections, "hand_rviz", False)),
            ros_arg("foxglove", bool_choice(selections, "hand_foxglove", False)),
        ]
    elif hand_launch == "wujihand.launch.py":
        hand_launch_args = [
            ros_arg("hand_name", choice(selections, "hand_name", "hand_0")),
            ros_arg("serial_number", choice(selections, "hand_serial", "")),
            ros_arg("hand_side", choice(selections, "hand_side", "")),
            ros_arg("publish_rate", choice(selections, "hand_publish_rate", "1000.0")),
            ros_arg("filter_cutoff_freq", choice(selections, "hand_filter_cutoff_freq", "10.0")),
            ros_arg("diagnostics_rate", choice(selections, "hand_diagnostics_rate", "10.0")),
            ros_arg("rviz", bool_choice(selections, "hand_rviz", False)),
            ros_arg("foxglove", bool_choice(selections, "hand_foxglove", False)),
        ]
    elif hand_launch == "home.launch.py":
        hand_launch_args = [
            ros_arg("hand_names", choice(selections, "hand_names", "hand_left,hand_right")),
            ros_arg("duration", choice(selections, "hand_home_duration", "2.0")),
            ros_arg("rate", choice(selections, "hand_home_rate", "100.0")),
        ]
    data_recorder_config = choice(selections, "data_recorder_config", config["data_recorder_config_file"])
    record_out_dir = choice(selections, "record_out_dir", DEFAULT_RECORDING_DIRS[0])
    require_cameras = bool_choice(selections, "require_cameras", True)
    single_config = choice(selections, "single_config", config["single_teleop_config_file"])
    single_side = choice(selections, "single_side", "right")
    single_robot_ip = choice(selections, "single_robot_ip", "172.16.0.3")
    single_robot_type = choice(selections, "single_robot_type", "fr3")
    single_load_gripper = bool_choice(selections, "single_load_gripper", False)
    single_start_rviz = bool_choice(selections, "single_start_rviz", False)
    single_left_wuji = "true" if single_side == "left" else "false"
    single_right_wuji = "false" if single_side == "left" else "true"

    dual_profile_prefix = (
        f"{setup} && ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py "
        f"{ros_arg('profile_file', teleop_profile_file)} "
        f"{ros_arg('wuji_keep_close', keep_close)} "
    )
    single_base_args = " ".join(
        [
            ros_arg("config_file", single_config),
            ros_arg("robot_ip", single_robot_ip),
            ros_arg("robot_type", single_robot_type),
            ros_arg("load_gripper", single_load_gripper),
            ros_arg("start_rviz", single_start_rviz),
            ros_arg("left_wuji_enabled", single_left_wuji),
            ros_arg("right_wuji_enabled", single_right_wuji),
        ]
    )
    return {
        "franka_stack": {
            "label": "Franka Stack",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch quest3_oculus_rviz simple_dual_franka_stack.launch.py",
        },
        "quest_teleop": {
            "label": "Quest Teleop",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch quest3_oculus_rviz simple_dual_quest_teleop.launch.py",
        },
        "all_in_one": {
            "label": "All In One",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch quest3_oculus_rviz simple_dual_all.launch.py",
        },
        "policy_profile": {
            "label": "Policy Profile",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch quest3_oculus_rviz simple_dual_policy.launch.py",
        },
        "impedance_franka_stack": {
            "label": "双臂 Franka Stack",
            "cwd": str(workspace),
            "cmd": f"{dual_profile_prefix}{ros_arg('profile', 'franka_stack')}",
        },
        "impedance_policy_stack": {
            "label": "Policy 阻抗栈",
            "cwd": str(workspace),
            "cmd": f"{dual_profile_prefix}{ros_arg('profile', 'policy_inference')}",
        },
        "wuji_trigger_service": {
            "label": "Wuji Service Mode",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 run quest3_oculus_rviz wuji_trigger_hand_node --ros-args "
                f"--params-file {shell_arg(wuji_config)} --params-file {shell_arg(wuji_policy)} "
                "-p left_enabled:=true -p right_enabled:=true"
            ),
        },
        "impedance_selected": {
            "label": "Selected Impedance Launch",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch serl_franka_controllers_ros2 {impedance_launch}",
        },
        "hand_selected": {
            "label": "Selected Wuji Launch",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch wujihand_bringup {hand_launch} {' '.join(hand_launch_args)}".strip(),
        },
        "teleop_selected": {
            "label": "Selected Teleop Profile",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz {teleop_launch} "
                f"{ros_arg('profile_file', teleop_profile_file)} {ros_arg('profile', teleop_profile)}"
            ),
        },
        "teleop_terminal1_franka_stack": {
            "label": "启动机械臂",
            "cwd": str(workspace),
            "cmd": f"{dual_profile_prefix}{ros_arg('profile', 'franka_stack')}",
        },
        "teleop_terminal2_quest": {
            "label": "启动手柄",
            "cwd": str(workspace),
            "cmd": f"{dual_profile_prefix}{ros_arg('profile', 'quest_teleop')}",
        },
        "teleop_data_recorder": {
            "label": "Quest Data Recorder",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz data_recorder.launch.py "
                f"{ros_arg('config_file', data_recorder_config)} "
                f"{ros_arg('out_data_dir', record_out_dir)} "
                f"{ros_arg('require_cameras', require_cameras)}"
            ),
        },
        "teleop_single_arm": {
            "label": "Single Arm Teleop",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py "
                f"{single_base_args}"
            ),
        },
        "teleop_single_arm_wuji": {
            "label": "Single Arm Teleop + Wuji",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py "
                f"{single_base_args} {ros_arg('start_wuji_trigger_hand', 'true')}"
            ),
        },
        "teleop_single_wuji_dry_run": {
            "label": "Single Wuji Button Dry Run",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py "
                f"{ros_arg('config_file', single_config)} "
                f"{ros_arg('start_impedance_stack', 'false')} "
                f"{ros_arg('start_wuji_trigger_hand', 'true')} "
                f"{ros_arg('wuji_dry_run', 'true')} "
                f"{ros_arg('left_wuji_enabled', single_left_wuji)} "
                f"{ros_arg('right_wuji_enabled', single_right_wuji)}"
            ),
        },
        "teleop_single_record_random": {
            "label": "Single Arm Recording + Random Pose",
            "cwd": str(workspace),
            "cmd": (
                f"{setup} && ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py "
                f"{single_base_args} "
                f"{ros_arg('start_wuji_trigger_hand', 'false')} "
                f"{ros_arg('start_data_recorder', 'true')} "
                f"{ros_arg('data_recorder_config_file', data_recorder_config)} "
                f"{ros_arg('out_data_dir', record_out_dir)} "
                f"{ros_arg('require_cameras', require_cameras)} "
                f"{ros_arg('random_pose_after_recording', 'true')}"
            ),
        },
        "easydp_server": {
            "label": "EasyDP Server",
            "cwd": str(easydp),
            "cmd": f"{shell_arg(easydp_python)} main.py --config-name {shell_arg(easydp_cfg)}",
        },
        "easydp_client_debug": {
            "label": "EasyDP Dual Client Debug",
            "cwd": str(easydp),
            "cmd": f"{shell_arg(easydp_python)} projects/task_insertion_stage2/client/client_dual.py --debug",
        },
        "easydp_client": {
            "label": "双臂推理客户端",
            "cwd": str(easydp),
            "cmd": f"{easydp_env} && ./scripts/run_pinned.sh {EASYDP_INFERENCE_CLIENT_REL}",
        },
        "easydp_reset": {
            "label": "一键恢复双臂位置",
            "cwd": str(easydp),
            "cmd": f"{easydp_env} && ./scripts/reset.sh",
        },
        "unified_control_stack": {
            "label": "统一阻抗与仲裁栈",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch unified_impedance_control unified_stack.launch.py",
        },
        "unified_inference_client": {
            "label": "统一栈 EasyDP 推理客户端",
            "cwd": str(easydp),
            "cmd": f"{shell_arg(easydp_python)} {EASYDP_INFERENCE_CLIENT_REL}",
        },
        "unified_quest_layer": {
            "label": "Quest 接管与录制层",
            "cwd": str(workspace),
            "cmd": f"{setup} && ros2 launch unified_impedance_control unified_quest_layer.launch.py",
        },
    }


def pgrep(pattern: str) -> list[str]:
    try:
        result = subprocess.run(
            ["pgrep", "-af", pattern],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=0.6,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    lines = []
    own_pid = os.getpid()
    for line in result.stdout.splitlines():
        if line.startswith(f"{own_pid} "):
            continue
        lines.append(line[:220])
    return lines


ROUTINE_HTTP_GET_RE = re.compile(
    r'\bHTTP\s+GET\b|"GET\s+/[^\s]*\s+HTTP/[\d.]+"\s+\d{3}',
    re.IGNORECASE,
)


def filter_runtime_log(text: str) -> str:
    """Remove periodic HTTP access probes while preserving errors and actions."""
    return "\n".join(line for line in text.splitlines() if not ROUTINE_HTTP_GET_RE.search(line))


class FKViewerState:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.lock = threading.Lock()
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_logs: dict[str, Path] = {}
        self.reported_process_exits: set[str] = set()
        self.events: list[dict[str, Any]] = []
        self.hand_topic_ready_until: dict[str, float] = {}
        self.logs_dir = APP_ROOT / "logs"
        self.logs_dir.mkdir(exist_ok=True)

    @property
    def live_control(self) -> bool:
        return bool(self.config.get("live_control", False))

    def add_event(self, level: str, message: str, detail: Any | None = None) -> None:
        event = {
            "time": time.strftime("%H:%M:%S"),
            "level": level,
            "message": message,
            "detail": detail,
        }
        with self.lock:
            self.events.append(event)
            self.events = self.events[-80:]

    def status(self) -> dict[str, Any]:
        timeout = float(self.config.get("request_timeout_sec", 0.7))
        profiles = parse_profile_file(Path(self.config["profile_file"]))
        arm_status = {}
        for arm, spec in self.config.get("arms", {}).items():
            base = str(spec.get("url", "")).rstrip("/")
            health = request_json("GET", f"{base}/health", timeout=timeout) if base else {"ok": False}
            state = request_json("GET", f"{base}/getstate", timeout=timeout) if health.get("ok") else None
            arm_status[arm] = {"url": base, "health": health, "state": state}
        wuji_url = str(self.config.get("wuji", {}).get("url", "")).rstrip("/")
        easydp_url = str(self.config.get("easydp", {}).get("url", "")).rstrip("/")
        authority_url = str(self.config.get("arms", {}).get("left", {}).get("url", "")).rstrip("/")
        authority = (
            request_json("GET", f"{authority_url}/control_authority", timeout=timeout)
            if authority_url
            else {"ok": False}
        )
        return {
            "ok": True,
            "live_control": self.live_control,
            "workspace": self.config["workspace"],
            "easydp_root": self.config["easydp_root"],
            "profiles": profiles,
            "commands": command_catalog(self.config),
            "arms": arm_status,
            "wuji": {
                "url": wuji_url,
                "health": request_json("GET", f"{wuji_url}/health", timeout=timeout) if wuji_url else {"ok": False},
            },
            "unified": authority,
            "easydp": {
                "url": easydp_url,
                "config": self.config.get("easydp", {}).get("config"),
                "health": self.probe_text(f"{easydp_url}/healthz", timeout=timeout) if easydp_url else {"ok": False},
            },
            "managed_processes": self.managed_processes(),
            "detected_processes": {
                "franka_stack": pgrep("simple_dual_franka_stack.launch.py"),
                "quest_teleop": pgrep("simple_dual_quest_teleop.launch.py"),
                "all_in_one": pgrep("simple_dual_all.launch.py"),
                "policy_profile": pgrep("simple_dual_policy.launch.py"),
                "impedance_franka_stack": pgrep("simple_dual_franka_stack.launch.py|profile:=franka_stack"),
                "impedance_policy_stack": pgrep("simple_dual_policy.launch.py|profile:=policy_inference"),
                "impedance_selected": pgrep("serl_franka_controllers_ros2"),
                "teleop_terminal1_franka_stack": pgrep("profile:=franka_stack"),
                "teleop_terminal2_quest": pgrep("profile:=quest_teleop"),
                "single_teleop": pgrep("simple_impedance_teleop.launch.py"),
                "data_recorder_launch": pgrep("data_recorder.launch.py"),
                "wuji": pgrep("wuji_trigger_hand_node"),
                "wujihand_driver": pgrep("wujihand_driver_node"),
                "hand_selected": pgrep("wujihand_dual.launch.py|wujihand.launch.py|home.launch.py"),
                "easydp": pgrep("main.py --config-name"),
                "easydp_client": pgrep("client_dual.py"),
                "easydp_reset": pgrep("move_dual_arm_to_pose.py"),
                "recorder": pgrep("data_recorder_node"),
                "unified_control_stack": pgrep("unified_stack.launch.py|unified_control_authority"),
                "unified_inference_client": pgrep("client_dual.py"),
                "unified_quest_layer": pgrep("unified_quest_layer.launch.py"),
            },
            "events": list(self.events),
            "now": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

    def yaml_config(self, target: str) -> dict[str, Any]:
        path = profile_file_for_target(self.config, target)
        return profile_file_payload(path)

    def hand_config(self, raw_path: Any = "") -> dict[str, Any]:
        path = hand_config_path(self.config, raw_path)
        data = extract_wuji_hand_config(path)
        return {"ok": True, "path": str(path), **data}

    def update_yaml_config(self, target: str, profiles: dict[str, Any]) -> dict[str, Any]:
        path = profile_file_for_target(self.config, target)
        normalized: dict[str, dict[str, Any]] = {}
        for profile, params in profiles.items():
            if isinstance(params, dict):
                normalized[str(profile)] = {str(key): value for key, value in params.items()}
        result = update_profile_file(path, normalized)
        label = "impedance YAML" if target == "impedance_profiles" else "teleop YAML"
        self.add_event("info", f"updated {label}", {"path": str(path), "changed": result["changed"]})
        return result

    def tail_logs(self, names: list[str], max_bytes: int = 32000) -> dict[str, Any]:
        out: dict[str, Any] = {}
        with self.lock:
            log_items = {name: self.process_logs.get(name) for name in names}
        for name, path in log_items.items():
            if path is None:
                out[name] = {"ok": False, "text": "", "error": "not started by FKViewer"}
                continue
            try:
                size = path.stat().st_size
                read_bytes = max_bytes * 8
                with path.open("rb") as handle:
                    if size > read_bytes:
                        handle.seek(size - read_bytes)
                    raw_text = handle.read(read_bytes).decode("utf-8", errors="replace")
                text = filter_runtime_log(raw_text)[-max_bytes:]
            except OSError as exc:
                out[name] = {"ok": False, "text": "", "error": str(exc)}
                continue
            out[name] = {"ok": True, "path": str(path), "text": text}
        return {"ok": True, "logs": out}

    def options(self, module: str = "all") -> dict[str, Any]:
        roots = self.config.get("code_roots", {})
        serl_root = Path(roots.get("impedance", DEFAULT_SERL_ROOT))
        wuji_ros2_root = Path(roots.get("wujihandros2", DEFAULT_WUJI_ROS2_ROOT))
        wuji_py_root = Path(roots.get("wujihandpy", DEFAULT_WUJI_PY_ROOT))
        teleop_root = Path(roots.get("teleop", DEFAULT_QUEST_ROOT))
        easydp_root = Path(roots.get("inference", self.config["easydp_root"]))
        unified_root = Path(roots.get("unified", DEFAULT_UNIFIED_ROOT))
        profile_file = Path(self.config["profile_file"])
        teleop_configs = list_safe_files(teleop_root / "config", (".yaml", ".yml", ".json"), max_depth=2)
        teleop_launches = list_safe_files(teleop_root / "launch", (".launch.py",), max_depth=2)
        data_recorder_configs = [
            item for item in teleop_configs if Path(item["path"]).name == "data_recorder.yaml"
        ] or [{"label": "data_recorder.yaml", "path": str(DEFAULT_DATA_RECORDER_FILE)}]
        single_teleop_configs = [
            item for item in teleop_configs if "impedance_teleop" in item["label"] or "teleop" in item["label"]
        ] or [{"label": "simple_impedance_teleop.yaml", "path": str(DEFAULT_SINGLE_TELEOP_FILE)}]
        wuji_config_data = extract_wuji_hand_config(Path(self.config["wuji_config_file"]))
        roots_data = {
            "impedance": str(serl_root),
            "hand": str(wuji_ros2_root),
            "wujihandros2": str(wuji_ros2_root),
            "wujihandpy": str(wuji_py_root),
            "teleop": str(teleop_root),
            "inference": str(easydp_root),
            "unified": str(unified_root),
        }
        hand_options = {
            "ros2_launches": list_safe_files(wuji_ros2_root / "wujihand_bringup/launch", (".launch.py",), max_depth=2),
            "ros2_scripts": list_safe_files(wuji_ros2_root / "wujihand_bringup/scripts", (".py",), max_depth=2),
            "python_examples": list_safe_files(wuji_py_root / "example", (".py",), max_depth=4),
            "bridge_configs": list_safe_files(teleop_root / "config", ("wuji_trigger_hand.yaml", "wuji_policy_bridge.yaml"), max_depth=2),
            "config_file": self.config["wuji_config_file"],
            "poses": wuji_config_data.get("poses", {}),
            "params": wuji_config_data.get("params", {}),
            "topic_choices": [
                {"label": "/hand_left/joint_commands", "value": "/hand_left/joint_commands"},
                {"label": "/hand_right/joint_commands", "value": "/hand_right/joint_commands"},
                {"label": "/hand_0/joint_commands", "value": "/hand_0/joint_commands"},
                {"label": "/hand_1/joint_commands", "value": "/hand_1/joint_commands"},
            ],
            "hand_names": [
                {"label": "hand_left,hand_right", "value": "hand_left,hand_right"},
                {"label": "hand_0,hand_1", "value": "hand_0,hand_1"},
                {"label": "hand_0", "value": "hand_0"},
            ],
            "hand_sides": [
                {"label": "auto", "value": ""},
                {"label": "left", "value": "left"},
                {"label": "right", "value": "right"},
            ],
            "boolean_choices": [
                {"label": "false", "value": "false"},
                {"label": "true", "value": "true"},
            ],
            "rate_choices": [
                {"label": "1000.0", "value": "1000.0"},
                {"label": "500.0", "value": "500.0"},
                {"label": "200.0", "value": "200.0"},
                {"label": "100.0", "value": "100.0"},
            ],
        }
        if module == "hand":
            return {"ok": True, "module": module, "hand": hand_options, "roots": roots_data}

        data = {
            "ok": True,
            "module": module,
            "roots": roots_data,
            "impedance": {
                "configs": list_safe_files(serl_root / "config", (".yaml", ".yml", ".json"), max_depth=3),
                "launches": list_safe_files(serl_root / "launch", (".launch.py",), max_depth=2),
                "profile_file": str(profile_file),
                "profiles": parse_profile_file(profile_file),
                "profile_files": [
                    {"label": _rel(profile_file, teleop_root), "path": str(profile_file)}
                ],
                "quest_configs": teleop_configs,
                "data_recorder_configs": data_recorder_configs,
                "boolean_choices": [
                    {"label": "true", "value": "true"},
                    {"label": "false", "value": "false"},
                ],
                "robot_types": [
                    {"label": "fr3", "value": "fr3"},
                    {"label": "panda", "value": "panda"},
                ],
                "robot_ips": [
                    {"label": "left / 172.16.0.2", "value": "172.16.0.2"},
                    {"label": "right / 172.16.0.3", "value": "172.16.0.3"},
                ],
                "control_modes": [
                    {"label": "service", "value": "service"},
                    {"label": "trigger", "value": "trigger"},
                ],
                "cpu_choices": [
                    {"label": "不绑核", "value": ""},
                    *[{"label": value, "value": value} for value in ["2-3", "4", "6", "8-9", "10", "12", "14", "16", "18", "20", "22", "24", "25"]],
                ],
                "port_choices": [
                    {"label": "5000", "value": "5000"},
                    {"label": "5001", "value": "5001"},
                ],
                "base_frames": [
                    {"label": "base", "value": "base"},
                    {"label": "fr3_link0", "value": "fr3_link0"},
                ],
                "wuji_serials": [
                    {"label": "left / 348534683533", "value": "348534683533"},
                    {"label": "right / 3671354F3333", "value": "3671354F3333"},
                ],
                "rate_choices": [
                    {"label": "100.0", "value": "100.0"},
                    {"label": "200.0", "value": "200.0"},
                    {"label": "500.0", "value": "500.0"},
                ],
            },
            "hand": hand_options,
            "teleop": {
                "configs": teleop_configs,
                "launches": teleop_launches,
                "profiles": parse_profile_file(profile_file),
                "profile_file": str(profile_file),
                "profile_files": [
                    {"label": _rel(profile_file, teleop_root), "path": str(profile_file)}
                ],
                "data_recorder_config_file": self.config["data_recorder_config_file"],
                "data_recorder_configs": data_recorder_configs,
                "single_config_file": self.config["single_teleop_config_file"],
                "single_configs": single_teleop_configs,
                "record_out_dir": recording_dir_options(self.config)[0]["path"],
                "recording_dirs": recording_dir_options(self.config),
                "single_robot_ips": [
                    {"label": "right / 172.16.0.3", "value": "172.16.0.3"},
                    {"label": "left / 172.16.0.2", "value": "172.16.0.2"},
                ],
                "single_sides": [
                    {"label": "right controller + right Wuji", "value": "right"},
                    {"label": "left controller + left Wuji", "value": "left"},
                ],
                "boolean_choices": [
                    {"label": "true", "value": "true"},
                    {"label": "false", "value": "false"},
                ],
            },
            "inference": {
                "client_file": str(easydp_root / EASYDP_INFERENCE_CLIENT_REL),
            },
            "unified": {
                "package_root": str(unified_root),
                "stack_launch": str(unified_root / "launch/unified_stack.launch.py"),
                "quest_launch": str(unified_root / "launch/unified_quest_layer.launch.py"),
                "record_out_dir": recording_dir_options(self.config)[0]["path"],
            },
        }
        if module != "all" and module in data:
            return {"ok": True, "module": module, module: data[module], "roots": data["roots"]}
        return data

    @staticmethod
    def probe_text(url: str, timeout: float) -> dict[str, Any]:
        try:
            with urlopen(url, timeout=timeout) as response:
                return {"ok": response.status < 400, "status": response.status, "body": response.read(256).decode("utf-8", "replace")}
        except (URLError, TimeoutError, OSError, HTTPError) as exc:
            return {"ok": False, "error": str(exc)}

    def managed_processes(self) -> dict[str, Any]:
        out = {}
        ended: list[tuple[str, int, int]] = []
        with self.lock:
            items = list(self.processes.items())
            for name, proc in items:
                code = proc.poll()
                out[name] = {"pid": proc.pid, "running": code is None, "returncode": code}
                if code is not None and name not in self.reported_process_exits:
                    self.reported_process_exits.add(name)
                    ended.append((name, proc.pid, code))
        for name, pid, code in ended:
            level = "info" if code in {0, -signal.SIGINT} else "err"
            self.add_event(level, f"{name} exited", {"pid": pid, "returncode": code})
        return out

    def run_catalog_command(self, name: str, selections: dict[str, Any] | None = None) -> dict[str, Any]:
        selections = selections or {}
        catalog = command_catalog(self.config, selections)
        if name not in catalog:
            raise ValueError(f"unknown command {name!r}")
        spec = catalog[name]
        if not self.live_control:
            detail = {"command": spec, "selections": selections}
            self.add_event("dry", f"{spec['label']} dry-run", detail)
            return {"ok": True, "dry_run": True, **detail}
        with self.lock:
            old = self.processes.get(name)
            if old is not None and old.poll() is None:
                return {"ok": True, "message": f"{name} is already running", "pid": old.pid, "command": spec}
            log_path = self.logs_dir / f"{name}-{int(time.time())}.log"
            log_handle = log_path.open("ab")
            proc = subprocess.Popen(
                ["bash", "-lc", spec["cmd"]],
                cwd=spec["cwd"],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.processes[name] = proc
            self.process_logs[name] = log_path
            self.reported_process_exits.discard(name)
        self.add_event("run", f"started {spec['label']}", {"pid": proc.pid, "log": str(log_path), "selections": selections})
        return {"ok": True, "dry_run": False, "pid": proc.pid, "log": str(log_path), "command": spec, "selections": selections}

    def stop_managed_process(self, name: str) -> dict[str, Any]:
        with self.lock:
            proc = self.processes.get(name)
        if proc is None:
            if not self.live_control:
                self.add_event("dry", f"stop {name} dry-run", {"managed": False})
                return {"ok": True, "dry_run": True, "message": f"{name} was not started by FKViewer"}
            return {"ok": False, "error": f"{name} was not started by FKViewer"}
        if proc.poll() is not None:
            return {"ok": True, "message": f"{name} already exited", "returncode": proc.returncode}
        if not self.live_control:
            self.add_event("dry", f"stop {name} dry-run", {"pid": proc.pid})
            return {"ok": True, "dry_run": True, "pid": proc.pid}
        os.killpg(proc.pid, signal.SIGINT)
        self.add_event("run", f"sent SIGINT to {name}", {"pid": proc.pid})
        return {"ok": True, "pid": proc.pid, "message": "SIGINT sent"}

    def arm_action(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = str(payload.get("arm", ""))
        op = str(payload.get("op", ""))
        if arm not in self.config.get("arms", {}):
            raise ValueError("arm must be left or right")
        endpoint = {
            "health": ("GET", "/health", True),
            "state": ("GET", "/getstate", True),
            "start_impedance": ("POST", "/startimp", False),
            "stop_impedance": ("POST", "/stopimp", False),
            "clear_error": ("POST", "/clearerr", False),
            "update_params": ("POST", "/update_param", False),
            "joint_reset": ("POST", "/jointreset", False),
        }.get(op)
        if endpoint is None:
            raise ValueError(f"unknown arm op {op!r}")
        method, path, read_only = endpoint
        base = str(self.config["arms"][arm]["url"]).rstrip("/")
        body = payload.get("body") if isinstance(payload.get("body"), dict) else {}
        if not read_only and not self.live_control:
            detail = {"method": method, "url": f"{base}{path}", "body": body}
            self.add_event("dry", f"{arm} {op} dry-run", detail)
            return {"ok": True, "dry_run": True, **detail}
        result = request_json(method, f"{base}{path}", body, timeout=float(self.config.get("request_timeout_sec", 0.7)))
        self.add_event("run" if result.get("ok") else "err", f"{arm} {op}", result)
        return result

    def hand_action(self, payload: dict[str, Any]) -> dict[str, Any]:
        op = str(payload.get("op", ""))
        side = str(payload.get("side", ""))
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        selections = payload.get("selections") if isinstance(payload.get("selections"), dict) else {}
        setup = "source setup_env.bash"
        workspace = Path(self.config["workspace"])
        config_path = hand_config_path(self.config, selections.get("wuji_config"))
        config_data = extract_wuji_hand_config(config_path)
        params = config_data.get("params", {})
        topic = str(selections.get(f"{side}_command_topic") or params.get(f"{side}_command_topic") or f"/hand_{side}/joint_commands")

        if op in {"released", "closed"}:
            pose = config_data.get("poses", {}).get(f"{side}_{op}_pose")
            if pose is None:
                raise ValueError(f"could not read {side}_{op}_pose from Wuji config")
            positions = validate_joint_positions(pose)
            command = ros2_joint_command(setup, topic, positions)
            detail = {"topic": topic, "positions": positions, "command": command, "config": str(config_path)}
        elif op == "pose":
            body = payload.get("body") if isinstance(payload.get("body"), dict) else {}
            positions = validate_joint_positions(body.get("positions"))
            command = ros2_joint_command(setup, topic, positions)
            detail = {"topic": topic, "positions": positions, "command": command, "config": str(config_path)}
        elif op in {"enable", "disable"}:
            service = str(selections.get(f"{side}_set_enabled_service") or f"/hand_{side}/set_enabled")
            request = {"finger_id": 255, "joint_id": 255, "enabled": op == "enable"}
            command = ros2_service_command(setup, service, "wujihand_msgs/srv/SetEnabled", request)
            detail = {"service": service, "request": request, "command": command}
        elif op == "reset_error":
            service = str(selections.get(f"{side}_reset_error_service") or f"/hand_{side}/reset_error")
            request = {"finger_id": 255, "joint_id": 255}
            command = ros2_service_command(setup, service, "wujihand_msgs/srv/ResetError", request)
            detail = {"service": service, "request": request, "command": command}
        else:
            raise ValueError("hand op must be released, closed, pose, enable, disable, or reset_error")

        if not self.live_control:
            self.add_event("dry", f"{side} hand {op} dry-run", detail)
            return {"ok": True, "dry_run": True, **detail}
        if "topic" in detail:
            topic = str(detail["topic"])
            now = time.monotonic()
            with self.lock:
                topic_ready = self.hand_topic_ready_until.get(topic, 0.0) > now
            if not topic_ready:
                info_cmd = f"{setup} && ros2 topic info --no-daemon --spin-time 0.5 {shell_arg(topic)}"
                try:
                    info = subprocess.run(
                        ["bash", "-lc", info_cmd],
                        cwd=str(workspace),
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        timeout=3.0,
                        check=False,
                    )
                except subprocess.TimeoutExpired as exc:
                    result = {
                        "ok": False,
                        "error": f"topic info timed out for {topic}",
                        "output": exc.stdout or "",
                        **detail,
                    }
                    self.add_event("err", f"{side} hand {op}", result)
                    return result
                info_output = info.stdout[-4000:]
                match = re.search(r"Subscription count:\s*(\d+)", info_output)
                if info.returncode != 0 or (match and int(match.group(1)) == 0):
                    with self.lock:
                        self.hand_topic_ready_until.pop(topic, None)
                    result = {
                        "ok": False,
                        "error": (
                            f"{topic} 没有检测到订阅者，请先启动官方 ROS2 Driver"
                            if match and int(match.group(1)) == 0
                            else info_output.strip() or f"ros2 topic info failed with return code {info.returncode}"
                        ),
                        "output": info_output,
                        **detail,
                    }
                    self.add_event("err", f"{side} hand {op}", result)
                    return result
                with self.lock:
                    self.hand_topic_ready_until[topic] = time.monotonic() + 4.0
        try:
            completed = subprocess.run(
                ["bash", "-lc", command],
                cwd=str(workspace),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=5.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            result = {"ok": False, "error": "ros2 command timed out", "output": exc.stdout or "", **detail}
            self.add_event("err", f"{side} hand {op}", result)
            return result
        output = completed.stdout[-4000:]
        result = {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "output": output,
            **detail,
        }
        if not result["ok"]:
            result["error"] = output.strip() or f"ros2 command failed with return code {completed.returncode}"
        self.add_event("run" if result["ok"] else "err", f"{side} hand {op}", result)
        return result


class FKRequestHandler(BaseHTTPRequestHandler):
    server: "FKHTTPServer"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/":
            self.serve_file(STATIC_ROOT / "index.html", "text/html; charset=utf-8")
            return
        if path == "/window":
            self.serve_file(STATIC_ROOT / "window.html", "text/html; charset=utf-8")
            return
        if path == "/api/status":
            json_response(self, self.server.state.status())
            return
        if path == "/api/options":
            query = parse_qs(parsed.query)
            module = query.get("module", ["all"])[0]
            json_response(self, self.server.state.options(module))
            return
        if path == "/api/yaml":
            query = parse_qs(parsed.query)
            target = query.get("target", ["teleop_split"])[0]
            try:
                json_response(self, self.server.state.yaml_config(target))
            except ValueError as exc:
                json_response(self, {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if path == "/api/hand_config":
            query = parse_qs(parsed.query)
            raw_path = query.get("path", [""])[0]
            try:
                json_response(self, self.server.state.hand_config(raw_path))
            except ValueError as exc:
                json_response(self, {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if path == "/api/logs":
            query = parse_qs(parsed.query)
            names = query.get("name", [])
            if not names:
                raw = query.get("names", [""])[0]
                names = [name for name in raw.split(",") if name]
            json_response(self, self.server.state.tail_logs(names))
            return
        if path == "/api/config":
            json_response(self, {"ok": True, "config": self.server.state.config})
            return
        if path.startswith("/static/"):
            rel = path.removeprefix("/static/")
            target = (STATIC_ROOT / rel).resolve()
            if not str(target).startswith(str(STATIC_ROOT.resolve())):
                self.send_error(HTTPStatus.FORBIDDEN.value)
                return
            ctype = "text/plain; charset=utf-8"
            if target.suffix == ".css":
                ctype = "text/css; charset=utf-8"
            elif target.suffix == ".js":
                ctype = "application/javascript; charset=utf-8"
            self.serve_file(target, ctype)
            return
        self.send_error(HTTPStatus.NOT_FOUND.value)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/yaml":
            try:
                payload = read_json(self)
                target = str(payload.get("target", "teleop_split"))
                profiles = payload.get("profiles") if isinstance(payload.get("profiles"), dict) else {}
                json_response(self, self.server.state.update_yaml_config(target, profiles))
            except ValueError as exc:
                json_response(self, {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            except Exception as exc:  # noqa: BLE001
                json_response(self, {"ok": False, "error": repr(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        if path != "/api/action":
            self.send_error(HTTPStatus.NOT_FOUND.value)
            return
        try:
            payload = read_json(self)
            action = str(payload.get("action", ""))
            if action == "launch":
                selections = payload.get("selections") if isinstance(payload.get("selections"), dict) else {}
                result = self.server.state.run_catalog_command(str(payload.get("name", "")), selections)
            elif action == "stop":
                result = self.server.state.stop_managed_process(str(payload.get("name", "")))
            elif action == "arm":
                result = self.server.state.arm_action(payload)
            elif action == "hand":
                result = self.server.state.hand_action(payload)
            else:
                raise ValueError(f"unknown action {action!r}")
            json_response(self, result)
        except ValueError as exc:
            json_response(self, {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:  # noqa: BLE001
            json_response(self, {"ok": False, "error": repr(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def serve_file(self, path: Path, content_type: str) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND.value)
            return
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK.value)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.address_string()} - {fmt % args}")


class FKHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], state: FKViewerState) -> None:
        self.state = state
        super().__init__(address, FKRequestHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description="FKViewer control console")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    state = FKViewerState(config)
    state.add_event("info", "FKViewer started", {"live_control": state.live_control})
    try:
        httpd = FKHTTPServer((args.host, args.port), state)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(
                f"FKViewer port is already in use: http://{args.host}:{args.port}\n"
                "Open that URL if FKViewer is already running, or choose another port, for example:\n"
                "  python3 FKViewer/server.py --host 127.0.0.1 --port 8788"
            )
            raise SystemExit(2) from None
        raise
    print(f"FKViewer listening on http://{args.host}:{args.port}")
    print(f"live_control={state.live_control}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nFKViewer stopped.")


if __name__ == "__main__":
    main()
