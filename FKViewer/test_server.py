from __future__ import annotations

from pathlib import Path

from FKViewer.server import DEFAULT_CONFIG, command_catalog


def test_keep_close_defaults_to_current_release_behavior() -> None:
    commands = command_catalog(DEFAULT_CONFIG)

    assert "wuji_keep_close:=false" in commands["impedance_policy_stack"]["cmd"]
    assert "wuji_keep_close:=false" in commands["teleop_terminal2_quest"]["cmd"]


def test_keep_close_is_forwarded_to_hand_owning_profiles() -> None:
    commands = command_catalog(DEFAULT_CONFIG, {"keep_close": True})

    assert "wuji_keep_close:=true" in commands["impedance_policy_stack"]["cmd"]
    assert "wuji_keep_close:=true" in commands["teleop_terminal2_quest"]["cmd"]


def test_teleop_checkbox_is_unchecked_and_sent_as_a_boolean() -> None:
    script = (Path(__file__).parent / "static" / "window.js").read_text(
        encoding="utf-8"
    )
    assert 'id="teleop-keep-close" type="checkbox"' in script
    assert 'id="teleop-keep-close" type="checkbox" checked' not in script
    assert 'keep_close: Boolean($("#teleop-keep-close")?.checked)' in script


def test_inference_control_uses_fixed_pinned_commands() -> None:
    commands = command_catalog(
        DEFAULT_CONFIG,
        {"easydp_config": "ignored", "checkpoint": "/tmp/ignored.ckpt"},
    )
    script = (Path(__file__).parent / "static" / "window.js").read_text(
        encoding="utf-8"
    )
    app_script = (Path(__file__).parent / "static" / "app.js").read_text(
        encoding="utf-8"
    )

    assert commands["easydp_client"]["cmd"].endswith(
        "./scripts/run_pinned.sh projects/task_insertion_stage2/client/client_dual.py"
    )
    assert "ignored" not in commands["easydp_client"]["cmd"]
    assert commands["easydp_reset"]["cmd"].endswith("&& ./scripts/reset.sh")
    assert "data-inference-reset" in script
    assert "Checkpoint Root" not in script
    assert "openInferenceSettings" not in script
    assert 'const hasSettings = ["teleop", "impedance", "hand"].includes(module);' in app_script
