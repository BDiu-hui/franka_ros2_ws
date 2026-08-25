from __future__ import annotations

from pathlib import Path

from FKViewer.server import DEFAULT_CONFIG, command_catalog


def test_keep_status_defaults_to_current_release_behavior() -> None:
    commands = command_catalog(DEFAULT_CONFIG)

    assert "wuji_keep_status:=false" in commands["impedance_policy_stack"]["cmd"]
    assert "wuji_keep_status:=false" in commands["teleop_terminal2_quest"]["cmd"]


def test_keep_status_is_forwarded_to_hand_owning_profiles() -> None:
    commands = command_catalog(DEFAULT_CONFIG, {"keep_status": True})

    assert "wuji_keep_status:=true" in commands["impedance_policy_stack"]["cmd"]
    assert "wuji_keep_status:=true" in commands["teleop_terminal2_quest"]["cmd"]


def test_teleop_checkbox_is_unchecked_and_sent_as_a_boolean() -> None:
    script = (Path(__file__).parent / "static" / "window.js").read_text(
        encoding="utf-8"
    )

    assert 'id="teleop-keep-status" type="checkbox"' in script
    assert 'id="teleop-keep-status" type="checkbox" checked' not in script
    assert 'keep_status: Boolean($("#teleop-keep-status")?.checked)' in script
