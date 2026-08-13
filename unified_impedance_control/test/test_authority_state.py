from unified_impedance_control.control_authority_node import AuthorityState, button_pressed


def test_y_toggles_only_on_rising_edges() -> None:
    state = AuthorityState()
    assert state.snapshot()["authority"] == "inference"
    assert state.update_buttons(
        {"Y": True}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (True, False)
    assert state.snapshot()["authority"] == "teleop"
    assert state.update_buttons(
        {"Y": True}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (False, False)
    assert state.update_buttons(
        {"Y": False}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (False, False)
    assert state.update_buttons(
        {"Y": [1.0]}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (True, False)
    assert state.snapshot()["authority"] == "inference"


def test_recording_observer_does_not_change_authority() -> None:
    state = AuthorityState()
    state.update_buttons({"Y": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"Y": 0.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"A": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    assert state.snapshot()["recording"] is True
    assert state.snapshot()["authority"] == "teleop"
    state.update_buttons({"A": 0.0, "B": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    assert state.snapshot()["recording"] is False


def test_y_is_ignored_until_recording_stops() -> None:
    state = AuthorityState()
    state.update_buttons({"Y": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"Y": 0.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"A": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"A": 0.0}, takeover_button="Y", start_button="A", stop_button="B")

    assert state.update_buttons(
        {"Y": 1.0}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (False, True)
    assert state.snapshot()["authority"] == "teleop"

    state.update_buttons({"Y": 0.0, "B": 1.0}, takeover_button="Y", start_button="A", stop_button="B")
    state.update_buttons({"B": 0.0}, takeover_button="Y", start_button="A", stop_button="B")
    assert state.update_buttons(
        {"Y": 1.0}, takeover_button="Y", start_button="A", stop_button="B"
    ) == (True, False)
    assert state.snapshot()["authority"] == "inference"


def test_button_value_shapes() -> None:
    assert button_pressed(True)
    assert button_pressed([0.7])
    assert not button_pressed([])
    assert not button_pressed("true")
