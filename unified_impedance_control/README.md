# Unified impedance control

This package leaves the existing Franka, Quest, EasyDP, Wuji, and recorder
implementations unchanged. It adds one authority gate in front of their shared
command topics and HTTP ports.

- Default authority is `inference`.
- A rising edge on Quest `Y` toggles `inference` / `teleop`.
- In `teleop`, policy `/pose`, `/pose_precise`, and Wuji HTTP commands are rejected.
- In `inference`, staged Quest arm and hand commands are discarded.
- The original recorder still owns A/B/X and writes the existing HDF5 schema.

Launch the shared hardware stack first, then the Quest layer and/or EasyDP client:

```bash
source setup_env.bash
ros2 launch unified_impedance_control unified_stack.launch.py
ros2 launch unified_impedance_control unified_quest_layer.launch.py
```

Public policy endpoints stay at `5000`, `5001`, and `8765`. The underlying
Franka HTTP servers bind loopback-only backend ports `5100` and `5101`.

