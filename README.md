# TRON2 Manipulation in Isaac Sim

This repository provides an Isaac Sim 5.1 motor simulator for the LimX DACH_TRON2A dual-arm manipulation controller. It uses the official robot description and the official controller's motor interface. [简体中文](README.zh-CN.md)

## Contents

- [Components](#components)
- [PhysX quick start](#physx-quick-start)
- [Controller integration](#controller-integration)
- [Verification and scope](#verification-and-scope)

## Components

| Component | Source or contract |
| --- | --- |
| Physics | Isaac Sim 5.1.0, PhysX, fixed-base DACH_TRON2A. [NVIDIA Python API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/index.html) |
| Robot model | Official `robot_grasper.urdf` at `9939c22e69d27653ec0ba8a505859a2903dd1a71`. The importer resolves the package mesh paths and fixes the gripper mechanism joints in a temporary URDF. [Robot description](https://github.com/limxdynamics/tron2-robot-description/tree/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A) |
| Controller | LimX prebuilt QP/PD runtime at `10af63c20e0dd2149168c6693453cc28f2c2dcbf`. It owns the `MoveJ`, `MoveH`, `MoveP`, `ServoJ`, `ServoP` and state-query WebSocket APIs. [Upstream controller](https://github.com/limxdynamics/tron2_manipulation_controller/tree/10af63c20e0dd2149168c6693453cc28f2c2dcbf) |
| Motor loop | `/motor/cmd` and `/motor/state` in MROS, in left arm × 7, right arm × 7, head pitch/yaw order. The default target rate is 300 Hz. Commands use radians, rad/s, N·m and per-axis PD gains. Applied effort is clipped to the official MJCF actuator limits. |

Isaac applies `tau + kp * (q_target - q) + kd * (dq_target - dq)` at each physics substep. At the default 300 Hz motor rate, four PhysX substeps keep the integration interval below 1 ms. PhysX reports joint position, velocity and effort. The initial pose is the upstream SDK's `DEFAULT_HOME_Q16` pose. Vendor assets and binaries are external dependencies.

## PhysX quick start

Run the commands from this repository's root on an Isaac Sim host. `ISAAC_PYTHON` is the Python launcher for your Isaac Sim 5.1 install, such as `python.sh` for the standalone installation or the virtual environment's `bin/python` for the pip installation.

1. Fetch the pinned controller and description into a directory outside this repository.

   ```bash
   export TRON2_UPSTREAM="$HOME/tron2-controller-upstream"
   ./scripts/get_upstream.sh "$TRON2_UPSTREAM"
   ```

2. Verify name mapping and motor effort in a headless run. Add `--gui` to open the Isaac window when a graphical display is available.

   ```bash
   "$ISAAC_PYTHON" -m tron2_isaac --upstream "$TRON2_UPSTREAM" --smoke
   ```

   A passing run prints `"physics_smoke": "passed"` with measured joint positions and tracking error.

## Controller integration

The integrated launcher uses Ubuntu 24.04 amd64, ROS 2 Jazzy, Git LFS and Linux user/network namespaces. Commands below run from this repository's root. Keep `ISAAC_PYTHON` and `TRON2_UPSTREAM` set as above.

1. Fetch the Jazzy build of the LimX MROS–ROS 2 bridge and set its install path. Isaac sends state to a system-Python ROS 2 subprocess over a Unix socket. The LimX bridge forwards `/motor/state` to MROS and `/motor/cmd` to ROS 2. [LimX ros2-bridger](https://github.com/limxdynamics/ros2-bridger/tree/e0deafa7fff9629573fe3be98b4575a6523a2c3e)

   ```bash
   export TRON2_BRIDGER="$HOME/tron2-ros2-bridger"
   GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/limxdynamics/ros2-bridger.git "$TRON2_BRIDGER"
   GIT_LFS_SKIP_SMUDGE=1 git -C "$TRON2_BRIDGER" checkout --detach e0deafa7fff9629573fe3be98b4575a6523a2c3e
   git -C "$TRON2_BRIDGER" lfs pull --include='amd64/jazzy/**' --exclude=''
   export TRON2_BRIDGE_INSTALL="$TRON2_BRIDGER/amd64/jazzy/install"
   ```

2. On Ubuntu 24.04, stage the legacy LimX binary dependencies in a separate directory. The helper uses the signed Ubuntu Focal archive and extracts the packages without changing the system libraries.

   ```bash
   ./scripts/get_controller_deps.sh "$HOME/tron2-controller-runtime"
   export TRON2_EXTRA_LIB_DIR="$HOME/tron2-controller-runtime/usr/lib/x86_64-linux-gnu"
   ```

3. Launch the headless simulator, official controller and joystick mode switch. The launcher opens a shell inside the isolated namespace after the WebSocket handshake. Exit that shell to stop the run. Add `--gui` to view the Isaac scene. The launcher prints `TRON2_RUN_DIR`, an external temporary directory for process logs and SDK results.

   ```bash
   ./scripts/run_sim.sh
   ```

4. Run the SDK smoke suite from this repository's root in the isolated shell, using a Python environment with `Tron2_arm_tests/requirements.txt` installed. The SDK entry splits the upstream test's 14-value `ServoP` pose into the controller's two seven-value fields, `data.left_pos` and `data.right_pos`. Send those fields when using a separate WebSocket client. The launcher also accepts `-- CLIENT [ARG ...]` to run a client without the interactive shell.

   ```bash
   python3 -m tron2_isaac.sdk --output-dir "$TRON2_RUN_DIR/sdk"
   ```

5. To reproduce the 300 Hz servo check, start a fresh session and run the commands below from this repository's root. They add the upstream `H_SAFE` and `J_PREP` cases before the four servo targets to reach their required starting pose. The ServoJ and ServoP targets and accuracy limits come from the upstream suite.

   ```bash
   python3 - <<'PY'
   import os
   from pathlib import Path

   import yaml

   run = Path(os.environ['TRON2_RUN_DIR'])
   config = Path(os.environ['TRON2_UPSTREAM']) / 'Tron2_arm_tests/config'
   servo = yaml.safe_load((config / 'servo_cases.yaml').read_text())
   smoke = yaml.safe_load((config / 'test_cases.yaml').read_text())
   servo['cases'] = [case for case in smoke['cases'] if case['id'] in ('H_SAFE', 'J_PREP')]
   defaults = yaml.safe_load((config / 'default.yaml').read_text())
   defaults['safety']['stop_on_failure'] = False
   (run / 'servo-prepared.yaml').write_text(yaml.safe_dump(servo))
   (run / 'sdk-continue.yaml').write_text(yaml.safe_dump(defaults))
   PY
   python3 -m tron2_isaac.sdk \
     --cases "$TRON2_RUN_DIR/servo-prepared.yaml" \
     --config "$TRON2_RUN_DIR/sdk-continue.yaml" \
     --output-dir "$TRON2_RUN_DIR/sdk-servo"
   ```

## Verification and scope

On Ubuntu 24.04 with Isaac Sim 5.1.0 and an RTX 5080, the upstream default SDK suite passed preflight and all five motion cases through the SDK entry. The pinned MuJoCo reference also passed all five with the corrected ServoP fields. Both runs used the upstream targets, send rates and accuracy limits.

| SDK case | Isaac Sim | Upstream MuJoCo |
| --- | --- | --- |
| `H_SAFE` · MoveH | Pass | Pass |
| `J_PREP` · MoveJ | Pass | Pass |
| `P_REACHABLE` · MoveP | Pass · 12.9 mm position error | Pass · 12.7 mm position error |
| `SJ_TRACK` · ServoJ | Pass | Pass |
| `SP_TRACK` · ServoP | Pass · 25.1 mm position error | Pass · 20.3 mm position error |

The pose errors come from the SDK's simulated end-effector estimate. With the upstream client's packed `data.pos` field, both simulators previously reported a 198.6 mm `SP_TRACK` error. With split fields, both stayed within the 30 mm tolerance after the same 301 packets at 100 Hz.

Isaac Sim and the pinned MuJoCo reference each passed **6/6** cases in the prepared 300 Hz suite. That suite scores servo pose accuracy after return to the starting pose. The 100 Hz `SP_TRACK` case scores arrival at the commanded outbound target. The standalone PhysX smoke confirmed all 16 joints and tracked four commanded axes with a maximum steady-state position error of 0.0403 rad.

The shipped scene exercises arm and head control. The imported gripper geometry is rigid, so the scene has no actuated fingers or object manipulation. It has no tabletop. For any TRON2 tabletop extension, resolve the physical base origin from the model's authored transforms and keep `physical_base_z = table_top_z + 0.45` m.
