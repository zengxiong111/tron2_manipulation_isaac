# TRON2 Manipulation in Isaac Sim

This repository provides an Isaac Sim 5.1 motor simulator for the LimX DACH_TRON2A dual-arm manipulation controller. It uses the official robot description and the official controller's motor interface. [简体中文](README.zh-CN.md)

## Contents

- [Components](#components)
- [PhysX quick start](#physx-quick-start)
- [Controller integration](#controller-integration)
- [Verification and scope](#verification-and-scope)
  - [Long motion and VLA playback](#long-motion-and-vla-playback)
  - [Matched motion traces](#matched-motion-traces)
  - [Motor stream timing](#motor-stream-timing)

## Components

| Component | Source or contract |
| --- | --- |
| Physics | Isaac Sim 5.1.0, PhysX, fixed-base DACH_TRON2A. [NVIDIA Python API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/index.html) |
| Robot model | Official `robot_grasper.urdf` at `9939c22e69d27653ec0ba8a505859a2903dd1a71`. The importer resolves the package mesh paths and fixes the gripper mechanism joints in a temporary URDF. [Robot description](https://github.com/limxdynamics/tron2-robot-description/tree/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A) |
| Controller | LimX prebuilt QP/PD runtime at `10af63c20e0dd2149168c6693453cc28f2c2dcbf`. It owns the `MoveJ`, `MoveH`, `MoveP`, `ServoJ`, `ServoP` and state-query WebSocket APIs. [Upstream controller](https://github.com/limxdynamics/tron2_manipulation_controller/tree/10af63c20e0dd2149168c6693453cc28f2c2dcbf) |
| Motor loop | `/motor/cmd` and `/motor/state` in MROS, in left arm × 7, right arm × 7, head pitch/yaw order. The default target rate is 300 Hz. Commands use radians, rad/s, N·m and per-axis PD gains. Applied effort is clipped to the official MJCF actuator limits. |

Isaac applies `tau + kp * (q_target - q) + kd * (dq_target - dq)` at each physics substep. At the default 300 Hz motor rate, four PhysX substeps keep the integration interval below 1 ms. PhysX reports joint position, velocity and effort. The initial pose is the upstream SDK's `DEFAULT_HOME_Q16` pose. Vendor assets and binaries are external dependencies.

During ServoP, Isaac computes gravity compensation from the articulation's current pose at each motor update. It adds the result to the powered joints of each arm whose controller feedforward vector is zero. The LimX `/move_cmd` ServoP event enables compensation for 100 ms, and each new ServoP sample refreshes that window. Keep sending the final target while holding a pose. [PhysX articulation gravity API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html#isaacsim.core.prims.Articulation.get_generalized_gravity_forces)

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

3. Check the pinned URDF and MJCF metadata and the imported PhysX articulation without starting the controller. Compare wire order, signed source axes, joint limits, motor effort limits, rotor inertia, mesh paths, the home state and an effort probe. The JSON also reports the root world positions for comparison with the native MuJoCo scene.

   ```bash
   "$ISAAC_PYTHON" -m tron2_isaac --upstream "$TRON2_UPSTREAM" --check-model
   ```

   A passing check prints `"model_check": "passed"`. The native MJCF authors `base_Link` at world z = 1.21 m and Isaac places the imported articulation at z = 0 m. SDK pose comparisons use the controller's base frame.

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

5. Run a prepared suite by choosing `servo` for the 300 Hz servo check, `move` for the full long-motion suite or `vla_servoj` for the timestamped VLA playback. Start a fresh isolated session for each suite. In that session, run the commands below from this repository's root with the SDK Python environment active. `H_SAFE` and `J_PREP` establish the starting pose. The pinned SDK supplies the trajectories, timing and limits. The SDK entry resolves the VLA fixture relative to the pinned SDK checkout when it cannot find it in the working directory.

   ```bash
   suite=servo
   python3 - "$suite" <<'PY'
   import os
   from pathlib import Path
   import sys

   import yaml

   run = Path(os.environ['TRON2_RUN_DIR'])
   config = Path(os.environ['TRON2_UPSTREAM']) / 'Tron2_arm_tests/config'
   suite = sys.argv[1]
   document = yaml.safe_load((config / f'{suite}_cases.yaml').read_text())
   smoke = yaml.safe_load((config / 'test_cases.yaml').read_text())
   document['cases'] = [case for case in smoke['cases'] if case['id'] in ('H_SAFE', 'J_PREP')]
   defaults = yaml.safe_load((config / 'default.yaml').read_text())
   defaults['safety']['stop_on_failure'] = False
   (run / f'{suite}-prepared.yaml').write_text(yaml.safe_dump(document))
   (run / 'sdk-continue.yaml').write_text(yaml.safe_dump(defaults))
   PY
   python3 -m tron2_isaac.sdk \
     --cases "$TRON2_RUN_DIR/$suite-prepared.yaml" \
     --config "$TRON2_RUN_DIR/sdk-continue.yaml" \
     --output-dir "$TRON2_RUN_DIR/sdk-$suite"
   ```

6. From the repository root, run the ServoP steady-state check with the SDK Python environment active. It streams the upstream `SP_TRACK` trajectory at 100 Hz, keeps sending the final target for 3 s, and checks the final SDK position error against 0.1 mm. Add `--no-servop-gravity-compensation` before `--` to run the same check with the LimX feedforward alone.

   ```bash
   ./scripts/run_sim.sh -- python3 tests/check_servop_hold.py
   ```

## Verification and scope

On Ubuntu 24.04 with Isaac Sim 5.1.0 and an RTX 5080, the upstream default SDK suite passed preflight and all five motion cases through the SDK entry. The pinned MuJoCo reference also passed all five with the corrected ServoP fields. Both runs used the upstream targets, send rates and accuracy limits.

| SDK case | Isaac Sim | Upstream MuJoCo |
| --- | --- | --- |
| `H_SAFE` · MoveH | Pass | Pass |
| `J_PREP` · MoveJ | Pass | Pass |
| `P_REACHABLE` · MoveP | Pass · 14.1 mm position error | Pass · 12.7 mm position error |
| `SJ_TRACK` · ServoJ | Pass | Pass |
| `SP_TRACK` · ServoP | Pass · 12.5 mm arrival error | Pass · 20.3 mm arrival error |

The pose errors come from the SDK's simulated end-effector estimate. With the upstream client's packed `data.pos` field, both simulators previously reported a 198.6 mm `SP_TRACK` error. With split fields, both stayed within the 30 mm tolerance after the same 301 packets at 100 Hz.

In the Isaac Sim A/B run, the same 100 Hz trajectories held their final targets for 3 s. Reported steady-state position error is the larger of the two arm errors.

| ServoP target | LimX feedforward alone | With PhysX gravity compensation |
| --- | --- | --- |
| `SP_TRACK` | 12.56 mm | 0.0048 mm |
| Both arms 25 mm forward from the prepared pose | 9.59 mm | 0.0109 mm |

The standalone 0.1 mm hold check ended at 0.0137 mm in a separate run.

Isaac Sim and the pinned MuJoCo reference each passed **6/6** cases in the prepared 300 Hz suite. That suite scores servo pose accuracy after return to the starting pose. The 100 Hz `SP_TRACK` case scores arrival at the commanded outbound target. The standalone PhysX smoke confirmed all 16 joints and tracked four commanded axes with a maximum steady-state position error of 0.0403 rad.

The shipped scene exercises arm and head control. The imported gripper geometry is rigid, so the scene has no actuated fingers or object manipulation. It has no tabletop. For any TRON2 tabletop extension, resolve the physical base origin from the model's authored transforms and keep `physical_base_z = table_top_z + 0.45` m.

The Isaac GUI rendered the robot and passed the standalone PhysX smoke check. In an integrated GUI session, the 3 s ServoP case passed with a measured SDK send rate of 299.99 Hz. The GUI ServoJ case stopped at SDK start-pose validation before motion. Use the preparation cases in the integration steps to establish the start pose.

### Long motion and VLA playback

Both simulators used the pinned controller and SDK with the same motion requests. The full Move suite includes two 65 s cases each for MoveJ, MoveH and MoveP. The VLA replay uses 3,138 source frames and 31,371 source-phase SDK sends over 104.57 s. Both engines completed every planned send and had zero hard-limit clamps.

| Result | Upstream MuJoCo | Isaac Sim |
| --- | --- | --- |
| Long Move SDK cases, including preparation | 8/8 | 8/8 |
| Worst SDK MoveP position error across the long cases | 13.72 mm | 12.56 mm |
| VLA SDK cases, including preparation | 3/3 | 3/3 |
| VLA total SDK sends, including warmup, transition and return | 32,119/32,119 | 32,120/32,120 |
| VLA SDK send rate | 300.01 Hz | 300.01 Hz |
| VLA maximum SDK joint tracking error | 0.000140 rad | 0.000535 rad |

### Matched motion traces

The comparison script sends identical 100 Hz ServoJ and ServoP references to the pinned LimX controller in each simulator. It samples joint position, velocity, effort and controller-reported end-effector pose through a separate SDK connection at 50 Hz. Comparison pairs observations in the same time slot whose local query midpoints are within 10 ms. Segment tracking and endpoint hold error are reported separately. The end-effector estimate uses the controller's base frame.

1. With the SDK Python environment active, capture Isaac from the repository root. The opt-out flag selects the LimX feedforward used for the paired baseline.

   ```bash
   ./scripts/run_sim.sh --no-servop-gravity-compensation -- \
     bash -c 'python3 tests/compare_dynamics.py capture isaac "$TRON2_RUN_DIR/isaac-track.json"'
   ```

2. In the isolated MuJoCo session with the pinned controller in developer mode, set `TRON2_UPSTREAM` to that checkout, `TRON2_ISAAC_REPO` to this repository, and `TRON2_RUN_DIR` to an external output directory. Capture the reference and compare the files offline. Set `MUJOCO_TRACK` and `ISAAC_TRACK` to the captured JSON files.

   ```bash
   python3 "$TRON2_ISAAC_REPO/tests/compare_dynamics.py" capture mujoco "$TRON2_RUN_DIR/mujoco-track.json"
   python3 "$TRON2_ISAAC_REPO/tests/compare_dynamics.py" compare "$MUJOCO_TRACK" "$ISAAC_TRACK"
   ```

| Paired result | MuJoCo | Isaac Sim, ServoP compensation disabled |
| --- | --- | --- |
| ServoJ joint tracking P95 during the 3 s ramp | 0.00154 rad | 0.00748 rad |
| ServoP position tracking P95 during the 3 s ramp | 13.01 mm | 16.49 mm |
| ServoP position error after the 3 s hold | 12.54 mm | 12.54 mm |

### Motor stream timing

Run the read-only timing observer inside the isolated simulation session. It records callback intervals and compatible source timestamps and writes a JSONL summary per topic. With `--direct-ros2` in Isaac, it observes `/motor/cmd` after the primary MROS-to-ROS 2 bridge and `/motor/state` at the Isaac sidecar publisher. Run it in the native MuJoCo session with `--engine mujoco` and without `--direct-ros2` to observe both native MROS topics through a separate one-way bridge. Set `TRON2_TIMING_ROS_DOMAIN_ID` to an unused domain if the primary bridge uses the observer's default domain 77.

```bash
./scripts/run_sim.sh -- bash -c '
  scripts/capture_mros_timing.sh --engine isaac --direct-ros2 \
    --bridge-install "$TRON2_BRIDGE_INSTALL" \
    --output "$TRON2_RUN_DIR/motor-timing.jsonl" --seconds 30
'
```

The timings include callback scheduling and any bridge in the capture path. Arrival gaps can estimate missed periods. Exact drop counts require a sequence number, which these messages lack. Isaac holds the last command during a publication gap while the transport stays connected. The observer uses a 20 ms threshold for reporting message age. The CPU CI workflow runs `python -m unittest discover -s tests -p 'test_*.py' -v` without Isaac Sim or ROS.

In separate 20 s headless captures with the same prepared SDK servo suite, both engines passed all six SDK cases. The observed command stream ran at about 300 Hz on both engines. Isaac published motor state at 85.15 Hz in this run, below its 300 Hz target. The sidecar publication timestamps showed the same rate.

| Observed stream | MuJoCo arrival rate | Isaac arrival rate | MuJoCo interval P95 | Isaac interval P95 |
| --- | ---: | ---: | ---: | ---: |
| `/motor/cmd` | 299.91 Hz | 300.04 Hz | 6.95 ms | 6.95 ms |
| `/motor/state` | 300.01 Hz | 85.15 Hz | 4.06 ms | 16.63 ms |

The captures observe different points in the bridge paths and give the cadence at those points. Controller-side receive timing remains unmeasured. A standalone headless profile on the same host found a mean of 2.71 ms per PhysX substep. The motor loop performs four substeps per cycle, which limits feedback cadence under this configuration.
