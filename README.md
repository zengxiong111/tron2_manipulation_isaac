# TRON2 Manipulation in Isaac Sim

This repository provides an Isaac Sim 5.1 motor simulator for the LimX DACH_TRON2A dual-arm manipulation controller. It uses the official robot description and the official controller's motor interface. [简体中文](README.zh-CN.md)

## Contents

- [Comparison with MuJoCo](#comparison-with-mujoco)
- [Components](#components)
- [PhysX quick start](#physx-quick-start)
- [Controller integration](#controller-integration)
- [Verification and scope](#verification-and-scope)
  - [Long motion and VLA playback](#long-motion-and-vla-playback)
  - [Matched motion traces](#matched-motion-traces)
  - [Motor stream timing](#motor-stream-timing)

## Comparison with MuJoCo

Both versions run the same DACH_TRON2A controller and pass the default SDK, prepared servo, long-motion and VLA suites in headless simulation. Isaac motor state reached 296.98–297.91 Hz against the 300 Hz target. [LimX MuJoCo runtime](https://github.com/limxdynamics/tron2_manipulation_controller/blob/10af63c20e0dd2149168c6693453cc28f2c2dcbf/Tron2_simulation_mujoco/README.md)

| Priority | Remaining difference | What to add or verify |
| --- | --- | --- |
| First | GUI feedback | Isaac GUI passed the prepared suite 6/6 with state feedback at 62.59 Hz. Separate rendering from motor updates and measure both GUIs with hardware acceleration. |
| Next | Command timeout | Both hold the last command by default during a connected publication gap. MuJoCo also exposes `--cmd-timeout`. Add the option and test interruption and recovery. |
| Next | Model selection | MuJoCo's `DACH_TRON2B` preset passed its 16-axis model check. Isaac currently supports `DACH_TRON2A`. Add the B model mapping and validate it with a matching controller configuration. |
| As needed | Launch options | MuJoCo offers custom model paths, render rate, real-time factor and motor topics. Isaac exposes `--rate` through its Python entry, with fewer options in `run_sim.sh`. |

On the isolated virtual display, MuJoCo used Mesa llvmpipe software rendering. That run measured 2.25 Hz state feedback and passed 2/6 cases. A hardware-accelerated MuJoCo GUI timing baseline is still pending. Isaac controller integration requires a ROS 2 Jazzy bridge, while the MuJoCo simulator speaks MROS directly.

Both shipped simulators cover 16 arm and head axes. The upstream box and tabletop are added for rendering the replay. For a future tabletop scene, resolve the physical base origin from the model transforms and keep `physical_base_z = table_top_z + 0.45` m. [Upstream demo scope](https://github.com/limxdynamics/tron2_manipulation_controller/blob/10af63c20e0dd2149168c6693453cc28f2c2dcbf/README.md#demo)

## Components

| Component | Source or contract |
| --- | --- |
| Physics | Isaac Sim 5.1.0, PhysX, fixed-base DACH_TRON2A. [NVIDIA Python API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/index.html) |
| Robot model | Official `robot_grasper.urdf` at `9939c22e69d27653ec0ba8a505859a2903dd1a71`. The importer resolves the package mesh paths and fixes the gripper mechanism joints in a temporary URDF. [Robot description](https://github.com/limxdynamics/tron2-robot-description/tree/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A) |
| Controller | LimX prebuilt QP/PD runtime at `10af63c20e0dd2149168c6693453cc28f2c2dcbf`. It owns the `MoveJ`, `MoveH`, `MoveP`, `ServoJ`, `ServoP` and state-query WebSocket APIs. [Upstream controller](https://github.com/limxdynamics/tron2_manipulation_controller/tree/10af63c20e0dd2149168c6693453cc28f2c2dcbf) |
| Motor loop | `/motor/cmd` and `/motor/state` in MROS, in left arm × 7, right arm × 7, head pitch/yaw order. The default target rate is 300 Hz. Commands use radians, rad/s, N·m and per-axis PD gains. Applied effort is clipped to the official MJCF actuator limits. |

Isaac applies `tau + kp * (q_target - q) + kd * (dq_target - dq)` at each physics substep. At the default 300 Hz motor rate, four PhysX substeps keep the integration interval below 1 ms. PhysX reports joint position, velocity and effort. The initial pose is the upstream SDK's `DEFAULT_HOME_Q16` pose. Vendor assets and binaries are external dependencies.

Headless runs read motor state directly from PhysX and skip per-substep writeback of transforms and velocities to USD. GUI runs publish transforms for the viewport. [Omni Physics 107.3 performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/physics-performance.html#render-transforms-in-runtime-fabric)

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

3. Activate a separate Python environment with pip for the SDK, install its dependencies, and launch the simulator, controller and joystick mode switch. After the WebSocket handshake the launcher opens an isolated shell. Exit that shell to stop the run. Add `--gui` to view the scene. Logs and SDK results can go in the external `TRON2_RUN_DIR` printed at startup.

   ```bash
   python3 -m pip install -r "$TRON2_UPSTREAM/Tron2_arm_tests/requirements.txt"
   ./scripts/run_sim.sh
   ```

4. Run the SDK smoke suite from this repository's root in the isolated shell. The SDK entry splits the ServoP target into `data.left_pos` and `data.right_pos`, seven values per arm. Use those fields in a custom client. The launcher also accepts `-- CLIENT [ARG ...]`.

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

The suites ran on Ubuntu 24.04 with Isaac Sim 5.1.0 and an RTX 5080. Both headless simulators used the same upstream motion targets, send rates and tolerances.

| SDK result | MuJoCo | Isaac Sim |
| --- | ---: | ---: |
| Default motion cases | 5/5 | 5/5 |
| Prepared 300 Hz servo cases | 6/6 | 6/6 |
| `P_REACHABLE` MoveP arrival error | 12.7 mm | 13.2 mm |
| `SP_TRACK` ServoP arrival error | 20.3 mm | 8.22 mm |

The SDK reports simulated end-effector positions. With the packed `data.pos` field, `SP_TRACK` previously reported 198.6 mm on both engines. Splitting the fields brought both within 30 mm after 301 sends at 100 Hz. The prepared 300 Hz suite scores the return to the start pose. `SP_TRACK` scores the outbound target.

In an Isaac A/B test, holding the target for 3 s with PhysX gravity compensation reduced `SP_TRACK` error from 12.56 to 0.0048 mm. A second target, 25 mm forward per arm, improved from 9.59 to 0.0109 mm. These errors use the larger of the two arm measurements. The latest standalone hold check measured 0.0224 mm against a 0.1 mm limit. The PhysX smoke check confirmed all 16 joints and measured a maximum steady-state error of 0.0403 rad across four commanded axes.

### Long motion and VLA playback

Both simulators used the pinned controller and SDK with the same motion requests. The full Move suite includes two cases each for MoveJ, MoveH and MoveP, with a configured active duration of 65 s per case. The SDK finishes the final segment, so runs with 2 s segments reach 66 s. The VLA replay uses 3,138 source frames and 31,371 source-phase SDK sends over 104.57 s. Both engines completed every planned send and had zero hard-limit clamps.

| Result | Upstream MuJoCo | Isaac Sim |
| --- | --- | --- |
| Long Move SDK cases, including preparation | 8/8 | 8/8 |
| Worst SDK MoveP position error across the long cases | 13.72 mm | 13.62 mm |
| VLA SDK cases, including preparation | 3/3 | 3/3 |
| VLA total SDK sends, including warmup, transition and return | 32,119/32,119 | 32,119/32,119 |
| VLA SDK send rate | 300.01 Hz | 300.01 Hz |
| VLA maximum SDK joint tracking error | 0.000140 rad | 0.000140 rad |

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
| ServoJ joint tracking P95 during the 3 s ramp | 0.00154 rad | 0.00169 rad |
| ServoP position tracking P95 during the 3 s ramp | 13.01 mm | 13.01 mm |
| ServoP position error after the 3 s hold | 12.54 mm | 12.53 mm |

Across 300 time-matched ServoP samples spanning motion and hold, the P95 difference between the two SDK end-effector positions was 0.157 mm.

### Motor stream timing

Run the read-only timing observer inside the isolated simulation session. It records callback intervals and compatible source timestamps and writes a JSONL summary per topic. With `--direct-ros2` in Isaac, it observes `/motor/cmd` after the primary MROS-to-ROS 2 bridge and `/motor/state` at the Isaac sidecar publisher. Run it in the native MuJoCo session with `--engine mujoco` and without `--direct-ros2` to observe both native MROS topics through a separate one-way bridge. Set `TRON2_TIMING_ROS_DOMAIN_ID` to an unused domain if the primary bridge uses the observer's default domain 77.

```bash
./scripts/run_sim.sh -- bash -c '
  scripts/capture_mros_timing.sh --engine isaac --direct-ros2 \
    --bridge-install "$TRON2_BRIDGE_INSTALL" \
    --output "$TRON2_RUN_DIR/motor-timing.jsonl" --seconds 30
'
```

The timings include bridge and callback scheduling. The observer reports gaps and counts messages older than 20 ms. The motor messages lack sequence numbers for counting exact drops. CPU CI runs `python -m unittest discover -s tests -p 'test_*.py' -v` without Isaac Sim or ROS.

In 20 s headless captures of the same prepared SDK servo suite, MuJoCo and Isaac passed all six SDK cases. Isaac state delivery rose from 85.15 Hz to 296.98–297.91 Hz after skipping per-substep USD writeback. The 50.00 s Isaac session containing the second capture counted 14,923 state frames in the motor loop, or 298.45 Hz. The default target is 300 Hz.

| Observed stream | Metric | MuJoCo | Isaac with USD writeback | Isaac headless, second run |
| --- | --- | ---: | ---: | ---: |
| `/motor/cmd` | Arrival rate | 299.91 Hz | 300.04 Hz | 300.36 Hz |
| `/motor/cmd` | Interarrival P95 | 6.95 ms | 6.95 ms | 6.95 ms |
| `/motor/state` | Arrival rate | 300.01 Hz | 85.15 Hz | 297.91 Hz |
| `/motor/state` | Interarrival P95 | 4.06 ms | 16.63 ms | 4.71 ms |

A standalone headless A/B ran 900 unpaced motor cycles per configuration, each with four PhysX substeps, using 20 Kit threads and eight PhysX workers in both runs. Turning off USD writeback increased throughput from 109.67 to 380.20 cycles/s. Recorded joint position, velocity and effort samples matched exactly.

The captures measure cadence at the bridge points described above. Controller-side receive timing remains unmeasured.
