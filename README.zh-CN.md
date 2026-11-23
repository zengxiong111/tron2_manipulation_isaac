# TRON2 双臂控制的 Isaac Sim 版本

本仓库为 LimX DACH_TRON2A 双臂操作控制器提供 Isaac Sim 5.1 电机仿真，使用官方机器人模型和官方控制器的电机接口。[English](README.md)

## 目录

- [组件](#组件)
- [PhysX 快速开始](#physx-快速开始)
- [控制器联调](#控制器联调)
- [验证与范围](#验证与范围)

## 组件

| 组件 | 来源或接口 |
| --- | --- |
| 物理仿真 | Isaac Sim 5.1.0、PhysX、固定底座的 DACH_TRON2A。[NVIDIA Python API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/index.html) |
| 机器人模型 | 官方 `robot_grasper.urdf`，提交 `9939c22e69d27653ec0ba8a505859a2903dd1a71`。导入时在临时 URDF 中解析包内网格路径并固定夹爪机构关节。[机器人描述](https://github.com/limxdynamics/tron2-robot-description/tree/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A) |
| 控制器 | LimX 预编译 QP/PD 运行时，提交 `10af63c20e0dd2149168c6693453cc28f2c2dcbf`。它提供 `MoveJ`、`MoveH`、`MoveP`、`ServoJ`、`ServoP` 和状态查询 WebSocket API。[上游控制器](https://github.com/limxdynamics/tron2_manipulation_controller/tree/10af63c20e0dd2149168c6693453cc28f2c2dcbf) |
| 电机环路 | MROS `/motor/cmd` 与 `/motor/state`，顺序为左臂 7 轴、右臂 7 轴、头部 pitch/yaw。默认目标频率为 300 Hz。命令使用 rad、rad/s、N·m 和各轴 PD 增益。施加力矩按官方 MJCF 执行器上限裁剪。 |

Isaac 在每个物理子步施加 `tau + kp * (q_target - q) + kd * (dq_target - dq)`。默认电机频率为 300 Hz，每周期执行四个 PhysX 子步，积分间隔小于 1 ms。PhysX 提供关节位置、速度和力矩反馈。初始姿态采用上游 SDK 的 `DEFAULT_HOME_Q16`。官方模型与二进制在仓库外获取。

## PhysX 快速开始

在安装 Isaac Sim 的主机上，从本仓库根目录执行。`ISAAC_PYTHON` 指向 Isaac Sim 5.1 的 Python 启动器，例如独立安装的 `python.sh` 或 pip 安装环境的 `bin/python`。

1. 将固定版本的控制器和模型下载到仓库外。

   ```bash
   export TRON2_UPSTREAM="$HOME/tron2-controller-upstream"
   ./scripts/get_upstream.sh "$TRON2_UPSTREAM"
   ```

2. 无界面验证关节名称映射和电机力矩。有图形显示时，附加 `--gui` 可打开 Isaac 窗口。

   ```bash
   "$ISAAC_PYTHON" -m tron2_isaac --upstream "$TRON2_UPSTREAM" --smoke
   ```

   通过时会输出 `"physics_smoke": "passed"`、关节实测位置和跟踪误差。

## 控制器联调

联调入口使用 Ubuntu 24.04 amd64、ROS 2 Jazzy、Git LFS 和 Linux 用户及网络命名空间。以下命令从本仓库根目录执行，保留前面设置的 `ISAAC_PYTHON` 和 `TRON2_UPSTREAM`。

1. 获取 LimX MROS–ROS 2 桥的 Jazzy 构建产物并设置安装路径。Isaac 通过 Unix socket 将状态送到系统 Python 的 ROS 2 子进程，LimX 桥将 `/motor/state` 转发到 MROS，将 `/motor/cmd` 转发到 ROS 2。[LimX ros2-bridger](https://github.com/limxdynamics/ros2-bridger/tree/e0deafa7fff9629573fe3be98b4575a6523a2c3e)

   ```bash
   export TRON2_BRIDGER="$HOME/tron2-ros2-bridger"
   GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/limxdynamics/ros2-bridger.git "$TRON2_BRIDGER"
   GIT_LFS_SKIP_SMUDGE=1 git -C "$TRON2_BRIDGER" checkout --detach e0deafa7fff9629573fe3be98b4575a6523a2c3e
   git -C "$TRON2_BRIDGER" lfs pull --include='amd64/jazzy/**' --exclude=''
   export TRON2_BRIDGE_INSTALL="$TRON2_BRIDGER/amd64/jazzy/install"
   ```

2. 在 Ubuntu 24.04 上将 LimX 二进制所需的旧版依赖解包到独立目录。脚本使用签名的 Ubuntu Focal 软件源，不更改系统库。

   ```bash
   ./scripts/get_controller_deps.sh "$HOME/tron2-controller-runtime"
   export TRON2_EXTRA_LIB_DIR="$HOME/tron2-controller-runtime/usr/lib/x86_64-linux-gnu"
   ```

3. 启动无界面仿真、官方控制器和模拟手柄模式切换。WebSocket 握手通过后，启动脚本会在隔离命名空间内打开一个 shell。退出该 shell 即停止本次运行。附加 `--gui` 可查看 Isaac 场景。启动脚本会打印仓库外的临时目录 `TRON2_RUN_DIR`，用于保存进程日志和 SDK 结果。

   ```bash
   ./scripts/run_sim.sh
   ```

4. 在隔离 shell 内，从本仓库根目录用已安装 `Tron2_arm_tests/requirements.txt` 的 Python 环境运行 SDK 冒烟用例。SDK 入口会将上游用例的 14 值 `ServoP` 位姿拆为控制器接收的两个 7 值字段 `data.left_pos` 和 `data.right_pos`。使用其他 WebSocket 客户端时直接发送这两个字段。启动脚本也接受 `-- CLIENT [ARG ...]`，可跳过交互 shell 直接运行客户端。

   ```bash
   python3 -m tron2_isaac.sdk --output-dir "$TRON2_RUN_DIR/sdk"
   ```

5. 复现 300 Hz 伺服检查时，启动一次新会话，并在本仓库根目录运行以下命令。在四个伺服目标前加入上游 `H_SAFE` 和 `J_PREP` 用例，使机器人达到所需的起始姿态。ServoJ 和 ServoP 的目标与精度阈值沿用上游套件。

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

## 验证与范围

在 Ubuntu 24.04、Isaac Sim 5.1.0 和 RTX 5080 上，通过本仓库的 SDK 入口运行上游默认套件，预检和 5 个运动用例全部通过。固定版本的 MuJoCo 基线采用修正后的 ServoP 字段，也通过了全部 5 个用例。两次运行均沿用上游目标、发送频率和精度阈值。

| SDK 用例 | Isaac Sim | 上游 MuJoCo |
| --- | --- | --- |
| `H_SAFE` · MoveH | 通过 | 通过 |
| `J_PREP` · MoveJ | 通过 | 通过 |
| `P_REACHABLE` · MoveP | 通过 · 位置误差 12.9 mm | 通过 · 位置误差 12.7 mm |
| `SJ_TRACK` · ServoJ | 通过 | 通过 |
| `SP_TRACK` · ServoP | 通过 · 位置误差 25.1 mm | 通过 · 位置误差 20.3 mm |

表中的位姿误差来自 SDK 的仿真末端估计。此前使用上游客户端的合并字段 `data.pos` 时，两种仿真器的 `SP_TRACK` 误差均为 198.6 mm。拆分字段后，两边都在以 100 Hz 发送同样的 301 帧后达到了 30 mm 的容差要求。

先进入准备姿态后，Isaac Sim 和固定版本的 MuJoCo 基线均通过 300 Hz 套件的 **6/6**。该套件在伺服回到起始位姿后评估精度，100 Hz 的 `SP_TRACK` 用例则在去程目标点评估到达误差。独立 PhysX 冒烟检查确认了全部 16 轴，并跟踪四个命令轴，最大稳态位置误差为 0.0403 rad。

当前场景用于双臂和头部控制。导入的夹爪为刚性几何体，场景不包含可驱动手指或物体操作。场景没有桌面。后续扩展 TRON2 桌面场景时，须按模型原始变换定位物理底座原点，并保持 `physical_base_z = table_top_z + 0.45` m。
