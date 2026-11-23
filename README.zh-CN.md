# TRON2 双臂控制的 Isaac Sim 版本

本仓库为 LimX DACH_TRON2A 双臂操作控制器提供 Isaac Sim 5.1 电机仿真，使用官方机器人模型和官方控制器的电机接口。[English](README.md)

## 目录

- [组件](#组件)
- [PhysX 快速开始](#physx-快速开始)
- [控制器联调](#控制器联调)
- [验证与范围](#验证与范围)
  - [长轨迹与 VLA 回放](#长轨迹与-vla-回放)
  - [同条件轨迹对照](#同条件轨迹对照)
  - [电机流时序](#电机流时序)

## 组件

| 组件 | 来源或接口 |
| --- | --- |
| 物理仿真 | Isaac Sim 5.1.0、PhysX、固定底座的 DACH_TRON2A。[NVIDIA Python API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/index.html) |
| 机器人模型 | 官方 `robot_grasper.urdf`，提交 `9939c22e69d27653ec0ba8a505859a2903dd1a71`。导入时在临时 URDF 中解析包内网格路径并固定夹爪机构关节。[机器人描述](https://github.com/limxdynamics/tron2-robot-description/tree/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A) |
| 控制器 | LimX 预编译 QP/PD 运行时，提交 `10af63c20e0dd2149168c6693453cc28f2c2dcbf`。它提供 `MoveJ`、`MoveH`、`MoveP`、`ServoJ`、`ServoP` 和状态查询 WebSocket API。[上游控制器](https://github.com/limxdynamics/tron2_manipulation_controller/tree/10af63c20e0dd2149168c6693453cc28f2c2dcbf) |
| 电机环路 | MROS `/motor/cmd` 与 `/motor/state`，顺序为左臂 7 轴、右臂 7 轴、头部 pitch/yaw。默认目标频率为 300 Hz。命令使用 rad、rad/s、N·m 和各轴 PD 增益。施加力矩按官方 MJCF 执行器上限裁剪。 |

Isaac 在每个物理子步施加 `tau + kp * (q_target - q) + kd * (dq_target - dq)`。默认电机频率为 300 Hz，每周期执行四个 PhysX 子步，积分间隔小于 1 ms。PhysX 提供关节位置、速度和力矩反馈。初始姿态采用上游 SDK 的 `DEFAULT_HOME_Q16`。官方模型与二进制在仓库外获取。

ServoP 运行时，Isaac 在每次电机更新时按机构当前姿态计算重力补偿，并补入控制器前馈全为零的手臂中启用位置控制的关节。LimX 的 `/move_cmd` ServoP 事件使补偿生效 100 ms，每个新采样点都会刷新该时间窗。需要维持位姿时，持续发送终点目标。[PhysX 机构重力 API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html#isaacsim.core.prims.Articulation.get_generalized_gravity_forces)

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

3. 无需启动控制器，检查固定版本的 URDF、MJCF 和导入后的 PhysX 机构。检查电机通信顺序、源模型有符号的关节轴、关节与电机力矩限位、转子惯量、网格路径、初始状态及力矩探测。JSON 也会报告根节点的世界坐标，供对照原版 MuJoCo 场景。

   ```bash
   "$ISAAC_PYTHON" -m tron2_isaac --upstream "$TRON2_UPSTREAM" --check-model
   ```

   通过时输出 `"model_check": "passed"`。原版 MJCF 将 `base_Link` 放在世界坐标 z = 1.21 m，Isaac 导入机构的 z = 0 m。SDK 位姿对照使用控制器底座坐标系。

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

5. 选择 `servo` 运行 300 Hz 伺服检查，选择 `move` 运行完整 Move 长轨迹，或选择 `vla_servoj` 回放带时间戳的 VLA 轨迹。每套用例均启动新的隔离会话。进入会话后，激活 SDK Python 环境，在本仓库根目录运行以下命令。`H_SAFE` 和 `J_PREP` 建立起始姿态。轨迹、时序和限位沿用固定版本 SDK。工作目录找不到 VLA 轨迹文件时，SDK 入口会在固定版本 SDK 目录下解析该相对路径。

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

6. 激活 SDK Python 环境后，从仓库根目录运行 ServoP 稳态检查。它以 100 Hz 发送上游 `SP_TRACK` 轨迹，续发终点 3 s，并按 0.1 mm 阈值检查 SDK 回读的末端位置误差。在 `--` 前加 `--no-servop-gravity-compensation`，即可使用原版 LimX 前馈运行同一检查。

   ```bash
   ./scripts/run_sim.sh -- python3 tests/check_servop_hold.py
   ```

## 验证与范围

在 Ubuntu 24.04、Isaac Sim 5.1.0 和 RTX 5080 上，通过本仓库的 SDK 入口运行上游默认套件，预检和 5 个运动用例全部通过。固定版本的 MuJoCo 基线采用修正后的 ServoP 字段，也通过了全部 5 个用例。两次运行均沿用上游目标、发送频率和精度阈值。

| SDK 用例 | Isaac Sim | 上游 MuJoCo |
| --- | --- | --- |
| `H_SAFE` · MoveH | 通过 | 通过 |
| `J_PREP` · MoveJ | 通过 | 通过 |
| `P_REACHABLE` · MoveP | 通过 · 位置误差 14.1 mm | 通过 · 位置误差 12.7 mm |
| `SJ_TRACK` · ServoJ | 通过 | 通过 |
| `SP_TRACK` · ServoP | 通过 · 到达误差 12.5 mm | 通过 · 到达误差 20.3 mm |

表中的位姿误差来自 SDK 的仿真末端估计。此前使用上游客户端的合并字段 `data.pos` 时，两种仿真器的 `SP_TRACK` 误差均为 198.6 mm。拆分字段后，两边都在以 100 Hz 发送同样的 301 帧后达到了 30 mm 的容差要求。

Isaac Sim 对照试验使用相同的 100 Hz 轨迹，并在终点续发 3 s。稳态位置误差取双臂中较大的一侧。

| ServoP 目标 | 原版 LimX 前馈 | 加入 PhysX 重力补偿 |
| --- | --- | --- |
| `SP_TRACK` | 12.56 mm | 0.0048 mm |
| 从准备姿态双臂各向前 25 mm | 9.59 mm | 0.0109 mm |

另一次独立运行的 0.1 mm 保持检查以 0.0137 mm 通过。

先进入准备姿态后，Isaac Sim 和固定版本的 MuJoCo 基线均通过 300 Hz 套件的 **6/6**。该套件在伺服回到起始位姿后评估精度，100 Hz 的 `SP_TRACK` 用例则在去程目标点评估到达误差。独立 PhysX 冒烟检查确认了全部 16 轴，并跟踪四个命令轴，最大稳态位置误差为 0.0403 rad。

当前场景用于双臂和头部控制。导入的夹爪为刚性几何体，场景不包含可驱动手指或物体操作。场景没有桌面。后续扩展 TRON2 桌面场景时，须按模型原始变换定位物理底座原点，并保持 `physical_base_z = table_top_z + 0.45` m。

Isaac GUI 已渲染机器人，并通过独立 PhysX 冒烟检查。在 GUI 联调会话中，3 s ServoP 用例通过，SDK 实测发送频率为 299.99 Hz。GUI ServoJ 用例在运动前停在 SDK 起始姿态校验。联调步骤中的准备动作可用于建立该起始姿态。

### 长轨迹与 VLA 回放

两个仿真器均使用固定版本控制器与 SDK，执行相同的运动请求。完整 Move 套件包括 MoveJ、MoveH 和 MoveP 各两个 65 s 用例。VLA 回放使用 3,138 帧源轨迹，在约 104.57 s 的源轨迹段发送 31,371 个 SDK 指令。两边均完成了全部计划发送，且均未触发关节硬限位裁剪。

| 结果 | 上游 MuJoCo | Isaac Sim |
| --- | --- | --- |
| Move 长轨迹 SDK 用例，含准备 | 8/8 | 8/8 |
| MoveP 长轨迹最差 SDK 位置误差 | 13.72 mm | 12.56 mm |
| VLA SDK 用例，含准备 | 3/3 | 3/3 |
| VLA SDK 指令总数，含预热、过渡及返回 | 32,119/32,119 | 32,120/32,120 |
| VLA SDK 发送频率 | 300.01 Hz | 300.01 Hz |
| VLA 最大 SDK 关节跟踪误差 | 0.000140 rad | 0.000535 rad |

### 同条件轨迹对照

对照脚本在两个仿真器上向固定版本的 LimX 控制器发送相同的 100 Hz ServoJ 和 ServoP 参考轨迹。独立 SDK 连接以 50 Hz 采样关节位置、速度、力矩和控制器报告的末端位姿。配对时要求观测属于同一时间槽，且本地查询中点相差不超过 10 ms。报告分别计算运动段跟踪误差和终点保持误差。末端估计使用控制器底座坐标系。

1. 激活 SDK Python 环境后，从仓库根目录采集 Isaac 轨迹。关闭附加补偿的选项使本次对照使用原版 LimX 前馈。

   ```bash
   ./scripts/run_sim.sh --no-servop-gravity-compensation -- \
     bash -c 'python3 tests/compare_dynamics.py capture isaac "$TRON2_RUN_DIR/isaac-track.json"'
   ```

2. 在固定版本控制器已进入开发模式的隔离 MuJoCo 会话中，将 `TRON2_UPSTREAM` 设为上游仓库路径，`TRON2_ISAAC_REPO` 设为本仓库路径，`TRON2_RUN_DIR` 设为仓库外的输出目录。采集参考轨迹，再离线对比。`MUJOCO_TRACK` 和 `ISAAC_TRACK` 指向采集得到的 JSON 文件。

   ```bash
   python3 "$TRON2_ISAAC_REPO/tests/compare_dynamics.py" capture mujoco "$TRON2_RUN_DIR/mujoco-track.json"
   python3 "$TRON2_ISAAC_REPO/tests/compare_dynamics.py" compare "$MUJOCO_TRACK" "$ISAAC_TRACK"
   ```

| 同条件对照结果 | MuJoCo | Isaac Sim，关闭 ServoP 补偿 |
| --- | --- | --- |
| ServoJ 运动 3 s 内的关节跟踪 P95 | 0.00154 rad | 0.00748 rad |
| ServoP 运动 3 s 内的位置跟踪 P95 | 13.01 mm | 16.49 mm |
| ServoP 保持 3 s 后的位置误差 | 12.54 mm | 12.54 mm |

### 电机流时序

在隔离仿真会话中运行只读时序观察器。它记录回调间隔和兼容的源时间戳，并输出各主题的 JSONL 摘要。Isaac 下使用 `--direct-ros2`，在主 MROS 到 ROS 2 桥之后观测 `/motor/cmd`，在 Isaac ROS 2 子进程的发布端观测 `/motor/state`。在原版 MuJoCo 的会话中用 `--engine mujoco` 运行并省略 `--direct-ros2`，通过独立的单向桥观测两个原生 MROS 主题。如果主桥使用了观察器默认的域 77，则把 `TRON2_TIMING_ROS_DOMAIN_ID` 设为未使用的域。

```bash
./scripts/run_sim.sh -- bash -c '
  scripts/capture_mros_timing.sh --engine isaac --direct-ros2 \
    --bridge-install "$TRON2_BRIDGE_INSTALL" \
    --output "$TRON2_RUN_DIR/motor-timing.jsonl" --seconds 30
'
```

时序测量包含回调调度和采集链路上的桥接开销。可用到达间隔估计缺失的周期，精确丢包计数需要消息序列号，而这些消息未提供序列号。通信仍连接但暂时没有新指令时，Isaac 沿用上一条指令。观察器使用 20 ms 阈值统计消息到达时的年龄。CPU CI 通过 `python -m unittest discover -s tests -p 'test_*.py' -v` 运行，无需 Isaac Sim 或 ROS。

分别在两个无界面会话中使用相同的准备版 SDK 伺服套件采集 20 s，两边均通过 6 个 SDK 用例。观测到的指令流均约为 300 Hz。该轮 Isaac 的电机状态发布频率为 85.15 Hz，低于 300 Hz 目标。ROS 2 子进程的发布消息时间戳也显示出这一频率。

| 观测流 | MuJoCo 到达频率 | Isaac 到达频率 | MuJoCo 间隔 P95 | Isaac 间隔 P95 |
| --- | ---: | ---: | ---: | ---: |
| `/motor/cmd` | 299.91 Hz | 300.04 Hz | 6.95 ms | 6.95 ms |
| `/motor/state` | 300.01 Hz | 85.15 Hz | 4.06 ms | 16.63 ms |

两次采集位于桥接链路的不同位置，数据反映各采集点的时序。控制器端的接收时序仍未测量。同一主机的独立无界面性能测试中，每个 PhysX 子步平均耗时 2.71 ms。电机循环每周期执行四个子步，该配置下的反馈频率受此限制。
