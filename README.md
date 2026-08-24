# robox_simulation

本仓库包含 Unitree H2 的 ONNX 策略和基于官方
[`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python)
实现的真机部署脚本。

> **安全警告**：这是直接发布 H2 `LowCmd` 的低层关节控制程序。第一次运行
> `passive`、`ready` 和 `policy` 时必须将机器人可靠悬吊、清空运动范围，并安排
> 一名操作员手持物理急停。不要跳过下面的分阶段检查，也不要第一次就在地面运行。

## 文件

- `deploy/h2/h2_policy_deploy_real.py`：真机部署程序，无 Isaac Sim/Isaac Lab 依赖。
- `deploy/h2/policy.onnx`：在 Isaac Sim 中验证过的策略，来自训练目录
  `2026-08-14_08-46-32_resume_to_20000`。

模型 SHA-256：

```text
6912db1b08e4309bdc6978b80309d35f02f4635e618b097266cbb84ac3249094
```

模型接口为：

- 输入 `obs`：`float32 [1, 400]`，由连续 4 帧、每帧 100 维的观测组成。
- 单帧观测：机身角速度 3 + 投影重力 3 + 速度命令 3 + 31 个关节位置偏差
  + 31 个关节速度 + 上一帧动作 29。
- 输出 `actions`：`float32 [1, 29]`，对应除头部外 29 个关节的归一化位置动作。

真机上的机身角速度和姿态四元数来自 pelvis IMU；关节位置、速度来自
`LowState`。策略本身不需要测量机身线速度，`vx/vy/wz` 是给策略的期望速度命令，
不是里程计测量值。

## 1. 准备真机电脑

建议先在 H2 的 PC2 上克隆本仓库：

```bash
git clone https://github.com/Michael0725/robox_simulation.git
cd robox_simulation
```

安装 Python 依赖：

```bash
python3 -m pip install numpy onnxruntime
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git ../unitree_sdk2_python
cd ../unitree_sdk2_python
python3 -m pip install -e .
cd ../robox_simulation
```

官方 SDK 要求 Python 3.8 或更高版本和 CycloneDDS 0.10.2。如果安装 SDK 时提示
找不到 CycloneDDS，请按 `unitree_sdk2_python` README 的源码安装说明设置
`CYCLONEDDS_HOME` 后重新执行 `pip install -e .`。

确认依赖和模型可被读取：

```bash
python3 -c "import numpy, onnxruntime, unitree_sdk2py; print('dependencies OK')"
sha256sum deploy/h2/policy.onnx
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode self-test \
  --model deploy/h2/policy.onnx
```

自检最后应输出 `SELF_TEST_OK`。

## 2. 连接 H2 并确定网卡

按照 Unitree 开发文档配置电脑与机器人的有线网络，然后查找连接 H2 的网卡名：

```bash
ip -br address
```

下面示例使用 `eth0`。如果实际名称是 `enp2s0` 等，请替换所有命令中的
`--interface eth0`。先确认官方 SDK 的 H2/low-level 状态读取示例能持续收到数据。

低层控制前，按 Unitree 的操作流程关闭高层运动服务 `sport_mode`，避免两套控制器
同时发送命令。脚本也会尝试通过 `MotionSwitcherClient` 释放当前运动模式，但不能以此
替代现场确认。

## 3. 分阶段悬吊测试

### 3.1 只读观测

`observe` 不发送任何电机命令，可以先确认 DDS、IMU、关节映射和遥控器数据：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode observe \
  --interface eth0 \
  --duration 10
```

检查输出中无电机错误，关节角和 IMU 数据合理，并确认状态没有超时。

### 3.2 被动阻尼

将机器人可靠悬吊并准备物理急停后，才执行：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode passive \
  --interface eth0 \
  --duration 10 \
  --enable-low-level \
  --confirm-suspended
```

确认 31 个关节方向正确、无抖动和异常发热。`--enable-low-level` 与
`--confirm-suspended` 是双重软件防误触开关，任何发送 `LowCmd` 的模式都必须同时提供。

### 3.3 缓慢进入准备姿态

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode ready \
  --interface eth0 \
  --duration 10 \
  --ready-duration 8 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

程序从当前关节位置平滑插值到保守准备姿态。先检查各关节运动方向、目标位置和跟踪
误差，再考虑提高增益；首次测试不要直接使用训练增益 `--gain-scale 1.0`。

### 3.4 零速度策略测试

`policy` 会先执行准备姿态，再以 50 Hz 推理并以 500 Hz 发布 LowCmd：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode policy \
  --interface eth0 \
  --model deploy/h2/policy.onnx \
  --vx 0.0 --vy 0.0 --wz 0.0 \
  --duration 10 \
  --ready-duration 8 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

默认准备姿态用于第一次关节级检查。完成该检查后，如需复现 Isaac 验证时使用的动作
初始姿态，可增加：

```text
--ready-pose validated-motion
```

确认悬吊零速度策略稳定后，再从很小的速度命令开始，例如 `--vx 0.05`，逐级扩大，
每一级都检查跟踪误差、推理耗时、饱和计数和电机温度。落地测试需要现场保护架与物理
急停，不能仅依据仿真结果直接进行。

## 急停与内置保护

- 运行过程中按 `Ctrl+C` 会进入被动阻尼后退出。
- 默认将 Unitree 遥控器 `B` 键作为锁存的软件急停；触发后需重新启动程序。
- 状态超时、电机报错、非有限数据、倾角过大、关节越界、跟踪误差过大、ONNX
  推理超时或控制周期严重超时都会停止主动位置控制并进入被动阻尼。
- `B` 键和 `Ctrl+C` 都不能替代物理急停与悬吊保护。

查看全部参数：

```bash
python3 deploy/h2/h2_policy_deploy_real.py --help
```

## 当前验证范围

该模型已在 Isaac Sim 中完成回放验证；仓库中的脚本已通过 ONNX 接口、零输入和
Isaac 观测 trace 的离线一致性自检。它尚未替代具体 H2 真机上的悬吊、关节方向、增益
和安全边界验证，因此必须严格按照上述顺序逐步放开。
