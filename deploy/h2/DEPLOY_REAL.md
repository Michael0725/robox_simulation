# Unitree H2 ONNX 策略真机部署作业指导书

本文档用于将 `policy.onnx` 通过 `h2_policy_deploy_real.py` 部署到 Unitree H2，
并完成从离线自检到悬吊策略验证的全过程。

> **安全警告**：该程序直接发布 H2 `rt/lowcmd`，属于底层关节控制。首次运行
> `passive`、`ready` 或 `policy` 必须可靠悬吊机器人、清空运动范围，并安排独立人员
> 手持物理急停。仿真验证通过不代表可以直接落地运行。

## 1. 适用范围与当前验证边界

本文档覆盖：

- 将脚本和模型复制到 H2 PC2；
- 安装并验证 Python、ONNX Runtime、CycloneDDS 和 `unitree_sdk2_python`；
- 配置并检查 DDS 有线网络；
- 执行 `self-test → observe → passive → ready → policy`；
- 判断每个阶段是否通过；
- 正常停止、异常急停与常见故障排查。

当前已完成：

- ONNX 接口验证：输入 `float32 [1, 400]`、输出 `float32 [1, 29]`；
- Isaac Sim 策略回放验证；
- 3000 帧 Isaac observation trace 一致性验证，动作和关节目标最大误差为 0；
- Unitree H2 `LowCmd` 类型、31 电机槽位与 CRC 构造检查。

当前尚未替代：

- 具体 H2 固件与 SDK 的关节索引确认；
- 真机关节正方向确认；
- 真机 IMU 坐标系和四元数顺序确认；
- 真机增益、摩擦、弹性、温度和电源验证；
- 足底接触、落地平衡和摔倒恢复验证。

因此本文档的最终目标是完成**悬吊策略验证**。从悬吊转入落地测试之前，还必须补充
现场保护架、温度监控、足底接触和更完整的急停验证。

## 2. 控制链路

```text
H2 rt/lowstate
  ├─ pelvis IMU gyroscope：3
  ├─ pelvis IMU quaternion → projected gravity：3
  ├─ velocity command vx/vy/wz：3
  ├─ joint position - default position：31
  ├─ joint velocity：31
  └─ previous raw action：29
              │
              ▼
      单帧 observation：100
              │ 最近 4 帧，旧 → 新
              ▼
       ONNX input：[1, 400]
              │ 50 Hz CPU inference
              ▼
       ONNX output：[1, 29]
              │ default q + action scale × action
              │ joint limit + target slew limit
              ▼
       31 个 H2 关节目标
              │ 500 Hz + CRC
              ▼
          H2 rt/lowcmd
```

ONNX 不需要真机机身线速度测量。`--vx/--vy/--wz` 是期望速度命令，不是里程计反馈。
头部两个电机参与观测但不由策略输出控制，脚本将其保持在默认位置。

## 3. 现场人员与机械保护

首次主动控制至少安排两人：

1. **电脑操作员**：执行命令、观察日志、随时按 `Ctrl+C`。
2. **急停操作员**：只负责观察机器人和触发物理急停，不同时操作电脑。

开始前逐项确认：

- [ ] 使用额定载荷足够且经过检查的悬吊架；
- [ ] 悬吊点和保险连接可靠；
- [ ] 双脚离地且最大动作范围内不会接触地面；
- [ ] 手臂、头部、腰部和腿部运动范围内无人、无设备；
- [ ] 电池、外壳和线缆固定可靠；
- [ ] 网线不会进入关节或运动范围；
- [ ] 物理急停已测试，急停操作员可立即触达；
- [ ] 现场已约定“开始、停止、急停”口令；
- [ ] 没有其他程序发布 `rt/lowcmd`。

人员扶持不能替代首次测试所需的机械悬吊。

## 4. 部署阶段总览

| Gate | 阶段 | 是否发送 LowCmd | 通过后才能做什么 |
|---|---|---:|---|
| 0 | 文件、版本、关节映射确认 | 否 | 安装与离线检查 |
| 1 | `self-test` | 否 | 连接机器人网络 |
| 2 | `observe` | 否 | 进入底层命令阶段 |
| 3 | `passive` | 是，仅阻尼 | 主动位置控制 |
| 4 | `ready default` | 是 | 验证动作初始姿态 |
| 5 | `ready validated-motion` | 是 | 零速度策略 |
| 6 | `policy` 零速度 | 是 | 悬吊小速度命令 |
| 7 | `policy` 小速度 | 是 | 规划保护架内落地测试 |

任何 Gate 失败，都必须停留在当前阶段。不要通过放宽阈值、关闭检查或提高增益强行通过。

## 5. Gate 0：复制并核验部署文件

### 5.1 从 GitHub 克隆

在 H2 PC2 上执行：

```bash
cd ~
git clone https://github.com/Michael0725/robox_simulation.git
cd robox_simulation
git rev-parse HEAD
```

首次部署文档对应的基础部署提交为：

```text
47f6546fe6df906b05636ebbedbef4d1f58d55fb
```

后续文档提交会使 `HEAD` 更新，因此还应使用文件哈希确认实际部署内容。

### 5.2 无法访问 GitHub 时使用 SCP

在保存本仓库的开发电脑上执行：

```bash
scp -r /path/to/robox_simulation <PC2_USER>@<PC2_IP>:~/
```

登录 PC2：

```bash
ssh <PC2_USER>@<PC2_IP>
cd ~/robox_simulation
```

`<PC2_USER>` 和 `<PC2_IP>` 必须替换为现场实际值，不要照抄占位符。

### 5.3 文件完整性

```bash
cd ~/robox_simulation
sha256sum deploy/h2/h2_policy_deploy_real.py deploy/h2/policy.onnx
ls -lh deploy/h2/
```

已验证文件哈希：

```text
314ad63a1f2a68f3fa359c82f2b4a456bc6cf788d8afa4ff2dff7a2bc9419501  deploy/h2/h2_policy_deploy_real.py
6912db1b08e4309bdc6978b80309d35f02f4635e618b097266cbb84ac3249094  deploy/h2/policy.onnx
```

模型约 1.5 MB。模型哈希不一致时不要继续，重新传输并再次校验。

```bash
chmod +x deploy/h2/h2_policy_deploy_real.py
```

## 6. PC2 Python 环境

### 6.1 基础信息

```bash
uname -m
python3 --version
which python3
```

要求：

- Python 3.8 或更高；
- 与 PC2 架构匹配的 ONNX Runtime；
- CycloneDDS 0.10.2；
- 官方 `unitree_sdk2_python`；
- NumPy。

### 6.2 建议创建虚拟环境

如果 PC2 的系统 Python 已经包含 Unitree SDK，可以继承系统包：

```bash
python3 -m venv --system-site-packages ~/venvs/h2_policy
source ~/venvs/h2_policy/bin/activate
```

之后每次运行前执行：

```bash
source ~/venvs/h2_policy/bin/activate
cd ~/robox_simulation
```

### 6.3 安装 NumPy 和 ONNX Runtime

```bash
python3 -m pip install --upgrade pip
python3 -m pip install numpy onnxruntime
```

检查：

```bash
python3 -c "import numpy, onnxruntime; print(numpy.__version__); print(onnxruntime.__version__); print(onnxruntime.get_available_providers())"
```

输出中必须包含 `CPUExecutionProvider`。脚本不要求 CUDA。若 aarch64 平台找不到兼容
wheel，应使用与 PC2 系统、Python 版本和架构匹配的官方或 Unitree 提供版本，不要安装
来源不明的二进制包。

## 7. 安装和验证 unitree_sdk2_python

### 7.1 优先检查 PC2 现有 SDK

```bash
python3 -c "import unitree_sdk2py; print(unitree_sdk2py.__file__)"
```

继续检查 H2 所需接口：

```bash
python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_; from unitree_sdk2py.utils.crc import CRC; from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient; print('Unitree H2 SDK imports OK')"
```

### 7.2 从官方仓库安装

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
python3 -m pip install -e .
```

当前脚本开发时参考的 SDK 提交是：

```text
e4cd91f051aaa77a70600e3d2bf7f50889db1980
```

为了复现环境可以执行 `git checkout` 固定提交，但最终版本必须与现场 H2 固件兼容。
不要在首次真机试验当天临时升级 SDK 或固件。

### 7.3 CycloneDDS 构建错误

若安装 SDK 时出现 `Could not locate cyclonedds`：

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ~/cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
cd ~/unitree_sdk2_python
python3 -m pip install -e .
```

Unitree 官方安装说明要求 Python ≥3.8、CycloneDDS 0.10.2，并在源码构建 DDS 时设置
`CYCLONEDDS_HOME`：

- <https://github.com/unitreerobotics/unitree_sdk2_python>
- <https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/README%20zh.md>

### 7.4 最终依赖检查

```bash
cd ~/robox_simulation
python3 -c "import numpy, onnxruntime, unitree_sdk2py; from unitree_sdk2py.core.channel import ChannelFactoryInitialize; from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_; from unitree_sdk2py.utils.crc import CRC; print('All dependencies OK')"
python3 deploy/h2/h2_policy_deploy_real.py --help
```

两条命令都成功后才进入离线自检。

## 8. Gate 1：离线 ONNX 自检

该模式不连接机器人，不发布 LowCmd：

```bash
cd ~/robox_simulation
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode self-test \
  --model deploy/h2/policy.onnx
```

预期：

```text
SELF_TEST_INTERFACE input=obs:['batch', 400] output=actions:['batch', 29]
SELF_TEST_ZERO action_min=-1.710654 action_max=1.365724
SELF_TEST_OK
```

通过条件：

- [ ] 输入最后一维为 400；
- [ ] 输出最后一维为 29；
- [ ] 推理输出没有 NaN/Inf；
- [ ] 最后一行是 `SELF_TEST_OK`。

`Unexpected ONNX interface` 表示模型和脚本不匹配；不要继续真机测试。

## 9. DDS 有线网络

按照 Unitree Quick Start 为当前 H2 配置网络：

- <https://support.unitree.com/home/zh/developer/Quick_start>

### 9.1 找到真实网卡名

```bash
ip -br link
ip -br address
```

后续示例使用 `eth0`。现场可能是 `enp2s0` 或其他名称，必须替换为连接 H2 的真实
有线网卡。

```bash
ip link show eth0
ip address show eth0
ip -s link show eth0
```

确认：

- [ ] 网卡为 `UP`；
- [ ] PC2 与机器人位于 Unitree 指定网段；
- [ ] RX/TX 没有持续增长的错误；
- [ ] 使用有线网络；
- [ ] VPN、Docker 网桥等没有让 DDS 选择错误接口。

不要照搬其他 Unitree 型号的固定 IP，应使用当前 H2 随机文档或现场配置。

## 10. Gate 0 补充：31 个关节索引

脚本使用下列 SDK 电机索引：

| SDK 索引 | 关节 | SDK 索引 | 关节 |
|---:|---|---:|---|
| 0 | left hip pitch | 6 | right hip pitch |
| 1 | left hip roll | 7 | right hip roll |
| 2 | left hip yaw | 8 | right hip yaw |
| 3 | left knee | 9 | right knee |
| 4 | left ankle roll | 10 | right ankle roll |
| 5 | left ankle pitch | 11 | right ankle pitch |
| 12 | waist yaw | 15 | left shoulder pitch |
| 13 | waist roll | 16 | left shoulder roll |
| 14 | waist pitch | 17 | left shoulder yaw |
| 18 | left elbow | 22 | right shoulder pitch |
| 19 | left wrist roll | 23 | right shoulder roll |
| 20 | left wrist pitch | 24 | right shoulder yaw |
| 21 | left wrist yaw | 25 | right elbow |
| 26 | right wrist roll | 29 | head pitch |
| 27 | right wrist pitch | 30 | head yaw |
| 28 | right wrist yaw |  |  |

重点核对 `12～14`、`19～21` 和 `26～28`。Unitree H2 示例的关节枚举曾发生调整，
不能只看枚举名称推断真机固件的数组含义。当前映射与策略资产和 Unitree H2 C++ 示例
一致：

- <https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/h2/low_level/h2_ankle_swing_example.cpp>

通过条件：

- [ ] 机器人型号确认为 H2；
- [ ] `LowState.motor_state` 有 31 个有效槽位；
- [ ] 腰部、手腕、头部索引经过现场 SDK/固件资料确认；
- [ ] 脚踝使用 PR 表示时 `4/5/10/11` 与脚本一致；
- [ ] pelvis IMU 来源和坐标系已确认。

未完成这些确认时，不运行 `ready` 或 `policy`。

## 11. 确认没有命令冲突

```bash
ps aux | grep -E "lowcmd|low_level|h2_policy|ankle_swing"
```

停止其他会发布 `rt/lowcmd` 的程序。底层控制前还需按 Unitree 操作流程关闭高层运动
服务；官方文档明确提示高层 `sport_mode` 与底层命令同时工作会造成指令冲突：

- <https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/README%20zh.md>

部署脚本也会通过 `MotionSwitcherClient` 检查并释放当前高层模式，但这不能替代现场确认。

不要把官方 `h2_ankle_swing_example.py` 当作只读测试运行；该示例会发布 LowCmd 并驱动
脚踝。纯只读检查应使用下一节的 `observe`。

## 12. Gate 2：observe 只读状态

```bash
cd ~/robox_simulation
source ~/venvs/h2_policy/bin/activate
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode observe \
  --interface eth0 \
  --duration 30 \
  --print-interval 1
```

典型日志：

```text
H2_OBSERVE age_ms=1.20 upright=0.9990 gravity=0.0010,-0.0030,-0.9990 gyro=0.0020,-0.0010,0.0030 q_min=-0.250 q_max=0.870 errors=[]
```

字段解释：

- `age_ms`：最新 LowState 的年龄；默认超过 50 ms 视为陈旧状态；
- `upright=-gravity_z`：直立时应接近 1；
- `gravity`：直立时应接近 `[0, 0, -1]`；
- `gyro`：静止时应接近零，仅有少量 IMU 噪声；
- `q_min/q_max`：当前 31 个关节位置范围；
- `errors`：电机错误索引，正常必须为空。

通过条件：

- [ ] 连续 30 秒收到有效 LowState；
- [ ] `age_ms` 稳定且通常为几毫秒；
- [ ] 直立悬吊时建议 `upright > 0.9`；
- [ ] 直立时 gravity 接近 `[0, 0, -1]`；
- [ ] 静止时 gyro 接近零；
- [ ] `errors=[]`；
- [ ] 无 CRC、回调或状态超时错误。

脚本的 `--min-upright 0.5` 是故障保护阈值，不是正常直立质量标准。如果重力方向相反
或与实际倾斜方向不一致，应先修正 IMU 坐标约定，不要降低阈值绕过。

## 13. 遥控器和急停说明

Unitree H2 官方 Python 示例使用无线遥控数据 bit 9 表示 B 键，脚本采用相同解析：

- <https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/h2/low_level/h2_ankle_swing_example.py>

当前版本的限制：

- B 键状态会在 LowState 回调中锁存；
- `policy` 主循环会检查锁存状态并停止；
- `passive`、ready 过渡和 ready 保持阶段尚未统一检查该锁存状态。

因此 B 键不能作为所有阶段唯一可靠急停。当前优先级应为：

1. 物理急停；
2. 操作员 `Ctrl+C`；
3. policy 阶段的遥控器 B 软件急停。

正式落地测试前，建议把 B 急停检查覆盖到所有发送 LowCmd 的阶段。

## 14. Gate 3：passive 被动阻尼

确认机器人已经悬吊、物理急停就位后，第一次只运行 3 秒：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode passive \
  --interface eth0 \
  --duration 3 \
  --enable-low-level \
  --confirm-suspended
```

脚本将：

1. 等待有效 LowState；
2. 检查并释放高层运动模式；
3. 启动 500 Hz `rt/lowcmd`；
4. 发送 `Kp=0`、被动 `Kd`、零前馈力矩；
5. 到期后保持短暂被动阻尼再停止线程。

正常表现：关节仅有阻尼，不主动前往某个姿态。出现以下任意情况立即物理急停：

- 任意关节主动快速运动；
- 高频抖动、持续啸叫；
- 腰或手臂突然甩动；
- 电机错误、异常发热或焦味；
- 状态中断后关节仍保持刚性；
- `Ctrl+C` 无法使程序退出。

3 秒测试通过后运行 10 秒：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode passive \
  --interface eth0 \
  --duration 10 \
  --enable-low-level \
  --confirm-suspended
```

## 15. Gate 4：default ready 准备姿态

该阶段开始主动位置控制：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode ready \
  --interface eth0 \
  --ready-pose default \
  --ready-duration 10 \
  --duration 5 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

含义：

- 10 秒从当前关节位置平滑进入保守默认姿态；
- 到达后保持 5 秒；
- Kp/Kd 使用训练增益的 35%；
- 总主动时间约为 15 秒。

观察全部腿、腰、臂、腕和头部，尤其核对腰部 `12～14`、左腕 `19～21`、右腕
`26～28`。正常情况下动作连续、缓慢、左右腿基本对称，无一步跳变和持续振荡。

以下任一情况表示失败：

- 本应弯膝却转腰；
- 脚踝 pitch/roll 交换；
- 腰 yaw/roll/pitch 交换；
- 腕 roll/pitch/yaw 交换；
- 关节向机械限位运动；
- 左右方向明显错误；
- 持续振荡、异响或跟踪失败。

不要用降低安全检查、提高目标速度或提高增益掩盖索引错误。建议至少重复三次
`default ready`，确认结果一致后再继续。

## 16. Gate 5：validated-motion ready

该姿态是 Isaac 验证初始化所用的低速度动作帧。只有 default ready 完整通过后才运行：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode ready \
  --interface eth0 \
  --ready-pose validated-motion \
  --ready-duration 10 \
  --duration 5 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

通过条件与 default ready 相同。记录准备姿态下每个关节的实际位置、机械干涉、异响和
电机状态。

## 17. Gate 6：悬吊零速度策略

第一次只运行 3 秒：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode policy \
  --interface eth0 \
  --model deploy/h2/policy.onnx \
  --ready-pose validated-motion \
  --vx 0.0 --vy 0.0 --wz 0.0 \
  --ready-duration 10 \
  --duration 3 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

程序顺序：

1. 接收 LowState；
2. 释放高层模式；
3. 启动被动阻尼；
4. 进入 validated-motion 准备姿态；
5. 用当前帧初始化四帧 observation history；
6. 50 Hz ONNX 推理；
7. 动作转换、关节限位和目标变化率限制；
8. 500 Hz 发布 LowCmd；
9. 到期或异常时进入被动阻尼退出。

典型日志：

```text
H2_POLICY t=1.00s step=51 cmd=0.000,0.000,0.000 upright=0.998 infer_ms=0.5 max_infer_ms=0.8 tracking_error=0.12 saturations=0
```

重点观察：

- `cmd`：第一次必须始终为 `0,0,0`；
- `upright`：悬吊直立时应接近 1；
- `infer_ms/max_infer_ms`：必须明显低于默认 20 ms 超时；
- `tracking_error`：应明显低于 1 rad 的保护阈值；
- `saturations`：应尽可能为 0，不应持续快速增加。

立即停止条件：

- 任意关节快速甩动或高频振荡；
- 手臂持续碰撞身体或支架；
- tracking error 持续增大；
- saturation 快速累计；
- inference 接近或超过 20 ms；
- upright 快速下降；
- 电机错误、状态超时或异常声音；
- Ctrl+C 后没有进入被动阻尼。

3 秒通过后，按 `5 秒 → 10 秒 → 30 秒` 逐级延长。每一级之间检查温度、电池、线缆、
悬吊结构、机械干涉和错误日志。

## 18. Gate 7：悬吊小速度命令

每次只改变一个命令维度。首先测试小幅前向命令：

```bash
python3 deploy/h2/h2_policy_deploy_real.py \
  --mode policy \
  --interface eth0 \
  --model deploy/h2/policy.onnx \
  --ready-pose validated-motion \
  --vx 0.05 --vy 0.0 --wz 0.0 \
  --ready-duration 10 \
  --duration 5 \
  --gain-scale 0.35 \
  --enable-low-level \
  --confirm-suspended
```

建议阶梯：

```text
vx: 0.00 → 0.05 → 0.10 → 0.15
vy: 0.00 → ±0.03 → ±0.05
wz: 0.00 → ±0.05 → ±0.10
```

先完整验证 `vx`，再验证 `vy`，最后验证 `wz`。首次测试不要同时提供三个非零命令，
也不要一步跳到大速度。

## 19. 正常停止与异常急停

### 19.1 正常停止

按 `Ctrl+C` 后，程序应输出：

```text
Entering passive damping before shutdown...
```

然后发送短暂被动阻尼并停止 LowCmd 线程。程序退出后，高层运动服务不保证自动恢复；
应按当前 H2 固件和 Unitree 操作流程恢复，不要假设机器人自动回到高层站立控制。

### 19.2 异常动作

优先级：

1. 急停操作员立即触发物理急停；
2. 电脑操作员按 `Ctrl+C`；
3. policy 阶段使用遥控器 B 软件急停。

物理异常时不要等待 Python 自己报错。急停后不要立即重启，先检查机械卡阻、限位、
温度、错误码、悬吊架、电源、网络和最后一条日志。

## 20. 故障排查

### 20.1 `No valid H2 LowState received`

检查网卡名、网线、IP 网段、DDS 接口、VPN/虚拟网卡和防火墙策略：

```bash
ip -br address
ip -s link show eth0
```

只回到 `observe` 排查，不尝试主动控制。

### 20.2 `LowState stale`

表示状态超过默认 50 ms 未更新。检查网络丢包、CPU 负载和 DDS 配置。首次部署不要直接
增大 `--max-state-age`。

### 20.3 `High-level mode is still active`

使用 Unitree App/官方流程关闭高层运动服务，重新运行 `observe` 后再测试。不要绕过
MotionSwitcher 检查。

### 20.4 `motor errors at indices [...]`

记录索引并立即停止。检查电机和固件错误，不要注释错误检查。Unitree H2 官方低层示例
同样检查 LowState CRC 和电机状态。

### 20.5 `upright below ...`

回到 `observe`，检查真实姿态、pelvis IMU、四元数顺序和重力投影。不要直接降低
`--min-upright`。

### 20.6 `joint tracking error exceeds ...`

检查关节索引、方向、目标、机械阻挡、限流和反馈。不要直接增大
`--max-tracking-error`。

### 20.7 `ONNX inference deadline miss`

检查 PC2 CPU 负载，停止不必要任务，确认模型在本地磁盘并使用 CPU provider。不要在
没有分析的情况下直接增大推理超时。

### 20.8 saturation 持续增加

表示关节目标触发机械限位或目标变化率限制，可能是动作幅度、映射或状态分布异常。
不要通过增大 `--max-target-speed` 或缩小限位余量绕过。

## 21. 从悬吊到落地前的附加条件

至少补齐：

- [ ] 遥控器急停覆盖 passive、ready 和 policy 全阶段；
- [ ] 每个关节实际位置、目标位置和跟踪误差日志；
- [ ] IMU 和 ONNX 原始动作日志；
- [ ] 电机温度、电压和电源状态监控；
- [ ] DDS 周期与控制循环抖动统计；
- [ ] 足底压力/接触状态验证；
- [ ] 摔倒检测和可靠停机策略；
- [ ] 低电量和过温保护；
- [ ] 保护架内轻触地测试；
- [ ] 现场风险评审和落地测试方案。

当前脚本不测量机身线速度，也不使用足底接触，因此悬吊稳定不能直接推导出落地稳定。

## 22. 试验记录模板

每次测试建议记录：

```text
日期/时间：
操作员：
急停操作员：
机器人序列号：
固件版本：
unitree_sdk2_python commit：
部署仓库 commit：
脚本 SHA-256：
模型 SHA-256：
网卡名：
运行模式：
ready pose：
gain scale：
vx/vy/wz：
duration：
最大 inference ms：
最大 tracking error：
saturation 次数：
电机错误：
测试结果 PASS/FAIL：
异常描述与视频文件：
```

## 23. 最终 Go/No-Go 清单

- [ ] 文件哈希正确；
- [ ] ONNX 输出 `SELF_TEST_OK`；
- [ ] SDK 与固件版本已记录；
- [ ] 31 个电机索引确认；
- [ ] IMU 重力方向确认；
- [ ] observe 连续 30 秒稳定；
- [ ] 没有电机错误；
- [ ] passive 无主动异常动作；
- [ ] default ready 全部关节方向正确；
- [ ] validated-motion ready 无机械干涉；
- [ ] 零速度策略无振荡；
- [ ] 推理耗时有足够余量；
- [ ] 跟踪误差正常；
- [ ] saturation 不持续增长；
- [ ] Ctrl+C 能进入被动阻尼；
- [ ] 物理急停有效；
- [ ] 悬吊架与现场保护有效。

任何一项未确认，都不得进入下一 Gate。

## 24. 官方参考

- Unitree SDK2 Python：<https://github.com/unitreerobotics/unitree_sdk2_python>
- Unitree SDK2 Python 中文说明：<https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/README%20zh.md>
- Unitree H2 Python 低层示例：<https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/h2/low_level/h2_ankle_swing_example.py>
- Unitree H2 C++ 低层示例：<https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/h2/low_level/h2_ankle_swing_example.cpp>
- Unitree Quick Start：<https://support.unitree.com/home/zh/developer/Quick_start>
