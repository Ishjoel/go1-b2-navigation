# go1_mujoco_locomotion

**Unitree Go1 四足运动控制节点**：MuJoCo 仿真 + 强化学习策略，封装成一个**独立可复用**的 ROS 2 节点。

它只做一件事：**把 `/cmd_vel` 速度指令变成四足行走**，并输出 `/odom`、`/scan`、`/joint_states`。
对外全部是标准消息类型与标准 frame 约定，**可以直接和任何自己写的 Nav2 / 导航栈组合**。

```
   你的导航栈                         这个包
   ┌──────────────┐   /cmd_vel   ┌──────────────────────┐
   │  Nav2        │─────────────>│                      │
   │  planner     │              │  go1_locomotion      │
   │  controller  │<─────────────│  (MuJoCo + RL 策略)   │
   │  costmap     │  /odom /scan │                      │
   └──────────────┘  /tf         └──────────────────────┘
```

---

## 1. 快速开始

### 依赖

```bash
# ROS 2 Jazzy（Ubuntu 24.04）
sudo apt install python3-numpy python3-venv
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install 'mujoco>=3.0'
```

> 模型随包提供的是**自包含的 `.mjb`**（内含全部 mesh 与场景几何），
> 所以**不需要**装 `mujoco_playground` / `mujoco_menagerie` / `etils`，
> 也不存在资源路径解析问题。

MuJoCo 安装在工作区虚拟环境中，运行 ROS 节点时需要将该环境加入 `PYTHONPATH`
（ROS 生成的 console script 使用系统 Python 的 shebang）：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages:$PYTHONPATH"
```

### 编译运行

```bash
# 在包含 go1_mujoco_locomotion 包的工作区根目录执行
cd ~/your_ws
colcon build --packages-select go1_mujoco_locomotion
source install/setup.bash

ros2 launch go1_mujoco_locomotion locomotion.launch.py mujoco_viewer:=true rviz:=true
```

`mujoco_viewer:=true` 打开官方 MuJoCo Simulate 窗口，显示策略正在驱动的机器人；
`rviz:=true` 同时打开 RViz 查看激光、TF 和里程计。

另一个终端里发个速度指令，机器人就会走：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### 接口自检（建议第一次就做）

```bash
ros2 run go1_mujoco_locomotion verify_interface
```

它会检查三个话题是否都有数据、frame 对不对、站立高度是否正常，
并把你组合 Nav2 时最容易踩的坑直接列出来。

---

## 2. 接口契约

这是唯一需要知道的东西。**全部是可配置的话题名**（见 §5 参数）。

| 方向 | 话题（默认） | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` 前进 m/s；`linear.y` 横移（策略未训练，恒 0）；`angular.z` 偏航 rad/s |
| 发布 | `/odom` | `nav_msgs/Odometry` | 位姿 + **机体系**速度（ROS 约定） |
| 发布 | `/scan` | `sensor_msgs/LaserScan` | 360 线，1° 分辨率，`frame_id = base_link` |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | 12 关节，可交给 `robot_state_publisher` |
| 发布 | `/tf` | `tf2` | `odom -> base_link`（可用 `publish_tf:=false` 关掉） |

### frame 约定

```
world（你的全局系）
  └── odom          ← 本节点发布（里程计漂移系）
        └── base_link   ← 本节点发布
              └── （你 URDF 里的腿/激光等）
```

**激光数据直接发布在 `base_link` 系里**，安装位（机体系前上方 `0.19, 0, 0.20`）已在内部补偿，
所以你**不需要**额外定义 `laser_frame`，也不需要为它配静态 TF。

### QoS

- 全部话题：**RELIABLE + VOLATILE**，depth 10（ROS 2 默认）
- 订阅 `/cmd_vel` 用默认 profile 即可

### 数据率

| 话题 | 频率 |
|---|---|
| `/odom` `/joint_states` | **50 Hz**（与策略同频） |
| `/scan` | 默认 **10 Hz**（可通过 `scan_rate` 调整）；与 Nav2 集成时使用 90 线 |

### 安全兜底

**`cmd_timeout`（默认 0.5 s）**：超过这个时间没收到 `/cmd_vel`，速度指令自动归零。
所以你的导航栈崩了或卡住时，机器人会自己停下，不会一直往前冲。

---

## 3. ★ 和 Nav2 组合时必看的三个坑

这三条都是实测踩出来的。**它们都不报错，只是让行为变得很怪**，
所以放在最显眼的位置。

### 坑 1：Nav2 的速度下限必须抬进"工作区间"

策略有**低速死区** —— 指令太小它跟不动：

| 线速度指令 | 实际跟踪率 | | 偏航角速度指令 | 实际跟踪率 |
|---|---|---|---|---|
| 0.09 m/s | **1%** | | 0.25 rad/s | **42%** |
| 0.15 m/s | **9%** | | 0.30 rad/s | 42% |
| 0.30 m/s | 82% | | 0.60 rad/s | **86%** |
| 0.50 m/s | 96% | | 1.00 rad/s | 92% |
| 1.00 m/s | **99%** | | | |

**如果 Nav2 允许它发小速度，就会出事**：

- `min_speed_xy: 0.01` → DWB 一路减速到 0.0x m/s，机器人**站着一步不走**
- `min_speed_theta: 0.0` → 需要掉头时发很小的角速度，**原地转半天转不过来**

**改法**（`nav2_params.yaml` 的 `controller_server`）：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # ★ 抬到策略的工作区间
      min_speed_xy: 0.28        # 不要用 0.01
      max_speed_xy: 0.55
      min_speed_theta: 0.60     # Go1 的稳定转速下限约 0.6 rad/s
      acc_lim_x: 4.50
      acc_lim_theta: 4.0
```

**注意一个 DWB 的隐含约束**：

```
min_speed_xy    <= acc_lim_x     / controller_frequency
min_speed_theta <= acc_lim_theta / controller_frequency
```

例如 `controller_frequency: 8.0`、`acc_lim_x: 4.5` → `min_speed_xy` 不能超过 0.5625。
**两个参数必须一起改**，否则 DWB 会拒绝这个配置（或在运行时行为异常）。

**为什么不能靠策略去适应小速度？** 我们试过重训（把 50% 的训练样本放进 ±0.3 m/s 窄带），
结果极低速段**毫无改善**（仍是 1%）—— 这更像**步态本身的物理下限**，
而不是采样问题。所以正确做法是在导航层避开它。

### 坑 2：全栈必须统一时间基准

本节点用**墙钟**（`use_sim_time=false`）。

如果你的栈设了 `use_sim_time=true` 但**没有人发布 `/clock`**，那么所有基于时间的逻辑
（等待、超时、沉降计时）都会失效 —— 表现为**状态机停住不动**，而且日志完全正常，极难排查。

- 你那边是 Gazebo / 其他会发 `/clock` 的仿真器 → 把 `use_sim_time:=true` 一起设上
- 纯本节点 + Nav2 的组合 → **全部用墙钟**（`false`）

### 坑 3：多终端一起用时导出 `ROS_LOCALHOST_ONLY=1`

如果本机有 DOWN/NO-CARRIER 的网卡（笔记本电脑插着网线但没连上时很常见），
DDS 会去绑一个不通的接口，导致**进程间发现失败**：

> launch 侧一切正常、话题 `ros2 topic list` 也看得到，
> 但某个节点**一条消息都收不到**。

```bash
export ROS_LOCALHOST_ONLY=1     # 在**每一个**终端里都执行
```

---

## 4. 能力与限制（先看这个再设计任务）

### ✅ 能做的

| 项 | 实测 |
|---|---|
| 前进速度 | 0.30 ~ 1.00 m/s，跟踪率 **82% ~ 99%** |
| 偏航角速度 | 0.60 ~ 1.00 rad/s，跟踪率 **86% ~ 92%** |
| 站立稳定性 | 高度 0.29 m，标准差 ≈ 0.000 |
| 位置精度（配合精对齐） | 实测可达 **0.046 m** |
| 姿态精度 | 实测可达 **1.2°** |

### ⚠️ 限制

| 限制 | 说明 |
|---|---|
| **速度死区** | 线速度 < 0.3 m/s、角速度 < 0.5 rad/s 时跟踪率很差（见坑 1）。**精细定位要靠"走一小步→停稳→测量→修正"的闭环，不要靠低速爬行。** |
| **横移未训练** | `linear.y` 当前策略没有训练，请保持 0 |
| **平地** | 策略在平地训练。场景里有台阶/斜坡需要重新训练 |
| **无地形感知** | 策略用的是本体感受（关节角/速度/姿态），**不是**视觉或高度图 |
| **仿真** | 这是 MuJoCo 里的仿真运动层，不是实机驱动。接实机需要换成 SDK 版本（见 §7） |

### 关于"精细定位"的建议

如果你的任务需要厘米级停位，**不要**让 Nav2 去低速逼近（会掉进死区）。
推荐做法（我们在本项目里实测有效）：

```
Nav2 走到目标附近（容差给大一点，比如 0.5 m）
    ↓
切到一个自定义的"精对齐"状态机：
    转正 → 停稳 1.2 s → 直行一小步(约 0.3~0.5 m，速度 0.3~0.45 m/s) → 停稳 → 测量
    重复直到位置/姿态都进容差
```

因为**精对齐需要的是"可靠的小增量位移"，不是"低速连续行走"** ——
用 0.3 m/s 走 0.3 秒就是 0.09 m，完全够用，而且正好落在策略最擅长的区间。

---

## 5. 参数

全部参数在 `config/locomotion.yaml`，可用 `ros2 param` 覆盖。
标了 **[可调]** 的是可能需要按你的场景改的；**其余不要动**（它们与训练一一对应）。

```yaml
go1_locomotion:
  ros__parameters:
    # 接口
    cmd_vel_topic: "cmd_vel"      # [可调]
    odom_topic: "odom"            # [可调]
    scan_topic: "scan"            # [可调]
    odom_frame: "odom"            # [可调] 与你的 global frame 对齐
    base_frame: "base_link"       # [可调] 与你的 URDF 对齐
    publish_tf: true              # [可调] 别处已发就设 false

    # 限幅与安全
    cmd_timeout: 0.5              # [可调] 收不到指令多久归零
    max_linear_x: 1.0             # [可调]
    max_angular_z: 1.2            # [可调]

    # 激光
    laser_samples: 360            # [可调]
    laser_x: 0.19                 # [可调] 安装位（机体系）
    laser_z: 0.20                 # [可调]
    laser_range: 10.0             # [可调]

    # ★ 以下与训练严格对应，改了策略就废了
    sim_dt: 0.004
    substeps: 5
    kp: 35.0
    kd: 0.5
    action_scale: 0.5
```

常用覆盖：

```bash
ros2 launch go1_mujoco_locomotion locomotion.launch.py \
    odom_frame:=map base_frame:=base_footprint publish_tf:=false
```

---

## 6. 它是怎么工作的

### 控制回路

```
/cmd_vel ─┐
          ├─> 组装 48 维观测 ─> 归一化 ─> MLP 策略 ─> 动作(12) ─┐
MuJoCo 状态┘                                                      │
                                                                  v
        ctrl = action × 0.5 + default_pose   （MuJoCo 位置执行器，Kp=35, Kd=0.5）
                                                                  │
                       每控制周期 5 × 0.004 s 物理步 <────────────┘
                                                                  │
                       50 Hz 发布 /odom /scan /joint_states <─────┘
```

- **策略**：MLP `48 → 512 → 256 → 128 → 24`，激活 **swish**，
  最后过 **tanh**（brax 的 `tanh_normal` 策略头），取前 12 维作动作均值。
- **观测 48 维**（顺序与 scale 都固定！改错不报错，只会让机器人变傻）：

  | 索引 | 内容 | 备注 |
  |---|---|---|
  | `[0:3]` | 机体系线速度 | **原始值，不打尺度** |
  | `[3:6]` | 机体系角速度 | 原始值 |
  | `[6:9]` | 投影重力 | 直立时应为 `[0,0,-1]` |
  | `[9:21]` | 关节角 − `default_pose` | |
  | `[21:33]` | 关节角速度 | |
  | `[33:45]` | 上一步动作 | |
  | `[45:48]` | 速度指令 `[vx,vy,wz]` | 原始值 |

- **必须做观测归一化** `(x − mean) / std`，统计量在 `config/go1_obs_normalizer.npz`。
  漏掉这一步，策略输出误差会从 `1e-7` 涨到 `0.43`，机器人会站不稳。
- 本文件的 NumPy 前向已与训练框架（MJX）**逐位比对过，最大误差 1.2e-07**。

### 训练方式

策略在 **MuJoCo 里训练**（`mujoco_playground` + MJX + brax PPO，2 亿步约 25 分钟）。

> 顺带说一句：我们一开始走的是「Isaac Gym 训练 + MuJoCo 部署」，
> **那条路走不通** —— 两个引擎的接触模型差异会让策略在 0.3 m/s 以上直接失稳
> （足端冲击造成的速度尖峰是训练噪声带的 8~33 倍）。
> 换成同引擎训练后问题彻底消失。如果你也要训四足策略，这一点值得注意。

---

## 7. 接实机怎么办

本节点是**仿真**运动层。接真机时**不需要改导航栈** —— 因为接口契约是标准话题。

只需把 `locomotion_node` 替换成一个基于
[`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) 的版本，保持：

- 同样订阅 `/cmd_vel`
- 同样发布 `/odom`（实机用 `SportModeState` 或状态估计）
- 同样发布 `/scan`（实机用真实激光雷达驱动）
- **同一个策略权重与同一套观测组装代码**（观测布局不变）

> ⚠️ 实机上有一个**已知的 sim-to-real 缺口**：观测里的 `[0:3]` 机体系线速度
> 在实机上不能直接测（没有这个传感器）。常见做法是用
> `SportModeState` 的速度估计，或者训练时就用可测量的量替代。
> 这是接实机前必须先解决的一项。

---

## 8. 排障

| 现象 | 最可能的原因 |
|---|---|
| **机器人站着一步不走** | Nav2 的 `min_speed_xy` 太小（掉进死区）→ 见坑 1 |
| **原地转半天转不过来** | `min_speed_theta` 太小 → 见坑 1 |
| **状态机停住不动，日志正常** | 时间基准不一致（`use_sim_time` 与 `/clock`）→ 见坑 2 |
| **话题看得到但收不到消息** | DDS 发现失败 → 导出 `ROS_LOCALHOST_ONLY=1`，见坑 3 |
| **启动就摔倒** | 模型/权重/归一化文件不匹配；或改了 `sim_dt`/`kp`/`action_scale` |
| **走起来但抖得厉害** | 检查 `sim_dt` 是否为 0.004（改成 0.005 会发散） |
| **激光全是 max_range** | 场景里没有障碍物几何。默认模型带通道；换场景请确保几何的 `contype/conaffinity` 与地板一致 |
| **`scan` 的障碍位置偏了** | 激光安装位参数 `laser_x/z` 与实际不符 |

快速确认节点是否正常：

```bash
ros2 run go1_mujoco_locomotion verify_interface
```

---

## 9. 文件说明

```
go1_mujoco_locomotion/
├── README.md                        ← 本文
├── package.xml / setup.py / setup.cfg
├── go1_mujoco_locomotion/
│   ├── locomotion_node.py           ← 主节点（全部逻辑，约 380 行，注释详尽）
│   └── verify_interface.py          ← 接口自检
├── config/
│   ├── go1_corridor.mjb             ← 自包含模型（含 mesh + 通道几何，47 MB）
│   ├── go1_policy_weights.npz       ← 策略权重（MLP）
│   ├── go1_obs_normalizer.npz       ← ★ 观测归一化统计量（不能省）
│   ├── go1_default_pose.npy         ← 默认关节角（备用，优先从模型关键帧读）
│   ├── locomotion.yaml              ← 参数
│   └── locomotion.rviz              ← 最小 RViz 配置（看 scan/odom/TF）
└── launch/
    └── locomotion.launch.py
```

### 换成你自己的场景

默认模型里带的是一条 4 m 宽、22 m 长的工业通道（货架 + 边界墙）。
换成你自己的环境有两种做法：

1. **直接改 MJCF**：用 MuJoCo 打开 `go1_corridor.mjb` 对应的 XML，替换环境几何后重新编译
2. **给 MuJoCo 一个自定义场景 XML**：用 `scene:=/path/to/your_scene.xml` 指定。
   XML 里 `<include>` Go1 模型后追加你的几何即可。

**加环境几何时注意**：Go1 模型的默认 geom class 把碰撞关了
（`contype="0" conaffinity="0"`），地板是靠显式声明打开的。所以你加的几何也要照抄地板那行：

```xml
<geom name="wall" type="box" size="1 0.2 1" pos="5 2 1"
      contype="1" conaffinity="0" priority="1" friction="0.6" condim="3"/>
```

**否则机器人会直接穿墙而过，而且不会报任何错。**

---

## 10. 许可与出处

- 代码：Apache-2.0
- 策略：本项目自行训练（MuJoCo + brax PPO）
- Go1 模型：来自 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)（BSD-3-Clause）
