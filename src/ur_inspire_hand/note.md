# ur_inspire_hand — 优傲机械臂 + 因时灵巧手 组合仿真工程

## 一.概述：
把 **优傲（Universal Robots）UR 系列机械臂** 和 **因时机器人（Inspire Robots）RH56DFX 仿人五指灵巧手** 组装成一个"肩—臂—肘—腕—手"的完整机械臂，并在 **ROS 2 Humble + Gazebo Classic 11** 中一键启动，可通过 `ros2_control` 控制器同时驱动手臂和手指。

## 二、主体命令：
### 1. 启动机械臂仿真：
```
ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py
```

### 2.启动数据监视器：
```
ros2 run ur_inspire_hand hand_arm_monitor.py
```

### 3.灵巧手常规命令：
```
ros2 run ur_inspire_hand hand_preset.py hand open      # 全部张开
ros2 run ur_inspire_hand hand_preset.py hand first     # 握拳
ros2 run ur_inspire_hand hand_preset.py hand pinch     # 拇指食指捏合
ros2 run ur_inspire_hand hand_preset.py hand point     # 食指伸出指点
ros2 run ur_inspire_hand hand_preset.py hand ok        # OK 手势
ros2 run ur_inspire_hand hand_preset.py hand first 1.0  # 指定 1 秒完成
```
自定义灵巧手姿态，七个数字分别对应关节right_thumb_1_joint, right_thumb_2_joint, right_index_1_joint, right_middle_1_joint, right_ring_1_joint, right_little_1_joint + 一个完成时长：
```
ros2 run ur_inspire_hand hand_preset.py hand custom 0.5 0.4 1.0 1.0 1.0 1.0 3.0
```

### 4.机械臂常规命令：
```
ros2 run ur_inspire_hand hand_preset.py arm home   # 回 home 位 (与仿真启动一致)
ros2 run ur_inspire_hand hand_preset.py arm ready  # 工作准备位 (shoulder_pan=0.6)
```
自定义机械臂姿态，七个数字分别对应关节shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint + 一个完成时长：
```
ros2 run ur_inspire_hand hand_preset.py arm custom 0.6 -1.5708 0 -1.5708 0 0 3.0
```
---

## 二. 资源来源（官网核查结论）

| 部件 | 官方渠道 | 结论 |
|---|---|---|
| UR 机械臂 | 官网文档指向 GitHub 官方仓库 [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)（本工程用 `humble` 分支，位于 `src/Universal_Robots_ROS2_Description/`） | **URDF/xacro 就在 GitHub 官方仓库下载** |
| 因时 RH56DFX 灵巧手 | 因时官网（inspire-robots.com）仅提供 **STEP CAD 模型**（RAR 打包，右/左手各一份，已下载至 `downloads/inspire_hand_*.rar` 并解压）；官网 GitHub 组织为空 | **官网没有现成 URDF**；本工程使用的 URDF 来自社区镜像（NYU `inspire-sdk`，位于 `inspire-sdk/`，仅作参考），网格模型源自官方 CAD |

> 机械臂和灵巧手拼接是：把因时灵巧手的安装板（`inspire_hand_base`）用一个固定 joint 直接装到 `tool0` 上。

## 三. 人体结构比喻（运动链映射）

```
world
 └─ base_link                     底座
     ├─ shoulder_pan_joint        肩（绕基座旋转）
     ├─ shoulder_lift_joint       肩（抬臂）
     ├─ elbow_joint               肘
     ├─ wrist_1/2/3_joint         腕（三轴）
     │   └─ tool0                 腕法兰
     │       └─ inspire_hand_base 手掌（因时 RH56DFX 右手）
     │           ├─ thumb_1/2_joint        拇指（2 主动关节）
     │           ├─ index_1_joint          食指
     │           ├─ middle_1_joint         中指
     │           ├─ ring_1_joint           无名指
     │           └─ little_1_joint         小指
     └─ （每根手指第 2 节为 mimic 从动关节，自动跟随第 1 节）
```

## 四. 相关依赖：
`ros-humble-gazebo-ros2-control`、`ros-humble-ros2-controllers`、
`ros-humble-joint-state-publisher-gui`、`ros-humble-ros2controlcli`、`ros-humble-tf2-ros`。

## 5. 运行

```bash
# 完整 Gazebo 仿真（会自动打开 Gazebo 客户端 + RViz）
ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py

# 无界面服务器（适合远程/无显示环境）
ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py gui:=false use_rviz:=false

# 换臂型 / 换世界 / 不自动摆姿势
ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py ur_type:=ur10 world:=/path/to.world run_initial_pose:=false

# 纯 RViz 查看模型（不启动物理仿真，带关节滑块）
ros2 launch ur_inspire_hand ur_inspire_view.launch.py
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ur_type` | `ur5` | ur3 / ur5 / ur10 / ur3e / ur5e / ur7e / ur10e / ur12e / ur16e / ur20 / ur30 |
| `name` | `ur_inspire_hand` | 机器人/Gazebo 模型名 |
| `gui` | `true` | 是否启动 Gazebo 客户端 |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_sim_time` | `true` | 使用仿真时钟 |
| `world` | gazebo_ros 的 empty.world | Gazebo 世界文件 |
| `hand_xyz` / `hand_rpy` | 见 §7.1 | 灵巧手在 `tool0` 上的安装位姿 |
| `run_initial_pose` | `true` | 启动后自动摆出初始姿态 |

启动后机器人会：生成模型 → 加载 18 个关节（6 臂 + 12 手）→ 激活 3 个控制器 →
摆出初始姿态（臂回 home、手指张开）。

## 6. 控制示例

```bash
# 机械臂预设姿势（日常推荐：hand_preset.py 封装了下面的 FollowJointTrajectory 细节）
ros2 run ur_inspire_hand hand_preset.py arm home     # 回 home 位（与仿真启动一致）
ros2 run ur_inspire_hand hand_preset.py arm ready    # 工作准备位（shoulder_pan=0.6，即上面那条长命令）
ros2 run ur_inspire_hand hand_preset.py arm custom 0.6 -1.5708 0 -1.5708 0 0 3.0  # 自定义 6 臂关节 + 时长

# 手部预设姿势（日常推荐：hand_preset.py 封装了下面的 FollowJointTrajectory 细节）
ros2 run ur_inspire_hand hand_preset.py hand first        # 握拳（拇指 0.5/0.4，四指 1.0 rad）
ros2 run ur_inspire_hand hand_preset.py hand open        # 张开（全 0）
ros2 run ur_inspire_hand hand_preset.py hand pinch       # 拇指食指捏合
ros2 run ur_inspire_hand hand_preset.py hand point       # 食指伸出指点
ros2 run ur_inspire_hand hand_preset.py hand ok          # OK 手势
ros2 run ur_inspire_hand hand_preset.py hand first 1.0    # 指定 1 秒完成
ros2 run ur_inspire_hand hand_preset.py hand custom 0.5 0.4 1.0 1.0 1.0 1.0 3.0   # 自定义 6 角度 + 时长
# （不带 hand 前缀的 hand_preset.py first 等老写法仍然兼容）

# 单指控制（日常推荐：hand_preset.py 封装了下面的 FollowJointTrajectory 细节）
ros2 run ur_inspire_hand hand_preset.py hand finger index 1.2      # 食指弯到 1.2 rad（其余保持原状）
ros2 run ur_inspire_hand hand_preset.py hand finger thumb_1 0.5    # 拇指第 1 节弯到 0.5
ros2 run ur_inspire_hand hand_preset.py hand finger middle 1.0 1.0 # 中指弯到 1.0，1 秒完成
# 关节名支持简写 thumb_1 thumb_2 index middle ring little，也可写完整名（如 right_index_1_joint）
# 依赖 hand_controller 的 allow_partial_joints_goal，可只列一个关节下发

# 底层 FollowJointTrajectory action（参考：hand_preset.py 本质上就是发这些；单指即只列一个关节）
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.6, -1.5708, 0.0, -1.5708, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
ros2 action send_goal /hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [right_thumb_1_joint, right_thumb_2_joint, right_index_1_joint, right_middle_1_joint, right_ring_1_joint, right_little_1_joint], points: [{positions: [0.5, 0.4, 1.0, 1.0, 1.0, 1.0], time_from_start: {sec: 2}}]}}"
ros2 action send_goal /hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [right_index_1_joint], points: [{positions: [1.2], time_from_start: {sec: 1}}]}}"

# 查看状态
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

手指关节限位（主动关节）：`thumb_1 ∈ [0, 1.310]`、`thumb_2 ∈ [0, 0.523]`、四指 `∈ [0, 1.344]`（rad）。

### 6.1 实时可视化监视窗口（matplotlib）

`hand_preset.py` 除了发 action 让机器人动，还会把两组目标值分别发布到两个 topic：

| topic | 消息类型 | 内容 |
|---|---|---|
| `/hand_joint_targets` | `std_msgs/Float64MultiArray` | 6 个手关节目标角（rad），顺序 thumb_1, thumb_2, index, middle, ring, little |
| `/arm_joint_targets` | `std_msgs/Float64MultiArray` | 6 个臂关节目标角（rad），顺序 pan, lift, elbow, wrist_1, wrist_2, wrist_3 |

QoS 为 `transient_local`（latched）：后启动的订阅者也能立刻收到最后一次的值；
消息的 `layout.dim[0].label` 里带了命令名（如 `arm ready`），界面会显示它。

`hand_arm_monitor.py` 订阅这两个 topic 并实时绘图：

```bash
# 方式一：ros2 run（推荐调试时用）
ros2 run ur_inspire_hand hand_arm_monitor.py

# 方式二：launch
ros2 launch ur_inspire_hand hand_arm_monitor.launch.py

# 调试用可选参数
ros2 run ur_inspire_hand hand_arm_monitor.py --snapshot /tmp/mon.png   # 每次刷新存一张截图
ros2 run ur_inspire_hand hand_arm_monitor.py --duration 10             # 10 秒后自动退出
```

界面布局（中间一条虚线分隔）：

```
┌────────────────────────────────────────────────────────────────────┐
│  ARM (UR5) 侧视示意图 X-Z  │ 底盘俯视(看 shoulder_pan) │ 6 关节角度条 │
├────────────────────────── 虚线 ───────────────────────────────────┤
│  HAND 五指示意（看弯曲程度）                        │ 6 关节角度条     │
└────────────────────────────────────────────────────────────────────┘
```

- 上方：按 DH 参数做的 **UR5 正运动学**简图，6 段连杆用与 Gazebo 一致的**红/橙/黄/绿/蓝/紫**着色；
  右侧小圆盘是**底盘俯视**，箭头表示 `shoulder_pan` 的朝向（俯视才能看清绕 Z 的旋转）。
- 下方：**五指弯曲示意**，食指等四指的第 2 节按 mimic 系数 1.1169 自动跟随。
- 两侧角度条给出每个关节的**角度（°）与弧度（rad）**。
- 收到命令前，界面先用 `/joint_states` 的**真实值**填充（状态栏显示 `actual`），
  之后跟随命令变成目标值（显示命令名 + 时间）。
- 图内文字用英文，避免系统缺少中文字体时显示成方块；终端日志仍是中文。

实测验证：`bash downloads/test_monitor.sh`（`MPLBACKEND=Agg` 无头 + `ROS_DOMAIN_ID=91` 隔离），
直接向两个 topic 发值、以及用 `hand_preset.py hand first` / `arm home` 触发，界面均正确刷新并生成截图。

## 7. 技术要点

### 7.1 末端执行器替换（安装位姿）

因时手的安装板通过固定 joint 装到 UR 的 `tool0`：

```xml
<joint name="tool0_inspire_hand_joint" type="fixed">
  <origin xyz="-0.001867 0.014404 0.034255" rpy="1.5707963267948966 0 3.141592653589793"/>
  <parent link="tool0"/><child link="inspire_hand_base"/>
</joint>
```

该位姿经计算使**手腕板中心精确落在 tool0**，且手指朝前（+Z）、食指朝上、掌心朝右。可用 `hand_xyz` / `hand_rpy` launch 参数在线调整。

### 7.2 mimic 从动关节

RH56DFX 每根手指第 2 节由第 1 节带动（thumb_3 ← thumb_2 ×1.1425；thumb_4 ← thumb_3 ×0.7508；index/middle/ring/little_2 ← _1 ×1.1169）。

- **sdformat 9.7 不支持 URDF 的 `<mimic>` 标签**（会在 URDF→SDF 转换时被丢弃），因此改用 **ros2_control 的原生 mimic**：在 `<ros2_control>` 的 joint 条目里加
  `<param name="mimic">right_thumb_2_joint</param>` 和 `<param name="multiplier">1.1425</param>`，
  gazebo_ros2_control 加载时会打印 `Joint 'right_thumb_3_joint' is mimicking ...` 并自动按乘子计算从动命令。
- Humble 的 gazebo_ros2_control 会把 mimic 关节的状态接口命名为 `<joint>_mimic`，导致 `robot_state_publisher` 找不到关节；`mimic_joint_fixer`（C++ 节点）订阅 `/joint_states`、去掉 `_mimic` 后缀后转发到 `/joint_states_fixed` 供 RSP 使用（RSP 的 `joint_states` 已重 remap 到该话题）。

### 7.3 ⚠️ rcl 的 YAML 陷阱（务必了解）

gazebo_ros2_control 插件内部把 URDF 字符串拼成命令行参数规则交给 rcl 解析：

```
--param robot_description:=<?xml version="1.0" ?> ...
```

**rcl 会把 `:=` 后面的值按 YAML 语法解析。** URDF 的 XML 注释里只要出现 `': '`（冒号+空格，例如 `<!-- Commit Version: 1.6.0 -->`），YAML 就会把它当作 mapping 语法，整个解析失败：

```
[gzserver-2] [ERROR] [gazebo_ros2_control]: parser error
Couldn't parse parameter override rule: '--param robot_description:=<?xml version="1.0" ?>...'
```

后果：controller_manager 创建失败，所有 spawner 一直等待。

**解决办法**：`launch/ur_inspire_gazebo.launch.py` 不直接调用 `xacro`，而是调用 `make_gazebo_urdf`（C++ 可执行文件）——它运行 xacro 后**剥掉所有 XML 注释**再输出。注释对 Gazebo 无语义（URDF→SDF 转换时本来就会丢弃），剥掉后 URDF 依然完全合法（多行内容会被 YAML 折叠成单行纯标量，XML 解析不受影响）。

> 推论：**以后给 URDF/xacro 加注释时，避免在注释里写 `': '`**；或者始终通过 `make_gazebo_urdf`（C++ 可执行文件）生成喂给 Gazebo 的 URDF。RViz 专用 launch（`ur_inspire_view.launch.py`）不走这条代码路径，注释无碍。

### 7.4 其他经验

- `launch` 的 `Command([...])` 会把列表元素**拼接成一个字符串再做 `shlex.split`**，所以含空格的参数值（如 `hand_xyz:="-0.001867 0.014404 0.034255"`）必须加引号。
- `ParameterValue(..., value_type=str)` 防止 launch_ros 把 URDF 字符串误当作 YAML 解析。
- ros2_control 的控制器**不会**随插件自动加载，必须用 `controller_manager spawner` 逐个激活（本工程在模型生成后 3 秒自动 spawn 三个控制器）。
- 出生姿态就是 ros2_control 的 `initial_value`（2026-10-27 实测确认）：臂用优傲官方 `initial_positions.yaml`（shoulder_lift / wrist_1 = -1.57，接近 home），手部全 0（自然张开）。`initial_pose`（C++ 节点）在控制器就绪后再发一次 home + 张手，防止控制器加载前关节漂移，同时验证 action 链路可用。
- `right_thumb_4_joint` 的上限从原始值 0.430866 放宽到 0.55 rad，避免 mimic 乘子导致的目标超限。
- 左手模型（`downloads/inspire_official_left/`）已随官方 RAR 解压，可按右手的方式扩展集成。

### 7.5 ⚠️ 灵巧手"碎成一地"= 末端穿进地面

Gazebo 默认 world（`gazebo_ros/worlds/empty.world`）里有一块 **z=0 的无限地面**。
机械臂摆得太平（典型：`shoulder_lift = 0`，整条臂水平，肩肘高度掉到 0.089 m）时，
末端会低于地面，手掌实体穿进地面 → 接触力把关节约束顶开 / 手掌被地面遮挡，
视觉上就是"灵巧手稀碎"。

实测（本工程 UR5 + 右手的安装位姿）：

| 臂关节 `[pan, lift, elbow, w1, w2, w3]` | 手掌安装面 z | 手掌最低点（估算） |
|---|---|---|
| `[1.3, 0, 0, -0.1, 0, 0]` | +0.0095 m | **-0.036 m（穿地 ❌）** |
| `[1.3, -0.05, 0, -0.1, 0, 0]` | +0.051 m | +0.006 m（临界 ⚠） |
| `[1.3, -0.1, 0, -0.1, 0, 0]` | +0.093 m | **+0.048 m ✅（实测无问题）** |
| `[1.3, -0.2, 0, -0.1, 0, 0]` | +0.175 m | +0.130 m ✅ |
| `[1.3, -0.4, 0, -0.1, 0, 0]` | +0.338 m | +0.293 m ✅ |
| `[0, -1.5708, 0, -1.5708, 0, 0]`（home） | +0.987 m | +0.942 m ✅ |

经验值：`shoulder_lift` 只要不高于 **-0.05 rad**（手掌最低点约 ±0）就安全；
**推荐 ≤ -0.1 rad**，能留约 5 cm 余量（2026-10-31 用户实测 `-0.1` 正常，`0` 会炸）。

**注意：这不是"手的 z 不能为负"** —— URDF 里对手没有世界坐标约束，手指尖在该姿态下其实朝上
（z≈+0.11~0.21 m），真正撞地的是**手掌根部 / 腕部转接板**。

三种处理方式：

1. **抬高姿态**（最简单）：`shoulder_lift` 取更负的值，推荐 **≤ -0.1 rad**，例如
   `hand_preset.py arm custom 1.3 -0.1 0 -0.1 0 0 2`（手掌约高 0.09 m，实测可用）。
2. **换一个没有地面的 world**（要贴地/桌面以下作业时）：
   `ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py world:=/path/to/no_ground.world`。
3. **工具强制拦截**：`hand_preset.py` 在发送臂目标前会检查 `shoulder_lift_joint`
   ——**高于 -0.1 rad 直接报错拒绝执行**（不下发目标，机器人保持原位），报错信息里写明
   当前角度、原因和改法；另外用正运动学估算手掌高度，低于 2 cm 再补一条警告。

## 8. 验证结果（2026-10-25，WSL 无头测试）

`bash downloads/test_headless.sh`（使用独立 `ROS_DOMAIN_ID=77` 与 `GAZEBO_MASTER_URI=11346`，避免干扰其他仿真）：

| 检查项 | 结果 |
|---|---|
| xacro 展开 | ✅ 1481 行 / 49 links / 66 joints |
| 模型生成 | ✅ `Successfully spawned entity [ur_inspire_hand]` |
| 控制器 | ✅ joint_state_broadcaster / arm_controller / hand_controller 全部 active |
| 18 关节加载 + mimic 注册 | ✅ 6 条 `is mimicking joint ... with multiplier` 日志 |
| 初始姿态（initial_pose） | ✅ 臂回 home（shoulder_lift/wrist_1 = -1.5708），手指全张 |
| 握拳目标（拇指 0.5/0.4，四指 1.0） | ✅ SUCCEEDED |
| **mimic 联动精度** | ✅ thumb_3=0.4572（期望 0.4568）、四指_2=1.1169（期望 1.1169），误差 < 0.0005 rad |
| 张手 / 转臂目标 | ✅ SUCCEEDED（shoulder_pan=0.6） |
| TF world→tool0 | ✅ 正常发布 |
| launch 日志 error | ✅ 无 |

2026-10-27 补充验证（`bash downloads/verify_hand_preset.sh`，同样隔离 domain/端口，`run_initial_pose:=false`）：

| 检查项 | 结果 |
|---|---|
| `hand_preset.py first` | ✅ SUCCEEDED；thumb_1=0.4999 / thumb_2=0.3999 / 四指 _1=1.0000 |
| mimic 联动 | ✅ thumb_3=0.4572（期望 0.4570）、thumb_4=0.3429、四指 _2=1.1169 |
| `open` / `custom 0.2 0.1 1.2 0 0 0 1.5` | ✅ SUCCEEDED |
| 不存在的预设名 | ✅ 打印用法并返回 1 |
| 出生姿态 | ✅ = initial_value（肩/腕 -1.57，手指全 0），修正了 §7.4 旧结论 |

目录结构：
```
ros_urrobot_inspire_hand/          ← colcon 工作区根目录
├── src/
│   ├── Universal_Robots_ROS2_Description/   优傲官方描述包（GitHub humble 分支）
│   └── ur_inspire_hand/            本工程的核心包
│       ├── urdf/
│       │   ├── ur_inspire.urdf.xacro        组合机器人主文件（臂 + 手 + ros2_control）
│       │   └── inc/inspire_hand.urdf.xacro  因时右手（改造自社区 URDF）
│       ├── launch/
│       │   ├── ur_inspire_gazebo.launch.py  Gazebo 仿真（主入口）
│       │   ├── ur_inspire_view.launch.py    纯 RViz 查看（无仿真）
│       │   └── hand_arm_monitor.launch.py   matplotlib 关节监视窗口
│       ├── src/                             C++ 可执行文件（ros2 run 调用）
│       │   ├── make_gazebo_urdf.cpp         xacro + 剥注释（关键！见 §7.3）
│       │   ├── mimic_joint_fixer.cpp        修正 mimic 关节名后缀
│       │   └── initial_pose.cpp             启动后把机器人摆到初始姿态
│       ├── scripts/
│       │   ├── hand_preset.py               预设姿势速发 + 发布目标 topic（hand/arm 两域）
│       │   └── hand_arm_monitor.py          matplotlib 实时可视化（订阅 hand/arm 两个 topic）
│       ├── config/ur_inspire_controllers.yaml  控制器配置
│       ├── meshes/                          37 个 STL（源自因时官方 CAD）
│       └── rviz/ur_inspire.rviz
├── downloads/                     官方 RAR、解压的 STEP、测试脚本
│   ├── test_headless.sh           无头功能测试（隔离运行，不干扰其他仿真）
│   ├── verify_mimic.py            mimic 联动比例校验
│   └── verify_hand_preset.sh      hand_preset.py 端到端验证（配 dump_joints.py）
└── build/ install/ log/           colcon 产物（符号链接农场安装）
```