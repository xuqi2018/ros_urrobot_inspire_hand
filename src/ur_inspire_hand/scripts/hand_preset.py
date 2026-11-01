#!/usr/bin/env python3
"""hand_preset.py - 用预设名字控制 UR 臂 + 灵巧手，并把 6 个目标值发布到 topic。

用法:
  手部 (hand 域):
    ros2 run ur_inspire_hand hand_preset.py hand open      # 全部张开
    ros2 run ur_inspire_hand hand_preset.py hand first     # 握拳
    ros2 run ur_inspire_hand hand_preset.py hand pinch     # 拇指食指捏合
    ros2 run ur_inspire_hand hand_preset.py hand point     # 食指伸出指点
    ros2 run ur_inspire_hand hand_preset.py hand ok        # OK 手势
    ros2 run ur_inspire_hand hand_preset.py hand first 1.0  # 指定 1 秒完成
    ros2 run ur_inspire_hand hand_preset.py hand custom 0.5 0.4 1.0 1.0 1.0 1.0 3.0
        # 自定义 6 个主动关节角度 (rad) + 时长 (s)
    ros2 run ur_inspire_hand hand_preset.py hand finger index 1.2
        # 单指控制：食指弯到 1.2 rad（其余手指保持原状）
    ros2 run ur_inspire_hand hand_preset.py hand finger thumb_1 0.5
        # 拇指第 1 节弯到 0.5（也支持不带 hand 前缀的 finger ...）

  机械臂 (arm 域):
    ros2 run ur_inspire_hand hand_preset.py arm home   # 回 home 位 (与仿真启动一致)
    ros2 run ur_inspire_hand hand_preset.py arm ready  # 工作准备位 (shoulder_pan=0.6)
    ros2 run ur_inspire_hand hand_preset.py arm home 2.0  # 指定 2 秒完成
    ros2 run ur_inspire_hand hand_preset.py arm custom 0.6 -1.5708 0 -1.5708 0 0 3.0
        # 自定义 6 个臂关节 (pan lift elbow w1 w2 w3) + 时长 (s)

发布的 topic (给可视化界面用):
    /hand_joint_targets   std_msgs/Float64MultiArray, 6 个手关节目标角 (rad)
    /arm_joint_targets    std_msgs/Float64MultiArray, 6 个臂关节目标角 (rad)
    QoS 为 transient_local (latched)，界面后启动也能立刻看到最后一次的目标值。

手部关节顺序: thumb_1, thumb_2, index_1, middle_1, ring_1, little_1
手部限位: thumb_1 <= 1.31, thumb_2 <= 0.523, 四指 <= 1.344
臂关节顺序: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3

安全约束: shoulder_lift_joint 必须 <= -0.1 rad。高于该值手臂接近水平，
灵巧手会穿进 Gazebo 的 z=0 地面导致模型散架，此时命令会被拒绝（详见 README §7.5）。
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectoryPoint

HAND_ACTION = "/hand_controller/follow_joint_trajectory"
ARM_ACTION = "/arm_controller/follow_joint_trajectory"

# 可视化界面订阅的两个 topic
HAND_TOPIC = "/hand_joint_targets"
ARM_TOPIC = "/arm_joint_targets"

HAND_JOINTS = [
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_index_1_joint",
    "right_middle_1_joint",
    "right_ring_1_joint",
    "right_little_1_joint",
]

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 手部预设姿势 (rad)，均在关节限位内，可自行增删
HAND_PRESETS = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "first": [0.5, 0.4, 1.0, 1.0, 1.0, 1.0],  # 握拳
    "pinch": [0.5, 0.4, 0.6, 1.0, 1.0, 1.0],
    "point": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    "ok":    [0.6, 0.5, 0.6, 0.0, 0.0, 0.0]
}

# 机械臂预设姿势 (rad)。home 与仿真启动时 initial_pose 发送的位姿一致；
# ready 是 home 基础上 shoulder_pan 转到 0.6 的"工作准备位"。
ARM_PRESETS = {
    "home": [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
    "ready": [0.6, -1.5708, 0.0, -1.5708, 0.0, 0.0],
}

# 单指快捷名 -> 完整关节名（hand_controller 已开 allow_partial_joints_goal，可只列一个关节）
FINGER_ALIASES = {
    "thumb_1": "right_thumb_1_joint",
    "thumb_2": "right_thumb_2_joint",
    "index": "right_index_1_joint",
    "middle": "right_middle_1_joint",
    "ring": "right_ring_1_joint",
    "little": "right_little_1_joint",
}

# 单指关节限位 (rad)，仅用于超出时给一句警告，不阻断下发
FINGER_LIMITS = {
    "right_thumb_1_joint": (0.0, 1.310),
    "right_thumb_2_joint": (0.0, 0.523),
    "right_index_1_joint": (0.0, 1.344),
    "right_middle_1_joint": (0.0, 1.344),
    "right_ring_1_joint": (0.0, 1.344),
    "right_little_1_joint": (0.0, 1.344),
}

# 不同域的默认运动时长 (s)
DEFAULT_DURATION = {"hand": 2.0, "arm": 3.0}

# ---- 末端离地检查 ---------------------------------------------------------
# Gazebo 默认 world（gazebo_ros 的 empty.world）里有一块 z=0 的无限地面。
# 机械臂摆得太平（如 shoulder_lift=0）时，手掌会穿进地面，Gazebo 的接触力会
# 把模型的关节约束顶开，看起来就是"灵巧手碎成一地"。这里用正运动学估算手掌
# 高度，低于阈值就提示（只警告，不阻断）。
UR5_DH = [  # (d, a, alpha)，UR5 标准 DH
    (0.089159, 0.0, 1.5707963267948966),
    (0.0, -0.425, 0.0),
    (0.0, -0.39225, 0.0),
    (0.10915, 0.0, 1.5707963267948966),
    (0.09465, 0.0, -1.5707963267948966),
    (0.0823, 0.0, 0.0),
]
HAND_MOUNT_XYZ = (-0.001867, 0.014404, 0.034255)  # tool0 -> inspire_hand_base
HAND_HALF_THICKNESS = 0.045   # 掌心/腕部实体厚度的一半（估算值）
GROUND_CLEARANCE = 0.02       # 最低点离地小于这个值就警告
# shoulder_lift_joint 的安全下限：再往上（更接近 0）手臂就接近水平，手掌会贴地/穿地。
# 实测（2026-10-31）：lift=0 时模型会碎开，lift=-0.1 正常，-0.05 只剩 6 mm 余量。
LIFT_SAFE_MIN = -0.1


def _mat4_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def hand_base_height(arm_positions):
    """给定 6 个臂关节角，返回手掌安装面中心的世界 z（m）。"""
    t = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for qi, (d, a, alpha) in zip(arm_positions, UR5_DH):
        t = _mat4_mul(t, _dh_transform(qi, d, a, alpha))
    # 手腕到手的安装变换：rpy=(pi/2, 0, pi)，只影响姿态，平移直接叠加即可
    return t[2][3] + HAND_MOUNT_XYZ[2] * t[2][2] + HAND_MOUNT_XYZ[1] * t[2][1] \
        + HAND_MOUNT_XYZ[0] * t[2][0]


def check_arm_safety(node, arm_positions):
    """发送臂目标前的安全检查。

    返回 True = 允许下发；False = 拦下（已打印错误信息）。
    规则：shoulder_lift_joint 必须 <= LIFT_SAFE_MIN(-0.1 rad)，
    否则手臂接近水平，灵巧手会穿进 Gazebo 的 z=0 地面。
    """
    lift = arm_positions[1]
    z = hand_base_height(arm_positions)
    lowest = z - HAND_HALF_THICKNESS

    if lift > LIFT_SAFE_MIN:
        node.get_logger().error(
            "拒绝执行：shoulder_lift_joint = %.3f rad (%.1f°) 高于安全下限 "
            "%.2f rad (%.1f°)。\n"
            "  原因：该角度下手臂接近水平，灵巧手会穿进 Gazebo 的 z=0 地面"
            "（估算手掌最低点 z=%+.3f m），接触力会把关节约束顶开，模型会碎成一地。\n"
            "  改法：把第 2 个参数（shoulder_lift）调小（更负），推荐 <= -0.1，例如\n"
            "    ros2 run ur_inspire_hand hand_preset.py arm custom 1.3 -0.1 0 -0.1 0 0 2\n"
            "  详见 README §7.5。未发送任何目标，机器人保持原位。"
            % (lift, math.degrees(lift), LIFT_SAFE_MIN,
               math.degrees(LIFT_SAFE_MIN), lowest)
        )
        return False

    if lowest < GROUND_CLEARANCE:
        node.get_logger().warn(
            "⚠ 该姿态下手掌估算最低点 z=%.3f m，已接近地面（z=0），"
            "若模型出现抖动/散架，请抬高手臂（减小 shoulder_lift）。" % lowest)
    return True

# topic 用的 latch QoS：后启动的界面也能拿到最后一次目标
LATCH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def parse_args(argv):
    """Return (action_name, joint_names, positions, duration, label), or None."""
    if not argv:
        return None

    # 兼容老写法：裸 finger ... 视作 hand finger ...
    if argv[0] == "finger":
        argv = ["hand", "finger"] + list(argv[1:])
    # 兼容老写法：裸 open/first/pinch/point/ok 视作 hand ...
    elif argv[0] in HAND_PRESETS:
        argv = ["hand"] + list(argv)

    # ---- 单指控制（hand 域子命令） ----
    if argv[0] == "hand" and len(argv) > 1 and argv[1] == "finger":
        rest = argv[2:]
        if len(rest) < 2:
            print("finger 需要: hand finger <关节名> <角度> [秒]")
            print("  关节名支持简写: thumb_1 thumb_2 index middle ring little")
            print("  也可写完整名，如 right_index_1_joint")
            return None
        name_in = rest[0]
        full = FINGER_ALIASES.get(name_in, name_in)
        if full not in HAND_JOINTS:
            print("未知关节 '%s'（支持: %s 或完整名）"
                  % (name_in, " ".join(FINGER_ALIASES)))
            return None
        try:
            pos = float(rest[1])
        except ValueError:
            print("角度必须是数字")
            return None
        duration = float(rest[2]) if len(rest) > 2 else DEFAULT_DURATION["hand"]
        lo, hi = FINGER_LIMITS[full]
        if pos < lo or pos > hi:
            print("⚠ 角度 %.3f 超出 %s 的限位 [%.3f, %.3f]，"
                  "控制器将夹取到边界" % (pos, name_in, lo, hi))
        return (HAND_ACTION, [full], [pos], duration,
                "hand finger %s" % name_in)

    # ---- 机械臂域 ----
    if argv[0] == "arm":
        rest = argv[1:]
        if not rest:
            return None
        sub = rest[0]
        if sub == "custom":
            if len(rest) < 7:
                print("arm custom 需要 6 个臂关节角度: "
                      "arm custom pan lift elbow w1 w2 w3 [秒]")
                return None
            try:
                positions = [float(v) for v in rest[1:7]]
            except ValueError:
                print("角度必须是数字")
                return None
            duration = float(rest[7]) if len(rest) > 7 else DEFAULT_DURATION["arm"]
            return ARM_ACTION, ARM_JOINTS, positions, duration, "arm custom"
        if sub not in ARM_PRESETS:
            return None
        positions = list(ARM_PRESETS[sub])
        duration = float(rest[1]) if len(rest) > 1 else DEFAULT_DURATION["arm"]
        return ARM_ACTION, ARM_JOINTS, positions, duration, "arm %s" % sub

    # ---- 手部域 ----
    if argv[0] == "hand":
        rest = argv[1:]
        if not rest:
            return None
        sub = rest[0]
        if sub == "custom":
            if len(rest) < 7:
                print("custom 需要 6 个角度: hand custom t1 t2 i m r l [秒]")
                return None
            try:
                positions = [float(v) for v in rest[1:7]]
            except ValueError:
                print("角度必须是数字")
                return None
            duration = float(rest[7]) if len(rest) > 7 else DEFAULT_DURATION["hand"]
            return HAND_ACTION, HAND_JOINTS, positions, duration, "hand custom"
        if sub not in HAND_PRESETS:
            return None
        positions = list(HAND_PRESETS[sub])
        duration = float(rest[1]) if len(rest) > 1 else DEFAULT_DURATION["hand"]
        return HAND_ACTION, HAND_JOINTS, positions, duration, "hand %s" % sub

    return None


def read_current_positions(node, joint_names, timeout=1.0):
    """从 /joint_states 读一组关节的当前值，用于把"单指"命令补成完整 6 维。"""
    found = {}

    def _cb(msg):
        for name, pos in zip(msg.name, msg.position):
            if name in joint_names:
                found[name] = pos

    sub = node.create_subscription(JointState, "/joint_states", _cb, 10)
    deadline = time.time() + timeout
    while time.time() < deadline and len(found) < len(joint_names):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return [float(found.get(n, 0.0)) for n in joint_names]


def publish_target(node, pub, topic, label, order, joint_names, positions):
    """把目标补成完整 6 维并发布，供可视化界面显示。"""
    full_values = list(positions)
    if len(joint_names) < len(order):
        # 单指控制：其余关节取当前实际值，避免界面上其它手指被"归零"
        full_values = read_current_positions(node, order, timeout=1.0)
        index_of = {name: i for i, name in enumerate(order)}
        for name, pos in zip(joint_names, positions):
            full_values[index_of[name]] = pos

    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension(label=label, size=len(full_values), stride=1)]
    msg.data = [float(v) for v in full_values]
    pub.publish(msg)
    node.get_logger().info("已发布 %s: %s"
                           % (topic, ["%.3f" % v for v in full_values]))


def main():
    parsed = parse_args(sys.argv[1:])
    if parsed is None:
        print(__doc__)
        return 1
    action_name, joint_names, positions, duration, label = parsed
    order = HAND_JOINTS if action_name == HAND_ACTION else ARM_JOINTS
    topic = HAND_TOPIC if action_name == HAND_ACTION else ARM_TOPIC

    rclpy.init()
    node = Node("hand_preset")
    client = ActionClient(node, FollowJointTrajectory, action_name)
    pub = node.create_publisher(Float64MultiArray, topic, LATCH_QOS)

    node.get_logger().info("等待 %s ..." % action_name)
    if action_name == ARM_ACTION:
        # 臂命令一定是完整 6 维，直接拿 positions 做安全检查
        if not check_arm_safety(node, positions):
            rclpy.shutdown()
            return 1
    publish_target(node, pub, topic, label, order, joint_names, positions)

    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(
            "等不到 %s，Gazebo 仿真启动了吗？（目标值已发布到 %s，界面仍可看到）"
            % (action_name, topic)
        )
        rclpy.shutdown()
        return 1

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(seconds=duration).to_msg()
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names
    goal.trajectory.points = [point]

    node.get_logger().info(
        "发送目标 (%.1fs): %s" % (duration, ["%.3f" % p for p in positions])
    )
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("目标被拒绝（%s 加载了吗？）" % action_name)
        rclpy.shutdown()
        return 1

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    status = result_future.result().status
    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info("完成 SUCCEEDED")
        rclpy.shutdown()
        return 0
    node.get_logger().error("结束但 status=%d（非 SUCCEEDED）" % status)
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
