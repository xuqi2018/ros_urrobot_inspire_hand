#!/usr/bin/env bash
# 验证 hand_arm_monitor.py + hand_preset.py 的 topic 发布链路
# 无头模式（MPLBACKEND=Agg），独立 ROS_DOMAIN_ID，不干扰正在运行的 Gazebo。
set -o pipefail
cd /mnt/d/xq/findWork/ROS/ros_urrobot_inspire_hand || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=91
export MPLBACKEND=Agg
rm -f /tmp/hand_arm_shot.png /tmp/monitor.log

echo "=== 1) 启动 monitor（无头，12s 后自动退出）==="
ros2 run ur_inspire_hand hand_arm_monitor.py \
  --snapshot /tmp/hand_arm_shot.png --duration 12 > /tmp/monitor.log 2>&1 &
MPID=$!
sleep 4

echo "=== 2) 直接往两个 topic 发 6 个值 ==="
ros2 topic pub --once -w 1 /hand_joint_targets std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.4, 1.0, 1.0, 1.0, 1.0]}" --qos-durability transient_local >/dev/null 2>&1
sleep 1
ros2 topic pub --once -w 1 /arm_joint_targets std_msgs/msg/Float64MultiArray \
  "{data: [0.6, -1.5708, 0.0, -1.5708, 0.0, 0.0]}" --qos-durability transient_local >/dev/null 2>&1
sleep 1

echo "=== 3) 用 hand_preset.py 发目标（无 action server 也应先发布 topic）==="
ros2 run ur_inspire_hand hand_preset.py hand first > /tmp/preset_hand.log 2>&1
ros2 run ur_inspire_hand hand_preset.py arm home > /tmp/preset_arm.log 2>&1

wait $MPID
echo "=== monitor 日志 ==="
tail -n 20 /tmp/monitor.log
echo "=== hand_preset 日志 ==="
tail -n 3 /tmp/preset_hand.log
tail -n 3 /tmp/preset_arm.log
echo "=== 截图 ==="
ls -l /tmp/hand_arm_shot.png 2>&1
