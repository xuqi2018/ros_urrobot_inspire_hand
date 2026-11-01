cmake_minimum_required(VERSION 3.8)
project(ur_inspire_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# ---- C++ helper executables (installed as ros2 run executables) ----------
add_executable(make_gazebo_urdf src/make_gazebo_urdf.cpp)

add_executable(mimic_joint_fixer src/mimic_joint_fixer.cpp)
ament_target_dependencies(mimic_joint_fixer
  rclcpp
  sensor_msgs
)

add_executable(initial_pose src/initial_pose.cpp)
ament_target_dependencies(initial_pose
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
)

install(TARGETS
  make_gazebo_urdf
  mimic_joint_fixer
  initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

# ---- python nodes (ros2 run ur_inspire_hand <script>) -------------------
# hand_preset.py     : 预设姿势控制，并把 hand/arm 6 个目标值发布到 topic
# hand_arm_monitor.py: matplotlib 实时可视化（matplotlib 只有 Python 绑定）
install(PROGRAMS
  scripts/hand_preset.py
  scripts/hand_arm_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- resources -----------------------------------------------------------
install(DIRECTORY launch urdf meshes config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
