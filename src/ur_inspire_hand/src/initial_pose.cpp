// Move the combined robot to a nice starting pose once the controllers are up.
//
// C++ replacement for the former scripts/initial_pose.py.
//
// - arm_controller: home pose from ur_description's config/initial_positions.yaml
//   (shoulder_pan 0, shoulder_lift -1.57, elbow 0, wrist_1 -1.57, wrist_2/3 0)
// - hand_controller: fully open hand (all actuated finger joints to 0)
//
// Without this node the robot would spawn in the all-zero joint configuration
// (Gazebo ignores the ros2_control initial_value entries for the initial sim
// pose), which looks like a stiff "reaching forward" pose.

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// Insertion order matters: it defines the joint order of the trajectory,
// exactly like the ordered dicts in the Python version.
const std::vector<std::pair<std::string, double>> kArmHome = {
    {"shoulder_pan_joint", 0.0},
    {"shoulder_lift_joint", -M_PI / 2.0},
    {"elbow_joint", 0.0},
    {"wrist_1_joint", -M_PI / 2.0},
    {"wrist_2_joint", 0.0},
    {"wrist_3_joint", 0.0},
};

const std::vector<std::pair<std::string, double>> kHandOpen = {
    {"right_thumb_1_joint", 0.0},
    {"right_thumb_2_joint", 0.0},
    {"right_index_1_joint", 0.0},
    {"right_middle_1_joint", 0.0},
    {"right_ring_1_joint", 0.0},
    {"right_little_1_joint", 0.0},
};

constexpr double kMoveTimeS = 3.0;

FollowJointTrajectory::Goal MakeGoal(
    const std::vector<std::pair<std::string, double>>& pose) {
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names.reserve(pose.size());
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.reserve(pose.size());
  for (const auto& entry : pose) {
    goal.trajectory.joint_names.push_back(entry.first);
    point.positions.push_back(entry.second);
  }
  point.time_from_start = rclcpp::Duration::from_seconds(kMoveTimeS);
  goal.trajectory.points.push_back(point);
  return goal;
}

class InitialPose : public rclcpp::Node {
public:
  InitialPose()
      : Node("initial_pose"),
        arm_client_(rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory")),
        hand_client_(rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/hand_controller/follow_joint_trajectory")) {
    // Wall timer: fires in real time.  The Python version used a ROS-time
    // timer (use_sim_time), but since it merely polls for the action servers
    // the two are interchangeable here.
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               std::bind(&InitialPose::try_send, this));
    done_ = false;
    RCLCPP_INFO(get_logger(), "initial_pose node started, waiting for controllers...");
  }

private:
  void try_send() {
    if (done_) {
      return;
    }
    if (!(arm_client_->wait_for_action_server(std::chrono::milliseconds(500)) &&
          hand_client_->wait_for_action_server(std::chrono::milliseconds(500)))) {
      return;
    }
    done_ = true;
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Controllers are up, sending initial poses");

    // The Python version discarded the futures from send_goal_async; the
    // futures are kept here only so the pending goals stay referenced.
    arm_goal_future_ = arm_client_->async_send_goal(MakeGoal(kArmHome));
    hand_goal_future_ = hand_client_->async_send_goal(MakeGoal(kHandOpen));
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr hand_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool done_;
  std::shared_future<GoalHandle::SharedPtr> arm_goal_future_;
  std::shared_future<GoalHandle::SharedPtr> hand_goal_future_;
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPose>());
  rclcpp::shutdown();
  return 0;
}
