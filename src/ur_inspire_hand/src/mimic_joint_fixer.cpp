// Republish /joint_states with mimic joint names fixed.
//
// C++ replacement for the former scripts/mimic_joint_fixer.py.
//
// gazebo_ros2_control exposes the Inspire hand's mimic joints with a "_mimic"
// suffix on their interface names (e.g. right_index_2_joint_mimic), so the
// joint_state_broadcaster publishes them under those suffixed names.
// robot_state_publisher then cannot match them to the URDF joints and the
// mimic joints would stay at their URDF default in TF/RViz.
//
// This node simply strips the "_mimic" suffix and republishes on
// /joint_states_fixed. The launch files remap robot_state_publisher's
// joint_states subscription to that topic.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MimicJointFixer : public rclcpp::Node {
public:
  MimicJointFixer() : Node("mimic_joint_fixer") {
    // Same QoS as the Python version (depth 10, reliable, volatile).
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&MimicJointFixer::joint_states_callback, this, std::placeholders::_1));
    publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states_fixed", 10);
    RCLCPP_INFO(get_logger(),
                "mimic_joint_fixer started: /joint_states -> /joint_states_fixed");
  }

private:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    sensor_msgs::msg::JointState fixed;
    fixed.header = msg->header;
    fixed.name.reserve(msg->name.size());
    fixed.position.reserve(msg->position.size());
    fixed.velocity.reserve(msg->velocity.size());
    fixed.effort.reserve(msg->effort.size());

    static const std::string suffix = "_mimic";
    for (std::size_t idx = 0; idx < msg->name.size(); ++idx) {
      const std::string& name = msg->name[idx];
      // Python: name[:-len("_mimic")] if name.endswith("_mimic") else name
      if (name.size() >= suffix.size() &&
          name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
        fixed.name.push_back(name.substr(0, name.size() - suffix.size()));
      } else {
        fixed.name.push_back(name);
      }
      if (idx < msg->position.size()) {
        fixed.position.push_back(msg->position[idx]);
      }
      if (idx < msg->velocity.size()) {
        fixed.velocity.push_back(msg->velocity[idx]);
      }
      if (idx < msg->effort.size()) {
        fixed.effort.push_back(msg->effort[idx]);
      }
    }
    publisher_->publish(fixed);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // rclcpp::spin returns on SIGINT, mirroring the Python
  // try/except KeyboardInterrupt -> shutdown behaviour.
  rclcpp::spin(std::make_shared<MimicJointFixer>());
  rclcpp::shutdown();
  return 0;
}
