#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Forwards partial joint_states from several source topics VERBATIM onto ONE
// target topic, with an EXPLICITLY RELIABLE publisher QoS.
//
// Why a node of our own instead of `topic_tools relay`:
// depending on its version, topic_tools relay publishes with
// best-effort/SensorDataQoS. move_group, however, subscribes to
// platform/joint_states as RELIABLE -> a best-effort publisher is NOT received
// there. The robot_state_publisher subscribes best-effort and does get the data
// -> TF/display fine, BUT MoveIt does not get the arm state -> planning fails
// (the classic symptom "state displayed correctly, planning fails"). One
// RELIABLE publisher serves BOTH: the reliable move_group and the best-effort
// robot_state_publisher.

class JointStateRelay : public rclcpp::Node
{
public:
  JointStateRelay() : Node("joint_state_relay")
  {
    // Relative names -> resolvable in the node namespace (/a200_0553).
    const std::vector<std::string> default_inputs = {
      "manipulators/joint_states",
      "manipulators/endeffectors/joint_states",
    };
    const auto inputs = this->declare_parameter("input_topics", default_inputs);
    const auto output = this->declare_parameter<std::string>("output_topic", "platform/joint_states");
    const int depth = this->declare_parameter<int>("depth", 20);

    // RELIABLE + VOLATILE + KEEP_LAST: compatible with the reliable move_group
    // subscriber AND the best-effort robot_state_publisher subscriber.
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(std::max(1, depth))));
    qos.reliable().durability_volatile();

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output, qos);
    for (const auto & topic : inputs) {
      subs_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(
        topic, qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { pub_->publish(*msg); }));
      RCLCPP_INFO(this->get_logger(), "relay %s -> %s", topic.c_str(), pub_->get_topic_name());
    }
  }

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
