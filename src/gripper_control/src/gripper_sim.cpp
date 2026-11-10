// gripper_sim: the simulation twin of a gripper WITHOUT hardware, for any model.
//
// AUTHORITATIVE is the surface every gripper of the stack offers -- on the real
// robot the RG6's rg6_grip_bridge, which commands it over XML-RPC at the
// OnRobot URCap.  What the mock reproduces of it is exactly that and no more:
//   action   <action_name> (control_msgs/GripperCommand), position = driver joint [rad]
//   topic    <state_topic> (flat JSON) plus the driver joint on joint_states
//
// A mock that reproduces a surface existing nowhere is a template for code
// written against something that does not exist.
//
// WHICH gripper is the caller's: the model's driver joint, its controller's
// action, its state topic and its linkage table (driver joint -> clear width)
// come in as parameters, read off the gripper model in the robot profile
// (clair.robot.contract, profiles/grippers/<model>.yaml) by whoever launches
// it.  Nothing here names a model.
//
// The driver joint on joint_states animates the model in RViz/Foxglove, which
// makes the MoveIt integration testable entirely without a robot.  The
// dependent joints hang on the driver via <mimic> and are derived by
// robot_state_publisher and move_group themselves.
//
// Motion model: the width travels towards the target width at a constant speed.
// With sim_object_width_m > 0 the closing stops at the object width ->
// grip_detected=true (like the tool DI0 signal of the real hardware).
//
// Plant mode (plant_command_topic set): a physics simulator moves the jaws, not
// this node.  The same ramp becomes the COMMAND, sent to a position controller
// of the driver joint (std_msgs/Float64MultiArray, position_controllers'
// JointGroupPositionController), and the width is what the plant measures on
// plant_state_topic.  A motion settles when the command has arrived and the
// measured joint has stood still for plant_settle_s; it stalled on something
// -- grip_detected -- when that happens short of a closing target by more than
// plant_grip_margin_m.  The plant publishes the driver joint itself, so this
// node does not.  Written for MARWIN 5's MuJoCo plant (deploy/marwin), whose
// RG6 is a force-limited servo in the physics; the same mode drives any hand a
// position controller moves, a real one included.
//
// The limit: this makes the gripper usable in the container, no more.  A real
// hand's pathologies stay uncovered (the RG6's AI2 sticks at 10 V on closed
// jaws, an injected grip tears ExternalControl off).  A success out of this
// node is marked as a non-hardware truth by the source field in /twin/result
// anyway.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_msgs/action/gripper_command.hpp"
#include "gripper_control/linkage.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

class GripperSimNode : public rclcpp::Node
{
public:
  GripperSimNode()
  : Node("gripper_sim")
  {
    // The model: its driver joint, its controller's action and state topic (relative to the node's namespace),
    // and its linkage table, driver joint [rad] ascending against the clear width [m] falling.
    driver_ = declare_parameter<std::string>("driver_joint", "");
    const auto action_name = declare_parameter<std::string>("action_name", "");
    const auto state_topic = declare_parameter<std::string>("state_topic", "");
    const auto qs = declare_parameter<std::vector<double>>("linkage_q_rad", std::vector<double>{});
    const auto ws = declare_parameter<std::vector<double>>("linkage_width_m", std::vector<double>{});
    if (driver_.empty() || action_name.empty()) {
      throw std::invalid_argument("gripper_sim needs driver_joint and action_name");
    }
    linkage_ = std::make_unique<gripper_control::Linkage>(qs, ws);
    // The default speed takes the model's full stroke in ~1 s, as the RG6 does.
    declare_parameter<double>("sim_speed_m_s", max_width_m());
    declare_parameter<double>("sim_object_width_m", 0.0);   // 0 = no object
    declare_parameter<double>("grip_default_force_n", 60.0);
    declare_parameter<double>("motion_timeout_s", 10.0);
    declare_parameter<double>("state_rate", 20.0);
    declare_parameter<double>("action_goal_angle_tol", 0.08);
    declare_parameter<std::string>("plant_command_topic", "");   // "" = the kinematic model above
    declare_parameter<std::string>("plant_state_topic", "/joint_states");
    declare_parameter<double>("plant_settle_s", 0.2);
    declare_parameter<double>("plant_still_rad", 0.002);
    declare_parameter<double>("plant_grip_margin_m", 0.003);

    width_ = max_width_m();
    target_width_ = width_;
    command_width_ = width_;

    blocking_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // The state as flat JSON, under the name the model's real driver uses as
    // well.  Without it the plan_server in the container reads into the void
    // -- the gripper state is then unanswerable on both rungs.
    if (!state_topic.empty()) {
      bridge_state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, rclcpp::QoS(10));
    }
    // 'joint_states' is relative -> put it on the wanted topic by a launch remap
    // (a200-0553: manipulators/endeffectors/joint_states).
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    const auto plant_command_topic = get_parameter("plant_command_topic").as_string();
    if (!plant_command_topic.empty()) {
      plant_ = true;
      command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(plant_command_topic, rclcpp::QoS(10));
      state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("plant_state_topic").as_string(), rclcpp::QoS(50),
        [this](const sensor_msgs::msg::JointState & msg) {
          for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
            if (msg.name[i] == driver_) {
              std::lock_guard<std::mutex> lk(mutex_);
              measured_angle_ = msg.position[i];
            }
          }
        });
      RCLCPP_INFO(get_logger(), "gripper sim in plant mode: commands on %s, width from %s",
        plant_command_topic.c_str(), get_parameter("plant_state_topic").as_string().c_str());
    }

    tick_timer_ = create_wall_timer(20ms, [this]() { tick(); });
    const double state_rate = get_parameter("state_rate").as_double();
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
      [this]() { publish_bridge_state(); });

    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this, action_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandleGripperCommand>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
        std::thread{[this, goal_handle]() { action_execute(goal_handle); }}.detach();
      },
      rcl_action_server_get_default_options(), blocking_cb_group_);

    RCLCPP_INFO(get_logger(),
      "gripper sim ready: %s, %.0f mm open, sim_object_width_m=%.3f",
      driver_.c_str(), width_ * 1000.0, get_parameter("sim_object_width_m").as_double());
  }

private:
  // --------- motion model (50 Hz tick) -------------------------------------
  void tick()
  {
    if (plant_) {
      plant_tick();
      return;
    }
    const double dt = 0.02;
    const double speed = get_parameter("sim_speed_m_s").as_double();
    const double object_w = get_parameter("sim_object_width_m").as_double();
    std::lock_guard<std::mutex> lk(mutex_);
    double effective_target = target_width_;
    // An object in the way? The closing stops at the object width -> grip detected.
    if (object_w > 0.0 && effective_target < object_w && width_ >= object_w) {
      effective_target = object_w;
    }
    const double delta = effective_target - width_;
    if (std::abs(delta) <= speed * dt) {
      width_ = effective_target;
      if (moving_) {
        moving_ = false;
        grip_detected_ = object_w > 0.0 && target_width_ < object_w &&
          std::abs(width_ - object_w) < 1e-6;
      }
    } else {
      width_ += (delta > 0 ? 1.0 : -1.0) * speed * dt;
      moving_ = true;
    }
    publish_joints_locked();
  }

  // Plant mode: ramp the COMMAND at sim_speed_m_s, read the width off the plant.
  void plant_tick()
  {
    const double dt = 0.02;
    const double speed = get_parameter("sim_speed_m_s").as_double();
    std::lock_guard<std::mutex> lk(mutex_);
    const double delta = target_width_ - command_width_;
    command_width_ = std::abs(delta) <= speed * dt ? target_width_ :
      command_width_ + (delta > 0 ? 1.0 : -1.0) * speed * dt;
    std_msgs::msg::Float64MultiArray command;
    command.data = {angle_from_width(command_width_)};
    command_pub_->publish(command);
    if (!std::isfinite(measured_angle_)) {
      return;
    }
    width_ = width_from_angle(measured_angle_);
    const bool still = std::abs(measured_angle_ - last_angle_) < get_parameter("plant_still_rad").as_double();
    last_angle_ = measured_angle_;
    still_s_ = still ? still_s_ + dt : 0.0;
    if (moving_ && command_width_ == target_width_ && still_s_ >= get_parameter("plant_settle_s").as_double()) {
      moving_ = false;
      grip_detected_ = target_width_ < width_ - get_parameter("plant_grip_margin_m").as_double();
    }
  }

  void publish_joints_locked()
  {
    // ONLY the driver joint -- exactly like rg6_grip_bridge on the real robot.
    //
    // The dependent joints (the rg6_v2's finger_joint_mirror and finger
    // links, the 2F-85's knuckles and tips) hang on the driver via <mimic>;
    // robot_state_publisher and move_group derive them themselves, a second
    // sender is superfluous.
    //
    // It would also be dangerous:  a RobotState carrying a joint name the model
    // does not know CRASHES move_group through RobotModel::getVariableIndex
    // (std::terminate, SIGABRT -- staged and reproduced twice).  Every consumer
    // that reads joint_states and hands it back into a MoveIt request would be a
    // launch pad for that.
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = {driver_};
    msg.position = {angle_from_width(width_)};
    joint_pub_->publish(msg);
  }

  // Width <-> driver joint: the model's linkage table (gripper_control/linkage.hpp).
  double max_width_m() const {return linkage_->max_width_m();}
  double min_width_m() const {return linkage_->min_width_m();}
  double width_from_angle(double q) const {return linkage_->width_from_angle(q);}
  double angle_from_width(double width_m) const {return linkage_->angle_from_width(width_m);}

  struct MotionResult
  {
    bool settled{false};
    bool grip_detected{false};
    double final_width_m{kNaN};
  };

  MotionResult start_motion_and_wait(
    double target_width_m, const std::function<bool()> & canceled = nullptr)
  {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      target_width_ = std::clamp(target_width_m, min_width_m(), max_width_m());
      grip_detected_ = false;
      moving_ = true;
      still_s_ = 0.0;  // plant mode: stillness counts from this command on
    }
    const auto timeout = rclcpp::Duration::from_seconds(
      get_parameter("motion_timeout_s").as_double());
    const auto start = now();
    MotionResult result;
    while (rclcpp::ok() && (now() - start) < timeout) {
      if (canceled && canceled()) {
        break;
      }
      std::this_thread::sleep_for(20ms);
      std::lock_guard<std::mutex> lk(mutex_);
      if (!moving_) {
        result.settled = true;
        result.grip_detected = grip_detected_;
        result.final_width_m = width_;
        break;
      }
    }
    if (!result.settled) {
      std::lock_guard<std::mutex> lk(mutex_);
      result.final_width_m = width_;
      result.grip_detected = grip_detected_;
    }
    return result;
  }

  // --------- GripperCommand action -----------------------------------------
  void action_execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    const auto goal = goal_handle->get_goal();

    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      {
        std::lock_guard<std::mutex> lk(mutex_);
        result->position = angle_from_width(width_);
      }
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_command_ = "GRIP";
    }

    std::atomic<bool> feedback_running{true};
    std::thread feedback_thread([this, goal_handle, &feedback_running]() {
        while (feedback_running && rclcpp::ok()) {
          auto fb = std::make_shared<GripperCommand::Feedback>();
          {
            std::lock_guard<std::mutex> lk(mutex_);
            fb->position = angle_from_width(width_);
          }
          goal_handle->publish_feedback(fb);
          std::this_thread::sleep_for(100ms);
        }
      });

    const double target_width = width_from_angle(goal->command.position);
    const auto motion = start_motion_and_wait(
      target_width, [goal_handle]() { return goal_handle->is_canceling(); });

    feedback_running = false;
    feedback_thread.join();

    result->position = angle_from_width(
      std::isfinite(motion.final_width_m) ? motion.final_width_m : target_width);
    result->effort = motion.grip_detected ?
      (goal->command.max_effort > 0.0 ? goal->command.max_effort :
      get_parameter("grip_default_force_n").as_double()) : 0.0;
    result->stalled = motion.grip_detected;
    result->reached_goal = motion.settled &&
      std::fabs(result->position - goal->command.position) <=
      get_parameter("action_goal_angle_tol").as_double();

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (motion.settled) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // --------- state publisher ------------------------------------------------
  // The fields of rg6_grip_bridge.status_payload(), character for character --
  // whoever renames something here makes the container a special case.  A
  // model without a state topic gets none.
  //
  // "status" and "safety_failed" come from rg_get_status / rg_get_safety_failed
  // on the device.  The sim has no fault to report and therefore writes
  // 0 / false; that is not a claim about hardware but the honest statement
  // "in this simulation there is nothing to disturb".
  void publish_bridge_state()
  {
    if (!bridge_state_pub_) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "{\"width_m\": " << width_
       << ", \"busy\": " << (moving_ ? "true" : "false")
       << ", \"grip_detected\": " << (grip_detected_ ? "true" : "false")
       << ", \"status\": 0"
       << ", \"safety_failed\": false"
       << ", \"last_command\": \"" << last_command_ << "\"}";
    std_msgs::msg::String out;
    out.data = os.str();
    bridge_state_pub_->publish(out);
  }

  rclcpp::CallbackGroup::SharedPtr blocking_cb_group_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridge_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::string driver_;
  std::unique_ptr<gripper_control::Linkage> linkage_;
  bool plant_{false};
  double command_width_{0.0};
  double measured_angle_{kNaN};
  double last_angle_{kNaN};
  double still_s_{0.0};
  rclcpp::TimerBase::SharedPtr tick_timer_, state_timer_;

  std::mutex mutex_;
  double width_{0.0};
  double target_width_{0.0};
  bool moving_{false};
  bool grip_detected_{false};
  const char * last_command_{"NONE"};

  std::mutex motion_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperSimNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
