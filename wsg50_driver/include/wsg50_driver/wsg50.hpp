// Copyright 2026 WSG50 ROS maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WSG50_DRIVER__WSG50_HPP_
#define WSG50_DRIVER__WSG50_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wsg50_msgs/action/command.hpp"
#include "wsg50_msgs/msg/state.hpp"

#include "wsg50_driver/transport.hpp"
#include "wsg50_driver/visibility_control.h"

namespace wsg50_driver
{

class GripperDriver : public rclcpp::Node
{
public:
  using StandardCommand = control_msgs::action::GripperCommand;
  using StandardGoalHandle = rclcpp_action::ServerGoalHandle<StandardCommand>;
  using AdvancedCommand = wsg50_msgs::action::Command;
  using AdvancedGoalHandle = rclcpp_action::ServerGoalHandle<AdvancedCommand>;

  WSG50_DRIVER_PUBLIC
  explicit GripperDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GripperDriver() override;

private:
  static constexpr double kMinimumWidth = 0.0;
  static constexpr double kMaximumWidth = 0.110;
  static constexpr double kMinimumSpeed = 0.005;
  static constexpr double kMaximumSpeed = 0.420;
  static constexpr double kMinimumAcceleration = 0.1;
  static constexpr double kMaximumAcceleration = 5.0;
  static constexpr double kMinimumForce = 5.0;
  static constexpr double kMaximumForce = 80.0;

  void load_parameters();
  void validate_parameters() const;
  void initialize_connection();
  void publish_state();
  void publish_diagnostics();

  rclcpp_action::GoalResponse handle_standard_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const StandardCommand::Goal> goal);
  rclcpp_action::CancelResponse handle_standard_cancel(
    const std::shared_ptr<StandardGoalHandle> & goal_handle);
  void handle_standard_accepted(const std::shared_ptr<StandardGoalHandle> & goal_handle);
  void execute_standard(
    const std::shared_ptr<StandardGoalHandle> & goal_handle, uint64_t generation);

  rclcpp_action::GoalResponse handle_advanced_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const AdvancedCommand::Goal> goal);
  rclcpp_action::CancelResponse handle_advanced_cancel(
    const std::shared_ptr<AdvancedGoalHandle> & goal_handle);
  void handle_advanced_accepted(const std::shared_ptr<AdvancedGoalHandle> & goal_handle);
  void execute_advanced(
    const std::shared_ptr<AdvancedGoalHandle> & goal_handle, uint64_t generation);

  std::future<CommandReply> start_motion(
    uint8_t mode, double width_m, double speed_m_s, bool stop_on_block);
  CommandReply set_acceleration(double acceleration_m_s2);
  CommandReply set_force(double force_n);
  CommandReply stop_motion();
  bool goal_is_valid(double width, double speed, double acceleration, double force) const;
  bool connection_is_ready() const;
  bool motion_reached(double target_m, bool grasp, const DeviceState & state) const;
  DeviceState wait_for_settled_state(double target_m, bool grasp, bool motion_ok) const;
  void finish_motion(uint64_t generation);
  void join_action_threads();

  void home_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void acknowledge_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void tare_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void fill_trigger_response(
    const CommandReply & reply, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string address_;
  int port_{1501};
  double default_speed_{0.01};
  double default_acceleration_{1.0};
  double default_grasp_force_{40.0};
  double goal_tolerance_{0.001};
  double state_publish_rate_{20.0};
  bool auto_acknowledge_faults_{false};
  bool auto_home_{false};
  int connect_timeout_ms_{2000};
  int response_timeout_ms_{2000};
  int motion_timeout_ms_{30000};
  int reconnect_interval_ms_{1000};
  int state_update_period_ms_{50};

  std::shared_ptr<Transport> transport_;
  std::atomic<uint64_t> initialized_generation_{0};
  std::atomic<bool> initialization_running_{false};
  std::atomic<bool> initialization_successful_{false};
  std::thread initialization_thread_;

  rclcpp_action::Server<StandardCommand>::SharedPtr standard_action_server_;
  rclcpp_action::Server<AdvancedCommand>::SharedPtr advanced_action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr acknowledge_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
  rclcpp::Publisher<wsg50_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;

  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> motion_active_{false};
  std::atomic<uint64_t> motion_generation_{0};
  std::mutex motion_execution_mutex_;
  std::mutex action_threads_mutex_;
  std::vector<std::thread> action_threads_;
};

}  // namespace wsg50_driver

#endif  // WSG50_DRIVER__WSG50_HPP_
