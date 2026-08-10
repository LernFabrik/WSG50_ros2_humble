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

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "wsg50_driver/wsg50.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace wsg50_driver
{
namespace
{
constexpr uint8_t kCommandHome = 0x20;
constexpr uint8_t kCommandMove = 0x21;
constexpr uint8_t kCommandStop = 0x22;
constexpr uint8_t kCommandAcknowledge = 0x24;
constexpr uint8_t kCommandGrasp = 0x25;
constexpr uint8_t kCommandRelease = 0x26;
constexpr uint8_t kCommandSetAcceleration = 0x30;
constexpr uint8_t kCommandSetForce = 0x32;
constexpr uint8_t kCommandTare = 0x38;

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

GripperDriver::GripperDriver(const rclcpp::NodeOptions & options)
: Node("wsg50_gripper_driver", options)
{
  load_parameters();
  validate_parameters();

  TransportOptions transport_options;
  transport_options.address = address_;
  transport_options.port = static_cast<uint16_t>(port_);
  transport_options.connect_timeout = std::chrono::milliseconds(connect_timeout_ms_);
  transport_options.response_timeout = std::chrono::milliseconds(response_timeout_ms_);
  transport_options.reconnect_interval = std::chrono::milliseconds(reconnect_interval_ms_);
  transport_options.state_update_period_ms = static_cast<uint16_t>(state_update_period_ms_);
  transport_ = std::make_shared<Transport>(transport_options);

  standard_action_server_ = rclcpp_action::create_server<StandardCommand>(
    this, "~/gripper_action",
    std::bind(&GripperDriver::handle_standard_goal, this, _1, _2),
    std::bind(&GripperDriver::handle_standard_cancel, this, _1),
    std::bind(&GripperDriver::handle_standard_accepted, this, _1));
  advanced_action_server_ = rclcpp_action::create_server<AdvancedCommand>(
    this, "~/command",
    std::bind(&GripperDriver::handle_advanced_goal, this, _1, _2),
    std::bind(&GripperDriver::handle_advanced_cancel, this, _1),
    std::bind(&GripperDriver::handle_advanced_accepted, this, _1));

  home_service_ = create_service<std_srvs::srv::Trigger>(
    "~/home", std::bind(&GripperDriver::home_service, this, _1, _2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "~/stop", std::bind(&GripperDriver::stop_service, this, _1, _2));
  acknowledge_service_ = create_service<std_srvs::srv::Trigger>(
    "~/ack_fault", std::bind(&GripperDriver::acknowledge_service, this, _1, _2));
  tare_service_ = create_service<std_srvs::srv::Trigger>(
    "~/tare", std::bind(&GripperDriver::tare_service, this, _1, _2));

  state_publisher_ = create_publisher<wsg50_msgs::msg::State>("~/state", 10);
  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10);

  const auto state_period = std::chrono::duration<double>(1.0 / state_publish_rate_);
  state_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(state_period),
    std::bind(&GripperDriver::publish_state, this));
  diagnostics_timer_ = create_wall_timer(1s, std::bind(&GripperDriver::publish_diagnostics, this));
  connection_timer_ = create_wall_timer(
    100ms, std::bind(
      &GripperDriver::initialize_connection,
      this));

  RCLCPP_INFO(
    get_logger(), "Connecting to WSG50 at %s:%d; automatic acknowledge=%s, home=%s",
    address_.c_str(), port_, auto_acknowledge_faults_ ? "true" : "false",
    auto_home_ ? "true" : "false");
  transport_->start();
}

GripperDriver::~GripperDriver()
{
  shutting_down_ = true;
  if (transport_) {
    transport_->stop();
  }
  if (initialization_thread_.joinable()) {
    initialization_thread_.join();
  }
  join_action_threads();
}

void GripperDriver::load_parameters()
{
  address_ = declare_parameter<std::string>("gripper_ip", "192.168.1.160");
  port_ = declare_parameter<int>("port", 1501);
  default_speed_ = declare_parameter<double>("default_speed", 0.01);
  default_acceleration_ = declare_parameter<double>("default_acceleration", 1.0);
  default_grasp_force_ = declare_parameter<double>("default_grasp_force", 40.0);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.001);
  state_publish_rate_ = declare_parameter<double>("state_publish_rate", 20.0);
  auto_acknowledge_faults_ = declare_parameter<bool>("auto_acknowledge_faults", false);
  auto_home_ = declare_parameter<bool>("auto_home", false);
  connect_timeout_ms_ = declare_parameter<int>("connect_timeout_ms", 2000);
  response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 2000);
  motion_timeout_ms_ = declare_parameter<int>("motion_timeout_ms", 30000);
  reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
  state_update_period_ms_ = declare_parameter<int>("state_update_period_ms", 50);
}

void GripperDriver::validate_parameters() const
{
  if (address_.empty() || port_ < 1 || port_ > 65535) {
    throw std::invalid_argument("gripper_ip and port must identify a valid IPv4 endpoint");
  }
  if (default_speed_ < kMinimumSpeed || default_speed_ > kMaximumSpeed) {
    throw std::invalid_argument("default_speed must be within [0.005, 0.420] m/s");
  }
  if (default_acceleration_ < kMinimumAcceleration ||
    default_acceleration_ > kMaximumAcceleration)
  {
    throw std::invalid_argument("default_acceleration must be within [0.1, 5.0] m/s^2");
  }
  if (default_grasp_force_ < kMinimumForce || default_grasp_force_ > kMaximumForce) {
    throw std::invalid_argument("default_grasp_force must be within [5, 80] N");
  }
  if (goal_tolerance_ <= 0.0 || goal_tolerance_ > kMaximumWidth || state_publish_rate_ <= 0.0) {
    throw std::invalid_argument("goal_tolerance and state_publish_rate must be positive");
  }
  if (connect_timeout_ms_ <= 0 || response_timeout_ms_ <= 0 || motion_timeout_ms_ <= 0 ||
    reconnect_interval_ms_ <= 0 || state_update_period_ms_ < 10 || state_update_period_ms_ > 1000)
  {
    throw std::invalid_argument(
            "timeouts must be positive and state_update_period_ms in [10, 1000]");
  }
}

void GripperDriver::initialize_connection()
{
  if (shutting_down_ || initialization_running_) {
    return;
  }
  const auto device_state = transport_->state();
  if (!device_state.connected || device_state.connection_generation == initialized_generation_) {
    return;
  }
  if (initialization_thread_.joinable()) {
    initialization_thread_.join();
  }
  initialization_running_ = true;
  initialization_successful_ = false;
  const uint64_t generation = device_state.connection_generation;
  initialization_thread_ = std::thread(
    [this, generation]() {
      CommandReply reply = set_acceleration(default_acceleration_);
      if (reply.success()) {
        reply = set_force(default_grasp_force_);
      }
      if (reply.success() && auto_acknowledge_faults_) {
        reply = transport_->command(
          kCommandAcknowledge, {'a', 'c', 'k'}, true,
          std::chrono::milliseconds(response_timeout_ms_)).get();
      }
      if (reply.success() && auto_home_) {
        reply = transport_->command(
          kCommandHome, {0x00}, true, std::chrono::milliseconds(motion_timeout_ms_)).get();
      }
      if (!reply.success() && !shutting_down_) {
        RCLCPP_WARN(get_logger(), "Connection initialization failed: %s", reply.message.c_str());
      }
      initialized_generation_ = generation;
      initialization_successful_ = reply.success();
      initialization_running_ = false;
    });
}

void GripperDriver::publish_state()
{
  const DeviceState state = transport_->state();
  wsg50_msgs::msg::State message;
  message.header.stamp = now();
  message.connected = state.connected;
  message.referenced = state.referenced;
  message.moving = state.moving;
  message.stalled = state.stalled;
  message.width = state.opening_mm / 1000.0;
  message.speed = state.speed_mm_s / 1000.0;
  message.force = state.force_n;
  message.system_state = state.system_state;
  message.grasping_state = state.grasping_state;
  message.status_code = state.last_status;
  message.status_message = state.status_message;
  state_publisher_->publish(message);

  if (!state.connected || !std::isfinite(message.width) || message.width < kMinimumWidth ||
    message.width > kMaximumWidth)
  {
    return;
  }
  sensor_msgs::msg::JointState joint_state;
  joint_state.header = message.header;
  joint_state.name = {"wsg50_finger_left_joint", "wsg50_finger_right_joint"};
  joint_state.position = {-message.width / 2.0, message.width / 2.0};
  joint_state.velocity = {-message.speed / 2.0, message.speed / 2.0};
  joint_state.effort = {message.force / 2.0, message.force / 2.0};
  joint_state_publisher_->publish(joint_state);
}

void GripperDriver::publish_diagnostics()
{
  const DeviceState state = transport_->state();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": WSG50");
  status.hardware_id = address_ + ":" + std::to_string(port_);

  const auto age = std::chrono::steady_clock::now() - state.updated_at;
  const bool stale = state.connected &&
    age > std::chrono::milliseconds(state_update_period_ms_ * 4);
  constexpr uint32_t error_mask =
    SYSTEM_SCRIPT_FAILURE | SYSTEM_COMMAND_FAILURE | SYSTEM_FINGER_FAULT |
    SYSTEM_CURRENT_FAULT | SYSTEM_POWER_FAULT | SYSTEM_TEMPERATURE_FAULT |
    SYSTEM_FAST_STOP;
  constexpr uint32_t warning_mask = SYSTEM_TEMPERATURE_WARNING;
  if (!state.connected) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "disconnected: " + state.status_message;
  } else if (stale) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "state stream is stale";
  } else if (initialized_generation_ != state.connection_generation || initialization_running_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "initializing connection";
  } else if (!initialization_successful_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "connection initialization failed";
  } else if ((state.system_state & error_mask) != 0U) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "gripper reports a fault";
  } else if ((state.system_state & warning_mask) != 0U) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "gripper reports a warning";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = state.referenced ? "ready" : "connected, not referenced";
  }
  status.values = {
    key_value("connected", state.connected ? "true" : "false"),
    key_value("referenced", state.referenced ? "true" : "false"),
    key_value("moving", state.moving ? "true" : "false"),
    key_value("width_m", std::to_string(state.opening_mm / 1000.0)),
    key_value("speed_m_s", std::to_string(state.speed_mm_s / 1000.0)),
    key_value("force_n", std::to_string(state.force_n)),
    key_value("system_state", std::to_string(state.system_state)),
    key_value("status_code", std::to_string(state.last_status)),
    key_value("status_message", state.status_message),
    key_value("last_error", state.last_error)};
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

rclcpp_action::GoalResponse GripperDriver::handle_standard_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const StandardCommand::Goal> goal)
{
  if (!std::isfinite(goal->command.max_effort) || goal->command.max_effort < 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const bool grasp = goal->command.max_effort > 0.0;
  if (!connection_is_ready() || !goal_is_valid(
      goal->command.position, default_speed_, default_acceleration_,
      grasp ? goal->command.max_effort : default_grasp_force_))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperDriver::handle_standard_cancel(
  const std::shared_ptr<StandardGoalHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperDriver::handle_standard_accepted(
  const std::shared_ptr<StandardGoalHandle> & goal_handle)
{
  const uint64_t generation = ++motion_generation_;
  motion_active_ = true;
  std::lock_guard<std::mutex> lock(action_threads_mutex_);
  action_threads_.emplace_back(&GripperDriver::execute_standard, this, goal_handle, generation);
}

void GripperDriver::execute_standard(
  const std::shared_ptr<StandardGoalHandle> & goal_handle, uint64_t generation)
{
  std::unique_lock<std::mutex> execution_lock(motion_execution_mutex_);
  auto result = std::make_shared<StandardCommand::Result>();
  if (generation != motion_generation_ || shutting_down_) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    finish_motion(generation);
    return;
  }

  const auto goal = goal_handle->get_goal();
  const bool grasp = goal->command.max_effort > 0.0;
  CommandReply reply = set_acceleration(default_acceleration_);
  if (reply.success()) {
    reply = set_force(grasp ? goal->command.max_effort : default_grasp_force_);
  }
  if (!reply.success()) {
    result->position = transport_->state().opening_mm / 1000.0;
    goal_handle->abort(result);
    finish_motion(generation);
    return;
  }

  auto future = start_motion(
    grasp ? AdvancedCommand::Goal::GRASP : AdvancedCommand::Goal::MOVE,
    goal->command.position, default_speed_, false);
  bool stop_sent = false;
  while (future.wait_for(50ms) != std::future_status::ready && !shutting_down_) {
    const DeviceState state = transport_->state();
    auto feedback = std::make_shared<StandardCommand::Feedback>();
    feedback->position = state.opening_mm / 1000.0;
    feedback->effort = state.force_n;
    feedback->stalled = state.stalled;
    feedback->reached_goal = motion_reached(goal->command.position, grasp, state);
    goal_handle->publish_feedback(feedback);
    if (!stop_sent && (goal_handle->is_canceling() || generation != motion_generation_)) {
      (void)stop_motion();
      stop_sent = true;
    }
  }
  reply = future.get();
  const bool motion_ok = reply.success() || (grasp && reply.status == STATUS_AXIS_BLOCKED);
  const DeviceState state = wait_for_settled_state(goal->command.position, grasp, motion_ok);
  result->position = state.opening_mm / 1000.0;
  result->effort = state.force_n;
  result->stalled = state.stalled || (grasp && reply.status == STATUS_AXIS_BLOCKED);
  result->reached_goal = motion_ok && motion_reached(goal->command.position, grasp, state);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (generation != motion_generation_) {
    goal_handle->abort(result);
  } else if (motion_ok && (result->reached_goal || (grasp && result->stalled))) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  finish_motion(generation);
}

rclcpp_action::GoalResponse GripperDriver::handle_advanced_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const AdvancedCommand::Goal> goal)
{
  if (goal->mode > AdvancedCommand::Goal::RELEASE || !connection_is_ready()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal_is_valid(goal->width, goal->speed, goal->acceleration, goal->force)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperDriver::handle_advanced_cancel(
  const std::shared_ptr<AdvancedGoalHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperDriver::handle_advanced_accepted(
  const std::shared_ptr<AdvancedGoalHandle> & goal_handle)
{
  const uint64_t generation = ++motion_generation_;
  motion_active_ = true;
  std::lock_guard<std::mutex> lock(action_threads_mutex_);
  action_threads_.emplace_back(&GripperDriver::execute_advanced, this, goal_handle, generation);
}

void GripperDriver::execute_advanced(
  const std::shared_ptr<AdvancedGoalHandle> & goal_handle, uint64_t generation)
{
  std::unique_lock<std::mutex> execution_lock(motion_execution_mutex_);
  auto result = std::make_shared<AdvancedCommand::Result>();
  if (generation != motion_generation_ || shutting_down_) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      result->status_code = STATUS_COMMAND_ABORTED;
      result->message = "preempted by a newer goal";
      goal_handle->abort(result);
    }
    finish_motion(generation);
    return;
  }
  const auto goal = goal_handle->get_goal();
  const bool grasp = goal->mode == AdvancedCommand::Goal::GRASP;
  CommandReply reply = set_acceleration(goal->acceleration);
  if (reply.success()) {
    reply = set_force(goal->force);
  }
  if (!reply.success()) {
    result->status_code = reply.status;
    result->message = reply.message;
    goal_handle->abort(result);
    finish_motion(generation);
    return;
  }

  auto future = start_motion(goal->mode, goal->width, goal->speed, goal->stop_on_block);
  bool stop_sent = false;
  while (future.wait_for(50ms) != std::future_status::ready && !shutting_down_) {
    const DeviceState state = transport_->state();
    auto feedback = std::make_shared<AdvancedCommand::Feedback>();
    feedback->width = state.opening_mm / 1000.0;
    feedback->speed = state.speed_mm_s / 1000.0;
    feedback->force = state.force_n;
    feedback->stalled = state.stalled;
    feedback->reached_goal = motion_reached(goal->width, grasp, state);
    feedback->status_code = state.last_status;
    feedback->message = state.status_message;
    goal_handle->publish_feedback(feedback);
    if (!stop_sent && (goal_handle->is_canceling() || generation != motion_generation_)) {
      (void)stop_motion();
      stop_sent = true;
    }
  }
  reply = future.get();
  const bool motion_ok = reply.success() || (grasp && reply.status == STATUS_AXIS_BLOCKED);
  const DeviceState state = wait_for_settled_state(goal->width, grasp, motion_ok);
  result->width = state.opening_mm / 1000.0;
  result->speed = state.speed_mm_s / 1000.0;
  result->force = state.force_n;
  result->stalled = state.stalled || (grasp && reply.status == STATUS_AXIS_BLOCKED);
  result->reached_goal = motion_ok && motion_reached(goal->width, grasp, state);
  result->status_code = reply.status;
  result->message = reply.message;

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (generation != motion_generation_) {
    result->status_code = STATUS_COMMAND_ABORTED;
    result->message = "preempted by a newer goal";
    goal_handle->abort(result);
  } else if (motion_ok && (result->reached_goal || (grasp && result->stalled))) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  finish_motion(generation);
}

std::future<CommandReply> GripperDriver::start_motion(
  uint8_t mode, double width_m, double speed_m_s, bool stop_on_block)
{
  std::vector<uint8_t> payload;
  const float width_mm = static_cast<float>(width_m * 1000.0);
  const float speed_mm_s = static_cast<float>(speed_m_s * 1000.0);
  uint8_t command_id = kCommandMove;
  if (mode == AdvancedCommand::Goal::MOVE) {
    payload.push_back(stop_on_block ? 0x02 : 0x00);
    Transport::append_float(payload, width_mm);
    Transport::append_float(payload, speed_mm_s);
  } else {
    command_id = mode == AdvancedCommand::Goal::GRASP ? kCommandGrasp : kCommandRelease;
    Transport::append_float(payload, width_mm);
    Transport::append_float(payload, speed_mm_s);
  }
  return transport_->command(
    command_id, std::move(payload), true, std::chrono::milliseconds(motion_timeout_ms_));
}

CommandReply GripperDriver::set_acceleration(double acceleration_m_s2)
{
  std::vector<uint8_t> payload;
  Transport::append_float(payload, static_cast<float>(acceleration_m_s2 * 1000.0));
  return transport_->command(
    kCommandSetAcceleration, std::move(payload), true,
    std::chrono::milliseconds(response_timeout_ms_)).get();
}

CommandReply GripperDriver::set_force(double force_n)
{
  std::vector<uint8_t> payload;
  Transport::append_float(payload, static_cast<float>(force_n));
  return transport_->command(
    kCommandSetForce, std::move(payload), true,
    std::chrono::milliseconds(response_timeout_ms_)).get();
}

CommandReply GripperDriver::stop_motion()
{
  return transport_->command(
    kCommandStop, {}, true, std::chrono::milliseconds(response_timeout_ms_), true).get();
}

bool GripperDriver::goal_is_valid(
  double width, double speed, double acceleration, double force) const
{
  return std::isfinite(width) && width >= kMinimumWidth && width <= kMaximumWidth &&
         std::isfinite(speed) && speed >= kMinimumSpeed && speed <= kMaximumSpeed &&
         std::isfinite(acceleration) && acceleration >= kMinimumAcceleration &&
         acceleration <= kMaximumAcceleration &&
         std::isfinite(force) && force >= kMinimumForce && force <= kMaximumForce;
}

bool GripperDriver::connection_is_ready() const
{
  const DeviceState state = transport_->state();
  return state.connected && state.referenced && initialization_successful_ &&
         initialized_generation_ == state.connection_generation;
}

bool GripperDriver::motion_reached(
  double target_m, bool grasp, const DeviceState & state) const
{
  const bool within_tolerance = std::abs(state.opening_mm / 1000.0 - target_m) <= goal_tolerance_;
  return within_tolerance || (grasp && state.stalled);
}

DeviceState GripperDriver::wait_for_settled_state(
  double target_m, bool grasp, bool motion_ok) const
{
  DeviceState state = transport_->state();
  const auto settle_duration = std::chrono::milliseconds(
    std::max(500, state_update_period_ms_ * 3));
  const auto settle_deadline = std::chrono::steady_clock::now() + settle_duration;
  while (state.connected && motion_ok &&
    (state.moving || !motion_reached(target_m, grasp, state)) &&
    std::chrono::steady_clock::now() < settle_deadline)
  {
    std::this_thread::sleep_for(10ms);
    state = transport_->state();
  }
  return state;
}

void GripperDriver::finish_motion(uint64_t generation)
{
  if (generation == motion_generation_) {
    motion_active_ = false;
  }
}

void GripperDriver::join_action_threads()
{
  std::vector<std::thread> threads;
  {
    std::lock_guard<std::mutex> lock(action_threads_mutex_);
    threads.swap(action_threads_);
  }
  for (auto & thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void GripperDriver::home_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(motion_execution_mutex_);
  if (!transport_->connected()) {
    response->message = "gripper is not connected";
    return;
  }
  fill_trigger_response(
    transport_->command(
      kCommandHome, {0x00}, true, std::chrono::milliseconds(motion_timeout_ms_)).get(),
    response);
}

void GripperDriver::stop_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  fill_trigger_response(stop_motion(), response);
}

void GripperDriver::acknowledge_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  fill_trigger_response(
    transport_->command(
      kCommandAcknowledge, {'a', 'c', 'k'}, true,
      std::chrono::milliseconds(response_timeout_ms_)).get(),
    response);
}

void GripperDriver::tare_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  fill_trigger_response(
    transport_->command(
      kCommandTare, {}, true, std::chrono::milliseconds(response_timeout_ms_)).get(),
    response);
}

void GripperDriver::fill_trigger_response(
  const CommandReply & reply, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = reply.success();
  response->message = reply.message;
}

}  // namespace wsg50_driver

RCLCPP_COMPONENTS_REGISTER_NODE(wsg50_driver::GripperDriver)
