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

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "wsg50_driver/transport.hpp"
#include "wsg50_driver/wsg50.hpp"
#include "wsg50_msgs/action/command.hpp"
#include "wsg50_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace
{
void append_u32(std::vector<uint8_t> & payload, uint32_t value)
{
  payload.push_back(static_cast<uint8_t>(value & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

class FakeGripper
{
public:
  struct ReceivedCommand
  {
    uint8_t id;
    std::vector<uint8_t> payload;
  };

  explicit FakeGripper(bool disconnect_on_move = false, bool reply_before_final_state = false)
  : disconnect_on_move_(disconnect_on_move), reply_before_final_state_(reply_before_final_state)
  {
    listen_socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    EXPECT_GE(listen_socket_, 0);
    int reuse = 1;
    (void)::setsockopt(listen_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    EXPECT_EQ(::bind(listen_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    EXPECT_EQ(::listen(listen_socket_, 1), 0);
    socklen_t size = sizeof(address);
    EXPECT_EQ(::getsockname(listen_socket_, reinterpret_cast<sockaddr *>(&address), &size), 0);
    port_ = ntohs(address.sin_port);
    thread_ = std::thread(&FakeGripper::run, this);
  }

  ~FakeGripper()
  {
    running_ = false;
    const int client_socket = client_socket_.load();
    if (client_socket >= 0) {
      ::shutdown(client_socket, SHUT_RDWR);
    }
    if (listen_socket_ >= 0) {
      ::shutdown(listen_socket_, SHUT_RDWR);
    }
    if (thread_.joinable()) {
      thread_.join();
    }
    if (client_socket >= 0) {
      ::close(client_socket);
    }
    if (listen_socket_ >= 0) {
      ::close(listen_socket_);
    }
  }

  uint16_t port() const {return port_;}

  void hold_motion_until_stop(bool enabled) {hold_motion_until_stop_ = enabled;}

  std::vector<ReceivedCommand> received_commands() const
  {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    return received_commands_;
  }

private:
  void run()
  {
    const int client_socket = ::accept(listen_socket_, nullptr, nullptr);
    client_socket_ = client_socket;
    std::vector<uint8_t> input;
    std::array<uint8_t, 1024> buffer{};
    while (running_ && client_socket >= 0) {
      const ssize_t count = ::recv(client_socket, buffer.data(), buffer.size(), 0);
      if (count <= 0) {
        break;
      }
      input.insert(input.end(), buffer.begin(), buffer.begin() + count);
      while (input.size() >= 8U) {
        const std::size_t payload_size =
          static_cast<std::size_t>(input[4]) | (static_cast<std::size_t>(input[5]) << 8U);
        const std::size_t frame_size = payload_size + 8U;
        if (input.size() < frame_size) {
          break;
        }
        const uint8_t id = input[3];
        EXPECT_EQ(wsg50_driver::Transport::crc16(input.data(), frame_size), 0U);
        const std::vector<uint8_t> payload(
          input.begin() + 6,
          input.begin() + static_cast<std::ptrdiff_t>(6U + payload_size));
        {
          std::lock_guard<std::mutex> lock(commands_mutex_);
          received_commands_.push_back({id, payload});
        }
        input.erase(input.begin(), input.begin() + static_cast<std::ptrdiff_t>(frame_size));
        if (id == 0x21 && disconnect_on_move_) {
          (void)::shutdown(client_socket, SHUT_RDWR);
          return;
        }
        respond(id);
      }
    }
  }

  void respond(uint8_t id)
  {
    if (id == 0x21) {
      send_payload(id, {26, 0});
      if (hold_motion_until_stop_) {
        pending_motion_id_ = id;
        std::vector<uint8_t> moving_state{0, 0};
        append_u32(
          moving_state,
          wsg50_driver::SYSTEM_REFERENCED | wsg50_driver::SYSTEM_MOVING);
        send_payload(0x40, moving_state);
        return;
      }
      if (reply_before_final_state_) {
        send_payload(id, {0, 0});
        std::this_thread::sleep_for(100ms);
      }
      std::vector<uint8_t> opening{0, 0};
      wsg50_driver::Transport::append_float(opening, 42.0F);
      send_payload(0x43, opening);
      std::vector<uint8_t> system_state{0, 0};
      append_u32(system_state, wsg50_driver::SYSTEM_REFERENCED);
      send_payload(0x40, system_state);
      if (!reply_before_final_state_) {
        send_payload(id, {0, 0});
      }
      return;
    }

    if (id == 0x22) {
      send_payload(id, {0, 0});
      const uint8_t pending_motion = pending_motion_id_.exchange(0);
      if (pending_motion != 0U) {
        std::vector<uint8_t> stopped_state{0, 0};
        append_u32(stopped_state, wsg50_driver::SYSTEM_REFERENCED);
        send_payload(0x40, stopped_state);
        send_payload(pending_motion, {wsg50_driver::STATUS_COMMAND_ABORTED, 0});
      }
      return;
    }

    std::vector<uint8_t> payload{0, 0};
    if (id == 0x40) {
      append_u32(payload, wsg50_driver::SYSTEM_REFERENCED);
    } else if (id == 0x41) {
      payload.push_back(0);
    } else if (id == 0x43) {
      wsg50_driver::Transport::append_float(payload, 110.0F);
    } else if (id == 0x44 || id == 0x45) {
      wsg50_driver::Transport::append_float(payload, 0.0F);
    }
    send_payload(id, payload);
  }

  void send_payload(uint8_t id, const std::vector<uint8_t> & payload)
  {
    const int client_socket = client_socket_.load();
    const auto frame = wsg50_driver::Transport::encode_frame(id, payload);
    if (!fragmented_sync_sent_) {
      const std::array<uint8_t, 8> leading_junk{0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0xaa, 0xaa};
      (void)::send(client_socket, leading_junk.data(), leading_junk.size(), MSG_NOSIGNAL);
      std::this_thread::sleep_for(5ms);
      (void)::send(client_socket, frame.data() + 2, frame.size() - 2, MSG_NOSIGNAL);
      fragmented_sync_sent_ = true;
      return;
    }
    const std::size_t split = std::min<std::size_t>(3, frame.size());
    (void)::send(client_socket, frame.data(), split, MSG_NOSIGNAL);
    (void)::send(client_socket, frame.data() + split, frame.size() - split, MSG_NOSIGNAL);
  }

  std::atomic<bool> running_{true};
  std::atomic<bool> hold_motion_until_stop_{false};
  std::atomic<uint8_t> pending_motion_id_{0};
  bool disconnect_on_move_{false};
  bool reply_before_final_state_{false};
  bool fragmented_sync_sent_{false};
  mutable std::mutex commands_mutex_;
  std::vector<ReceivedCommand> received_commands_;
  int listen_socket_{-1};
  std::atomic<int> client_socket_{-1};
  uint16_t port_{0};
  std::thread thread_;
};
}  // namespace

TEST(TransportEncoding, ProducesValidFrame)
{
  const std::vector<uint8_t> payload{0x01, 0x02, 0x03};
  auto frame = wsg50_driver::Transport::encode_frame(0x21, payload);
  ASSERT_EQ(frame.size(), payload.size() + 8U);
  EXPECT_EQ(frame[0], 0xaa);
  EXPECT_EQ(frame[1], 0xaa);
  EXPECT_EQ(frame[2], 0xaa);
  EXPECT_EQ(frame[3], 0x21);
  EXPECT_EQ(frame[4], payload.size());
  EXPECT_EQ(frame[5], 0x00);
  EXPECT_EQ(wsg50_driver::Transport::crc16(frame.data(), frame.size()), 0);
}

TEST(TransportEncoding, MatchesWsgProtocolCrcVectors)
{
  const std::array<uint8_t, 3> preamble{0xaa, 0xaa, 0xaa};
  EXPECT_EQ(wsg50_driver::Transport::crc16(preamble.data(), preamble.size()), 0x50f5);

  const std::vector<uint8_t> system_state_query{
    0xaa, 0xaa, 0xaa, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(
    wsg50_driver::Transport::crc16(system_state_query.data(), system_state_query.size()),
    0x70f5);
}

TEST(TransportEncoding, FloatRoundTripIsLittleEndian)
{
  std::vector<uint8_t> payload;
  wsg50_driver::Transport::append_float(payload, 123.5F);
  ASSERT_EQ(payload.size(), 4U);
  EXPECT_FLOAT_EQ(wsg50_driver::Transport::decode_float(payload.data()), 123.5F);
}

TEST(TransportStatus, KnownAndUnknownStatusText)
{
  EXPECT_EQ(wsg50_driver::Transport::status_text(0), "success");
  EXPECT_EQ(wsg50_driver::Transport::status_text(29), "axis blocked");
  EXPECT_NE(wsg50_driver::Transport::status_text(1234).find("1234"), std::string::npos);
}

TEST(TransportStatus, SystemStateFlagsMatchTheWsgCommandSet)
{
  EXPECT_EQ(wsg50_driver::SYSTEM_REFERENCED, 1U << 0U);
  EXPECT_EQ(wsg50_driver::SYSTEM_MOVING, 1U << 1U);
  EXPECT_EQ(wsg50_driver::SYSTEM_BLOCKED_MINUS, 1U << 2U);
  EXPECT_EQ(wsg50_driver::SYSTEM_BLOCKED_PLUS, 1U << 3U);
  EXPECT_EQ(wsg50_driver::SYSTEM_FAST_STOP, 1U << 12U);
  EXPECT_EQ(wsg50_driver::SYSTEM_COMMAND_FAILURE, 1U << 18U);
}

TEST(TransportIntegration, HandlesFragmentedFramesPendingRepliesAndStateUpdates)
{
  FakeGripper gripper;
  wsg50_driver::TransportOptions options;
  options.address = "127.0.0.1";
  options.port = gripper.port();
  options.connect_timeout = 200ms;
  options.response_timeout = 500ms;
  options.reconnect_interval = 50ms;
  options.state_update_period_ms = 50;
  wsg50_driver::Transport transport(options);
  transport.start();

  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  while ((!transport.connected() || !transport.state().referenced) &&
    std::chrono::steady_clock::now() < ready_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(transport.connected());
  ASSERT_TRUE(transport.state().referenced);

  auto future = transport.command(0x21, {}, true, 1s);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto reply = future.get();
  EXPECT_TRUE(reply.success()) << reply.message;
  EXPECT_FLOAT_EQ(transport.state().opening_mm, 42.0F);
  transport.stop();
}

TEST(TransportIntegration, RejectsCommandsWhileDisconnected)
{
  wsg50_driver::TransportOptions options;
  options.address = "127.0.0.1";
  options.port = 1;
  wsg50_driver::Transport transport(options);
  auto future = transport.command(0x21, {}, true, 100ms);
  ASSERT_EQ(future.wait_for(100ms), std::future_status::ready);
  EXPECT_FALSE(future.get().transport_ok);
}

TEST(TransportIntegration, FailsPendingCommandAndClearsStateOnDisconnect)
{
  FakeGripper gripper(true);
  wsg50_driver::TransportOptions options;
  options.address = "127.0.0.1";
  options.port = gripper.port();
  options.connect_timeout = 200ms;
  options.response_timeout = 500ms;
  options.reconnect_interval = 5s;
  options.state_update_period_ms = 50;
  wsg50_driver::Transport transport(options);
  transport.start();

  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  while ((!transport.connected() || !transport.state().referenced) &&
    std::chrono::steady_clock::now() < ready_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(transport.state().referenced);

  auto future = transport.command(0x21, {}, true, 1s);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().transport_ok);
  EXPECT_FALSE(transport.connected());
  EXPECT_FALSE(transport.state().referenced);
  transport.stop();
}

TEST(DriverIntegration, SafeStartupAndBothActionsUseValidatedSiUnits)
{
  FakeGripper gripper(false, true);
  int argc = 0;
  rclcpp::init(argc, nullptr);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("gripper_ip", "127.0.0.1"),
    rclcpp::Parameter("port", static_cast<int>(gripper.port())),
    rclcpp::Parameter("auto_acknowledge_faults", false),
    rclcpp::Parameter("auto_home", false),
    rclcpp::Parameter("state_publish_rate", 50.0),
    rclcpp::Parameter("state_update_period_ms", 20),
    rclcpp::Parameter("connect_timeout_ms", 200),
    rclcpp::Parameter("response_timeout_ms", 500),
    rclcpp::Parameter("motion_timeout_ms", 2000),
    rclcpp::Parameter("reconnect_interval_ms", 100)});

  auto driver = std::make_shared<wsg50_driver::GripperDriver>(options);
  auto client_node = std::make_shared<rclcpp::Node>("wsg50_driver_integration_client");
  std::atomic<bool> state_ready{false};
  wsg50_msgs::msg::State last_state;
  std::mutex state_mutex;
  auto state_subscription = client_node->create_subscription<wsg50_msgs::msg::State>(
    "/wsg50_gripper_driver/state", 10,
    [&state_ready, &last_state, &state_mutex](const wsg50_msgs::msg::State & message) {
      std::lock_guard<std::mutex> lock(state_mutex);
      last_state = message;
      state_ready = message.connected && message.referenced;
    });
  (void)state_subscription;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  auto cleanup = std::shared_ptr<void>(
    nullptr, [&](void *) {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      executor.remove_node(client_node);
      executor.remove_node(driver);
      driver.reset();
      client_node.reset();
      rclcpp::shutdown();
    });

  const auto state_deadline = std::chrono::steady_clock::now() + 3s;
  while (!state_ready && std::chrono::steady_clock::now() < state_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(state_ready);
  const auto initialization_deadline = std::chrono::steady_clock::now() + 2s;
  bool initialization_commands_received = false;
  while (!initialization_commands_received &&
    std::chrono::steady_clock::now() < initialization_deadline)
  {
    const auto initialization_commands = gripper.received_commands();
    const bool acceleration_received = std::any_of(
      initialization_commands.begin(), initialization_commands.end(),
      [](const FakeGripper::ReceivedCommand & command) {return command.id == 0x30;});
    const bool force_received = std::any_of(
      initialization_commands.begin(), initialization_commands.end(),
      [](const FakeGripper::ReceivedCommand & command) {return command.id == 0x32;});
    initialization_commands_received = acceleration_received && force_received;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(initialization_commands_received);
  std::this_thread::sleep_for(50ms);
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    EXPECT_NEAR(last_state.width, 0.110, 1e-6);
  }

  auto commands = gripper.received_commands();
  EXPECT_TRUE(
    std::none_of(
      commands.begin(), commands.end(), [](const FakeGripper::ReceivedCommand & command) {
        return command.id == 0x20 || command.id == 0x21 || command.id == 0x25 ||
        command.id == 0x26;
      }));

  using AdvancedCommand = wsg50_msgs::action::Command;
  auto advanced_client = rclcpp_action::create_client<AdvancedCommand>(
    client_node, "/wsg50_gripper_driver/command");
  ASSERT_TRUE(advanced_client->wait_for_action_server(2s));

  AdvancedCommand::Goal invalid_goal;
  invalid_goal.mode = AdvancedCommand::Goal::MOVE;
  invalid_goal.width = 0.2;
  invalid_goal.speed = 0.01;
  invalid_goal.acceleration = 0.5;
  invalid_goal.force = 10.0;
  auto invalid_future = advanced_client->async_send_goal(invalid_goal);
  ASSERT_EQ(invalid_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(invalid_future.get(), nullptr);

  AdvancedCommand::Goal advanced_goal;
  advanced_goal.mode = AdvancedCommand::Goal::MOVE;
  advanced_goal.width = 0.042;
  advanced_goal.speed = 0.010;
  advanced_goal.acceleration = 0.5;
  advanced_goal.force = 10.0;
  advanced_goal.stop_on_block = true;
  auto advanced_goal_future = advanced_client->async_send_goal(advanced_goal);
  ASSERT_EQ(advanced_goal_future.wait_for(2s), std::future_status::ready);
  auto advanced_handle = advanced_goal_future.get();
  ASSERT_NE(advanced_handle, nullptr);
  auto advanced_result_future = advanced_client->async_get_result(advanced_handle);
  ASSERT_EQ(advanced_result_future.wait_for(3s), std::future_status::ready);
  const auto advanced_result = advanced_result_future.get();
  EXPECT_EQ(advanced_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(advanced_result.result->width, 0.042, 1e-6);
  EXPECT_TRUE(advanced_result.result->reached_goal);

  using StandardCommand = control_msgs::action::GripperCommand;
  auto standard_client = rclcpp_action::create_client<StandardCommand>(
    client_node, "/wsg50_gripper_driver/gripper_action");
  ASSERT_TRUE(standard_client->wait_for_action_server(2s));
  StandardCommand::Goal standard_goal;
  standard_goal.command.position = 0.042;
  standard_goal.command.max_effort = 0.0;
  auto standard_goal_future = standard_client->async_send_goal(standard_goal);
  ASSERT_EQ(standard_goal_future.wait_for(2s), std::future_status::ready);
  auto standard_handle = standard_goal_future.get();
  ASSERT_NE(standard_handle, nullptr);
  auto standard_result_future = standard_client->async_get_result(standard_handle);
  ASSERT_EQ(standard_result_future.wait_for(3s), std::future_status::ready);
  const auto standard_result = standard_result_future.get();
  EXPECT_EQ(standard_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(standard_result.result->position, 0.042, 1e-6);

  gripper.hold_motion_until_stop(true);
  AdvancedCommand::Goal cancel_goal = advanced_goal;
  cancel_goal.width = 0.050;
  auto cancel_goal_future = advanced_client->async_send_goal(cancel_goal);
  ASSERT_EQ(cancel_goal_future.wait_for(2s), std::future_status::ready);
  auto cancel_handle = cancel_goal_future.get();
  ASSERT_NE(cancel_handle, nullptr);
  std::this_thread::sleep_for(100ms);
  auto cancel_future = advanced_client->async_cancel_goal(cancel_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  ASSERT_EQ(cancel_future.get()->return_code, 0);
  auto canceled_result_future = advanced_client->async_get_result(cancel_handle);
  ASSERT_EQ(canceled_result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(canceled_result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  gripper.hold_motion_until_stop(false);

  commands = gripper.received_commands();
  const auto move_command = std::find_if(
    commands.begin(), commands.end(), [](const FakeGripper::ReceivedCommand & command) {
      return command.id == 0x21 && command.payload.size() == 9U;
    });
  ASSERT_NE(move_command, commands.end());
  EXPECT_EQ(move_command->payload[0], 0x02);
  EXPECT_FLOAT_EQ(wsg50_driver::Transport::decode_float(move_command->payload.data() + 1), 42.0F);
  EXPECT_FLOAT_EQ(wsg50_driver::Transport::decode_float(move_command->payload.data() + 5), 10.0F);

  const auto acceleration_command = std::find_if(
    commands.begin(), commands.end(), [](const FakeGripper::ReceivedCommand & command) {
      return command.id == 0x30 && command.payload.size() == 4U &&
      wsg50_driver::Transport::decode_float(command.payload.data()) == 500.0F;
    });
  EXPECT_NE(acceleration_command, commands.end());
  const auto force_command = std::find_if(
    commands.begin(), commands.end(), [](const FakeGripper::ReceivedCommand & command) {
      return command.id == 0x32 && command.payload.size() == 4U &&
      wsg50_driver::Transport::decode_float(command.payload.data()) == 10.0F;
    });
  EXPECT_NE(force_command, commands.end());
  EXPECT_TRUE(
    std::any_of(
      commands.begin(), commands.end(), [](const FakeGripper::ReceivedCommand & command) {
        return command.id == 0x22;
      }));

  cleanup.reset();
}
