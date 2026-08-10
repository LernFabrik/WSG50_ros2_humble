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

#ifndef WSG50_DRIVER__TRANSPORT_HPP_
#define WSG50_DRIVER__TRANSPORT_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace wsg50_driver
{

constexpr uint16_t STATUS_SUCCESS = 0;
constexpr uint16_t STATUS_NOT_INITIALIZED = 3;
constexpr uint16_t STATUS_COMMAND_ABORTED = 19;
constexpr uint16_t STATUS_COMMAND_PENDING = 26;
constexpr uint16_t STATUS_AXIS_BLOCKED = 29;
constexpr uint16_t STATUS_TRANSPORT_ERROR = 0xffff;

constexpr uint32_t SYSTEM_REFERENCED = 1U << 0U;
constexpr uint32_t SYSTEM_MOVING = 1U << 1U;
constexpr uint32_t SYSTEM_BLOCKED_MINUS = 1U << 2U;
constexpr uint32_t SYSTEM_BLOCKED_PLUS = 1U << 3U;
constexpr uint32_t SYSTEM_FAST_STOP = 1U << 12U;
constexpr uint32_t SYSTEM_TEMPERATURE_WARNING = 1U << 13U;
constexpr uint32_t SYSTEM_TEMPERATURE_FAULT = 1U << 14U;
constexpr uint32_t SYSTEM_POWER_FAULT = 1U << 15U;
constexpr uint32_t SYSTEM_CURRENT_FAULT = 1U << 16U;
constexpr uint32_t SYSTEM_FINGER_FAULT = 1U << 17U;
constexpr uint32_t SYSTEM_COMMAND_FAILURE = 1U << 18U;
constexpr uint32_t SYSTEM_SCRIPT_FAILURE = 1U << 20U;

struct CommandReply
{
  bool transport_ok{false};
  uint16_t status{STATUS_TRANSPORT_ERROR};
  std::string message;
  std::vector<uint8_t> payload;

  bool success() const {return transport_ok && status == STATUS_SUCCESS;}
};

struct DeviceState
{
  bool connected{false};
  bool referenced{false};
  bool moving{false};
  bool stalled{false};
  double opening_mm{0.0};
  double speed_mm_s{0.0};
  double force_n{0.0};
  uint32_t system_state{0};
  uint8_t grasping_state{0};
  uint16_t last_status{STATUS_TRANSPORT_ERROR};
  std::string status_message{"disconnected"};
  std::string last_error{"not connected"};
  std::chrono::steady_clock::time_point updated_at{};
  uint64_t connection_generation{0};
};

struct TransportOptions
{
  std::string address{"192.168.1.160"};
  uint16_t port{1501};
  std::chrono::milliseconds connect_timeout{2000};
  std::chrono::milliseconds response_timeout{2000};
  std::chrono::milliseconds reconnect_interval{1000};
  uint16_t state_update_period_ms{50};
  std::size_t max_payload_size{4096};
};

class Transport
{
public:
  explicit Transport(TransportOptions options);
  ~Transport();

  Transport(const Transport &) = delete;
  Transport & operator=(const Transport &) = delete;

  void start();
  void stop();
  bool connected() const;
  DeviceState state() const;

  std::future<CommandReply> command(
    uint8_t id, std::vector<uint8_t> payload, bool allow_pending,
    std::chrono::milliseconds timeout, bool priority = false);

  static std::vector<uint8_t> encode_frame(uint8_t id, const std::vector<uint8_t> & payload);
  static uint16_t crc16(const uint8_t * data, std::size_t size);
  static float decode_float(const uint8_t * bytes);
  static void append_float(std::vector<uint8_t> & payload, float value);
  static std::string status_text(uint16_t status);

private:
  struct Request
  {
    uint8_t id{0};
    std::vector<uint8_t> payload;
    bool allow_pending{false};
    bool internal{false};
    std::chrono::milliseconds timeout{0};
    std::chrono::steady_clock::time_point deadline{};
    std::promise<CommandReply> promise;
  };

  void run();
  bool connect_socket();
  void disconnect(const std::string & reason);
  bool send_all(const std::vector<uint8_t> & frame);
  bool receive_available();
  void parse_frames();
  void handle_frame(uint8_t id, const std::vector<uint8_t> & payload);
  void update_state(uint8_t id, uint16_t status, const std::vector<uint8_t> & payload);
  void dispatch_requests();
  void expire_requests();
  void queue_state_streaming();
  void fail_request(const std::shared_ptr<Request> & request, const std::string & reason);
  void fail_all(const std::string & reason);

  TransportOptions options_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread worker_;
  int socket_{-1};

  mutable std::mutex state_mutex_;
  DeviceState state_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::shared_ptr<Request>> queue_;
  std::deque<std::shared_ptr<Request>> priority_queue_;
  std::map<uint8_t, std::shared_ptr<Request>> in_flight_;
  std::vector<uint8_t> receive_buffer_;
};

}  // namespace wsg50_driver

#endif  // WSG50_DRIVER__TRANSPORT_HPP_
