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
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "wsg50_driver/transport.hpp"

namespace wsg50_driver
{
namespace
{
constexpr uint8_t kPreamble = 0xaa;
constexpr std::size_t kHeaderSize = 6;
constexpr std::size_t kFrameOverhead = 8;

uint16_t read_u16(const uint8_t * bytes)
{
  return static_cast<uint16_t>(bytes[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8U);
}

uint32_t read_u32(const uint8_t * bytes)
{
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}
}  // namespace

Transport::Transport(TransportOptions options)
: options_(std::move(options))
{
}

Transport::~Transport()
{
  stop();
}

void Transport::start()
{
  if (running_.exchange(true)) {
    return;
  }
  worker_ = std::thread(&Transport::run, this);
}

void Transport::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool Transport::connected() const
{
  return connected_;
}

DeviceState Transport::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

std::future<CommandReply> Transport::command(
  uint8_t id, std::vector<uint8_t> payload, bool allow_pending,
  std::chrono::milliseconds timeout, bool priority)
{
  auto request = std::make_shared<Request>();
  request->id = id;
  request->payload = std::move(payload);
  request->allow_pending = allow_pending;
  request->timeout = timeout;
  auto future = request->promise.get_future();

  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (connected_) {
      if (priority) {
        priority_queue_.push_back(request);
      } else {
        queue_.push_back(request);
      }
      queued = true;
    }
  }
  if (!queued) {
    fail_request(request, "gripper is not connected");
    return future;
  }
  queue_cv_.notify_one();
  return future;
}

std::vector<uint8_t> Transport::encode_frame(
  uint8_t id, const std::vector<uint8_t> & payload)
{
  if (payload.size() > std::numeric_limits<uint16_t>::max()) {
    return {};
  }
  std::vector<uint8_t> frame;
  frame.reserve(kFrameOverhead + payload.size());
  frame.insert(frame.end(), 3, kPreamble);
  frame.push_back(id);
  const auto size = static_cast<uint16_t>(payload.size());
  frame.push_back(static_cast<uint8_t>(size & 0xffU));
  frame.push_back(static_cast<uint8_t>((size >> 8U) & 0xffU));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xffU));
  frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0xffU));
  return frame;
}

uint16_t Transport::crc16(const uint8_t * data, std::size_t size)
{
  uint16_t crc = 0xffff;
  for (std::size_t offset = 0; offset < size; ++offset) {
    uint16_t table_value = static_cast<uint16_t>((crc ^ data[offset]) & 0x00ffU) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      table_value = (table_value & 0x8000U) != 0U ?
        static_cast<uint16_t>((table_value << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(table_value << 1U);
    }
    crc = static_cast<uint16_t>(table_value ^ (crc >> 8U));
  }
  return crc;
}

float Transport::decode_float(const uint8_t * bytes)
{
  const uint32_t bits = read_u32(bytes);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits), "WSG float must be 32 bit");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void Transport::append_float(std::vector<uint8_t> & payload, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  payload.push_back(static_cast<uint8_t>(bits & 0xffU));
  payload.push_back(static_cast<uint8_t>((bits >> 8U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((bits >> 16U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((bits >> 24U) & 0xffU));
}

std::string Transport::status_text(uint16_t status)
{
  static const std::array<const char *, 31> messages = {
    "success", "not available", "no sensor", "not initialized", "already running",
    "feature not supported", "inconsistent data", "timeout", "read error", "write error",
    "insufficient resources", "checksum error", "no parameter expected", "not enough parameters",
    "unknown command", "command format error", "access denied", "already open", "command failed",
    "command aborted", "invalid handle", "not found", "not open", "I/O error",
    "invalid parameter", "index out of bounds", "command pending", "overrun", "range error",
    "axis blocked", "file exists"};
  if (status < messages.size()) {
    return messages[status];
  }
  if (status == STATUS_TRANSPORT_ERROR) {
    return "transport error";
  }
  return "unknown status " + std::to_string(status);
}

void Transport::run()
{
  auto next_connect = std::chrono::steady_clock::now();
  while (running_) {
    if (socket_ < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now < next_connect) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_until(lock, next_connect, [this]() {return !running_;});
        continue;
      }
      if (!connect_socket()) {
        next_connect = now + options_.reconnect_interval;
        continue;
      }
      if (!running_) {
        break;
      }
      queue_state_streaming();
    }

    dispatch_requests();

    pollfd descriptor{};
    descriptor.fd = socket_;
    descriptor.events = POLLIN;
    const int result = ::poll(&descriptor, 1, 20);
    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      disconnect("socket disconnected");
      next_connect = std::chrono::steady_clock::now() + options_.reconnect_interval;
      continue;
    }
    if (result > 0 && (descriptor.revents & POLLIN) != 0 && !receive_available()) {
      next_connect = std::chrono::steady_clock::now() + options_.reconnect_interval;
      continue;
    }
    if (result < 0 && errno != EINTR) {
      disconnect(std::string("poll failed: ") + std::strerror(errno));
      next_connect = std::chrono::steady_clock::now() + options_.reconnect_interval;
      continue;
    }
    expire_requests();
  }
  disconnect("transport stopped");
}

bool Transport::connect_socket()
{
  const int socket_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (socket_fd < 0) {
    return false;
  }
  const int original_flags = ::fcntl(socket_fd, F_GETFL, 0);
  if (original_flags < 0 || ::fcntl(socket_fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
    ::close(socket_fd);
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(options_.port);
  if (::inet_pton(AF_INET, options_.address.c_str(), &address.sin_addr) != 1) {
    ::close(socket_fd);
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.status_message = "invalid IPv4 address";
    return false;
  }

  int result = ::connect(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  bool connection_ready = result == 0;
  if (!connection_ready && result < 0 && errno == EINPROGRESS) {
    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLOUT;
    result = ::poll(&descriptor, 1, static_cast<int>(options_.connect_timeout.count()));
    if (result > 0) {
      int socket_error = 0;
      socklen_t error_size = sizeof(socket_error);
      connection_ready =
        ::getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_size) == 0 &&
        socket_error == 0;
    }
  }
  if (!connection_ready) {
    ::close(socket_fd);
    return false;
  }

  int enabled = 1;
  (void)::setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
  socket_ = socket_fd;
  receive_buffer_.clear();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.connected = true;
    state_.last_status = STATUS_SUCCESS;
    state_.status_message = "connected";
    state_.updated_at = std::chrono::steady_clock::now();
    ++state_.connection_generation;
  }
  connected_ = true;
  return true;
}

void Transport::disconnect(const std::string & reason)
{
  connected_ = false;
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  fail_all(reason);
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.connected = false;
  state_.referenced = false;
  state_.moving = false;
  state_.stalled = false;
  state_.system_state = 0;
  state_.last_status = STATUS_TRANSPORT_ERROR;
  state_.status_message = reason;
  state_.last_error = reason;
  state_.updated_at = std::chrono::steady_clock::now();
}

bool Transport::send_all(const std::vector<uint8_t> & frame)
{
  std::size_t offset = 0;
  while (running_ && offset < frame.size()) {
    const ssize_t count = ::send(
      socket_, frame.data() + offset, frame.size() - offset, MSG_NOSIGNAL);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd descriptor{};
      descriptor.fd = socket_;
      descriptor.events = POLLOUT;
      if (::poll(&descriptor, 1, static_cast<int>(options_.response_timeout.count())) > 0) {
        continue;
      }
    }
    disconnect(std::string("send failed: ") + std::strerror(errno));
    return false;
  }
  return offset == frame.size();
}

bool Transport::receive_available()
{
  std::array<uint8_t, 2048> buffer{};
  while (running_) {
    const ssize_t count = ::recv(socket_, buffer.data(), buffer.size(), 0);
    if (count > 0) {
      receive_buffer_.insert(receive_buffer_.end(), buffer.begin(), buffer.begin() + count);
      parse_frames();
      continue;
    }
    if (count == 0) {
      disconnect("peer closed connection");
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    disconnect(std::string("receive failed: ") + std::strerror(errno));
    return false;
  }
  return false;
}

void Transport::parse_frames()
{
  while (receive_buffer_.size() >= kFrameOverhead) {
    static constexpr std::array<uint8_t, 3> preamble_bytes{kPreamble, kPreamble, kPreamble};
    auto preamble = std::search(
      receive_buffer_.begin(), receive_buffer_.end(),
      preamble_bytes.begin(), preamble_bytes.end());
    if (preamble == receive_buffer_.end()) {
      std::size_t keep = 0;
      for (auto iterator = receive_buffer_.rbegin();
        iterator != receive_buffer_.rend() && keep < 2U && *iterator == kPreamble;
        ++iterator)
      {
        ++keep;
      }
      receive_buffer_.erase(
        receive_buffer_.begin(),
        receive_buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      return;
    }
    if (preamble != receive_buffer_.begin()) {
      receive_buffer_.erase(receive_buffer_.begin(), preamble);
    }
    if (receive_buffer_.size() < kFrameOverhead) {
      return;
    }
    const std::size_t payload_size = read_u16(receive_buffer_.data() + 4);
    if (payload_size > options_.max_payload_size) {
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }
    const std::size_t frame_size = kFrameOverhead + payload_size;
    if (receive_buffer_.size() < frame_size) {
      return;
    }
    if (crc16(receive_buffer_.data(), frame_size) != 0) {
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }
    const uint8_t id = receive_buffer_[3];
    std::vector<uint8_t> payload(
      receive_buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
      receive_buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
    receive_buffer_.erase(
      receive_buffer_.begin(), receive_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    handle_frame(id, payload);
  }
}

void Transport::handle_frame(uint8_t id, const std::vector<uint8_t> & payload)
{
  if (payload.size() < 2) {
    return;
  }
  const uint16_t status = read_u16(payload.data());
  update_state(id, status, payload);

  std::shared_ptr<Request> request;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto found = in_flight_.find(id);
    if (found == in_flight_.end()) {
      return;
    }
    request = found->second;
    if (request->allow_pending && status == STATUS_COMMAND_PENDING) {
      request->deadline = std::chrono::steady_clock::now() + request->timeout;
      return;
    }
    in_flight_.erase(found);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.last_status = status;
    state_.status_message = status_text(status);
    if (status != STATUS_SUCCESS) {
      state_.last_error = state_.status_message;
    }
  }

  CommandReply reply;
  reply.transport_ok = true;
  reply.status = status;
  reply.message = status_text(status);
  reply.payload = payload;
  request->promise.set_value(std::move(reply));
  queue_cv_.notify_one();
}

void Transport::update_state(
  uint8_t id, uint16_t status, const std::vector<uint8_t> & payload)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (status != STATUS_SUCCESS) {
    state_.last_status = status;
    state_.status_message = status_text(status);
    state_.last_error = state_.status_message;
    return;
  }
  if (id == 0x40 && payload.size() >= 6) {
    state_.system_state = read_u32(payload.data() + 2);
    state_.referenced = (state_.system_state & SYSTEM_REFERENCED) != 0U;
    state_.moving = (state_.system_state & SYSTEM_MOVING) != 0U;
    state_.stalled =
      (state_.system_state & (SYSTEM_BLOCKED_PLUS | SYSTEM_BLOCKED_MINUS)) != 0U;
  } else if (id == 0x41 && payload.size() >= 3) {
    state_.grasping_state = payload[2];
  } else if (id == 0x43 && payload.size() >= 6) {
    state_.opening_mm = decode_float(payload.data() + 2);
  } else if (id == 0x44 && payload.size() >= 6) {
    state_.speed_mm_s = decode_float(payload.data() + 2);
  } else if (id == 0x45 && payload.size() >= 6) {
    state_.force_n = decode_float(payload.data() + 2);
  } else {
    return;
  }
  state_.updated_at = std::chrono::steady_clock::now();
}

void Transport::dispatch_requests()
{
  std::shared_ptr<Request> request;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!priority_queue_.empty()) {
      request = priority_queue_.front();
      if (in_flight_.count(request->id) != 0) {
        return;
      }
      priority_queue_.pop_front();
    } else if (in_flight_.empty() && !queue_.empty()) {
      request = queue_.front();
      queue_.pop_front();
    } else {
      return;
    }
    request->deadline = std::chrono::steady_clock::now() + request->timeout;
    in_flight_[request->id] = request;
  }

  const auto frame = encode_frame(request->id, request->payload);
  if (frame.empty() || !send_all(frame)) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    in_flight_.erase(request->id);
    fail_request(request, "failed to send command");
  }
}

void Transport::expire_requests()
{
  const auto now = std::chrono::steady_clock::now();
  bool expired = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (const auto & entry : in_flight_) {
      if (now >= entry.second->deadline) {
        expired = true;
        break;
      }
    }
  }
  if (expired) {
    disconnect("command response timeout");
  }
}

void Transport::queue_state_streaming()
{
  const uint16_t period = options_.state_update_period_ms;
  const std::vector<uint8_t> payload = {
    0x01, static_cast<uint8_t>(period & 0xffU), static_cast<uint8_t>((period >> 8U) & 0xffU)};
  std::lock_guard<std::mutex> lock(queue_mutex_);
  for (const uint8_t id : {0x40, 0x41, 0x43, 0x44, 0x45}) {
    auto request = std::make_shared<Request>();
    request->id = id;
    request->payload = payload;
    request->allow_pending = false;
    request->internal = true;
    request->timeout = options_.response_timeout;
    queue_.push_back(request);
  }
}

void Transport::fail_request(
  const std::shared_ptr<Request> & request, const std::string & reason)
{
  CommandReply reply;
  reply.message = reason;
  try {
    request->promise.set_value(std::move(reply));
  } catch (const std::future_error &) {
  }
}

void Transport::fail_all(const std::string & reason)
{
  std::vector<std::shared_ptr<Request>> requests;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (auto & entry : in_flight_) {
      requests.push_back(entry.second);
    }
    requests.insert(requests.end(), queue_.begin(), queue_.end());
    requests.insert(requests.end(), priority_queue_.begin(), priority_queue_.end());
    in_flight_.clear();
    queue_.clear();
    priority_queue_.clear();
  }
  for (const auto & request : requests) {
    fail_request(request, reason);
  }
}

}  // namespace wsg50_driver
