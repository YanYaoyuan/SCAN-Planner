// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file zsibot_sdk_proxy.cpp @brief UDP-to-ZsiBot SDK proxy. */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "zsibot_cmd_bridge/sdk_owner_lock.hpp"

#if defined(ZSIBOT_MODEL_ZSL_1)
#include "zsl-1/highlevel.h"
namespace zsibot_model = mc_sdk::zsl_1;
#elif defined(ZSIBOT_MODEL_ZSL_1W)
#include "zsl-1w/highlevel.h"
namespace zsibot_model = mc_sdk::zsl_1w;
#else
#error "A ZsiBot model compile definition is required"
#endif

namespace
{
using Clock = std::chrono::steady_clock;

std::atomic_bool g_running{true};

/** @brief Requests orderly proxy shutdown. @param signal_number Received POSIX signal number. */
void handleSignal(int signal_number)
{
  (void)signal_number;
  g_running = false;
}

/** @brief Sanitizes and limits a command component. @param value Requested value. @param limit Absolute limit. @return Limited finite value. */
double clampFinite(double value, double limit)
{
  if (!std::isfinite(value))
    return 0.0;
  const double abs_limit = std::max(0.0, std::abs(limit));
  return std::clamp(value, -abs_limit, abs_limit);
}

/** @brief Command-line and SDK networking options for the proxy process. */
struct Options
{
  bool enable_deprecated_transport{false};
  std::string listen_ip{"0.0.0.0"};
  int listen_port{44000};
  std::string sdk_local_ip{"127.0.0.1"};
  std::string sdk_dog_ip{"127.0.0.1"};
  int sdk_local_port{43988};
  double publish_rate{50.0};
  double cmd_timeout{0.3};
  double max_vx{0.3};
  double max_vy{0.15};
  double max_yaw_rate{0.5};
  bool auto_stand{true};
  bool require_standing_before_move{true};
  double standup_check_period{0.2};
  double standup_retry_period{1.0};
  double standup_wait_timeout{10.0};
  double status_log_period{2.0};
};

/** @brief Reads a string option value. @param argc Argument count. @param argv Argument vector. @param[in,out] i Current argument index. @param[out] value Parsed value. @return True when present. */
bool readOption(int argc, char** argv, int& i, std::string& value)
{
  if (i + 1 >= argc)
    return false;
  value = argv[++i];
  return true;
}

/** @brief Reads an integer option value. @param argc Argument count. @param argv Argument vector. @param[in,out] i Current argument index. @param[out] value Parsed value. @return True when present. */
bool readOption(int argc, char** argv, int& i, int& value)
{
  std::string text;
  if (!readOption(argc, argv, i, text))
    return false;
  value = std::stoi(text);
  return true;
}

/** @brief Reads a floating-point option value. @param argc Argument count. @param argv Argument vector. @param[in,out] i Current argument index. @param[out] value Parsed value. @return True when present. */
bool readOption(int argc, char** argv, int& i, double& value)
{
  std::string text;
  if (!readOption(argc, argv, i, text))
    return false;
  value = std::stod(text);
  return true;
}

/** @brief Reads a Boolean option value. @param argc Argument count. @param argv Argument vector. @param[in,out] i Current argument index. @param[out] value Parsed value. @return True when present. */
bool readOption(int argc, char** argv, int& i, bool& value)
{
  std::string text;
  if (!readOption(argc, argv, i, text))
    return false;
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  value = text == "1" || text == "true" || text == "yes" || text == "on";
  return true;
}

/** @brief Prints command-line usage. @param argv0 Executable name. */
void printUsage(const char* argv0)
{
  std::cerr
      << "usage: " << argv0 << " [options]\n"
      << "  --enable-deprecated-transport REQUIRED explicit compatibility opt-in\n"
      << "  --listen-ip IP                 UDP listen IP, default 0.0.0.0\n"
      << "  --listen-port PORT             UDP listen port, default 44000\n"
      << "  --sdk-local-ip IP              SDK local IP, default 127.0.0.1\n"
      << "  --sdk-local-port PORT          SDK local port, default 43988\n"
      << "  --sdk-dog-ip IP                SDK dog IP, default 127.0.0.1\n"
      << "  --publish-rate HZ              SDK move resend rate, default 50\n"
      << "  --cmd-timeout SEC              UDP command timeout, default 0.3\n"
      << "  --max-vx MPS                   default 0.3\n"
      << "  --max-vy MPS                   default 0.15\n"
      << "  --max-yaw-rate RADPS           default 0.5\n"
      << "  --auto-stand true|false        default true\n"
      << "  --require-standing true|false  default true\n";
}

/** @brief Parses and validates proxy options. @param argc Argument count. @param argv Argument vector. @return Validated options. */
Options parseOptions(int argc, char** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    bool ok = true;
    if (arg == "--help" || arg == "-h")
    {
      printUsage(argv[0]);
      std::exit(0);
    }
    else if (arg == "--enable-deprecated-transport") options.enable_deprecated_transport = true;
    else if (arg == "--listen-ip") ok = readOption(argc, argv, i, options.listen_ip);
    else if (arg == "--listen-port") ok = readOption(argc, argv, i, options.listen_port);
    else if (arg == "--sdk-local-ip") ok = readOption(argc, argv, i, options.sdk_local_ip);
    else if (arg == "--sdk-local-port") ok = readOption(argc, argv, i, options.sdk_local_port);
    else if (arg == "--sdk-dog-ip") ok = readOption(argc, argv, i, options.sdk_dog_ip);
    else if (arg == "--publish-rate") ok = readOption(argc, argv, i, options.publish_rate);
    else if (arg == "--cmd-timeout") ok = readOption(argc, argv, i, options.cmd_timeout);
    else if (arg == "--max-vx") ok = readOption(argc, argv, i, options.max_vx);
    else if (arg == "--max-vy") ok = readOption(argc, argv, i, options.max_vy);
    else if (arg == "--max-yaw-rate") ok = readOption(argc, argv, i, options.max_yaw_rate);
    else if (arg == "--auto-stand") ok = readOption(argc, argv, i, options.auto_stand);
    else if (arg == "--require-standing") ok = readOption(argc, argv, i, options.require_standing_before_move);
    else
    {
      throw std::runtime_error("unknown argument: " + arg);
    }
    if (!ok)
      throw std::runtime_error("missing value for argument: " + arg);
  }

  if (!options.enable_deprecated_transport)
    throw std::runtime_error(
        "deprecated SDK proxy is disabled; pass --enable-deprecated-transport "
        "only for an isolated compatibility test");
  if (options.listen_port <= 0 || options.listen_port > 65535)
    throw std::runtime_error("listen_port must be in 1..65535");
  if (options.sdk_local_port <= 0 || options.sdk_local_port > 65535)
    throw std::runtime_error("sdk_local_port must be in 1..65535");
  if (options.publish_rate <= 0.0 || options.cmd_timeout <= 0.0)
    throw std::runtime_error("publish_rate and cmd_timeout must be positive");
  return options;
}

/** @brief One decoded UDP body-velocity command. */
struct Command
{
  uint64_t seq{0};
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
};

/** @brief Decodes one versioned text command. @param data Datagram bytes. @param size Datagram size. @param[out] command Parsed command. @return True when valid. */
bool parsePacket(const char* data, size_t size, Command& command)
{
  std::string text(data, size);
  std::istringstream stream(text);
  std::string magic;
  uint64_t seq = 0;
  double vx = 0.0;
  double vy = 0.0;
  double yaw = 0.0;
  stream >> magic >> seq >> vx >> vy >> yaw;
  if (!stream || magic != "ZSIBOT_CMD_V1")
    return false;
  command.seq = seq;
  command.vx = vx;
  command.vy = vy;
  command.yaw_rate = yaw;
  return true;
}

/** @brief Creates and binds the receive socket. @param options Network options. @return Socket descriptor. */
int createUdpSocket(const Options& options)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));

  int reuse = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(options.listen_port));
  if (::inet_pton(AF_INET, options.listen_ip.c_str(), &addr.sin_addr) != 1)
  {
    ::close(fd);
    throw std::runtime_error("Invalid listen_ip: " + options.listen_ip);
  }
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    ::close(fd);
    throw std::runtime_error(std::string("bind() failed: ") + std::strerror(errno));
  }
  return fd;
}
}  // namespace

/** @brief Runs the UDP-to-SDK proxy process. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char** argv)
{
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  try
  {
    const Options options = parseOptions(argc, argv);
    const zsibot_cmd_bridge::SdkOwnerLock sdk_owner_lock("deprecated_zsibot_sdk_proxy");
    const int udp_fd = createUdpSocket(options);

    std::cout << "ZsiBot SDK proxy listening on " << options.listen_ip << ':'
              << options.listen_port << std::endl;
    std::cout << "Connecting SDK locally: " << options.sdk_local_ip << ':'
              << options.sdk_local_port << " -> " << options.sdk_dog_ip << std::endl;

    zsibot_model::HighLevel highlevel;
    highlevel.initRobot(options.sdk_local_ip, options.sdk_local_port, options.sdk_dog_ip);

    bool waiting_for_standup = false;
    bool move_enabled = true;
    auto standup_start = Clock::now();
    auto last_standup_request = Clock::now();
    auto last_standup_check = Clock::now() - std::chrono::seconds(10);

    if (options.auto_stand)
    {
      const uint32_t ret = highlevel.standUp();
      std::cout << "standUp() returned " << ret << std::endl;
      if (options.require_standing_before_move)
      {
        waiting_for_standup = true;
        move_enabled = false;
        standup_start = Clock::now();
        last_standup_request = standup_start;
        std::cout << "Waiting for standUp state before sending move commands" << std::endl;
      }
    }

    Command latest_cmd;
    bool have_cmd = false;
    bool last_sent_zero = false;
    auto last_cmd_time = Clock::now();
    auto last_send_time = Clock::now();
    auto last_status_time = Clock::now();
    const auto send_period = std::chrono::duration<double>(1.0 / options.publish_rate);

    auto send_move = [&](double vx, double vy, double yaw) {
      const uint32_t ret = highlevel.move(static_cast<float>(vx), static_cast<float>(vy),
                                          static_cast<float>(yaw));
      if (ret != 0)
        std::cout << "HighLevel::move returned " << ret << std::endl;
    };

    while (g_running)
    {
      const auto now = Clock::now();
      const auto until_next_send = last_send_time + send_period - now;
      timeval timeout{};
      const auto wait_us = std::max<int64_t>(
          1000, std::chrono::duration_cast<std::chrono::microseconds>(until_next_send).count());
      timeout.tv_sec = static_cast<long>(wait_us / 1000000);
      timeout.tv_usec = static_cast<long>(wait_us % 1000000);

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(udp_fd, &read_set);
      const int ready = ::select(udp_fd + 1, &read_set, nullptr, nullptr, &timeout);
      if (ready > 0 && FD_ISSET(udp_fd, &read_set))
      {
        char buffer[256];
        sockaddr_in remote{};
        socklen_t remote_len = sizeof(remote);
        const ssize_t count = ::recvfrom(udp_fd, buffer, sizeof(buffer) - 1, 0,
                                         reinterpret_cast<sockaddr*>(&remote), &remote_len);
        if (count > 0)
        {
          buffer[count] = '\0';
          Command parsed;
          if (parsePacket(buffer, static_cast<size_t>(count), parsed))
          {
            latest_cmd.seq = parsed.seq;
            latest_cmd.vx = clampFinite(parsed.vx, options.max_vx);
            latest_cmd.vy = clampFinite(parsed.vy, options.max_vy);
            latest_cmd.yaw_rate = clampFinite(parsed.yaw_rate, options.max_yaw_rate);
            last_cmd_time = Clock::now();
            have_cmd = true;
          }
          else
          {
            std::cout << "Ignored malformed UDP packet: " << buffer << std::endl;
          }
        }
      }

      const auto loop_now = Clock::now();
      if (waiting_for_standup &&
          std::chrono::duration<double>(loop_now - last_standup_check).count() >= options.standup_check_period)
      {
        last_standup_check = loop_now;
        const uint32_t ctrlmode = highlevel.getCurrentCtrlmode();
        if (ctrlmode == 1 || ctrlmode == 3)
        {
          waiting_for_standup = false;
          move_enabled = true;
          std::cout << "standUp state confirmed: ctrlmode=" << ctrlmode
                    << "; move commands enabled" << std::endl;
          send_move(0.0, 0.0, 0.0);
          last_sent_zero = true;
        }
        else
        {
          const double wait_s = std::chrono::duration<double>(loop_now - standup_start).count();
          if (std::chrono::duration<double>(loop_now - last_standup_request).count() >=
              options.standup_retry_period)
          {
            last_standup_request = loop_now;
            const uint32_t ret = highlevel.standUp();
            std::cout << "Retrying standUp() while ctrlmode=" << ctrlmode
                      << "; returned " << ret << std::endl;
          }
          if (wait_s > options.standup_wait_timeout)
          {
            std::cout << "Still waiting for standUp state after " << wait_s
                      << "s: ctrlmode=" << ctrlmode << ". Move commands remain blocked."
                      << std::endl;
          }
        }
      }

      if (move_enabled && Clock::now() - last_send_time >= send_period)
      {
        last_send_time = Clock::now();
        const bool timed_out = !have_cmd ||
            std::chrono::duration<double>(last_send_time - last_cmd_time).count() > options.cmd_timeout;
        if (timed_out)
        {
          if (!last_sent_zero)
            std::cout << "UDP cmd timeout; sending zero velocity" << std::endl;
          send_move(0.0, 0.0, 0.0);
          last_sent_zero = true;
        }
        else
        {
          send_move(latest_cmd.vx, latest_cmd.vy, latest_cmd.yaw_rate);
          last_sent_zero = latest_cmd.vx == 0.0 && latest_cmd.vy == 0.0 &&
              latest_cmd.yaw_rate == 0.0;
        }
      }

      if (std::chrono::duration<double>(Clock::now() - last_status_time).count() >=
          options.status_log_period)
      {
        last_status_time = Clock::now();
        const bool connected = highlevel.checkConnect();
        const uint32_t battery = highlevel.getBatteryPower();
        const uint32_t ctrlmode = highlevel.getCurrentCtrlmode();
        std::cout << "SDK status: connected=" << (connected ? "true" : "false")
                  << " battery=" << battery << "% ctrlmode=" << ctrlmode
                  << " udp=" << (have_cmd ? "received" : "none")
                  << " seq=" << latest_cmd.seq
                  << " vx=" << latest_cmd.vx
                  << " vy=" << latest_cmd.vy
                  << " yaw=" << latest_cmd.yaw_rate << std::endl;
      }
    }

    if (move_enabled)
      send_move(0.0, 0.0, 0.0);
    ::close(udp_fd);
  }
  catch (const std::exception& e)
  {
    std::cerr << "zsibot_sdk_proxy fatal: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
