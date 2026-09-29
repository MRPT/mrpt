/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/CServerTCPSocket.h>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mrpt::hwdrivers::testing
{
/** A TCP server on the loopback interface that plays the role of a networked
 * sensor: it accepts one connection and, for every chunk of bytes received from
 * the driver, it calls a handler that returns the bytes to send back (an empty
 * string sends nothing). Bytes can also be pushed with no request at all (the
 * `spontaneous` constructor argument), for sensors that stream data on their
 * own, and the connection can be closed right after the first reply, like an
 * HTTP/1.0 server does.
 */
class FakeTcpDevice
{
 public:
  using Handler = std::function<std::string(const std::string& request)>;

  explicit FakeTcpDevice(
      Handler handler, std::string spontaneous = {}, bool closeAfterReply = false) :
      m_handler(std::move(handler)),
      m_spontaneous(std::move(spontaneous)),
      m_closeAfterReply(closeAfterReply)
  {
    for (unsigned short p = FIRST_PORT; p <= LAST_PORT && !m_server; p++)
    {
      try
      {
        auto s = std::make_unique<mrpt::comms::CServerTCPSocket>(
            p, "127.0.0.1", 4, mrpt::system::LVL_ERROR);
        if (s->isListening())
        {
          m_port = p;
          m_server = std::move(s);
        }
      }
      catch (const std::exception&)
      {
        // Port in use: try the next one.
      }
    }
    if (!m_server)
    {
      return;
    }
    m_thread = std::thread([this]() { run(); });
  }

  ~FakeTcpDevice()
  {
    m_stop = true;
    if (m_thread.joinable())
    {
      m_thread.join();
    }
  }

  FakeTcpDevice(const FakeTcpDevice&) = delete;
  FakeTcpDevice& operator=(const FakeTcpDevice&) = delete;
  FakeTcpDevice(FakeTcpDevice&&) = delete;
  FakeTcpDevice& operator=(FakeTcpDevice&&) = delete;

  [[nodiscard]] bool isReady() const { return m_server != nullptr; }
  [[nodiscard]] unsigned short port() const { return m_port; }

  /** Everything the driver has sent so far, one entry per received chunk. */
  [[nodiscard]] std::vector<std::string> requests() const
  {
    std::lock_guard<std::mutex> lck(m_mtx);
    return m_requests;
  }

  /** Hang up the connection (the next driver read sees a closed socket). */
  void disconnect() { m_disconnect = true; }

 private:
  static constexpr unsigned short FIRST_PORT = 18700;
  // 18790-18799 are left for tests that need a fixed or an unused port
  static constexpr unsigned short LAST_PORT = 18789;

  void run()
  {
    try
    {
      auto client = m_server->accept(5000);
      if (!client)
      {
        return;
      }
      if (!m_spontaneous.empty())
      {
        client->sendString(m_spontaneous);
      }
      std::array<char, 8192> buf{};
      while (!m_stop && !m_disconnect)
      {
        const size_t n = client->readAsync(buf.data(), buf.size(), 20, 2);
        if (n == 0)
        {
          if (!client->isConnected())
          {
            break;  // the driver hung up
          }
          continue;
        }
        const std::string req(buf.data(), n);
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          m_requests.push_back(req);
        }
        const std::string reply = m_handler(req);
        if (!reply.empty())
        {
          client->sendString(reply);
          if (m_closeAfterReply)
          {
            break;
          }
        }
      }
      client->close();
    }
    catch (const std::exception&)
    {
      // Nothing to do: the test will notice the missing traffic.
    }
  }

  Handler m_handler;
  std::string m_spontaneous;
  bool m_closeAfterReply;
  std::unique_ptr<mrpt::comms::CServerTCPSocket> m_server;
  unsigned short m_port = 0;
  std::thread m_thread;
  std::atomic<bool> m_stop{false};
  std::atomic<bool> m_disconnect{false};
  mutable std::mutex m_mtx;
  std::vector<std::string> m_requests;
};

}  // namespace mrpt::hwdrivers::testing
