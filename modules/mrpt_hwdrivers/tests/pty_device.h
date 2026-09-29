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

// A pseudo-terminal pair to stand in for a serial device. Only used on Linux,
// where the driver tests were validated: use `MRPT_HWDRIVERS_TESTS_HAVE_PTY`.
// (On macOS a pty delivers data differently, and drivers that block on the port
// without a timeout would hang the test run.)
#if defined(__linux__)
#define MRPT_HWDRIVERS_TESTS_HAVE_PTY 1

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace mrpt::hwdrivers::testing
{
/** The master side of a pseudo-terminal: drivers open `slaveName()` as if it
 * were a serial port, and the test plays the device through the master side.
 *
 * Besides the raw `send()` / `receive()`, `startResponder()` launches a thread
 * that answers what the driver writes, and `startStreaming()` one that keeps
 * pushing data, for sensors that transmit on their own.
 */
class PtyDevice
{
 public:
  PtyDevice()
  {
    m_master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (m_master < 0)
    {
      return;
    }
    if (::grantpt(m_master) != 0 || ::unlockpt(m_master) != 0)
    {
      closeMaster();
      return;
    }
    const char* n = ::ptsname(m_master);
    if (n == nullptr)
    {
      closeMaster();
      return;
    }
    m_slaveName = n;

    // Raw mode, so that bytes cross the pty unmodified (no echo, no CR/LF
    // translation): the slave side inherits it until the driver reconfigures
    // the port.
    termios t{};
    if (::tcgetattr(m_master, &t) == 0)
    {
      ::cfmakeraw(&t);
      ::tcsetattr(m_master, TCSANOW, &t);
    }
  }

  ~PtyDevice()
  {
    stopThreads();
    closeMaster();
  }

  PtyDevice(const PtyDevice&) = delete;
  PtyDevice& operator=(const PtyDevice&) = delete;
  PtyDevice(PtyDevice&&) = delete;
  PtyDevice& operator=(PtyDevice&&) = delete;

  [[nodiscard]] bool ok() const { return m_master >= 0; }
  [[nodiscard]] const std::string& slaveName() const { return m_slaveName; }

  /** Sends bytes to the driver, as if they came from the device. */
  bool send(const std::string& s) const
  {
    std::lock_guard<std::mutex> lck(m_writeMtx);
    return ::write(m_master, s.data(), s.size()) == static_cast<ssize_t>(s.size());
  }

  /** Reads what the driver wrote, waiting up to `timeoutMs` for the first
   * bytes. */
  [[nodiscard]] std::string receive(size_t maxBytes = 4096, int timeoutMs = 2000) const
  {
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(m_master, &rd);
    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = 1000 * (timeoutMs % 1000);
    if (::select(m_master + 1, &rd, nullptr, nullptr, &tv) <= 0)
    {
      return {};
    }
    std::string out(maxBytes, '\0');
    const ssize_t n = ::read(m_master, out.data(), maxBytes);
    out.resize(n > 0 ? static_cast<size_t>(n) : 0);
    return out;
  }

  /** Runs `handler` on every chunk the driver writes, and sends back what it
   * returns. */
  void startResponder(std::function<std::string(const std::string&)> handler)
  {
    m_responder = std::thread(
        [this, handler = std::move(handler)]()
        {
          while (!m_stop)
          {
            const std::string req = receive(4096, 50);
            if (req.empty())
            {
              continue;
            }
            const std::string reply = handler(req);
            if (!reply.empty())
            {
              send(reply);
            }
          }
        });
  }

  /** Keeps sending `data` every `periodMs` until destroyed. */
  void startStreaming(std::string data, int periodMs = 20)
  {
    m_streamer = std::thread(
        [this, data = std::move(data), periodMs]()
        {
          while (!m_stop)
          {
            send(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
          }
        });
  }

 private:
  void stopThreads()
  {
    m_stop = true;
    if (m_responder.joinable())
    {
      m_responder.join();
    }
    if (m_streamer.joinable())
    {
      m_streamer.join();
    }
  }
  void closeMaster()
  {
    if (m_master >= 0)
    {
      ::close(m_master);
      m_master = -1;
    }
  }

  int m_master = -1;
  std::string m_slaveName;
  std::atomic<bool> m_stop{false};
  mutable std::mutex m_writeMtx;
  std::thread m_responder;
  std::thread m_streamer;
};

}  // namespace mrpt::hwdrivers::testing

#endif  // Linux
