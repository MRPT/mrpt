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

#include <gtest/gtest.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/core/config.h>  // MRPT_OS_*()

#include <array>
#include <string>

using mrpt::comms::CSerialPort;

// ---------------------------------------------------------------------------
// Tests that need no serial device at all:
// ---------------------------------------------------------------------------

TEST(CSerialPort, closedPortRejectsEverything)
{
  CSerialPort port;

  EXPECT_FALSE(port.isOpen());

  std::array<char, 4> buf{};
  EXPECT_ANY_THROW(port.Read(buf.data(), buf.size()));
  EXPECT_ANY_THROW(port.Write(buf.data(), buf.size()));
  EXPECT_ANY_THROW(port.ReadString());
  EXPECT_ANY_THROW(port.purgeBuffers());
  EXPECT_ANY_THROW(port.setConfig(115200));
  EXPECT_ANY_THROW(port.setTimeouts(1, 0, 100, 1, 100));
}

TEST(CSerialPort, unsupportedStreamOperationsThrow)
{
  CSerialPort port;

  EXPECT_ANY_THROW(port.Seek(0));
  EXPECT_ANY_THROW((void)port.getTotalBytesCount());
  EXPECT_ANY_THROW((void)port.getPosition());
}

TEST(CSerialPort, closingAnAlreadyClosedPortIsHarmless)
{
  CSerialPort port;
  EXPECT_NO_THROW(port.close());
  EXPECT_FALSE(port.isOpen());
}

TEST(CSerialPort, openingAnEmptyOrMissingDeviceThrows)
{
  {
    CSerialPort port;
    EXPECT_ANY_THROW(port.open());  // no name was ever set
  }
  {
    CSerialPort port;
    EXPECT_ANY_THROW(port.open("/dev/mrpt-no-such-serial-device"));
    EXPECT_FALSE(port.isOpen());
  }
}

// ---------------------------------------------------------------------------
// Tests driving a real termios device, using a pseudo-terminal pair so that no
// physical serial hardware is required.
// ---------------------------------------------------------------------------

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <thread>

namespace
{
/** Master side of a pseudo-terminal pair; `slaveName()` is a device path that
 * CSerialPort can open just like a real serial port. */
class PtyPair
{
 public:
  PtyPair()
  {
    m_master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (m_master < 0)
    {
      return;
    }
    if (::grantpt(m_master) != 0 || ::unlockpt(m_master) != 0)
    {
      ::close(m_master);
      m_master = -1;
      return;
    }
    const char* n = ::ptsname(m_master);
    if (n == nullptr)
    {
      ::close(m_master);
      m_master = -1;
      return;
    }
    m_slaveName = n;
  }

  ~PtyPair()
  {
    if (m_master >= 0)
    {
      ::close(m_master);
    }
  }

  PtyPair(const PtyPair&) = delete;
  PtyPair& operator=(const PtyPair&) = delete;
  PtyPair(PtyPair&&) = delete;
  PtyPair& operator=(PtyPair&&) = delete;

  [[nodiscard]] bool ok() const { return m_master >= 0; }
  [[nodiscard]] const std::string& slaveName() const { return m_slaveName; }

  /** Injects data as if it had arrived from the far end of the cable. */
  [[nodiscard]] bool sendToPort(const std::string& s) const
  {
    return ::write(m_master, s.data(), s.size()) == static_cast<ssize_t>(s.size());
  }

  /** Reads back whatever the port wrote out, giving up after `timeoutMs`. */
  [[nodiscard]] std::string receiveFromPort(size_t maxBytes, int timeoutMs = 3000) const
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

 private:
  int m_master = -1;
  std::string m_slaveName;
};

/** Opens `port` on the pseudo-terminal. Returns false only when the platform
 * provides no pseudo-terminal at all (e.g. a container without /dev/pts), in
 * which case the caller skips the test. */
bool openPtyPort(const PtyPair& pty, CSerialPort& port)
{
  if (!pty.ok())
  {
    return false;
  }
  port.open(pty.slaveName());
  return port.isOpen();
}

}  // namespace

/** Message used by every test that needs a real termios device. */
#define SKIP_IF_NO_PTY "No pseudo-terminal available in this platform."

TEST(CSerialPort, openAndCloseARealDevice)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  // Opening again with the same name is a no-op, with a different one an error:
  EXPECT_NO_THROW(port.open(pty.slaveName()));
  EXPECT_ANY_THROW(port.open("/dev/ttyOther"));
  EXPECT_ANY_THROW(port.setSerialPortName("/dev/ttyOther"));

  port.close();
  EXPECT_FALSE(port.isOpen());
}

TEST(CSerialPort, setConfigAcceptsStandardBaudRates)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  for (const int baud : {50, 300, 1200, 9600, 19200, 38400, 57600, 115200, 230400})
  {
    EXPECT_NO_THROW(port.setConfig(baud, 0, 8, 1, false));
  }

  // Parity, character size and stop bits:
  EXPECT_NO_THROW(port.setConfig(9600, 1, 7, 2, false));
  EXPECT_NO_THROW(port.setConfig(9600, 2, 5, 1, false));
  EXPECT_NO_THROW(port.setConfig(9600, 0, 6, 1, true));
}

TEST(CSerialPort, setConfigRejectsInvalidParameters)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  EXPECT_ANY_THROW(port.setConfig(9600, 0, 9, 1, false));  // character size
  EXPECT_ANY_THROW(port.setConfig(9600, 7, 8, 1, false));  // parity
  EXPECT_ANY_THROW(port.setConfig(9600, 0, 8, 3, false));  // stop bits
  EXPECT_ANY_THROW(port.setConfig(0, 0, 8, 1, false));     // baud rate
}

TEST(CSerialPort, writeReachesTheOtherEnd)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  port.setConfig(115200, 0, 8, 1, false);
  port.setTimeouts(1, 0, 200, 1, 200);

  const std::string msg = "PING\r\n";

  // CSerialPort::Write() ends with tcdrain(), which on some platforms blocks
  // until the far end consumes the data, so read it concurrently:
  std::string received;
  std::thread reader([&]() { received = pty.receiveFromPort(64); });

  EXPECT_EQ(port.Write(msg.data(), msg.size()), msg.size());
  reader.join();

  EXPECT_EQ(received, msg);
}

TEST(CSerialPort, readReceivesIncomingBytes)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  port.setConfig(115200, 0, 8, 1, false);
  port.setTimeouts(1, 0, 500, 1, 500);

  const std::string msg = "0123456789";
  ASSERT_TRUE(pty.sendToPort(msg));

  std::array<char, 16> buf{};
  const size_t n = port.Read(buf.data(), msg.size());
  EXPECT_EQ(std::string(buf.data(), n), msg);
}

TEST(CSerialPort, readOfZeroBytesIsANoOp)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  char dummy = 0;
  EXPECT_EQ(port.Read(&dummy, 0), 0U);
}

TEST(CSerialPort, readTimesOutWhenNothingArrives)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  port.setTimeouts(1, 0, 50, 1, 50);

  std::array<char, 8> buf{};
  EXPECT_EQ(port.Read(buf.data(), buf.size()), 0U);
}

TEST(CSerialPort, readStringUpToTheEndOfLine)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  port.setConfig(115200, 0, 8, 1, false);
  ASSERT_TRUE(pty.sendToPort("$GPGGA,1234\r\n"));

  bool timedOut = true;
  const std::string line = port.ReadString(2000, &timedOut);
  EXPECT_EQ(line, "$GPGGA,1234");
  EXPECT_FALSE(timedOut);
}

TEST(CSerialPort, readStringReportsTimeoutOnAnIncompleteLine)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  ASSERT_TRUE(pty.sendToPort("no end of line here"));

  bool timedOut = false;
  const std::string partial = port.ReadString(200, &timedOut);
  EXPECT_TRUE(timedOut);
  EXPECT_EQ(partial, "no end of line here");
}

TEST(CSerialPort, purgeBuffersDiscardsPendingInput)
{
  PtyPair pty;
  CSerialPort port;
  if (!openPtyPort(pty, port))
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  port.setTimeouts(1, 0, 50, 1, 50);

  ASSERT_TRUE(pty.sendToPort("to be discarded"));
  // Let the bytes reach the port's input queue before flushing it:
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_NO_THROW(port.purgeBuffers());

  std::array<char, 32> buf{};
  EXPECT_EQ(port.Read(buf.data(), buf.size()), 0U);
}

TEST(CSerialPort, constructorCanOpenTheDeviceRightAway)
{
  PtyPair pty;
  if (!pty.ok())
  {
    GTEST_SKIP() << SKIP_IF_NO_PTY;
  }

  CSerialPort port(pty.slaveName(), true);
  EXPECT_TRUE(port.isOpen());
}

#endif  // Linux / Apple
