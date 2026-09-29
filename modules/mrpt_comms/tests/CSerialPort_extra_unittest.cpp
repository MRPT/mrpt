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
#include <mrpt/core/config.h>

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <utility>
#include <vector>

using mrpt::comms::CSerialPort;

namespace
{
/** Master side of a pseudo-terminal, standing for a serial device. */
class Pty
{
 public:
  Pty()
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
    m_slave = n;
  }
  ~Pty()
  {
    if (m_master >= 0)
    {
      ::close(m_master);
    }
  }
  Pty(const Pty&) = delete;
  Pty& operator=(const Pty&) = delete;
  [[nodiscard]] bool ok() const { return m_master >= 0; }
  [[nodiscard]] const std::string& slave() const { return m_slave; }

 private:
  int m_master = -1;
  std::string m_slave;
};

/** The configuration that the kernel holds for the pseudo-terminal. */
bool readTermios(const std::string& slaveName, termios& t)
{
  const int fd = ::open(slaveName.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    return false;
  }
  const bool ok = ::tcgetattr(fd, &t) == 0;
  ::close(fd);
  return ok;
}
}  // namespace

TEST(CSerialPort, StandardBaudRatesAreAppliedToTheDevice)
{
  Pty pty;
  if (!pty.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }

  const std::vector<std::pair<int, speed_t>> rates = {
      {     50,      B50},
      {     75,      B75},
      {    110,     B110},
      {    134,     B134},
      {    150,     B150},
      {    200,     B200},
      {    300,     B300},
      {    600,     B600},
      {   1200,    B1200},
      {   1800,    B1800},
      {   2400,    B2400},
      {   4800,    B4800},
      {   9600,    B9600},
      {  19200,   B19200},
      {  38400,   B38400},
      {  57600,   B57600},
      { 115200,  B115200},
      { 230400,  B230400},
#ifdef B460800
      { 460800,  B460800},
#endif
#ifdef B500000
      { 500000,  B500000},
#endif
#ifdef B4000000
      { 576000,  B576000},
      { 921600,  B921600},
      {1000000, B1000000},
      {1152000, B1152000},
      {1500000, B1500000},
      {2000000, B2000000},
      {2500000, B2500000},
      {3000000, B3000000},
      {3500000, B3500000},
      {4000000, B4000000},
#endif
  };

  CSerialPort port(pty.slave());
  ASSERT_TRUE(port.isOpen());
  for (const auto& [rate, expected] : rates)
  {
    ASSERT_NO_THROW(port.setConfig(rate)) << "rate " << rate;
    termios t{};
    ASSERT_TRUE(readTermios(pty.slave(), t));
    EXPECT_EQ(cfgetospeed(&t), expected) << "rate " << rate;
    EXPECT_EQ(cfgetispeed(&t), expected) << "rate " << rate;
  }
}

TEST(CSerialPort, ParityStopBitsAndFlowControl)
{
  Pty pty;
  if (!pty.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  CSerialPort port(pty.slave());
  ASSERT_TRUE(port.isOpen());

  // (Linux pseudo-terminals always force 8 data bits and no parity, so only
  // the other settings can be observed here)
  termios t{};
  // parity: 0:none, 1:odd, 2:even
  port.setConfig(9600, 1, 8, 2, true);
  ASSERT_TRUE(readTermios(pty.slave(), t));
  EXPECT_NE(t.c_cflag & PARODD, 0u) << "odd parity";
  EXPECT_NE(t.c_cflag & CSTOPB, 0u) << "2 stop bits";
  EXPECT_NE(t.c_cflag & CRTSCTS, 0u) << "hardware flow control";

  port.setConfig(115200, 2, 8, 1, false);
  ASSERT_TRUE(readTermios(pty.slave(), t));
  EXPECT_EQ(t.c_cflag & PARODD, 0u) << "even parity";
  EXPECT_EQ(t.c_cflag & CSTOPB, 0u) << "1 stop bit";
  EXPECT_EQ(t.c_cflag & CRTSCTS, 0u) << "no flow control";
}

TEST(CSerialPort, ConfigurationErrors)
{
  // Not open:
  {
    CSerialPort closed;
    EXPECT_ANY_THROW(closed.setConfig(9600));
  }
  Pty pty;
  if (!pty.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  CSerialPort port(pty.slave());
  ASSERT_TRUE(port.isOpen());
  EXPECT_ANY_THROW(port.setConfig(0));
  EXPECT_ANY_THROW(port.setConfig(-9600));
  // Not a valid number of data bits, parity or stop bits:
  EXPECT_ANY_THROW(port.setConfig(9600, 0, 9, 1));
  EXPECT_ANY_THROW(port.setConfig(9600, 7, 8, 1));
  EXPECT_ANY_THROW(port.setConfig(9600, 0, 8, 3));
}
#endif  // Linux or macOS
