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
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/CServerTCPSocket.h>
#include <mrpt/serialization/CMessage.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "comms_test_server.h"

using namespace mrpt::comms;
using namespace std::chrono_literals;

namespace
{
/** A connected client/server socket pair over the loopback interface. */
struct SocketPair
{
  std::unique_ptr<CServerTCPSocket> server;
  std::unique_ptr<CClientTCPSocket> serverSide;
  CClientTCPSocket clientSide;

  bool connected = false;

  SocketPair()
  {
    unsigned short port = 0;
    server = comms_test::listenOnFreePort(port);
    if (!server)
    {
      return;
    }

    std::thread th([&]() { serverSide = server->accept(5000); });
    try
    {
      clientSide.connect("127.0.0.1", port, 5000);
    }
    catch (...)
    {
      // The accept thread must be joined before it is destroyed, whatever
      // happens on this side:
      th.join();
      throw;
    }
    th.join();

    connected = serverSide != nullptr;
  }
};
}  // namespace

TEST(CClientTCPSocket, unconnectedSocketIsInert)
{
  CClientTCPSocket sock;

  EXPECT_FALSE(sock.isConnected());
  EXPECT_EQ(sock.getReadPendingBytes(), 0U);

  std::array<char, 4> buf{};
  EXPECT_EQ(sock.readAsync(buf.data(), buf.size(), 100, 100), 0U);
  EXPECT_EQ(sock.writeAsync(buf.data(), buf.size(), 100), 0U);

  // Closing an already-closed socket is a no-op, not an error:
  EXPECT_NO_THROW(sock.close());
}

TEST(CClientTCPSocket, unsupportedStreamOperationsThrow)
{
  CClientTCPSocket sock;

  // A socket is not seekable nor does it have a length:
  EXPECT_ANY_THROW(sock.Seek(0));
  EXPECT_ANY_THROW((void)sock.getTotalBytesCount());
  EXPECT_ANY_THROW((void)sock.getPosition());
}

TEST(CClientTCPSocket, connectToUnresolvableHostThrows)
{
  CClientTCPSocket sock;
  // ".invalid" is reserved by RFC 2606 and must never resolve:
  EXPECT_ANY_THROW(sock.connect("mrpt-unit-test-host.invalid", 80, 1000));
  EXPECT_FALSE(sock.isConnected());
}

TEST(CClientTCPSocket, connectToClosedPortThrows)
{
#ifdef _WIN32
  // connect() waits for the connection attempt to complete only on Linux and
  // Apple; on Windows it goes straight to getsockopt(SO_ERROR), which has not
  // seen the refusal yet, so the failure is never noticed.
  GTEST_SKIP() << "connect() does not detect a refused connection on Windows.";
#else
  // Bind a port and immediately release it, so we know nothing is listening:
  unsigned short port = 0;
  {
    auto s = comms_test::listenOnFreePort(port);
    ASSERT_TRUE(s);
  }

  CClientTCPSocket sock;
  EXPECT_ANY_THROW(sock.connect("127.0.0.1", port, 1000));
  EXPECT_FALSE(sock.isConnected());
#endif
}

TEST(CClientTCPSocket, sendAndReceiveOverLoopback)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  EXPECT_TRUE(p.clientSide.isConnected());
  EXPECT_TRUE(p.serverSide->isConnected());

  const std::string msg = "the quick brown fox";
  p.clientSide.sendString(msg);

  std::array<char, 64> buf{};
  const size_t n = p.serverSide->readAsync(buf.data(), msg.size(), 5000, 500);
  EXPECT_EQ(std::string(buf.data(), n), msg);
}

TEST(CClientTCPSocket, readAsyncTimesOutWhenNothingIsSent)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  std::array<char, 8> buf{};
  // Nobody writes anything: the call must give up and report zero bytes.
  EXPECT_EQ(p.serverSide->readAsync(buf.data(), buf.size(), 200, 100), 0U);
  EXPECT_TRUE(p.serverSide->isConnected());
}

TEST(CClientTCPSocket, readAsyncDetectsRemoteHangUp)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  p.clientSide.close();

  std::array<char, 8> buf{};
  EXPECT_EQ(p.serverSide->readAsync(buf.data(), buf.size(), 2000, 500), 0U);
  // A graceful close on the far end closes this side too:
  EXPECT_FALSE(p.serverSide->isConnected());
}

TEST(CClientTCPSocket, pendingBytesReflectUnreadData)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  const std::string msg = "0123456789";
  p.clientSide.sendString(msg);

  // Give the loopback a moment to deliver:
  size_t pending = 0;
  for (int i = 0; i < 50 && pending < msg.size(); i++)
  {
    pending = p.serverSide->getReadPendingBytes();
    if (pending < msg.size())
    {
      std::this_thread::sleep_for(20ms);
    }
  }
  EXPECT_EQ(pending, msg.size());
}

TEST(CClientTCPSocket, socketOptions)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  // Note: the getter returns whatever the OS stores for the flag, which is
  // only guaranteed to be non-zero when enabled (BSD-derived stacks report a
  // bit mask rather than 1):
  EXPECT_EQ(p.clientSide.setTCPNoDelay(1), 0);
  EXPECT_NE(p.clientSide.getTCPNoDelay(), 0);
  EXPECT_EQ(p.clientSide.setTCPNoDelay(0), 0);
  EXPECT_EQ(p.clientSide.getTCPNoDelay(), 0);

  EXPECT_EQ(p.clientSide.setSOSendBufffer(16384), 0);
  // The kernel is free to round the requested size up, so only require that
  // it reports something sensible back:
  EXPECT_GT(p.clientSide.getSOSendBufffer(), 0);
}

TEST(CClientTCPSocket, socketOptionsOnUnconnectedSocketFail)
{
  CClientTCPSocket sock;
  EXPECT_EQ(sock.getTCPNoDelay(), -1);
}

TEST(CClientTCPSocket, messageRoundTrip)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  mrpt::serialization::CMessage out;
  out.type = 0x42;
  out.content = {1, 2, 3, 4};
  ASSERT_TRUE(p.clientSide.sendMessage(out, 5000));

  mrpt::serialization::CMessage in;
  ASSERT_TRUE(p.serverSide->receiveMessage(in, 5000, 5000));
  EXPECT_EQ(in.type, out.type);
  EXPECT_EQ(in.content, out.content);
}

TEST(CClientTCPSocket, receiveMessageRejectsGarbage)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  // Not the "MRPTMessage" magic word:
  p.clientSide.sendString("NOTAMESSAGE!!!!!!!!!!!!!!");

  mrpt::serialization::CMessage in;
  EXPECT_FALSE(p.serverSide->receiveMessage(in, 2000, 500));
}

TEST(CClientTCPSocket, receiveMessageOnSilentPeerTimesOut)
{
  SocketPair p;
  ASSERT_TRUE(p.connected);

  mrpt::serialization::CMessage in;
  EXPECT_FALSE(p.serverSide->receiveMessage(in, 200, 100));
}

TEST(CServerTCPSocket, listeningAndAcceptTimeout)
{
  unsigned short port = 0;
  auto s = comms_test::listenOnFreePort(port);
  ASSERT_TRUE(s);

  EXPECT_TRUE(s->isListening());
  // Nobody connects: accept() must return an empty pointer, not block forever.
  EXPECT_EQ(s->accept(200), nullptr);
}

TEST(CServerTCPSocket, bindingAnAlreadyUsedPortThrows)
{
  unsigned short port = 0;
  auto s = comms_test::listenOnFreePort(port);
  ASSERT_TRUE(s);

  EXPECT_ANY_THROW(CServerTCPSocket(port, "127.0.0.1", 10, mrpt::system::LVL_ERROR));
}
