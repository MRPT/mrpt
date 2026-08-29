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

#include "comms_test_server.h"

#include <array>
#include <exception>
#include <thread>
#include <utility>

using namespace comms_test;

namespace
{
/** Ports are picked from this range, high enough to be outside the usual
 * ephemeral and well-known ranges. */
constexpr unsigned short FIRST_PORT = 18500;
constexpr unsigned short LAST_PORT = 18599;
}  // namespace

std::unique_ptr<mrpt::comms::CServerTCPSocket> comms_test::listenOnFreePort(unsigned short& outPort)
{
  for (unsigned short p = FIRST_PORT; p <= LAST_PORT; p++)
  {
    try
    {
      auto s = std::make_unique<mrpt::comms::CServerTCPSocket>(
          p, "127.0.0.1", 10, mrpt::system::LVL_ERROR);
      if (s->isListening())
      {
        outPort = p;
        return s;
      }
    }
    catch (const std::exception&)
    {
      // Port in use: try the next one.
    }
  }
  outPort = 0;
  return {};
}

struct OneShotServer::Impl
{
  std::unique_ptr<mrpt::comms::CServerTCPSocket> server;
  unsigned short port = 0;
  std::thread th;
  std::string request;
};

OneShotServer::OneShotServer(std::string reply) : m_impl(std::make_unique<Impl>())
{
  m_impl->server = listenOnFreePort(m_impl->port);
  if (!m_impl->server)
  {
    return;
  }

  m_impl->th = std::thread(
      [this, reply = std::move(reply)]()
      {
        try
        {
          auto client = m_impl->server->accept(5000);
          if (!client)
          {
            return;
          }

          // One read is enough for the small requests these tests send: the
          // "between bytes" timeout ends it once the client goes quiet.
          std::array<char, 8192> buf{};
          const size_t n = client->readAsync(buf.data(), buf.size(), 3000, 200);
          m_impl->request.assign(buf.data(), n);

          client->sendString(reply);
          client->close();
        }
        catch (const std::exception&)
        {
          // Leave the request empty; the test will notice.
        }
      });
}

OneShotServer::~OneShotServer()
{
  if (m_impl->th.joinable())
  {
    m_impl->th.join();
  }
}

bool OneShotServer::isReady() const { return m_impl->server != nullptr; }

unsigned short OneShotServer::port() const { return m_impl->port; }

std::string OneShotServer::waitAndGetRequest()
{
  if (m_impl->th.joinable())
  {
    m_impl->th.join();
  }
  return m_impl->request;
}
