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

#include <memory>
#include <string>

namespace comms_test
{
/** Opens a listening socket on 127.0.0.1 at some free port, so that tests never
 * depend on one particular port being available. Returns a null pointer if no
 * port in the search range could be bound. */
std::unique_ptr<mrpt::comms::CServerTCPSocket> listenOnFreePort(unsigned short& outPort);

/** A one-shot TCP server that accepts a single connection, reads whatever the
 * client sends and answers with a canned reply, then hangs up. Used to drive
 * the HTTP client in net_utils without needing a network. */
class OneShotServer
{
 public:
  /** Starts the server thread. `isReady()` tells whether a port was bound. */
  explicit OneShotServer(std::string reply);
  ~OneShotServer();

  OneShotServer(const OneShotServer&) = delete;
  OneShotServer& operator=(const OneShotServer&) = delete;
  OneShotServer(OneShotServer&&) = delete;
  OneShotServer& operator=(OneShotServer&&) = delete;

  [[nodiscard]] bool isReady() const;
  [[nodiscard]] unsigned short port() const;

  /** Blocks until the server thread is done, then returns what the client
   * sent. */
  [[nodiscard]] std::string waitAndGetRequest();

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace comms_test
