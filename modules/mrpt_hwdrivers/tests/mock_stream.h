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

#include <mrpt/core/exceptions.h>
#include <mrpt/io/CStream.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <string>

namespace mrpt::hwdrivers::testing
{
/** A CStream standing in for a serial port or a socket, so that sensor drivers
 *  can be exercised without any hardware attached.
 *
 *  Two ways to feed the driver:
 *   - pushRx(): queue bytes to be returned by the next Read() calls.
 *   - replyTo(): register a canned answer that is queued as soon as the driver
 *     writes that exact command, which is what request/response protocols need.
 *
 *  Everything the driver writes is kept in tx() for assertions.
 */
class MockStream : public mrpt::io::CStream
{
 public:
  MockStream() = default;

  /** Queue bytes for the driver to read. */
  void pushRx(const std::string& data) { m_rx += data; }

  /** Queue `reply` as soon as the driver writes exactly `cmd`. */
  void replyTo(const std::string& cmd, const std::string& reply) { m_replies[cmd] = reply; }

  /** All the bytes written by the driver so far. */
  const std::string& tx() const { return m_tx; }

  void clearTx() { m_tx.clear(); }

  /** Pending bytes not yet read by the driver. */
  size_t pendingRx() const { return m_rx.size() - m_rxPos; }

  /** Make the next Read() return 0 bytes, to exercise timeout paths. */
  void setReadsReturnNothing(bool b) { m_readsReturnNothing = b; }

  /** Make Write() throw, to exercise I/O error paths. */
  void setWritesThrow(bool b) { m_writesThrow = b; }

  size_t Read(void* Buffer, size_t Count) override
  {
    if (m_readsReturnNothing)
    {
      return 0;
    }

    const size_t avail = m_rx.size() - m_rxPos;
    const size_t n = std::min(Count, avail);
    if (n > 0)
    {
      std::memcpy(Buffer, &m_rx[m_rxPos], n);
    }
    m_rxPos += n;
    return n;
  }

  size_t Write(const void* Buffer, size_t Count) override
  {
    if (m_writesThrow)
    {
      THROW_EXCEPTION("MockStream: simulated write error");
    }

    const std::string chunk(static_cast<const char*>(Buffer), Count);
    m_tx += chunk;

    if (auto it = m_replies.find(chunk); it != m_replies.end())
    {
      m_rx += it->second;
    }

    return Count;
  }

  uint64_t Seek(int64_t /*Offset*/, CStream::TSeekOrigin /*Origin*/ = sFromBeginning) override
  {
    return 0;
  }
  uint64_t getTotalBytesCount() const override { return m_rx.size(); }
  uint64_t getPosition() const override { return m_rxPos; }

 private:
  std::string m_rx;
  std::string m_tx;
  size_t m_rxPos = 0;
  bool m_readsReturnNothing = false;
  bool m_writesThrow = false;
  std::map<std::string, std::string> m_replies;
};

}  // namespace mrpt::hwdrivers::testing
