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

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/system/crc.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <iostream>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

namespace
{
// Builds the message for a CRC-validated frame (header + body + CRC): the
// dedicated class for its ID if there is one, the generic container otherwise.
template <class GENERIC_FRAME, class HEADER>
gnss::gnss_message_ptr buildNovatelMessage(const HEADER& hdr, const std::vector<uint8_t>& frame)
{
  using namespace mrpt::obs::gnss;

  const auto msg_type = static_cast<gnss_message_type_t>(NV_OEM6_MSG2ENUM + hdr.msg_id);
  if (!gnss_message::FactoryKnowsMsgType(msg_type))
  {
    auto msg = std::make_shared<GENERIC_FRAME>();
    msg->header = hdr;
    const auto body_begin = frame.begin() + sizeof(HEADER);
    msg->msg_body.assign(body_begin, body_begin + hdr.msg_len);
    return msg;
  }

  // Dedicated classes deserialize from: msg type, frame length, raw frame.
  mrpt::io::CMemoryStream tmpStream;
  auto arch = mrpt::serialization::archiveFrom(tmpStream);
  arch << static_cast<int32_t>(msg_type);
  arch << static_cast<uint32_t>(frame.size());
  arch.WriteBuffer(frame.data(), frame.size());

  tmpStream.Seek(0);
  return gnss_message_ptr(gnss_message::readAndBuildFromStream(arch));
}
}  // namespace

bool CGPSInterface::implement_parser_NOVATEL_OEM6(size_t& out_minimum_rx_buf_to_decide)
{
  // to be grabbed from the last Message_NV_OEM6_IONUTC msg
  static uint32_t num_leap_seconds = getenv("MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS") == nullptr
                                         ? 18
                                         : atoi(getenv("MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS"));

  using namespace mrpt::obs::gnss;

  out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);

  const size_t nBytesAval = m_rx_buffer.size();                // Available for read
  if (nBytesAval < out_minimum_rx_buf_to_decide) return true;  // no need to skip 1 byte

  // If the synch bytes do not match, it is not a valid frame:
  uint8_t peek_buffer[3];
  m_rx_buffer.peek_many(&peek_buffer[0], 3);

  // Short header?
  const bool is_short_hdr = peek_buffer[0] == nv_oem6_short_header_t::SYNCH0 &&
                            peek_buffer[1] == nv_oem6_short_header_t::SYNCH1 &&
                            peek_buffer[2] == nv_oem6_short_header_t::SYNCH2;

  const bool is_regular_hdr = peek_buffer[0] == nv_oem6_header_t::SYNCH0 &&
                              peek_buffer[1] == nv_oem6_header_t::SYNCH1 &&
                              peek_buffer[2] == nv_oem6_header_t::SYNCH2;

  if (!is_short_hdr && !is_regular_hdr) return false;  // skip 1 byte, we dont recognize this format

  if (is_short_hdr)
  {
    if (nBytesAval < sizeof(nv_oem6_short_header_t))
    {
      out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);
      return true;  // we must wait for more data in the buffer
    }
    nv_oem6_short_header_t hdr;
    m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
    hdr.fixEndianness();
    const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
    if (nBytesAval < expected_total_msg_len)
    {
      out_minimum_rx_buf_to_decide = expected_total_msg_len;
      return true;  // we must wait for more data in the buffer
    }

    std::vector<uint8_t> buf(expected_total_msg_len);
    m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
    m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/);

    // Check CRC:
    const uint32_t crc_computed = mrpt::system::compute_CRC32(&buf[0], expected_total_msg_len - 4);
    const uint32_t crc_read =
        (buf[expected_total_msg_len - 1] << 24) | (buf[expected_total_msg_len - 2] << 16) |
        (buf[expected_total_msg_len - 3] << 8) | (buf[expected_total_msg_len - 4] << 0);
    if (crc_read != crc_computed) return false;  // skip 1 byte, we dont recognize this format

    const gnss_message_ptr msg = buildNovatelMessage<Message_NV_OEM6_GENERIC_SHORT_FRAME>(hdr, buf);
    if (!msg.get())
    {
      std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error "
                   "parsing binary packet msg_id="
                << hdr.msg_id << "\n";
      return true;
    }
    m_parsed_messages->messages[msg->message_type] = msg;
    m_parsed_messages->originalReceivedTimestamp = mrpt::Clock::now();
    if (!CObservationGPS::GPS_time_to_UTC(
            hdr.week, hdr.ms_in_week * 1e-3, num_leap_seconds, m_parsed_messages->timestamp))
      m_parsed_messages->timestamp = mrpt::Clock::now();
    else
      m_parsed_messages->has_satellite_timestamp = true;

    m_parsed_messages->sensorLabel = msg->getMessageTypeAsString();

    flushParsedMessagesNow();
    return true;
  }  // end short hdr

  if (is_regular_hdr)
  {
    if (nBytesAval < sizeof(nv_oem6_header_t))
    {
      out_minimum_rx_buf_to_decide = sizeof(nv_oem6_header_t);
      return true;  // we must wait for more data in the buffer
    }
    nv_oem6_header_t hdr;
    m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
    hdr.fixEndianness();
    const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
    if (nBytesAval < expected_total_msg_len)
    {
      out_minimum_rx_buf_to_decide = expected_total_msg_len;
      return true;  // we must wait for more data in the buffer
    }

    std::vector<uint8_t> buf(expected_total_msg_len);
    m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
    m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/);

    // Check CRC:
    const uint32_t crc_computed = mrpt::system::compute_CRC32(&buf[0], expected_total_msg_len - 4);
    const uint32_t crc_read =
        (buf[expected_total_msg_len - 1] << 24) | (buf[expected_total_msg_len - 2] << 16) |
        (buf[expected_total_msg_len - 3] << 8) | (buf[expected_total_msg_len - 4] << 0);
    if (crc_read != crc_computed) return false;  // skip 1 byte, we dont recognize this format

    const gnss_message_ptr msg = buildNovatelMessage<Message_NV_OEM6_GENERIC_FRAME>(hdr, buf);
    if (!msg.get())
    {
      std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error "
                   "parsing binary packet msg_id="
                << hdr.msg_id << "\n";
      return true;
    }
    m_parsed_messages->messages[msg->message_type] = msg;
    m_parsed_messages->originalReceivedTimestamp = mrpt::Clock::now();
    {
      // Detect NV_OEM6_IONUTC msgs to learn about the current leap
      // seconds:
      const auto* ionutc = dynamic_cast<const gnss::Message_NV_OEM6_IONUTC*>(msg.get());
      if (ionutc) num_leap_seconds = ionutc->fields.deltat_ls;
    }
    if (!CObservationGPS::GPS_time_to_UTC(
            hdr.week, hdr.ms_in_week * 1e-3, num_leap_seconds, m_parsed_messages->timestamp))
      m_parsed_messages->timestamp = mrpt::Clock::now();
    else
      m_parsed_messages->has_satellite_timestamp = true;

    m_parsed_messages->sensorLabel = msg->getMessageTypeAsString();
    flushParsedMessagesNow();
    return true;
  }  // end regular hdr

  // Shouldnt arrive here, but MSVC complies anyway:
  return false;
}
