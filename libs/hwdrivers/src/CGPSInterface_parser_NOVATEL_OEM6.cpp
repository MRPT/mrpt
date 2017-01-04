/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/crc.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

bool  CGPSInterface::implement_parser_NOVATEL_OEM6(size_t &out_minimum_rx_buf_to_decide)
{
	// to be grabbed from the last Message_NV_OEM6_IONUTC msg
	static uint32_t num_leap_seconds = getenv("MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS")==NULL ? 
		17 : atoi(getenv("MRPT_HWDRIVERS_DEFAULT_LEAP_SECONDS"));

	using namespace mrpt::obs::gnss;

	out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);

	const size_t nBytesAval = m_rx_buffer.size();  // Available for read
	if (nBytesAval<out_minimum_rx_buf_to_decide) 
		return true; // no need to skip 1 byte

	// If the synch bytes do not match, it is not a valid frame:
	uint8_t peek_buffer[3];
	m_rx_buffer.peek_many(&peek_buffer[0],3);
	// Short header?
	const bool is_short_hdr = 
		peek_buffer[0]==nv_oem6_short_header_t::SYNCH0 && 
		peek_buffer[1]==nv_oem6_short_header_t::SYNCH1 && 
		peek_buffer[2]==nv_oem6_short_header_t::SYNCH2;

	const bool is_regular_hdr = 
		peek_buffer[0]==nv_oem6_header_t::SYNCH0 && 
		peek_buffer[1]==nv_oem6_header_t::SYNCH1 && 
		peek_buffer[2]==nv_oem6_header_t::SYNCH2;
			
	if (!is_short_hdr && !is_regular_hdr)
		return false; // skip 1 byte, we dont recognize this format

	if (is_short_hdr)
	{
		if (nBytesAval<sizeof(nv_oem6_short_header_t)) {
			out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);
			return true; // we must wait for more data in the buffer
		}
		nv_oem6_short_header_t hdr;
		m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
		const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
		if (nBytesAval<expected_total_msg_len) {
			out_minimum_rx_buf_to_decide = expected_total_msg_len;
			return true; // we must wait for more data in the buffer
		}

		std::vector<uint8_t> buf(expected_total_msg_len);
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/ );

		// Check CRC:
		const uint32_t crc_computed = mrpt::utils::compute_CRC32(&buf[0], expected_total_msg_len-4);
		const uint32_t crc_read = 
			(buf[expected_total_msg_len-1] << 24) | 
			(buf[expected_total_msg_len-2] << 16) | 
			(buf[expected_total_msg_len-3] << 8) | 
			(buf[expected_total_msg_len-4] << 0);
		if (crc_read!=crc_computed)
			return false; // skip 1 byte, we dont recognize this format

		// Deserialize the message:
		// 1st, test if we have a specific data structure for this msg_id:
		const bool use_generic_container = !gnss_message::FactoryKnowsMsgType( (gnss_message_type_t)(NV_OEM6_MSG2ENUM + hdr.msg_id ) );
		// ------ Serialization format:
		//const int32_t msg_id = message_type;
		//out << msg_id;
		//this->internal_writeToStream(out);  == >  out << static_cast<uint32_t>(DATA_LEN); out.WriteBuffer(DATA_PTR,DATA_LEN); }
		// ------
		mrpt::utils::CMemoryStream tmpStream;
		const uint32_t msg_id = use_generic_container ? 
			(uint32_t)(NV_OEM6_GENERIC_SHORT_FRAME)
			: 
			(uint32_t) hdr.msg_id+NV_OEM6_MSG2ENUM;
		tmpStream << (uint32_t )(msg_id);
		tmpStream << (uint32_t)(expected_total_msg_len);  // This len = hdr + hdr.msg_len + 4 (crc);
		tmpStream.WriteBuffer(&buf[0],buf.size());

		tmpStream.Seek(0);
		gnss_message_ptr msg( gnss_message::readAndBuildFromStream(tmpStream) );
		if (!msg.get()) {
			std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error parsing binary packet msg_id="<< hdr.msg_id<<"\n";
			return true;
		}
		m_just_parsed_messages.messages[msg->message_type] = msg;
		m_just_parsed_messages.originalReceivedTimestamp = mrpt::system::now();
		if (!CObservationGPS::GPS_time_to_UTC(hdr.week,hdr.ms_in_week*1e-3,num_leap_seconds, m_just_parsed_messages.timestamp))
			m_just_parsed_messages.timestamp =  mrpt::system::now();
		else m_just_parsed_messages.has_satellite_timestamp = true;

		m_just_parsed_messages.sensorLabel = msg->getMessageTypeAsString();

		flushParsedMessagesNow();
		return true;
	} // end short hdr

	if (is_regular_hdr)
	{
		if (nBytesAval<sizeof(nv_oem6_header_t)) {
			out_minimum_rx_buf_to_decide = sizeof(nv_oem6_header_t);
			return true; // we must wait for more data in the buffer
		}
		nv_oem6_header_t hdr;
		m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
		const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
		if (nBytesAval<expected_total_msg_len)
		{
			out_minimum_rx_buf_to_decide = expected_total_msg_len;
			return true; // we must wait for more data in the buffer
		}

		std::vector<uint8_t> buf(expected_total_msg_len);
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/ );

		// Check CRC:
		const uint32_t crc_computed = mrpt::utils::compute_CRC32(&buf[0], expected_total_msg_len-4);
		const uint32_t crc_read = 
			(buf[expected_total_msg_len-1] << 24) | 
			(buf[expected_total_msg_len-2] << 16) | 
			(buf[expected_total_msg_len-3] << 8) | 
			(buf[expected_total_msg_len-4] << 0);
		if (crc_read!=crc_computed)
			return false; // skip 1 byte, we dont recognize this format

		// Deserialize the message:
		// 1st, test if we have a specific data structure for this msg_id:
		const bool use_generic_container = !gnss_message::FactoryKnowsMsgType( (gnss_message_type_t)(NV_OEM6_MSG2ENUM + hdr.msg_id ) );
		// ------ Serialization format:
		//const int32_t msg_id = message_type;
		//out << msg_id;
		//this->internal_writeToStream(out);  == >  out << static_cast<uint32_t>(DATA_LEN); out.WriteBuffer(DATA_PTR,DATA_LEN); }
		// ------
		mrpt::utils::CMemoryStream tmpStream;
		const int32_t msg_id = use_generic_container ? 
			(uint32_t)(NV_OEM6_GENERIC_FRAME)
			: 
			(uint32_t) hdr.msg_id+NV_OEM6_MSG2ENUM;
		tmpStream << msg_id;
		tmpStream << (uint32_t)(expected_total_msg_len);
		tmpStream.WriteBuffer(&buf[0],buf.size());

		tmpStream.Seek(0);
		gnss_message_ptr msg( gnss_message::readAndBuildFromStream(tmpStream) );
		if (!msg.get()) {
			std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error parsing binary packet msg_id="<< hdr.msg_id<<"\n";
			return true;
		}
		m_just_parsed_messages.messages[msg->message_type] = msg;
		m_just_parsed_messages.originalReceivedTimestamp = mrpt::system::now();
		{
			// Detect NV_OEM6_IONUTC msgs to learn about the current leap seconds:
			const gnss::Message_NV_OEM6_IONUTC *ionutc = dynamic_cast<const gnss::Message_NV_OEM6_IONUTC *>(msg.get());
			if (ionutc) 
				num_leap_seconds = ionutc->fields.deltat_ls;
		}
		if (!CObservationGPS::GPS_time_to_UTC(hdr.week,hdr.ms_in_week*1e-3,num_leap_seconds,m_just_parsed_messages.timestamp))
			m_just_parsed_messages.timestamp =  mrpt::system::now();
		else m_just_parsed_messages.has_satellite_timestamp = true;

		m_just_parsed_messages.sensorLabel = msg->getMessageTypeAsString();
		flushParsedMessagesNow();
		return true;
	} // end regular hdr

	// Shouldnt arrive here, but MSVC complies anyway:
	return false;
}
