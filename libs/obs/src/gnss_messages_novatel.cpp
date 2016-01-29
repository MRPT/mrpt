/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_novatel.h>
#include <map>

using namespace std;
using namespace mrpt::obs::gnss;

nv_oem6_header_t::nv_oem6_header_t() :
	hdr_len(0),
	msg_id(0),
	msg_type(0),
	port_addr(0),
	msg_len(0),
	seq_number(0),
	idle_percent(0),
	time_status(0),
	week(0),
	ms_in_week(0),
	receiver_status(0),
	reserved(0),
	receiver_sw_version(0)
{
	synch[0]=synch[1]=synch[2]=0;
}

nv_oem6_short_header_t::nv_oem6_short_header_t() :
	msg_len(),
	msg_id(),
	week(),
	ms_in_week()
{
	synch[0]=synch[1]=synch[2]=0;
}


// ------------
void Message_NV_OEM6_GENERIC_FRAME::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 GENERIC FRAME]\n");
	out.printf(" Message ID: %u\n", (unsigned)this->header.msg_id);
}
void Message_NV_OEM6_GENERIC_FRAME::internal_writeToStream(mrpt::utils::CStream &out) const
{
	out.WriteBuffer(&header,sizeof(header));
	out << static_cast<uint32_t>(msg_body.size());
	if (!msg_body.empty())
		out.WriteBuffer(&msg_body[0],msg_body.size());
}
void Message_NV_OEM6_GENERIC_FRAME::internal_readFromStream(mrpt::utils::CStream &in)
{
	in.ReadBuffer(&header,sizeof(header));
	uint32_t nBytesInStream;
	in >> nBytesInStream;
	msg_body.resize(nBytesInStream);
	if (nBytesInStream)
		in.ReadBuffer(&msg_body[0],sizeof(nBytesInStream));
}
// ------------
void Message_NV_OEM6_GENERIC_FRAME_SHORT::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 GENERIC SHORT FRAME]\n");
	out.printf(" Message ID: %u\n", (unsigned)this->header.msg_id);
}
void Message_NV_OEM6_GENERIC_FRAME_SHORT::internal_writeToStream(mrpt::utils::CStream &out) const
{
	out.WriteBuffer(&header,sizeof(header));
	out << static_cast<uint32_t>(msg_body.size());
	if (!msg_body.empty())
		out.WriteBuffer(&msg_body[0],msg_body.size());
}
void Message_NV_OEM6_GENERIC_FRAME_SHORT::internal_readFromStream(mrpt::utils::CStream &in)
{
	in.ReadBuffer(&header,sizeof(header));
	uint32_t nBytesInStream;
	in >> nBytesInStream;
	msg_body.resize(nBytesInStream);
	if (nBytesInStream)
		in.ReadBuffer(&msg_body[0],sizeof(nBytesInStream));
}

// ------------
Message_NV_OEM6_BESTPOS::content_t::content_t() :
	solution_stat(), position_type(),
	lat(),lon(),hgt(),
	undulation(), datum_id(),
	lat_sigma(), lon_sigma(), hgt_sigma(),
	diff_age(), sol_age(),
	num_sats_tracked(), num_sats_sol(), num_sats_sol_L1(), num_sats_sol_multi(),
	reserved(), ext_sol_stat(),
	galileo_beidou_mask(),gps_glonass_mask(),
	crc()
{
	base_station_id[0]=0;
}

const std::string OBS_IMPEXP & nv_oem6_solution_status::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int,std::string> val2str;
	if (!init_map) {
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_solution_status::_NAME] = #_NAME;
	DEF_TYPESTR(SOL_COMPUTED)	DEF_TYPESTR(INSUFFICIENT_OBS)	DEF_TYPESTR(NO_CONVERGENCE)
	DEF_TYPESTR(SINGULARITY)	DEF_TYPESTR(COV_TRACE)			DEF_TYPESTR(TEST_DIST)
	DEF_TYPESTR(COLD_START)		DEF_TYPESTR(V_H_LIMIT)			DEF_TYPESTR(VARIANCE)
	DEF_TYPESTR(RESIDUALS)		DEF_TYPESTR(DELTA_POS)			DEF_TYPESTR(NEGATIVE_VAR)
	DEF_TYPESTR(INTEGRITY_WARNING) DEF_TYPESTR(INS_INACTIVE)	DEF_TYPESTR(INS_ALIGNING)
	DEF_TYPESTR(INS_BAD)		DEF_TYPESTR(IMU_UNPLUGGED)		DEF_TYPESTR(PENDING) DEF_TYPESTR(INVALID_FIX)
#undef DEF_TYPESTR
	}
	std::map<int,std::string>::const_iterator it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}

const std::string OBS_IMPEXP & nv_oem6_position_type::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int,std::string> val2str;
	if (!init_map) {
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_position_type::_NAME] = #_NAME;
		DEF_TYPESTR(NONE)			DEF_TYPESTR(FIXEDPOS)		DEF_TYPESTR(FIXEDHEIGHT)	DEF_TYPESTR(Reserved)
		DEF_TYPESTR(FLOATCONV)		DEF_TYPESTR(WIDELANE)		DEF_TYPESTR(NARROWLANE)	DEF_TYPESTR(DOPPLER_VELOCITY)
		DEF_TYPESTR(SINGLE)		DEF_TYPESTR(PSRDIFF)		DEF_TYPESTR(WAAS)			DEF_TYPESTR(PROPOGATED)
		DEF_TYPESTR(OMNISTAR)		DEF_TYPESTR(L1_FLOAT)		DEF_TYPESTR(IONOFREE_FLOAT)	DEF_TYPESTR(NARROW_FLOAT)
		DEF_TYPESTR(L1_INT)		DEF_TYPESTR(WIDE_INT)		DEF_TYPESTR(NARROW_INT)		DEF_TYPESTR(RTK_DIRECT_INS)
		DEF_TYPESTR(INS)			DEF_TYPESTR(INS_PSRSP)		DEF_TYPESTR(INS_PSRDIFF)		DEF_TYPESTR(INS_RTKFLOAT)
		DEF_TYPESTR(INS_RTKFIXED)	DEF_TYPESTR(OMNISTAR_HP)	DEF_TYPESTR(OMNISTAR_XP)		DEF_TYPESTR(CDGPS)
#undef DEF_TYPESTR
	}
	std::map<int,std::string>::const_iterator it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}

const std::string OBS_IMPEXP & nv_oem6_ins_status_type::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int,std::string> val2str;
	if (!init_map) {
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_ins_status_type::_NAME] = #_NAME;
		DEF_TYPESTR(INS_INACTIVE)		DEF_TYPESTR(INS_ALIGNING)		DEF_TYPESTR(INS_HIGH_VARIANCE)
		DEF_TYPESTR(INS_SOLUTION_GOOD)	DEF_TYPESTR(INS_SOLUTION_FREE)	DEF_TYPESTR(INS_ALIGNMENT_COMPLETE)
		DEF_TYPESTR(DETERMINING_ORIENTATION)	DEF_TYPESTR(WAITING_INITIALPOS)
#undef DEF_TYPESTR
	}
	std::map<int,std::string>::const_iterator it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}



void Message_NV_OEM6_BESTPOS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 BESTPOS]\n");
	out.printf(" Solution status: `%s`\n", nv_oem6_solution_status::enum2str(fields.solution_stat).c_str() );
	out.printf(" Position type  : `%s`\n", nv_oem6_position_type::enum2str(fields.position_type).c_str() );
	out.printf(" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n", fields.lon, fields.lat, fields.hgt );
}

// ------------
Message_NV_OEM6_INSPVAS::content_t::content_t() 
{
}

void Message_NV_OEM6_INSPVAS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 INSPVAS]\n");
	out.printf(" INS status: `%s`\n", nv_oem6_ins_status_type::enum2str(fields.ins_status).c_str());
	out.printf(" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n", fields.lon, fields.lat, fields.hgt );
	out.printf(" Velocities: North: %.05f  East: %.05f  Up: %.05f\n", fields.vel_north, fields.vel_east, fields.vel_up);
	out.printf(" Attitude: Roll: %.05f  Pitch: %.05f  Azimuth: %.05f\n", fields.roll, fields.pitch, fields.azimuth);
}


