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

const char* nv_solution_status_t_str[] = {
	"SOL_COMPUTED",
	"INSUFFICIENT_OBS",
	"NO_CONVERGENCE",
	"SINGULARITY",
	"COV_TRACE",
	"TEST_DIST",
	"COLD_START",
	"V_H_LIMIT",
	"VARIANCE",
	"RESIDUALS",
	"DELTA_POS",
	"NEGATIVE_VAR",
	"(UNKNOWN)",
	"INTEGRITY_WARNING",
	"INS_INACTIVE",
	"INS_ALIGNING",
	"INS_BAD",
	"IMU_UNPLUGGED",
	"PENDING",
	"INVALID_FIX"
};

void Message_NV_OEM6_BESTPOS::dumpToStream( mrpt::utils::CStream &out ) const
{
	static bool init_map = false;
	static std::map<int,const char*> nv_position_type_t_str;
	if (!init_map) 
	{
		init_map = true;
#define DEF_POS_TYPE_STR(_NAME) nv_position_type_t_str[nv_oem6_position_type::_NAME] = #_NAME;

		DEF_POS_TYPE_STR(NONE)
		DEF_POS_TYPE_STR(FIXEDPOS)
		DEF_POS_TYPE_STR(FIXEDHEIGHT)
		DEF_POS_TYPE_STR(Reserved)
		DEF_POS_TYPE_STR(FLOATCONV)
		DEF_POS_TYPE_STR(WIDELANE)
		DEF_POS_TYPE_STR(NARROWLANE)
		DEF_POS_TYPE_STR(DOPPLER_VELOCITY)
		DEF_POS_TYPE_STR(SINGLE)
		DEF_POS_TYPE_STR(PSRDIFF)
		DEF_POS_TYPE_STR(WAAS)
		DEF_POS_TYPE_STR(PROPOGATED)
		DEF_POS_TYPE_STR(OMNISTAR)
		DEF_POS_TYPE_STR(L1_FLOAT)
		DEF_POS_TYPE_STR(IONOFREE_FLOAT)
		DEF_POS_TYPE_STR(NARROW_FLOAT)
		DEF_POS_TYPE_STR(L1_INT)
		DEF_POS_TYPE_STR(WIDE_INT)
		DEF_POS_TYPE_STR(NARROW_INT)
		DEF_POS_TYPE_STR(RTK_DIRECT_INS)
		DEF_POS_TYPE_STR(INS)
		DEF_POS_TYPE_STR(INS_PSRSP)
		DEF_POS_TYPE_STR(INS_PSRDIFF)
		DEF_POS_TYPE_STR(INS_RTKFLOAT)
		DEF_POS_TYPE_STR(INS_RTKFIXED)
		DEF_POS_TYPE_STR(OMNISTAR_HP)
		DEF_POS_TYPE_STR(OMNISTAR_XP)
		DEF_POS_TYPE_STR(CDGPS)
	}

	out.printf("[Novatel OEM6 BESTPOS]\n");
	out.printf(" Solution status: %u\n", 
		((unsigned)fields.solution_stat<sizeof(nv_solution_status_t_str)/sizeof(nv_solution_status_t_str[0])) ? 
			nv_solution_status_t_str[(unsigned)fields.solution_stat] : "???" );
	out.printf(" Position type  : %u\n", nv_position_type_t_str[(unsigned)fields.position_type]);
	out.printf(" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
		fields.lon,
		fields.lat,
		fields.hgt );
}

// ------------
Message_NV_OEM6_INSPVAS::content_t::content_t() 
{
}

void Message_NV_OEM6_INSPVAS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 INSPVAS]\n");
	out.printf(" INS status: %u\n", (unsigned)fields.ins_status);
	out.printf(" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n", fields.lon, fields.lat, fields.hgt );
	out.printf(" Velocities: Nort: %.05f  East: %.05f  Up: %.05f\n", fields.vel_north, fields.vel_east, fields.vel_up);
	out.printf(" Attitude: Roll: %.05f  Pitch: %.05f  Azimuth: %.05f\n", fields.roll, fields.pitch, fields.azimuth);
}


