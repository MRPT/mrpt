/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/gnss_messages_novatel.h>
#include <map>
#include <iostream>

using namespace std;
using namespace mrpt::obs::gnss;

// ------------
void Message_NV_OEM6_GENERIC_FRAME::dumpToStream(std::ostream& out) const
{
	out << mrpt::format(
		"[Novatel OEM6 GENERIC FRAME]\n"
		" Message ID: %u\n",
		(unsigned)this->header.msg_id);
}
void Message_NV_OEM6_GENERIC_FRAME::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	out.WriteBuffer(&header, sizeof(header));
	out << static_cast<uint32_t>(msg_body.size());
	if (!msg_body.empty()) out.WriteBuffer(&msg_body[0], msg_body.size());
}
void Message_NV_OEM6_GENERIC_FRAME::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	in.ReadBuffer(&header, sizeof(header));
	uint32_t nBytesInStream;
	in >> nBytesInStream;
	msg_body.resize(nBytesInStream);
	if (nBytesInStream) in.ReadBuffer(&msg_body[0], sizeof(nBytesInStream));
}
// ------------
void Message_NV_OEM6_GENERIC_SHORT_FRAME::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 GENERIC SHORT FRAME]\n");
	out << mrpt::format(" Message ID: %u\n", (unsigned)this->header.msg_id);
}
void Message_NV_OEM6_GENERIC_SHORT_FRAME::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	out.WriteBuffer(&header, sizeof(header));
	out << static_cast<uint32_t>(msg_body.size());
	if (!msg_body.empty()) out.WriteBuffer(&msg_body[0], msg_body.size());
}
void Message_NV_OEM6_GENERIC_SHORT_FRAME::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	in.ReadBuffer(&header, sizeof(header));
	uint32_t nBytesInStream;
	in >> nBytesInStream;
	msg_body.resize(nBytesInStream);
	if (nBytesInStream) in.ReadBuffer(&msg_body[0], sizeof(nBytesInStream));
}

// ------------
const std::string& nv_oem6_solution_status::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int, std::string> val2str;
	if (!init_map)
	{
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_solution_status::_NAME] = #_NAME;
		DEF_TYPESTR(SOL_COMPUTED)
		DEF_TYPESTR(INSUFFICIENT_OBS)
		DEF_TYPESTR(NO_CONVERGENCE)
		DEF_TYPESTR(SINGULARITY)
		DEF_TYPESTR(COV_TRACE)
		DEF_TYPESTR(TEST_DIST)
		DEF_TYPESTR(COLD_START)
		DEF_TYPESTR(V_H_LIMIT)
		DEF_TYPESTR(VARIANCE)
		DEF_TYPESTR(RESIDUALS)
		DEF_TYPESTR(DELTA_POS)
		DEF_TYPESTR(NEGATIVE_VAR)
		DEF_TYPESTR(INTEGRITY_WARNING)
		DEF_TYPESTR(INS_INACTIVE)
		DEF_TYPESTR(INS_ALIGNING)
		DEF_TYPESTR(INS_BAD)
		DEF_TYPESTR(IMU_UNPLUGGED)
		DEF_TYPESTR(PENDING)
		DEF_TYPESTR(INVALID_FIX)
#undef DEF_TYPESTR
	}
	auto it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}

const std::string& nv_oem6_position_type::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int, std::string> val2str;
	if (!init_map)
	{
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_position_type::_NAME] = #_NAME;
		DEF_TYPESTR(NONE)
		DEF_TYPESTR(FIXEDPOS)
		DEF_TYPESTR(FIXEDHEIGHT)
		DEF_TYPESTR(Reserved)
		DEF_TYPESTR(FLOATCONV)
		DEF_TYPESTR(WIDELANE)
		DEF_TYPESTR(NARROWLANE)
		DEF_TYPESTR(DOPPLER_VELOCITY)
		DEF_TYPESTR(SINGLE)
		DEF_TYPESTR(PSRDIFF)
		DEF_TYPESTR(WAAS)
		DEF_TYPESTR(PROPAGATED)
		DEF_TYPESTR(OMNISTAR)
		DEF_TYPESTR(L1_FLOAT)
		DEF_TYPESTR(IONOFREE_FLOAT)
		DEF_TYPESTR(NARROW_FLOAT)
		DEF_TYPESTR(L1_INT)
		DEF_TYPESTR(WIDE_INT)
		DEF_TYPESTR(NARROW_INT)
		DEF_TYPESTR(RTK_DIRECT_INS)
		DEF_TYPESTR(INS)
		DEF_TYPESTR(INS_PSRSP)
		DEF_TYPESTR(INS_PSRDIFF)
		DEF_TYPESTR(INS_RTKFLOAT)
		DEF_TYPESTR(INS_RTKFIXED)
		DEF_TYPESTR(OMNISTAR_HP)
		DEF_TYPESTR(OMNISTAR_XP)
		DEF_TYPESTR(CDGPS)
#undef DEF_TYPESTR
	}
	auto it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}

const std::string& nv_oem6_ins_status_type::enum2str(int val)
{
	static bool init_map = false;
	static std::map<int, std::string> val2str;
	if (!init_map)
	{
		init_map = true;
#define DEF_TYPESTR(_NAME) val2str[nv_oem6_ins_status_type::_NAME] = #_NAME;
		DEF_TYPESTR(INS_INACTIVE)
		DEF_TYPESTR(INS_ALIGNING)
		DEF_TYPESTR(INS_HIGH_VARIANCE)
		DEF_TYPESTR(INS_SOLUTION_GOOD)
		DEF_TYPESTR(INS_SOLUTION_FREE)
		DEF_TYPESTR(INS_ALIGNMENT_COMPLETE)
		DEF_TYPESTR(DETERMINING_ORIENTATION)
		DEF_TYPESTR(WAITING_INITIALPOS)
#undef DEF_TYPESTR
	}
	auto it = val2str.find(val);
	static const std::string nullstr("???");
	return (it == val2str.end()) ? nullstr : it->second;
}

void generic_dump_BESTPOS(
	const Message_NV_OEM6_BESTPOS::content_t& fields, std::ostream& out)
{
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
	out << mrpt::format(
		" Solution status: `%s`\n",
		nv_oem6_solution_status::enum2str(fields.solution_stat).c_str());
	out << mrpt::format(
		" Position type  : `%s`\n",
		nv_oem6_position_type::enum2str(fields.position_type).c_str());
	out << mrpt::format(
		" Longitude: %.09f deg (std dev: %.06f m)  Latitude: %.09f deg (std "
		"dev: %.06f m)\n",
		fields.lon, fields.lon_sigma, fields.lat, fields.lat_sigma);
	out << mrpt::format(
		" Height (sea level): %.06f m (std dev: %.06f m) Undulation: %.04f m   "
		"(Sum: %.04f m)\n",
		fields.hgt, fields.hgt_sigma, fields.undulation,
		fields.hgt + fields.undulation);
	out << mrpt::format(
		" Diff age: %.03f  Solution age: %.03f\n", fields.diff_age,
		fields.sol_age);
	out << mrpt::format(
		" Base station ID: `%.*s`\n", 4, fields.base_station_id);
	out << mrpt::format(
		" Num sat tracked: %u  Num sat in solution: %u\n",
		(unsigned)fields.num_sats_tracked, (unsigned)fields.num_sats_sol);
}

void Message_NV_OEM6_BESTPOS::dumpToStream(std::ostream& out) const
{
	out << "[Novatel OEM6 BESTPOS]\n";
	generic_dump_BESTPOS(fields, out);
}

bool Message_NV_OEM6_BESTPOS::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms solution_stat position_type lon_deg lat_deg hgt_m "
		 "undulation_m lon_sigma_m lat_sigma_m hgt_sigma_m diff_age sol_age "
		 "num_sats_tracked num_sats_sol";
	return true;
}
bool Message_NV_OEM6_BESTPOS::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%u.%08u %u %u %.09f %.09f %.06f %.04f %.06f %.06f %.06f %.03f %.03f "
		"%u %u",
		(unsigned)fields.header.week, (unsigned)(fields.header.ms_in_week),
		(unsigned)fields.solution_stat, (unsigned)fields.position_type,
		fields.lon, fields.lat, fields.hgt, fields.undulation, fields.lon_sigma,
		fields.lat_sigma, fields.hgt_sigma, fields.diff_age, fields.sol_age,
		(unsigned)fields.num_sats_tracked, (unsigned)fields.num_sats_sol);
	return true;
}

// ------------
void Message_NV_OEM6_INSPVAS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 INSPVAS]\n");
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
	out << mrpt::format(
		" INS status: `%s`\n",
		nv_oem6_ins_status_type::enum2str(fields.ins_status).c_str());
	out << mrpt::format(
		" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
		fields.lon, fields.lat, fields.hgt);
	out << mrpt::format(
		" Velocities: North: %.05f  East: %.05f  Up: %.05f\n", fields.vel_north,
		fields.vel_east, fields.vel_up);
	out << mrpt::format(
		" Attitude: Roll: %.05f  Pitch: %.05f  Azimuth: %.05f\n", fields.roll,
		fields.pitch, fields.azimuth);
}

bool Message_NV_OEM6_INSPVAS::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms ins_status lon_deg lat_deg ellip_height_WGS84 "
		 "vel_north vel_east vel_up roll_deg pitch_deg azimuth_deg";
	return true;
}
bool Message_NV_OEM6_INSPVAS::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%u.%08u %u %.09f %.09f %.06f %.05f %.05f %.05f %.05f %.05f %.05f",
		(unsigned)fields.header.week, (unsigned)(fields.header.ms_in_week),
		(unsigned)fields.ins_status, fields.lon, fields.lat, fields.hgt,
		fields.vel_north, fields.vel_east, fields.vel_up, fields.roll,
		fields.pitch, fields.azimuth);
	return true;
}

// ------------
void Message_NV_OEM6_INSCOVS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 INSCOVS]\n");
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
	out << mrpt::format(
		" Position cov: %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f "
		"%9.03f %9.03f\n",
		fields.pos_cov[0], fields.pos_cov[1], fields.pos_cov[2],
		fields.pos_cov[3], fields.pos_cov[4], fields.pos_cov[5],
		fields.pos_cov[6], fields.pos_cov[7], fields.pos_cov[8]);
	out << mrpt::format(
		" Attitude cov: %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f "
		"%9.03f %9.03f\n",
		fields.att_cov[0], fields.att_cov[1], fields.att_cov[2],
		fields.att_cov[3], fields.att_cov[4], fields.att_cov[5],
		fields.att_cov[6], fields.att_cov[7], fields.att_cov[8]);
	out << mrpt::format(
		" Velocity cov: %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f "
		"%9.03f %9.03f\n",
		fields.vel_cov[0], fields.vel_cov[1], fields.vel_cov[2],
		fields.vel_cov[3], fields.vel_cov[4], fields.vel_cov[5],
		fields.vel_cov[6], fields.vel_cov[7], fields.vel_cov[8]);
}

bool Message_NV_OEM6_INSCOVS::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms pos_cov(*9) att-cov(*9) vel_cov(*9)";
	return true;
}
bool Message_NV_OEM6_INSCOVS::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%u.%08u "
		"%9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f "
		"%9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f "
		"%9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f  %9.03f %9.03f %9.03f ",
		(unsigned)fields.header.week, (unsigned)(fields.header.ms_in_week),
		fields.pos_cov[0], fields.pos_cov[1], fields.pos_cov[2],
		fields.pos_cov[3], fields.pos_cov[4], fields.pos_cov[5],
		fields.pos_cov[6], fields.pos_cov[7], fields.pos_cov[8],
		fields.att_cov[0], fields.att_cov[1], fields.att_cov[2],
		fields.att_cov[3], fields.att_cov[4], fields.att_cov[5],
		fields.att_cov[6], fields.att_cov[7], fields.att_cov[8],
		fields.vel_cov[0], fields.vel_cov[1], fields.vel_cov[2],
		fields.vel_cov[3], fields.vel_cov[4], fields.vel_cov[5],
		fields.vel_cov[6], fields.vel_cov[7], fields.vel_cov[8]);
	return true;
}

// ------------
void Message_NV_OEM6_RANGECMP::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 RANGECMP]\n");
	out << mrpt::format(
		" Number of SAT observations: %u\n",
		static_cast<unsigned int>(this->num_obs));
}

void Message_NV_OEM6_RANGECMP::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint32_t msg_len = sizeof(header) + header.msg_len + 4;
	out << msg_len;
	out.WriteBuffer(&header, sizeof(header));
	out << num_obs;
	ASSERT_EQUAL_(num_obs, obs_data.size());
	if (num_obs)
		out.WriteBuffer(&obs_data[0], sizeof(obs_data[0]) * obs_data.size());
}

void Message_NV_OEM6_RANGECMP::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	uint32_t expected_msg_len;
	in >> expected_msg_len;
	in.ReadBuffer(&header, sizeof(header));
	in >> num_obs;
	ASSERT_BELOW_(num_obs, 2000);
	obs_data.resize(num_obs);
	if (num_obs)
		in.ReadBuffer(&obs_data[0], sizeof(obs_data[0]) * obs_data.size());
}

// ------------
void Message_NV_OEM6_RXSTATUS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 RXSTATUS]\n");
	out << mrpt::format(
		" Error code: 0x%04X\n", static_cast<unsigned int>(this->fields.error));
}

// ------------
void Message_NV_OEM6_RAWEPHEM::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 RAWEPHEM]\n");
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
}

// ------------
void Message_NV_OEM6_VERSION::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 VERSION]\n");
	out << mrpt::format(
		" Number of components: %u\n",
		static_cast<unsigned int>(this->num_comps));
	for (unsigned i = 0; i < components.size(); i++)
	{
		out << mrpt::format(
			" Component #%u:\n  Model: `%.*s`\n  Serial: `%.*s`\n  SW "
			"version:`%.*s`\n",
			i, (int)sizeof(components[i].model), components[i].model,
			(int)sizeof(components[i].serial), components[i].serial,
			(int)sizeof(components[i].swversion), components[i].swversion);
	}
}

void Message_NV_OEM6_VERSION::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint32_t msg_len = sizeof(header) + header.msg_len + 4;
	out << msg_len;
	out.WriteBuffer(&header, sizeof(header));
	out << num_comps;
	ASSERT_EQUAL_(num_comps, components.size());
	if (num_comps)
		out.WriteBuffer(
			&components[0], sizeof(components[0]) * components.size());
}

void Message_NV_OEM6_VERSION::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	uint32_t expected_msg_len;
	in >> expected_msg_len;
	in.ReadBuffer(&header, sizeof(header));
	in >> num_comps;
	ASSERT_BELOW_(num_comps, 2000);
	components.resize(num_comps);
	if (num_comps)
		in.ReadBuffer(
			&components[0], sizeof(components[0]) * components.size());
}

// ------------
void Message_NV_OEM6_RAWIMUS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 RAWIMUS]\n");
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
	out << mrpt::format(" Status: 0x%08lu\n", (long)fields.imu_status);
	out << mrpt::format(
		" Acel: X=%li Y=%li Z=%li\n", (long)fields.accel_x,
		-(long)fields.accel_y_neg, (long)fields.accel_z);
	out << mrpt::format(
		" Gyro: X=%li Y=%li Z=%li\n", (long)fields.gyro_x,
		-(long)fields.gyro_y_neg, (long)fields.gyro_z);
}

bool Message_NV_OEM6_RAWIMUS::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms imu_status acc_x acc_y acc_z gyro_x gyro_y gyro_z";
	return true;
}
bool Message_NV_OEM6_RAWIMUS::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%u.%08u %u %li %li %li %li %li %li", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week), (unsigned)fields.imu_status,
		(long)fields.accel_x, -(long)fields.accel_y_neg, (long)fields.accel_z,
		(long)fields.gyro_x, -(long)fields.gyro_y_neg, (long)fields.gyro_z);
	return true;
}

// ------------
void generic_dump_MARKTIME(
	const Message_NV_OEM6_MARKTIME::content_t& fields, std::ostream& out)
{
	out << mrpt::format(" Clock status: 0x%08lu\n", (long)fields.clock_status);
	out << mrpt::format(
		" GPS week: %lu Seconds: %f\n", (long)fields.week, fields.week_seconds);
	out << mrpt::format(
		" Clock offset: %f  (std dev = %e)\n", fields.clock_offset,
		fields.clock_offset_std);
	out << mrpt::format(" UTC offset: %f\n", fields.utc_offset);
}

void Message_NV_OEM6_MARKTIME::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 MARKTIME]\n");
	generic_dump_MARKTIME(fields, out);
}

bool Message_NV_OEM6_MARKTIME::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms clock_status week week_seconds utc_offset";
	return true;
}
void generic_getFieldValues_MARKTIME(
	const Message_NV_OEM6_MARKTIME::content_t& fields, std::ostream& o)
{
	o << mrpt::format(
		"%u.%08u %u %lu %f %f", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week), (unsigned)fields.clock_status,
		(long unsigned)fields.week, fields.week_seconds, fields.utc_offset);
}
bool Message_NV_OEM6_MARKTIME::getAllFieldValues(std::ostream& o) const
{
	generic_getFieldValues_MARKTIME(fields, o);
	return true;
}
// ------------
void Message_NV_OEM6_MARK2TIME::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 MARK2TIME]\n");
	generic_dump_MARKTIME(
		*reinterpret_cast<const Message_NV_OEM6_MARKTIME::content_t*>(&fields),
		out);
}
bool Message_NV_OEM6_MARK2TIME::getAllFieldDescriptions(std::ostream& o) const
{
	o << "gps_week.gps_ms clock_status week week_seconds utc_offset";
	return true;
}
bool Message_NV_OEM6_MARK2TIME::getAllFieldValues(std::ostream& o) const
{
	generic_getFieldValues_MARKTIME(
		*reinterpret_cast<const Message_NV_OEM6_MARKTIME::content_t*>(&fields),
		o);
	return true;
}

// ------------
void Message_NV_OEM6_MARKPOS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel OEM6 MARKPOSE]\n");
	generic_dump_BESTPOS(
		*reinterpret_cast<const Message_NV_OEM6_BESTPOS::content_t*>(&fields),
		out);
}

// ------------
void Message_NV_OEM6_IONUTC::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[Novatel NV_OEM6_IONUTC]\n");
	out << mrpt::format(
		" GPS week: %u  ms in week: %u\n", (unsigned)fields.header.week,
		(unsigned)(fields.header.ms_in_week));
	out << mrpt::format(
		" UTC ref week: %u  Tot: %u\n", (unsigned)fields.utc_wn,
		(unsigned)fields.tot);
	out << mrpt::format(
		" Leap seconds delta_t: %u  future: %u\n", (unsigned)fields.deltat_ls,
		(unsigned)fields.deltat_lsf);
}
