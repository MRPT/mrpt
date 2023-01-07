/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include "gnss_messages_common.h"

namespace mrpt::obs::gnss
{
// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push, 1)

/** Novatel OEM6 regular header structure \sa mrpt::obs::CObservationGPS  */
struct nv_oem6_header_t
{
	enum : uint8_t
	{
		SYNCH0 = 0xAA,
		SYNCH1 = 0X44,
		SYNCH2 = 0x12
	};

	uint8_t synch[3]{0, 0, 0};
	uint8_t hdr_len{0};
	uint16_t msg_id{0};
	uint8_t msg_type{0};
	uint8_t port_addr{0};
	uint16_t msg_len{0};
	uint16_t seq_number{0};
	uint8_t idle_percent{0};
	uint8_t time_status{0};
	uint16_t week{0};
	uint32_t ms_in_week{0};
	uint32_t receiver_status{0};
	uint16_t reserved{0};
	uint16_t receiver_sw_version{0};

	void fixEndianness()
	{
#if MRPT_IS_BIG_ENDIAN
		mrpt::reverseBytesInPlace(msg_id);
		mrpt::reverseBytesInPlace(msg_len);
		mrpt::reverseBytesInPlace(seq_number);
		mrpt::reverseBytesInPlace(week);
		mrpt::reverseBytesInPlace(ms_in_week);
		mrpt::reverseBytesInPlace(receiver_status);
		mrpt::reverseBytesInPlace(reserved);
		mrpt::reverseBytesInPlace(receiver_sw_version);
#endif
	}
};

/** Novatel OEM6 short header structure \sa mrpt::obs::CObservationGPS  */
struct nv_oem6_short_header_t
{
	enum : uint8_t
	{
		SYNCH0 = 0xAA,
		SYNCH1 = 0X44,
		SYNCH2 = 0x13
	};
	uint8_t synch[3]{0, 0, 0};
	uint8_t msg_len{0};
	uint16_t msg_id{0};
	uint16_t week{0};
	uint32_t ms_in_week{0};

	void fixEndianness()
	{
#if MRPT_IS_BIG_ENDIAN
		mrpt::reverseBytesInPlace(msg_id);
		mrpt::reverseBytesInPlace(msg_len);
		mrpt::reverseBytesInPlace(week);
		mrpt::reverseBytesInPlace(ms_in_week);
#endif
	}
};

namespace nv_oem6_position_type
{
/** Novatel OEM6 firmware reference, table 84; Novatel SPAN on OEM6 firmware
 * manual, table 26. */
enum nv_position_type_t : uint32_t
{
	NONE = 0,
	FIXEDPOS = 1,
	FIXEDHEIGHT = 2,
	Reserved = 3,
	FLOATCONV = 4,
	WIDELANE = 5,
	NARROWLANE = 6,
	DOPPLER_VELOCITY = 8,
	SINGLE = 16,
	PSRDIFF = 17,
	WAAS = 18,
	PROPAGATED = 19,
	OMNISTAR = 20,
	L1_FLOAT = 32,
	IONOFREE_FLOAT = 33,
	NARROW_FLOAT = 34,
	L1_INT = 48,
	WIDE_INT = 49,
	NARROW_INT = 50,
	RTK_DIRECT_INS = 51,
	INS = 52,
	INS_PSRSP = 53,
	INS_PSRDIFF = 54,
	INS_RTKFLOAT = 55,
	INS_RTKFIXED = 56,
	OMNISTAR_HP = 64,
	OMNISTAR_XP = 65,
	CDGPS = 66
};
/** for nv_position_type_t */
const std::string& enum2str(int val);
}  // namespace nv_oem6_position_type

namespace nv_oem6_solution_status
{
/** Novatel OEM6 firmware reference, table 85 */
enum nv_solution_status_t : uint32_t
{
	/** solution computed */
	SOL_COMPUTED = 0,
	/** insufficient observations */
	INSUFFICIENT_OBS,
	/** noconvergence */
	NO_CONVERGENCE,
	/** singularity at parameters matrix */
	SINGULARITY,
	/** covariance trace exceeds maximum (trace>1000m) */
	COV_TRACE,
	/** test distance exceeded (max of 3 rejections if distance > 10km) */
	TEST_DIST,
	/** not yet converged from cold start */
	COLD_START,
	/** height or velocity limits exceeded */
	V_H_LIMIT,
	/** variance exceeds limits */
	VARIANCE,
	/** residuals are too large */
	RESIDUALS,
	/** delta position is too large */
	DELTA_POS,
	/** negative variance */
	NEGATIVE_VAR,
	/** large residuals make position unreliable */
	INTEGRITY_WARNING = 13,
	/** ins has not started yet */
	INS_INACTIVE,
	/** ins doing its coarse alignment */
	INS_ALIGNING,
	/** ins position is bad */
	INS_BAD,
	/** no imu detected */
	IMU_UNPLUGGED,
	/** when a fix position command is entered, the receiver computes its own
	   position and determines if the fixed position is valid */
	PENDING = 18,
	/** the fixed position entered using the fix position command is not valid
	 */
	INVALID_FIX
};
/** for nv_solution_status_t */
const std::string& enum2str(int val);
}  // namespace nv_oem6_solution_status
namespace nv_oem6_ins_status_type
{
/** Novatel SPAN on OEM6 firmware reference, table 33 */
enum nv_ins_status_type_t : uint32_t
{
	INS_INACTIVE = 0,  // IMU logs are present, but the alignment routine has
	// not started; INS is inactive.
	INS_ALIGNING = 1,  // INS is in alignment mode.
	INS_HIGH_VARIANCE = 2,	// The INS solution is in navigation mode but the
	// azimuth solution uncertainty has exceeded the
	// threshold.
	INS_SOLUTION_GOOD = 3,	// The INS filter is in navigation mode and the INS
	// solution is good.
	INS_SOLUTION_FREE = 6,	// The INS filter is in navigation mode and the GNSS
	// solution is suspected to be in error.
	INS_ALIGNMENT_COMPLETE = 7,	 // The INS filter is in navigation mode, but
	// not enough vehicle dynamics have been
	// experienced for the system to be within
	// specifications.
	DETERMINING_ORIENTATION =
		8,	// INS is determining the IMU axis aligned with gravity.
	WAITING_INITIALPOS = 9	// The INS filter has determined the IMU orientation
	// and is awaiting an initial position estimate to
	// begin the alignment process.
};
/** for nv_ins_status_type_t */
const std::string& enum2str(int val);
}  // namespace nv_oem6_ins_status_type

/** Novatel generic frame (to store frames without a parser at the present
 * time). \sa mrpt::obs::CObservationGPS  */
struct Message_NV_OEM6_GENERIC_FRAME : public gnss_message
{
	Message_NV_OEM6_GENERIC_FRAME()
		: gnss_message((gnss_message_type_t)NV_OEM6_GENERIC_FRAME)
	{
	}
	/** Frame header */
	nv_oem6_header_t header;
	std::vector<uint8_t> msg_body;

	void fixEndianness() override { header.fixEndianness(); }

	void dumpToStream(std::ostream& out) const override;  // See docs in base
   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
};

/** Novatel generic short-header frame (to store frames without a parser at the
 * present time). \sa mrpt::obs::CObservationGPS  */
struct Message_NV_OEM6_GENERIC_SHORT_FRAME : public gnss_message
{
	Message_NV_OEM6_GENERIC_SHORT_FRAME()
		: gnss_message((gnss_message_type_t)NV_OEM6_GENERIC_SHORT_FRAME)
	{
	}
	/** Frame header */
	nv_oem6_short_header_t header;
	std::vector<uint8_t> msg_body;

	void fixEndianness() override { header.fixEndianness(); }

	void dumpToStream(std::ostream& out) const override;  // See docs in base
   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
};

/** Novatel frame: NV_OEM6_BESTPOS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_BESTPOS)
/** Frame header */
nv_oem6_header_t header;
nv_oem6_solution_status::nv_solution_status_t solution_stat =
	nv_oem6_solution_status::INVALID_FIX;
nv_oem6_position_type::nv_position_type_t position_type =
	nv_oem6_position_type::NONE;
/** [deg], [deg], hgt over sea level[m] */
double lat = 0, lon = 0, hgt = 0;
float undulation = 0;
uint32_t datum_id = 0;
/** Uncertainties (all in [m]) */
float lat_sigma = 0, lon_sigma = 0, hgt_sigma = 0;
char base_station_id[4]{0, 0, 0, 0};
float diff_age = 0, sol_age = 0;
uint8_t num_sats_tracked = 0, num_sats_sol = 0, num_sats_sol_L1 = 0,
		num_sats_sol_multi = 0;
uint8_t reserved = 0;
uint8_t ext_sol_stat = 0;
uint8_t galileo_beidou_mask = 0;
uint8_t gps_glonass_mask = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure
 * (requires linking against mrpt-topography)
 *   Call as: getAsStruct<TGeodeticCoords>(); */
template <class TGEODETICCOORDS>
inline TGEODETICCOORDS getAsStruct() const
{
	return TGEODETICCOORDS(fields.lat, fields.lon, fields.hgt);
}
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace_enum(fields.solution_stat);
	mrpt::reverseBytesInPlace_enum(fields.position_type);
	mrpt::reverseBytesInPlace(fields.lat);
	mrpt::reverseBytesInPlace(fields.lon);
	mrpt::reverseBytesInPlace(fields.hgt);
	mrpt::reverseBytesInPlace(fields.undulation);
	mrpt::reverseBytesInPlace(fields.datum_id);
	mrpt::reverseBytesInPlace(fields.lat_sigma);
	mrpt::reverseBytesInPlace(fields.lon_sigma);
	mrpt::reverseBytesInPlace(fields.hgt_sigma);
	mrpt::reverseBytesInPlace(fields.diff_age);
	mrpt::reverseBytesInPlace(fields.sol_age);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_INSPVAS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_INSPVAS)
/** Frame header */
nv_oem6_short_header_t header;
uint32_t week = 0;
double seconds_in_week = 0;
double lat = 0, lon = 0, hgt = 0;
double vel_north = 0, vel_east = 0, vel_up = 0;
double roll = 0, pitch = 0, azimuth = 0;
nv_oem6_ins_status_type::nv_ins_status_type_t ins_status =
	nv_oem6_ins_status_type::INS_INACTIVE;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure
 * (requires linking against mrpt-topography)
 *   Call as: getAsStruct<TGeodeticCoords>(); */
template <class TGEODETICCOORDS>
inline TGEODETICCOORDS getAsStruct() const
{
	return TGEODETICCOORDS(fields.lat, fields.lon, fields.hgt);
}
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.week);
	mrpt::reverseBytesInPlace(fields.seconds_in_week);
	mrpt::reverseBytesInPlace(fields.lat);
	mrpt::reverseBytesInPlace(fields.lon);
	mrpt::reverseBytesInPlace(fields.hgt);
	mrpt::reverseBytesInPlace(fields.vel_north);
	mrpt::reverseBytesInPlace(fields.vel_east);
	mrpt::reverseBytesInPlace(fields.vel_up);
	mrpt::reverseBytesInPlace(fields.roll);
	mrpt::reverseBytesInPlace(fields.pitch);
	mrpt::reverseBytesInPlace(fields.azimuth);
	mrpt::reverseBytesInPlace_enum(fields.ins_status);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_INSCOVS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_INSCOVS)
/** Frame header */
nv_oem6_short_header_t header;
uint32_t week = 0;
double seconds_in_week = 0;
/** Position covariance matrix in local level frame (metres squared)
 * xx,xy,xz,yx,yy,yz,zx,zy,zz */
double pos_cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
/** Attitude covariance matrix of the SPAN frame to the local level frame.  (deg
 * sq) xx,xy,xz,yx,yy,yz,zx,zy,zz */
double att_cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
/** Velocity covariance matrix in local level frame. (metres/second squared)
 * xx,xy,xz,yx,yy,yz,zx,zy,zz */
double vel_cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.week);
	mrpt::reverseBytesInPlace(fields.seconds_in_week);
	for (int i = 0; i < 9; i++)
	{
		mrpt::reverseBytesInPlace(fields.pos_cov[i]);
		mrpt::reverseBytesInPlace(fields.att_cov[i]);
		mrpt::reverseBytesInPlace(fields.vel_cov[i]);
	}
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_RANGECMP. \sa mrpt::obs::CObservationGPS  */
struct Message_NV_OEM6_RANGECMP : public gnss_message
{
	Message_NV_OEM6_RANGECMP()
		: gnss_message((gnss_message_type_t)NV_OEM6_RANGECMP)
	{
	}
	struct TCompressedRangeLog
	{
		uint8_t data[24];
	};

	/** Frame header */
	nv_oem6_header_t header;
	uint32_t num_obs{0};
	std::vector<TCompressedRangeLog> obs_data;
	uint32_t crc;

	void dumpToStream(std::ostream& out) const override;  // See docs in base
   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
};

/** Novatel frame: NV_OEM6_RXSTATUS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_RXSTATUS)
/** Frame header */
nv_oem6_header_t header;
uint32_t error = 0, num_stats = 0;
uint32_t rxstat = 0, rxstat_pri = 0, rxstat_set = 0, rxstat_clear = 0;
uint32_t aux1stat = 0, aux1stat_pri = 0, aux1stat_set = 0, aux1stat_clear = 0;
uint32_t aux2stat = 0, aux2stat_pri = 0, aux2stat_set = 0, aux2stat_clear = 0;
uint32_t aux3stat = 0, aux3stat_pri = 0, aux3stat_set = 0, aux3stat_clear = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.error);
	mrpt::reverseBytesInPlace(fields.num_stats);
	mrpt::reverseBytesInPlace(fields.rxstat);
	mrpt::reverseBytesInPlace(fields.rxstat_pri);
	mrpt::reverseBytesInPlace(fields.rxstat_set);
	mrpt::reverseBytesInPlace(fields.rxstat_clear);
	mrpt::reverseBytesInPlace(fields.aux1stat);
	mrpt::reverseBytesInPlace(fields.aux1stat_pri);
	mrpt::reverseBytesInPlace(fields.aux1stat_set);
	mrpt::reverseBytesInPlace(fields.aux1stat_clear);
	mrpt::reverseBytesInPlace(fields.aux2stat);
	mrpt::reverseBytesInPlace(fields.aux2stat_pri);
	mrpt::reverseBytesInPlace(fields.aux2stat_set);
	mrpt::reverseBytesInPlace(fields.aux2stat_clear);
	mrpt::reverseBytesInPlace(fields.aux3stat);
	mrpt::reverseBytesInPlace(fields.aux3stat_pri);
	mrpt::reverseBytesInPlace(fields.aux3stat_set);
	mrpt::reverseBytesInPlace(fields.aux3stat_clear);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_RAWEPHEM. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_RAWEPHEM)
/** Frame header */
nv_oem6_header_t header;
uint32_t sat_prn = 0, ref_week = 0, ref_secs = 0;
uint8_t subframe1[30], subframe2[30], subframe3[30];
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.sat_prn);
	mrpt::reverseBytesInPlace(fields.ref_week);
	mrpt::reverseBytesInPlace(fields.ref_secs);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_VERSION. \sa mrpt::obs::CObservationGPS  */
struct Message_NV_OEM6_VERSION : public gnss_message
{
	Message_NV_OEM6_VERSION()
		: gnss_message(static_cast<gnss_message_type_t>(NV_OEM6_VERSION))
	{
	}
	struct TComponentVersion
	{
		uint32_t type;
		char model[16], serial[16];
		char hwversion[16], swversion[16], bootversion[16];
		char compdate[12], comptime[12];
	};

	/** Frame header */
	nv_oem6_header_t header;
	uint32_t num_comps{0};
	std::vector<TComponentVersion> components;
	uint32_t crc;

	void dumpToStream(std::ostream& out) const override;  // See docs in base
	void fixEndianness() override
	{
#if MRPT_IS_BIG_ENDIAN
		header.fixEndianness();
		for (auto& c : components)
			mrpt::reverseBytesInPlace(c.type);
		mrpt::reverseBytesInPlace(crc);
#endif
	}

   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
};

/** Novatel frame: NV_OEM6_RAWIMUS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_RAWIMUS)
/** Frame header */
nv_oem6_short_header_t header;
uint32_t week = 0;
double week_seconds = 0;
uint32_t imu_status = 0;
int32_t accel_z = 0, accel_y_neg = 0, accel_x = 0;
int32_t gyro_z = 0, gyro_y_neg = 0, gyro_x = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.week);
	mrpt::reverseBytesInPlace(fields.week_seconds);
	mrpt::reverseBytesInPlace(fields.imu_status);
	mrpt::reverseBytesInPlace(fields.accel_z);
	mrpt::reverseBytesInPlace(fields.accel_y_neg);
	mrpt::reverseBytesInPlace(fields.accel_x);
	mrpt::reverseBytesInPlace(fields.gyro_z);
	mrpt::reverseBytesInPlace(fields.gyro_y_neg);
	mrpt::reverseBytesInPlace(fields.gyro_x);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_MARKPOS. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_MARKPOS)
/** Frame header */
nv_oem6_header_t header;
nv_oem6_solution_status::nv_solution_status_t solution_stat =
	nv_oem6_solution_status::INVALID_FIX;
nv_oem6_position_type::nv_position_type_t position_type =
	nv_oem6_position_type::NONE;
/** [deg], [deg], hgt over sea level[m] */
double lat = 0, lon = 0, hgt = 0;
float undulation = 0;
uint32_t datum_id = 0;
float lat_sigma = 0, lon_sigma = 0, hgt_sigma = 0;
char base_station_id[4] = {0, 0, 0, 0};
float diff_age = 0, sol_age = 0;
uint8_t num_sats_tracked = 0, num_sats_sol = 0, num_sats_sol_L1 = 0,
		num_sats_sol_multi = 0;
uint8_t reserved = 0;
uint8_t ext_sol_stat = 0;
uint8_t galileo_beidou_mask = 0;
uint8_t gps_glonass_mask = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure
 * (requires linking against mrpt-topography)
 *   Call as: getAsStruct<TGeodeticCoords>(); */
template <class TGEODETICCOORDS>
inline TGEODETICCOORDS getAsStruct() const
{
	return TGEODETICCOORDS(fields.lat, fields.lon, fields.hgt);
}
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace_enum(fields.solution_stat);
	mrpt::reverseBytesInPlace_enum(fields.position_type);
	mrpt::reverseBytesInPlace(fields.lat);
	mrpt::reverseBytesInPlace(fields.lon);
	mrpt::reverseBytesInPlace(fields.hgt);
	mrpt::reverseBytesInPlace(fields.undulation);
	mrpt::reverseBytesInPlace(fields.datum_id);
	mrpt::reverseBytesInPlace(fields.lat_sigma);
	mrpt::reverseBytesInPlace(fields.lon_sigma);
	mrpt::reverseBytesInPlace(fields.hgt_sigma);
	mrpt::reverseBytesInPlace(fields.diff_age);
	mrpt::reverseBytesInPlace(fields.sol_age);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_MARKTIME. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_MARKTIME)
/** Frame header */
nv_oem6_header_t header;
uint32_t week = 0;
double week_seconds = 0;
double clock_offset = 0, clock_offset_std = 0;
double utc_offset = 0;
uint32_t clock_status = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.week);
	mrpt::reverseBytesInPlace(fields.week_seconds);
	mrpt::reverseBytesInPlace(fields.clock_offset);
	mrpt::reverseBytesInPlace(fields.clock_offset_std);
	mrpt::reverseBytesInPlace(fields.utc_offset);
	mrpt::reverseBytesInPlace(fields.clock_status);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_MARK2TIME. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_MARK2TIME)
/** Frame header */
nv_oem6_header_t header;
uint32_t week = 0;
double week_seconds = 0;
double clock_offset = 0, clock_offset_std = 0;
double utc_offset = 0;
uint32_t clock_status = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
bool getAllFieldDescriptions(std::ostream& o) const override;
bool getAllFieldValues(std::ostream& o) const override;
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.week);
	mrpt::reverseBytesInPlace(fields.week_seconds);
	mrpt::reverseBytesInPlace(fields.clock_offset);
	mrpt::reverseBytesInPlace(fields.clock_offset_std);
	mrpt::reverseBytesInPlace(fields.utc_offset);
	mrpt::reverseBytesInPlace(fields.clock_status);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

/** Novatel frame: NV_OEM6_IONUTC. \sa mrpt::obs::CObservationGPS  */
GNSS_BINARY_MSG_DEFINITION_START(NV_OEM6_IONUTC)
/** Frame header */
nv_oem6_header_t header;
/** Ionospheric alpha and beta constant terms parameters */
double a0 = 0, a1 = 0, a2 = 0, a3 = 0, b0 = 0, b1 = 0, b2 = 0, b3 = 0;
/** UTC reference week number */
uint32_t utc_wn = 0;
/** Reference time of UTC params */
uint32_t tot = 0;
/** UTC constant and 1st order terms */
double A0 = 0, A1 = 0;
/** Future week number */
uint32_t wn_lsf = 0;
/** Day number (1=sunday, 7=saturday) */
uint32_t dn = 0;
/** Delta time due to leap seconds */
uint32_t deltat_ls = 0;
/** Delta time due to leap seconds (future) */
uint32_t deltat_lsf = 0;
uint32_t reserved = 0;
uint32_t crc = 0;
GNSS_BINARY_MSG_DEFINITION_MID
void fixEndianness() override
{
#if MRPT_IS_BIG_ENDIAN
	fields.header.fixEndianness();
	mrpt::reverseBytesInPlace(fields.a0);
	mrpt::reverseBytesInPlace(fields.a1);
	mrpt::reverseBytesInPlace(fields.a2);
	mrpt::reverseBytesInPlace(fields.a3);
	mrpt::reverseBytesInPlace(fields.b0);
	mrpt::reverseBytesInPlace(fields.b1);
	mrpt::reverseBytesInPlace(fields.b2);
	mrpt::reverseBytesInPlace(fields.b3);
	mrpt::reverseBytesInPlace(fields.utc_wn);
	mrpt::reverseBytesInPlace(fields.tot);
	mrpt::reverseBytesInPlace(fields.A0);
	mrpt::reverseBytesInPlace(fields.A1);
	mrpt::reverseBytesInPlace(fields.wn_lsf);
	mrpt::reverseBytesInPlace(fields.dn);
	mrpt::reverseBytesInPlace(fields.deltat_ls);
	mrpt::reverseBytesInPlace(fields.deltat_lsf);
	mrpt::reverseBytesInPlace(fields.reserved);
	mrpt::reverseBytesInPlace(fields.crc);
#endif
}
GNSS_BINARY_MSG_DEFINITION_MID_END

#pragma pack(pop)  // End of pack = 1
}  // namespace mrpt::obs::gnss
