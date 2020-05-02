
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSRAWGNSSPVTDATA_H
#define XSRAWGNSSPVTDATA_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#elif defined (__ICCARM__)
_Pragma("pack(push, 1)")
#endif

#ifndef __cplusplus
#define XSRAWGNSSPVTDATA_INITIALIZER { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#else
#include <cstring>
#endif

// The valid flag from the PVT package has the following fields, x is the valid flag.
#define GNSS_PVT_VALID_DATE(x)		(0x01 & x)	// UTC date is valid
#define GNSS_PVT_VALID_TIME(x)		(0x02 & x)	// UTC time of day is valid
#define GNSS_PVT_VALID_RESOLVE(x)	(0x04 & x)	// UTC time of day has been fully resolved

/*! \brief A container for GNSS Position Velocity and Time data
*/
struct XsRawGnssPvtData
{
	uint32_t	m_itow;		//!< GNSS time of week (ms)
	uint16_t	m_year;		//!< Year (UTC)
	uint8_t		m_month;	//!< Month (UTC)
	uint8_t		m_day;		//!< Day of Month (UTC)
	uint8_t		m_hour;		//!< Hour of day 0..23 (UTC)
	uint8_t		m_min;		//!< Minute of hour 0..59 (UTC)
	uint8_t		m_sec;		//!< Seconds of minute 0..60 (UTC)
	uint8_t		m_valid;	/*!< Validity Flags
								bit(0) = Set if UTC Date is valid
								bit(1) = Set if UTC Time of Day if valid
								bit(2) = Set if UTC Time of Day has been fully resolved (no seconds uncertainty)
							*/
	uint32_t	m_tAcc;		//!< Time accuracy estimate (ns) (UTC)
	int32_t		m_nano;		//!< Fraction of second (ns) -1e9..1e9 (UTC)
	uint8_t		m_fixType;	/*!< GNSSfix Type, range 0..5
								0x00 = No Fix
								0x01 = Dead Reckoning only
								0x02 = 2D-Fix
								0x03 = 3D-Fix
								0x04 = GNSS + dead reckoning combined
								0x05 = Time only fix
								0x06..0xff: reserved
							*/
	uint8_t		m_flags;	/*!< Fix Status Flags
								bit(0) = Set if there is a valid fix (i.e. within DOP & accuracy masks)
								bit(1) = Set if differential corrections were applied
								bit(2..4) = Reserved (Ignore)
								bit(5) = Set if heading of vehicle is valid
								*/

	uint8_t		m_numSv;	//!< Number of satellites used in Nav Solution
	uint8_t		m_res1;		//!< Reserved for future use (1)
	int32_t		m_lon;		//!< Longitude (deg) (scaling 1e-7)
	int32_t		m_lat;		//!< Latitude (deg) (scaling 1e-7)
	int32_t		m_height;	//!< Height above ellipsoid (mm)
	int32_t		m_hMsl;		//!< Height above mean sea level (mm)

	uint32_t	m_hAcc;		//!< Horizontal accuracy estimate (mm)
	uint32_t	m_vAcc;		//!< Vertical accuracy estimate (mm)

	int32_t		m_velN;		//!< NED north velocity (mm/s)
	int32_t		m_velE;		//!< NED east velocity (mm/s)
	int32_t		m_velD;		//!< NED down velocity (mm/s)
	int32_t		m_gSpeed;	//!< 2D Ground Speed (mm/s)
	int32_t		m_headMot;	//!< 2D Heading of motion (deg) (scaling 1e-5)

	uint32_t	m_sAcc;		//!< Speed accuracy estimate (mm/s)
	uint32_t	m_headAcc;	//!< Heading accuracy estimate (both motion and vehicle) (deg) (scaling 1-e5)
	int32_t		m_headVeh;	//!< 2D Heading of vehicle (deg) (scaling 1e-5)

	uint16_t	m_gdop;		//!< Geometric DOP (scaling 0.01)
	uint16_t	m_pdop;		//!< Position DOP (scaling 0.01)
	uint16_t	m_tdop;		//!< Time DOP (scaling 0.01)
	uint16_t	m_vdop;		//!< Vertical DOP (scaling 0.01)
	uint16_t	m_hdop;		//!< Horizontal DOP (scaling 0.01)
	uint16_t	m_ndop;		//!< Northing DOP (scaling 0.01)
	uint16_t	m_edop;		//!< Easting DOP (scaling 0.01)

#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsRawGnssPvtData& other) const
	{
		// direct memory comparison is allowed because all fields line up properly
		return std::memcmp(this, &other, sizeof(XsRawGnssPvtData)) == 0;
	}
#endif

};
typedef struct XsRawGnssPvtData XsRawGnssPvtData;

#ifdef _MSC_VER
#pragma pack(pop)
#elif defined (__ICCARM__)
_Pragma("pack(pop)")
#endif

#endif
