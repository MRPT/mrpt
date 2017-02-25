/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSRAWGPSSVINFO_H
#define XSRAWGPSSVINFO_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSSVINFO_INITIALIZER	{ 0, 0, 0, 0, 0, 0, 0, 0 }
#define XSRAWGPSSVINFO_INITIALIZER	{ 0, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER, XSSVINFO_INITIALIZER };
#endif

//! \brief A container for information of one satellite
struct XsSvInfo 
{
	uint8_t m_chn;		//!< channel number, range 0..NCH-1
	uint8_t m_svid;		//!< Satellite ID
	uint8_t m_flags;	/*!< Bitmask, made up of the following bit values
							0x01 = SV is used for navigation
							0x02 = Differential correction data is available for this SV
							0x04 = Orbit information is available for this SV (Ephemeris or Almanach)
							0x08 = Orbit information is Ephemeris
							0x10 = SV is unhealthy / shall not be used
							0x20 = reserved
							0x40 = reserved
							0x80 = reserved
						*/
	int8_t  m_qi;		/*!< Signal Quality indicator (range 0..7). The following list shows the meaning of the different QI values:
							0: This channel is idle
							1, 2: Channel is searching
							3: Signal detected but unusable
							4: Code Lock on Signal
							5, 6: Code and Carrier locked
							7: Code and Carrier locked, receiving 50bps data
						*/
	uint8_t	m_cno;		//!< Carrier to Noise Ratio (Signal Strength) (dbHz)
	int8_t	m_elev;		//!< Elevation in integer (deg)
	int16_t	m_azim;		//!< Azimuth in integer (deg)
	int32_t m_prres;	//!< Pseudo range residual (cm)
};
typedef struct XsSvInfo XsSvInfo;

/*! \brief A container for NAV-SVINFO data
*/
struct XsRawGpsSvInfo
{
	uint32_t m_itow;		//!< Gps time of week (ms)
	uint8_t  m_nch;			//!< Number of channels range 0..16
	uint8_t  m_res1;		//!< Reserved
	uint16_t m_res2;		//!< Reserved
	XsSvInfo m_svInfos[16];	//!< The information of all satellites, maxmimum 16
};
typedef struct XsRawGpsSvInfo XsRawGpsSvInfo;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif // file guard
