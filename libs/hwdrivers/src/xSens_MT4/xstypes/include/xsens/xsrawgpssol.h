/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSRAWGPSSOL_H
#define XSRAWGPSSOL_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSRAWGPSSOL_INITIALIZER                           \
	{                                                     \
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
	}
#endif

/*! \brief A container for NAV-SOL data
 */
struct XsRawGpsSol
{
	/** Gps time of week (ms) */
	uint32_t m_itow;
	/** Nanoseconds remainder of rounded ms above, range -500000 .. 500000 */
	int32_t m_frac;
	/** GPS week (GPS time) */
	int16_t m_week;
	uint8_t m_gpsfix; /*!< GPSfix Type, range 0..4
						  0x00 = No Fix
						  0x01 = Dead Reckoning only
						  0x02 = 2D-Fix
						  0x03 = 3D-Fix
						  0x04 = GPS + dead reckoning combined
						  0x05..0xff: reserved
					  */
	uint8_t m_flags; /*!< 0x01=GPSfixOK (i.e. within DOP & ACC Masks)
						  0x02=DiffSoln (is DGPS used)
						  0x04=WKNSET (is Week Number valid)
						  0x08=TOWSET (is Time of Week valid)
						  0x?0=reserved
					 */
	/** ECEF X position (cm) */
	int32_t m_ecef_x;
	/** ECEF Y position (cm) */
	int32_t m_ecef_y;
	/** ECEF Z position (cm) */
	int32_t m_ecef_z;
	/** Position Accuracy Estimate (cm) */
	uint32_t m_pacc;
	/** ECEF X velocity (cm/s) */
	int32_t m_ecef_vx;
	/** ECEF Y velocity (cm/s) */
	int32_t m_ecef_vy;
	/** ECEF Z velocity (cm/s) */
	int32_t m_ecef_vz;
	/** Speed Accuracy Estimate (cm/s) */
	uint32_t m_sacc;
	/** Position DOP */
	uint16_t m_pdop;
	/** Reserved */
	uint8_t m_res1;
	/** Number of SVs used in Nav Solution */
	uint8_t m_numsv;
	/** Reserved */
	uint32_t m_res2;
};
typedef struct XsRawGpsSol XsRawGpsSol;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif  // file guard
