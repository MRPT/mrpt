/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSRAWGPSTIMEUTC_H
#define XSRAWGPSTIMEUTC_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSRAWGPSTIMEUTC_INITIALIZER  \
	{                                \
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
	}
#endif

/*! \brief A container for NAV-TIMEUTC data
 */
struct XsRawGpsTimeUtc
{
	/** Gps time of week (ms) */
	uint32_t m_itow;
	/** Time Accuracy Estimate (ns) */
	uint32_t m_tacc;
	/** Nanoseconds of second, range -500000000 .. 500000000 (UTC) */
	int32_t m_nano;
	/** Year, range 1999..2099 (UTC) */
	uint16_t m_year;
	/** Month, range 1..12 (UTC) */
	uint8_t m_month;
	/** Day of Month, range 1..31 (UTC) */
	uint8_t m_day;
	/** Hour of Day, range 0..23 (UTC) */
	uint8_t m_hour;
	/** Minute of Hour, range 0..59 (UTC) */
	uint8_t m_min;
	/** Minute of Hour, range 0..59 (UTC) */
	uint8_t m_sec;
	uint8_t m_valid; /*!< Valid	0x01 = Valid Time of Week
								 0x02 = Valid Week Number
								 0x04 = Valid UTC (Leap Seconds already known?)
					 */
};
typedef struct XsRawGpsTimeUtc XsRawGpsTimeUtc;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif  // file guard
