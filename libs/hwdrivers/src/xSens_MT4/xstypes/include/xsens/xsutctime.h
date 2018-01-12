/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSUTCTIME_H
#define XSUTCTIME_H

#include "pstdint.h"

/*! \brief A structure for storing UTC Time values. */
struct XsUtcTime
{
	/** \brief Nanosecond part of the time */
	uint32_t m_nano;
	/** \brief The year (if date is valid) */
	uint16_t m_year;
	/** \brief The month (if date is valid) */
	uint8_t m_month;
	/** \brief The day of the month (if date is valid) */
	uint8_t m_day;
	/** \brief The hour (if time is valid) */
	uint8_t m_hour;
	/** \brief The minute (if time is valid) */
	uint8_t m_minute;
	/** \brief The second (if time is valid) */
	uint8_t m_second;
	/** \brief Validity indicator \details When set to 1, the time is valid,
	 * when set to 2, the time part is valid, but the date may not be valid.
	 * when set to 0, the time is invalid and should be ignored. */
	uint8_t m_valid;
};
typedef struct XsUtcTime XsUtcTime;

#endif  // file guard
