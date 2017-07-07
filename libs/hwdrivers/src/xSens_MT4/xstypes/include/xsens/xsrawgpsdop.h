/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSRAWGPSDOP_H
#define XSRAWGPSDOP_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSRAWGPSDOP_INITIALIZER \
	{                           \
		0, 0, 0, 0, 0, 0, 0, 0  \
	}
#endif

/*! \brief A container for NAV-DOP data
	\details DOP values are dimensionless.
	All dop values are scaled by a factor of 100. that is, if the unit transmits
   a value of e.g. 156,
	it means that the DOP value is 1.56.
*/
struct XsRawGpsDop
{
	/** Gps time of week (ms) */
	uint32_t m_itow;
	/** Geometric DOP */
	uint16_t m_gdop;
	/** Position DOP */
	uint16_t m_pdop;
	/** Time DOP */
	uint16_t m_tdop;
	/** Vertical DOP */
	uint16_t m_vdop;
	/** Horizontal DOP */
	uint16_t m_hdop;
	/** Northing DOP */
	uint16_t m_ndop;
	/** Easting DOP */
	uint16_t m_edop;
};
typedef struct XsRawGpsDop XsRawGpsDop;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif  // file guard
