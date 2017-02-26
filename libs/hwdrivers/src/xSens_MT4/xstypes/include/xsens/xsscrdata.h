/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSCRDATA_H
#define XSSCRDATA_H

#include "pstdint.h"
#include "xsushortvector.h"

#ifndef __cplusplus
#define XSSCRDATA_INITIALIZER {XSUSHORTVECTOR_INITIALIZER, XSUSHORTVECTOR_INITIALIZER, XSUSHORTVECTOR_INITIALIZER, {0, 0, 0, 0}}
#endif

#define XS_EXTRA_TEMPERATURE_CHANNELS 3
#define XS_MAX_TEMPERATURE_CHANNELS ((XS_EXTRA_TEMPERATURE_CHANNELS) + 1)

/*! \brief Container for raw sensor measurement data
	\details This structure contains raw measurement data from the sensors on the device.
	This data is unscaled, the bias has not been subtracted and no error correction has been applied.
*/
struct XsScrData {
	XsUShortVector	m_acc;	//!< The raw accelerometer data
	XsUShortVector	m_gyr;	//!< The raw gyroscope data
	XsUShortVector	m_mag;	//!< The raw magnetometer data
	uint16_t		m_temp[XS_MAX_TEMPERATURE_CHANNELS];	//!< The temperature data
};
typedef struct XsScrData XsScrData;

#endif // file guard
