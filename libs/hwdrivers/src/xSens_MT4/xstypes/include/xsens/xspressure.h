/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSPRESSURE_H
#define XSPRESSURE_H

#include "pstdint.h"

/*! \brief Pressure data.
	\details Contains the pressure data and the pressure age
*/
struct XsPressure {
	double		m_pressure;		//!< Pressure in Pascal
	uint8_t		m_pressureAge;	//!< Age of pressure data in samples
};
typedef struct XsPressure XsPressure;

#endif // file guard
