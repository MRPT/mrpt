/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsrssi.h"

/*! \namespace XsRssi
	\brief Contains Received Signal Strength Indication (RSSI) constants
*/
/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief The maximum RSSI value. */
XSTYPES_DLL_API int XsRssi_max()
{
	return XS_RSSI_MAX;
}

/*! \brief The RSSI value that was reserved for when the RSSI is unknown. */
XSTYPES_DLL_API int XsRssi_unknown()
{
	return XS_RSSI_UNKNOWN;
}

/*! \brief Returns the raw RSSI value transformed into a usable (unbiased) number.
	\details Actual rssi is calculated by \a raw + XS_RSSI_MAX, where \a raw is a negative number.
	\param raw The RSSI value as reported by the device.
	\returns The unbiased RSSI value.
*/
XSTYPES_DLL_API int XsRssi_unbiased(int raw)
{
	return raw + XS_RSSI_MAX;
}

/*! @} */
