/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSRSSI_H
#define XSRSSI_H

#include "xstypesconfig.h"

#define XS_RSSI_MAX		(128)
#define XS_RSSI_UNKNOWN	(-XS_RSSI_MAX)

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API int XsRssi_max();
XSTYPES_DLL_API int XsRssi_unknown();
XSTYPES_DLL_API int XsRssi_unbiased(int raw);

#ifdef __cplusplus
} // extern "C"
namespace XsRssi {
	/*! \brief The maximum RSSI value. */
	static const int max = XS_RSSI_MAX;			
	/*! \brief The RSSI value that was reserved for when the RSSI is unknown. */
	static const int unknown = XS_RSSI_UNKNOWN;
	/*! \copydoc XsRssi_unbiased */
	inline int unbiased(int raw)
	{
		return XsRssi_unbiased(raw);
	}
}
#endif

#endif // file guard
