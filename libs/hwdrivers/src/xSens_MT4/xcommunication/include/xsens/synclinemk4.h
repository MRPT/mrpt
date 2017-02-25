/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSYNCLINEMK4_H
#define XSSYNCLINEMK4_H

#include <xsens/xssyncline.h>

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization line identifiers for the Mk4 devices, only to be used directly in Xbus messages */
enum SyncLineMk4
{
	XSL4_ClockIn = 0,		//!< External clock sync \ref XSL_ClockIn
	XSL4_GpsClockIn = 1,	//!< GPS clock sync \ref XSL_GpsClockIn
	XSL4_In = 2,			//!< Send data line \ref XSL_In1
	XSL4_BiIn = 3,			//!< Bidirectional sync line, configured as input \ref XSL_Bi1In
	XSL4_BiOut = 4,			//!< Bidirectional sync line, configured as output \ref XSL_Bi1Out
	XSL4_ExtTimepulseIn = 5,//!< External Timepulse input \ref XSL_ExtTimepulseIn
	XSL4_ReqData = 6,		//!< Serial data sync option, use XMID_ReqData message id for this \ref XSL_ReqData

	XSL4_Invalid
};
/*! @} */
typedef enum SyncLineMk4 SyncLineMk4;

#ifdef __cplusplus
extern "C" {
#endif

XsSyncLine xsl4ToXsl(SyncLineMk4 mk4Line);
SyncLineMk4 xslToXsl4(XsSyncLine line);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // file guard
