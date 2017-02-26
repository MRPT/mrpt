/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "synclinemk4.h"

/*! \addtogroup cinterface C Interface
	@{
*/

//! \brief Translate an SyncLineMk4 into a generic XsSyncLine
XsSyncLine xsl4ToXsl(SyncLineMk4 mk4Line)
{
	switch (mk4Line)
	{
	case XSL4_ClockIn:			return XSL_ClockIn;
	case XSL4_GpsClockIn:		return XSL_GpsClockIn;
	case XSL4_ReqData:			return XSL_ReqData;
	case XSL4_In:				return XSL_In1;
	case XSL4_BiIn:				return XSL_Bi1In;
	case XSL4_BiOut:			return XSL_Bi1Out;
	case XSL4_ExtTimepulseIn:	return XSL_ExtTimepulseIn;
	default:					return XSL_Invalid;
	}
}

//! \brief Translate an XsSyncLine into a Mk4-specififc SyncLineMk4
SyncLineMk4 xslToXsl4(XsSyncLine line)
{
	switch (line)
	{
	case XSL_ClockIn:			return XSL4_ClockIn;
	case XSL_GpsClockIn:		return XSL4_GpsClockIn;
	case XSL_In1:				return XSL4_In;
	case XSL_ReqData:			return XSL4_ReqData;
	case XSL_Bi1In:				return XSL4_BiIn;
	case XSL_Bi1Out:			return XSL4_BiOut;
	case XSL_ExtTimepulseIn:	return XSL4_ExtTimepulseIn;
	default:					return XSL4_Invalid;
	}
}

/*! @} */ 
