/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xssyncsetting.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Returns whether the selected line is configured as an input line 
*/
int XsSyncSetting_isInput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_In1:
	case XSL_In2:
	case XSL_Bi1In:
	case XSL_ClockIn:
	case XSL_CtsIn:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Returns whether the selected line is configured as an output line 
*/
int XsSyncSetting_isOutput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_Out1:
	case XSL_Out2:
	case XSL_Bi1Out:
	case XSL_RtsOut:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Swap the contents of \a a with \a b
*/
void XsSyncSetting_swap(XsSyncSetting* a, XsSyncSetting* b)
{
	XsSyncSetting tmp;
	memcpy(&tmp, a, sizeof(XsSyncSetting));
	memcpy(a, b, sizeof(XsSyncSetting));
	memcpy(b, &tmp, sizeof(XsSyncSetting));
}

/*! @} */ 
