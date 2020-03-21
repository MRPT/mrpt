/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */
#include "xstriggerindicationdata.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Destroy the %XsTriggerIndicationData object */
void XsTriggerIndicationData_destruct(XsTriggerIndicationData* thisPtr)
{
	memset(thisPtr, 0, sizeof(XsTriggerIndicationData));
}

/*! \brief Returns true if the object is valid (line and polarity may not be 0) */
int XsTriggerIndicationData_valid(const XsTriggerIndicationData* thisPtr)
{
	return thisPtr->m_line != 0 && thisPtr->m_polarity != 0;
}

/*! @} */
