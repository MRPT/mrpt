/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xstypedefs.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Convert the %XsDataFlags to a human readable string
	\param f The flags to translate
	\returns A pointer to a statically allocated memory buffer. Do not free this buffer.
	\note This function is NOT reentrant, multiple simultaneous calls may cause crashes.
	Also, later calls will invalidate the results of earlier calls.
*/
const char *XsDataFlags_toString(XsDataFlags f)
{
	static char rv[4*20];
	if (f == XSDF_None)
		return "XSDF_None";

	rv[0] = 0;
	if (f & XSDF_Managed)
		strcpy(rv, "XSDF_Managed");
	if (f & XSDF_FixedSize)
	{
		if (rv[0])
			strcat(rv, " | ");
		strcat(rv, "XSDF_FixedSize");
	}
	if (f & XSDF_Empty)
	{
		if (rv[0])
			strcat(rv, " | ");
		strcat(rv, "XSDF_Empty");
	}
	return rv;
}

/*! @} */ 
