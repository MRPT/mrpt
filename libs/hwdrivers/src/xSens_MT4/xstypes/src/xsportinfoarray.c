/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsportinfoarray.h"

/*! \struct XsPortInfoArray
	\brief A list of XsPortInfo values
	\sa XsArray
*/

void copyPortInfo(XsPortInfo* dest, XsPortInfo const* src)
{
	memcpy(dest, src, sizeof(XsPortInfo));
}

int comparePortInfo(XsPortInfo const* a, XsPortInfo const* b)
{
	return memcmp(a, b, sizeof(XsPortInfo));
}

//! \brief Descriptor for XsPortInfoArray
XsArrayDescriptor const g_xsPortInfoArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsPortInfo),
	XSEXPCASTITEMSWAP XsPortInfo_swap,	// swap
	XSEXPCASTITEMMAKE XsPortInfo_clear,	// construct
	XSEXPCASTITEMCOPY copyPortInfo,		// copy construct
	0,									// destruct
	XSEXPCASTITEMCOPY copyPortInfo,		// copy
	XSEXPCASTITEMCOMP comparePortInfo	// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsPortInfoArray_construct(XsPortInfoArray* thisPtr, XsSize count, XsPortInfo const* src)
{
	XsArray_construct(thisPtr, &g_xsPortInfoArrayDescriptor, count, src);
}
