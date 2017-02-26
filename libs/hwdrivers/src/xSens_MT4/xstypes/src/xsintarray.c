/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsintarray.h"

/*! \struct XsIntArray
	\brief A list of XsInt values
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemSwap \note Specialization for int
void swapInt(int* a, int* b)
{
	int tmp = *a;
	*a = *b;
	*b = tmp;
}

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for int
void copyInt(int* to, int const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for int
int compareInt(int const* a, int const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}


//! \brief Descriptor for XsIntArray
XsArrayDescriptor const g_xsIntArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(int),
	XSEXPCASTITEMSWAP swapInt,		// swap
	0,								// construct
	XSEXPCASTITEMCOPY copyInt,		// copy construct
	0,								// destruct
	XSEXPCASTITEMCOPY copyInt,		// copy
	XSEXPCASTITEMCOMP compareInt	// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsIntArray_construct(XsIntArray* thisPtr, XsSize count, int const* src)
{
	XsArray_construct(thisPtr, &g_xsIntArrayDescriptor, count, src);
}
