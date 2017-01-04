/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsint64array.h"

/*! \struct XsInt64Array
	\brief A list of int64_t values
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemSwap \note Specialization for int64_t
void swapInt64(int64_t* a, int64_t* b)
{
	int64_t tmp = *a;
	*a = *b;
	*b = tmp;
}

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for int64_t
void copyInt64(int64_t* to, int64_t const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for int64_t
int compareInt64(int64_t const* a, int64_t const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}

//! \brief Descriptor for XsInt64Array
XsArrayDescriptor const g_xsInt64ArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(int64_t),
	XSEXPCASTITEMSWAP swapInt64,	// swap
	0,								// construct
	XSEXPCASTITEMCOPY copyInt64,	// copy construct
	0,								// destruct
	XSEXPCASTITEMCOPY copyInt64,	// copy
	XSEXPCASTITEMCOMP compareInt64	// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsInt64Array
*/
void XsInt64Array_construct(XsInt64Array* thisPtr, XsSize count, int64_t const* src)
{
	XsArray_construct(thisPtr, &g_xsInt64ArrayDescriptor, count, src);
}
