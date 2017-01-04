/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsbytearray.h"
//#include "xsstring.h"

/*! \struct XsByteArray
	\brief A list of uint8_t values
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemSwap \note Specialization for uint8_t
void swapUint8(uint8_t* a, uint8_t* b)
{
	uint8_t tmp = *a;
	*a = *b;
	*b = tmp;
}

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for uint8_t
void copyUint8(uint8_t* to, uint8_t const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for uint8_t
int compareUint8(uint8_t const* a, uint8_t const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}

//! \brief Descriptor for XsByteArray
XsArrayDescriptor const g_xsByteArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(uint8_t),
	XSEXPCASTITEMSWAP swapUint8,	// swap
	0,								// construct
	XSEXPCASTITEMCOPY copyUint8,	// copy construct
	0,								// destruct
	XSEXPCASTITEMCOPY copyUint8,	// copy
	XSEXPCASTITEMCOMP compareUint8	// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsByteArray
*/
void XsByteArray_construct(XsByteArray* thisPtr, XsSize count, uint8_t const* src)
{
	XsArray_construct(thisPtr, &g_xsByteArrayDescriptor, count, src);
}
