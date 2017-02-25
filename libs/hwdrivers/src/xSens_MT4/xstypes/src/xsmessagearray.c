/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsmessagearray.h"
#include "xsmessage.h"

/*! \struct XsMessageArray
	\brief A list of XsMessage values
	\sa XsArray
*/

//! \brief Descriptor for XsMessageArray
XsArrayDescriptor const g_xsMessageArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsMessage),
	XSEXPCASTITEMSWAP XsMessage_swap,
	XSEXPCASTITEMMAKE XsMessage_construct,
	XSEXPCASTITEMCOPY XsMessage_copyConstruct,
	XSEXPCASTITEMMAKE XsMessage_destruct,
	XSEXPCASTITEMCOPY XsMessage_copy,
	XSEXPCASTITEMCOMP XsMessage_compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsMessageArray_construct(XsMessageArray* thisPtr, XsSize count, XsMessage const* src)
{
	XsArray_construct(thisPtr, &g_xsMessageArrayDescriptor, count, src);
}
