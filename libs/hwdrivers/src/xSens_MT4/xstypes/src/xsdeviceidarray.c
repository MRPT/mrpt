/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsdeviceidarray.h"
#include "xsdeviceid.h"

/*! \struct XsDeviceIdArray
	\brief A list of XsDeviceId values
	\sa XsArray
*/

void initDeviceId(XsDeviceId* did)
{
	did->m_deviceId = 0;
}

void initDeviceIdToValue(XsDeviceId* did, XsDeviceId const* src)
{
	did->m_deviceId = src->m_deviceId;
}

int compareDeviceIds(XsDeviceId const* a, XsDeviceId const* b)
{
	return ((int) a->m_deviceId) - ((int) b->m_deviceId);
}

//! \brief Descriptor for XsDeviceIdArray
XsArrayDescriptor const g_xsDeviceIdArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsDeviceId),
	XSEXPCASTITEMSWAP XsDeviceId_swap,		// swap
	XSEXPCASTITEMMAKE initDeviceId,			// construct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy construct
	0,										// destruct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy
	XSEXPCASTITEMCOMP compareDeviceIds		// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsDeviceIdArray_construct(XsDeviceIdArray* thisPtr, XsSize count, XsDeviceId const* src)
{
	XsArray_construct(thisPtr, &g_xsDeviceIdArrayDescriptor, count, src);
}
