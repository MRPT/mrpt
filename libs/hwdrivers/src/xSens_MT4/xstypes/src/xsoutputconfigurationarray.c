/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsoutputconfigurationarray.h"
#include "xsoutputconfiguration.h"

/*! \struct XsOutputConfigurationArray
	\brief A list of XsOutputConfiguration values
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemSwap \note Specialization for XsOutputConfiguration
void swapXsOutputConfiguration(XsOutputConfiguration* a, XsOutputConfiguration* b)
{
	XsOutputConfiguration tmp = *a;
	*a = *b;
	*b = tmp;
}

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for XsOutputConfiguration
void copyXsOutputConfiguration(XsOutputConfiguration* to, XsOutputConfiguration const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for XsOutputConfiguration
int compareXsOutputConfiguration(XsOutputConfiguration const* a, XsOutputConfiguration const* b)
{
	if (a->m_dataIdentifier != b->m_dataIdentifier || a->m_frequency != b->m_frequency)
	{
		if (a->m_dataIdentifier < b->m_dataIdentifier || a->m_frequency < b->m_frequency)
			return -1;
		else
			return 1;
	}

	return 0;
}


//! \brief Descriptor for XsOutputConfigurationArray
XsArrayDescriptor const g_xsOutputConfigurationArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsOutputConfiguration),
	XSEXPCASTITEMSWAP swapXsOutputConfiguration,	// swap
	0,												// construct
	XSEXPCASTITEMCOPY copyXsOutputConfiguration,	// copy construct
	0,												// destruct
	XSEXPCASTITEMCOPY copyXsOutputConfiguration,	// copy
	XSEXPCASTITEMCOMP compareXsOutputConfiguration	// compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsOutputConfigurationArray_construct(XsOutputConfigurationArray* thisPtr, XsSize count, XsOutputConfiguration const* src)
{
	XsArray_construct(thisPtr, &g_xsOutputConfigurationArrayDescriptor, count, src);
}
