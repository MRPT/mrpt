/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */
#include "xsoutputconfiguration.h"

void XsOutputConfiguration_swap(struct XsOutputConfiguration* a, struct XsOutputConfiguration* b)
{
	{
		XsDataIdentifier t = a->m_dataIdentifier;
		a->m_dataIdentifier = b->m_dataIdentifier;
		b->m_dataIdentifier = t;
	}
	
	{
		uint16_t t = a->m_frequency;
		a->m_frequency = b->m_frequency;
		b->m_frequency = t;
	}
}
