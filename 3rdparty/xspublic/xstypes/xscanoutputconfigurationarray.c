
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xscanoutputconfigurationarray.h"
#include "xscanoutputconfiguration.h"

/*! \struct XsCanOutputConfigurationArray
	\brief A list of XsCanOutputConfiguration values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for XsCanOutputConfiguration*/
void swapXsCanOutputConfiguration(XsCanOutputConfiguration* a, XsCanOutputConfiguration* b)
{
	XsCanOutputConfiguration tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for XsCanOutputConfiguration*/
void copyXsCanOutputConfiguration(XsCanOutputConfiguration* to, XsCanOutputConfiguration const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for XsCanOutputConfiguration*/
int compareXsCanOutputConfiguration(XsCanOutputConfiguration const* a, XsCanOutputConfiguration const* b)
{
	if (a->m_dataIdentifier != b->m_dataIdentifier || a->m_frequency != b->m_frequency || a->m_id != b->m_id || a->m_frameFormat != b->m_frameFormat)
	{
		if (a->m_dataIdentifier == b->m_dataIdentifier)
		{
			if (a->m_frequency == b->m_frequency)
			{
				if (a->m_id == b->m_id)
				{
					return (a->m_frameFormat < b->m_frameFormat) ? -1 : 1;
				}
				return (a->m_id < b->m_id) ? -1 : 1;
			}
			return (a->m_frequency < b->m_frequency) ? -1 : 1;
		}
		return (a->m_dataIdentifier < b->m_dataIdentifier) ? -1 : 1;
	}

	return 0;
}


//! \brief Descriptor for XsCanOutputConfigurationArray
XsArrayDescriptor const g_xsCanOutputConfigurationArrayDescriptor = {
	sizeof(XsCanOutputConfiguration),
	XSEXPCASTITEMSWAP swapXsCanOutputConfiguration,	// swap
	0,												// construct
	XSEXPCASTITEMCOPY copyXsCanOutputConfiguration,	// copy construct
	0,												// destruct
	XSEXPCASTITEMCOPY copyXsCanOutputConfiguration,	// copy
	XSEXPCASTITEMCOMP compareXsCanOutputConfiguration,	// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy	// raw copy
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsCanOutputConfigurationArray
*/
void XsCanOutputConfigurationArray_construct(XsCanOutputConfigurationArray* thisPtr, XsSize count, XsCanOutputConfiguration const* src)
{
	XsArray_construct(thisPtr, &g_xsCanOutputConfigurationArrayDescriptor, count, src);
}
