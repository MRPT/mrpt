
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

#include "xsfilterprofilearray.h"
#include "xsfilterprofile.h"
#include <memory.h>

/*! \struct XsFilterProfileArray
	\brief A list of XsFilterProfile values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for XsFilterProfile
*/
void swapFilterProfile(XsFilterProfile* a, XsFilterProfile* b)
{
	XsFilterProfile tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for XsFilterProfile
*/
void copyFilterProfile(XsFilterProfile* to, XsFilterProfile const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for XsFilterProfile
*/
int compareFilterProfile(XsFilterProfile const* a, XsFilterProfile const* b)
{
	if (a->m_filterType != b->m_filterType)
		return (a->m_filterType < b->m_filterType)?-1:1;
	if (a->m_type != b->m_type)
		return (a->m_type < b->m_type)?-1:1;
	if (a->m_filterMajor != b->m_filterMajor)
		return (a->m_filterMajor < b->m_filterMajor)?-1:1;
	if (a->m_filterMinor != b->m_filterMinor)
		return (a->m_filterMinor < b->m_filterMinor)?-1:1;
	if (a->m_version != b->m_version)
		return (a->m_version < b->m_version)?-1:1;
	return 0;
}

//! \brief zero the pointer value
void zeroFilterProfile(XsFilterProfile* a)
{
	memset(a, 0, sizeof(XsFilterProfile));
}

//! \brief Descriptor for XsFilterProfileArray
XsArrayDescriptor const g_xsFilterProfileArrayDescriptor = {
	sizeof(XsFilterProfile),
	XSEXPCASTITEMSWAP swapFilterProfile,	// swap
	XSEXPCASTITEMMAKE zeroFilterProfile,	// construct
	XSEXPCASTITEMCOPY copyFilterProfile,	// copy construct
	XSEXPCASTITEMMAKE zeroFilterProfile,	// destruct
	XSEXPCASTITEMCOPY copyFilterProfile,	// copy
	XSEXPCASTITEMCOMP compareFilterProfile,	// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsFilterProfileArray
*/
void XsFilterProfileArray_construct(XsFilterProfileArray* thisPtr, XsSize count, XsFilterProfile const* src)
{
	XsArray_construct(thisPtr, &g_xsFilterProfileArrayDescriptor, count, src);
}
