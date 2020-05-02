
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

#include "xsdeviceidarray.h"
#include "xsdeviceid.h"

/*! \struct XsDeviceIdArray
	\brief A list of XsDeviceId values
	\sa XsArray
*/

void initDeviceId(XsDeviceId* did)
{
	static const XsDeviceId cleared = XSDEVICEID_INITIALIZER;
	*did = cleared;
}

void initDeviceIdToValue(XsDeviceId* did, XsDeviceId const* src)
{
	memcpy(did, src, sizeof(XsDeviceId));
}

int compareDeviceIds(XsDeviceId const* a, XsDeviceId const* b)
{
	int64_t diff = (int64_t)a->m_deviceId - (int64_t)b->m_deviceId;
	if (diff < 0)
		return -1;
	else if (diff > 0)
		return 1;

	return 0;
}

//! \brief Descriptor for XsDeviceIdArray
XsArrayDescriptor const g_xsDeviceIdArrayDescriptor = {
	sizeof(XsDeviceId),
	XSEXPCASTITEMSWAP XsDeviceId_swap,		// swap
	XSEXPCASTITEMMAKE initDeviceId,			// construct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy construct
	0,										// destruct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy
	XSEXPCASTITEMCOMP compareDeviceIds,		// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy		// raw copy
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsStringArray
*/
void XsDeviceIdArray_construct(XsDeviceIdArray* thisPtr, XsSize count, XsDeviceId const* src)
{
	XsArray_construct(thisPtr, &g_xsDeviceIdArrayDescriptor, count, src);
}
