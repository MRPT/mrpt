
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

#include "xsstringoutputtypearray.h"
#include <memory.h>
#include <string.h>

/*! \struct XsStringOutputTypeArray
	\brief A list of XsStringOutputType values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for XsStringOutputType
*/
void swapStringOutputType(XsStringOutputType* a, XsStringOutputType* b)
{
	XsStringOutputType tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for XsStringOutputType
*/
void copyStringOutputType(XsStringOutputType* to, XsStringOutputType const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for XsStringOutputType
*/
int compareStringOutputType(XsStringOutputType const* a, XsStringOutputType const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}

//! \brief Descriptor for XsStringOutputTypeArray
XsArrayDescriptor const g_xsStringOutputTypeArrayDescriptor = {
	sizeof(XsStringOutputType),
	XSEXPCASTITEMSWAP swapStringOutputType,			// swap
	0,												// construct
	(XsArrayItemCopyFunc)copyStringOutputType,		// copy construct
	0,												// destruct
	(XsArrayItemCopyFunc)copyStringOutputType,		// copy
	(XsArrayItemCompareFunc)compareStringOutputType,// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsStringOutputTypeArray
*/
void XsStringOutputTypeArray_construct(XsStringOutputTypeArray* thisPtr, XsSize count, XsStringOutputType const* src)
{
	XsArray_construct(thisPtr, &g_xsStringOutputTypeArrayDescriptor, count, src);
}
