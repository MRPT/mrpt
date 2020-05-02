
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

#include "xsintarray.h"

/*! \struct XsIntArray
	\brief A list of XsInt values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for int*/
void swapInt(int* a, int* b)
{
	int tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for int*/
void copyInt(int* to, int const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for int*/
int compareInt(int const* a, int const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}


//! \brief Descriptor for XsIntArray
XsArrayDescriptor const g_xsIntArrayDescriptor = {
	sizeof(int),
	XSEXPCASTITEMSWAP swapInt,		// swap
	0,								// construct
	XSEXPCASTITEMCOPY copyInt,		// copy construct
	0,								// destruct
	XSEXPCASTITEMCOPY copyInt,		// copy
	XSEXPCASTITEMCOMP compareInt,	// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy	// raw copy
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsStringArray
*/
void XsIntArray_construct(XsIntArray* thisPtr, XsSize count, int const* src)
{
	XsArray_construct(thisPtr, &g_xsIntArrayDescriptor, count, src);
}
