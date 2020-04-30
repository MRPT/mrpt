
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

#include "xsmatrix3x3.h"
#include <string.h>

/*! \class XsMatrix3x3
	\brief A class that represents a fixed size (3x3) matrix
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsMatrix3x3 \brief Init the %XsMatrix3x3 */
void XsMatrix3x3_construct(XsMatrix3x3* thisPtr)
{
	XsMatrix_ref(&thisPtr->m_matrix, 3, 3, 3, (XsReal*) thisPtr->m_fixedData, XSDF_FixedSize);
}

/*! \relates XsMatrix3x3 \brief Init the %XsMatrix3x3 and copy the data from \a src into the matrix if \a src is not null */
void XsMatrix3x3_assign(XsMatrix3x3* thisPtr, const XsReal* src, XsSize srcStride)
{
	XsSize r, c;

	if (src)
	{
		if (srcStride == 0 || srcStride == 3)
			memcpy(thisPtr->m_matrix.m_data, src, 3*3*sizeof(XsReal));
		else
		{
			for (r = 0; r < 3; ++r)
				for (c = 0; c < 3; ++c)
					thisPtr->m_matrix.m_data[r*3+c] = src[r*srcStride + c];
		}
	}
}

/*! \relates XsMatrix3x3 \brief Frees the Matrix3x3 */
void XsMatrix3x3_destruct(XsMatrix3x3* thisPtr)
{
	// don't do anything, no memory needs to be freed, which is what  XsMatrix_destruct will figure out
	assert(thisPtr->m_matrix.m_flags & XSDF_FixedSize);
	(void) thisPtr;
	//XsMatrix_destruct(&thisPtr->m_matrix);
}

/*! \relates XsMatrix3x3 \brief Copy the contents of the %XsMatrix3x3 to \a copy */
void XsMatrix3x3_copy(XsMatrix* copy, XsMatrix3x3 const* src)
{
	XsMatrix_copy(copy, &src->m_matrix);
}

/*! @} */
