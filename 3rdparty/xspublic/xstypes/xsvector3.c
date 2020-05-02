
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

#include "xsvector3.h"
#include <string.h>

/*! \class XsVector3
	\brief A class that represents a fixed size (3) vector
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVector3 \brief Init the %XsVector3 and copy the data from \a src into the vector if \a src is not null */
void XsVector3_construct(XsVector3* thisPtr, const XsReal* src)
{
	XsVector_ref(&thisPtr->m_vector, 3, (XsReal*) thisPtr->m_fixedData, XSDF_FixedSize);
	if (src)
		memcpy((XsReal*) thisPtr->m_fixedData, src, 3*sizeof(XsReal));
}

/*! \relates XsVector3 \brief Init the %XsVector3 and copy the data from \a src into the vector if \a src is not null */
void XsVector3_assign(XsVector3* thisPtr, const XsReal* src)
{
	if (src)
		memcpy((XsReal*) thisPtr->m_fixedData, (XsReal*) src, 3*sizeof(XsReal));
}

/*! \relates XsVector3 \brief Frees the XsVector3 */
void XsVector3_destruct(XsVector3* thisPtr)
{
	// don't do anything, no memory needs to be freed
	assert(thisPtr->m_vector.m_flags & XSDF_FixedSize);
	(void)thisPtr;
}

/*! \relates XsVector3 \brief Copy the contents of the %XsVector3 to \a copy */
void XsVector3_copy(XsVector* copy, XsVector3 const* src)
{
	XsVector_copy(copy, &src->m_vector);
}

/*! @} */
