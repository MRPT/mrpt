
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


#include "xssimpleversion.h"

/*! \class XsSimpleVersion
	\brief A class to store version information
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsSimpleVersion \brief Test if this is a null-version. */
int XsSimpleVersion_empty(const XsSimpleVersion* thisPtr)
{
#ifdef __cplusplus
	return thisPtr->major() == 0 && thisPtr->minor() == 0 && thisPtr->revision() == 0;
#else
	return thisPtr->m_major == 0 && thisPtr->m_minor == 0 && thisPtr->m_revision == 0;
#endif
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsSimpleVersion_swap(struct XsSimpleVersion* a, struct XsSimpleVersion* b)
{
	XsSimpleVersion tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \brief Compare two XsSimpleVersion objects.
	\param a The left hand side of the comparison
	\param b The right hand side of the comparison
	\return 0 when they're equal
*/
int XsSimpleVersion_compare(XsSimpleVersion const* a, XsSimpleVersion const* b)
{
#ifdef __cplusplus
	return a->major() != b->major() || a->minor() != b->minor() || a->revision() != b->revision();
#else
	return a->m_major != b->m_major || a->m_minor != b->m_minor || a->m_revision != b->m_revision;
#endif
}

/*! @} */
