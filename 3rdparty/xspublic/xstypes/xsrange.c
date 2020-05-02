
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

#include "xsrange.h"

/*! \class XsRange
	\brief A class whose objects can be used to store a range. It provides method to
		   check whether a value is inside the range.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsRange \brief Get the number of values in the range.
    \note The range is *inclusive* [first, last] instead of [first, last>. So count [1, 2] = 2
	\returns The number of values in the range (inclusive)
*/
int XsRange_count(const XsRange* thisPtr)
{
	if (thisPtr->m_last < thisPtr->m_first)
		return 0;
	return 1 + thisPtr->m_last - thisPtr->m_first;
}

/*! \relates XsRange \brief Get the number of values in the range.
    \note The range is *exclusive* [first, last> instead of [first, last]. So interval [1, 2] = 1
	\returns The number of values in the range (exclusive)
*/
int XsRange_interval(const XsRange* thisPtr)
{
	if (thisPtr->m_last <= thisPtr->m_first)
		return 0;
	return thisPtr->m_last - thisPtr->m_first;
}

/*! \relates XsRange \brief Test if the range contains the given value \a i. */
int XsRange_contains(const XsRange* thisPtr, int i)
{
	return (i >= thisPtr->m_first && i <= thisPtr->m_last);
}

/*! \relates XsRange \brief Set a new range. */
void XsRange_setRange(XsRange* thisPtr, int f, int l)
{
	thisPtr->m_first = f;
	thisPtr->m_last = l;
}

/*!	\relates XsRange \brief Test if the range is empty.
	\details An empty range has a last element that is lower than its first element.
	\returns true if the range is empty, false otherwise
*/
int XsRange_empty(const XsRange* thisPtr)
{
	return thisPtr->m_last < thisPtr->m_first;
}

/*! @} */
