/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
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
