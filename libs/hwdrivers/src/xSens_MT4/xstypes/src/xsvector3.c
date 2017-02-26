/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsvector3.h"
#include <string.h>

//lint -e641 conversion from enum to int should not be a problem

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
	// don't do anything, no memory needs to be freed, but call parent destructor just to be sure
	assert(thisPtr->m_vector.m_flags & XSDF_FixedSize);	
	//XsVector_destruct(thisPtr);
	thisPtr->m_vector.m_data[0] = XsMath_zero;
	thisPtr->m_vector.m_data[1] = XsMath_zero;
	thisPtr->m_vector.m_data[2] = XsMath_zero;
}

/*! \relates XsVector3 \brief Copy the contents of the %XsVector3 to \a copy */
void XsVector3_copy(XsVector* copy, XsVector3 const* src)
{
	XsVector_copy(copy, &src->m_vector);
}

/*! @} */
