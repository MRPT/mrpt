
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

#include "xsvector.h"
#include <stdlib.h>
#include <math.h>
#include "xsdebugcounters.h"
#include <string.h>
#include "xsmalloc.h"
#include "xsquaternion.h"
#include "xscopy.h"
#include <assert.h>
#include "xsfloatmath.h"

#define realSwap(a,b) { XsReal t = *a; *a = *b; *b = t; }

/*! \class XsVector
	\brief A class that represents a vector of real numbers
*/

/*! \addtogroup cinterface C Interface
	@{
*/

//! \relates XsVector \brief Initialize the %XsVector to refer to the supplied buffer
void XsVector_ref(XsVector* thisPtr, XsSize sz, XsReal* buffer, XsDataFlags flags)
{
	assert(sz==0 || buffer != 0);
	*((XsReal**) &thisPtr->m_data) = buffer;
	*((XsSize*) &thisPtr->m_size) = sz;
	*((XsSize*) &thisPtr->m_flags) = flags;
}

//! \relates XsVector \brief Initialize the %XsVector using \a sz number of items from \a src
void XsVector_construct(XsVector* thisPtr, XsSize sz, const XsReal* src)
{
	if (sz)
	{
		// init to size
		XsReal* data = (XsReal*) xsMathMalloc(sz*sizeof(XsReal));
		assert(data);
		*((XsReal**) &thisPtr->m_data) = data;
		XsVector_incAllocCount();
	}
	else
		*((XsReal**) &thisPtr->m_data) = 0;
	*((XsSize*) &thisPtr->m_flags) = XSDF_Managed;
	*((XsSize*) &thisPtr->m_size) = sz;
	if (src && sz)
		memcpy(thisPtr->m_data, src, sz*sizeof(XsReal));
}

/*! \relates XsVector
	\brief Initialize the XsVector using \a sz number of items from \a src
	\param sz The desired size of the vector
	\param src 0 or a pointer to a buffer containing \a sz items to copy into the %XsVector
*/
void XsVector_assign(XsVector* thisPtr, XsSize sz, const XsReal* src)
{
	if (thisPtr->m_flags == XSDF_FixedSize)
	{
		if (sz == 0)
		{
			*((XsSize*) &thisPtr->m_flags) |= XSDF_Empty;
			return;
		}
		assert(sz == thisPtr->m_size);
		*((XsSize*) &thisPtr->m_flags) &= ~XSDF_Empty;
	}

	if (sz > thisPtr->m_size || sz == 0)
	{
		XsVector_destruct(thisPtr);
		if (sz)
		{
			// init to size
			XsReal* data = (XsReal*) xsMathMalloc(sz*sizeof(XsReal));
			assert(data);
			*((XsReal**) &thisPtr->m_data) = data;
			*((XsSize*) &thisPtr->m_flags) = XSDF_Managed;
			XsVector_incAllocCount();
		}
	}
	*((XsSize*) &thisPtr->m_size) = sz;
	if (src && sz)
		memcpy(thisPtr->m_data, src, sz*sizeof(XsReal));
}

//! \relates XsVector \brief Release and clear the contents of the vector
void XsVector_destruct(XsVector* thisPtr)
{
	if (thisPtr->m_data && (thisPtr->m_flags & XSDF_Managed))
	{
		// clear contents
		xsMathFree((void*) thisPtr->m_data);
		XsVector_incFreeCount();
	}
	// init to 0
	if (!(thisPtr->m_flags & XSDF_FixedSize))
	{
		*((XsReal**) &thisPtr->m_data) = 0;
		*((XsSize*) &thisPtr->m_size) = 0;
		*((XsSize*) &thisPtr->m_flags) = 0;
	}
	else
		*((XsSize*) &thisPtr->m_flags) |= XSDF_Empty;
}

//! \relates XsVector \brief Copy the contents of the %XsVector to \a copy
void XsVector_copy(XsVector* copy, XsVector const* src)
{
	if (copy == src)
		return;

	if (src->m_flags & XSDF_Empty)
		XsVector_destruct(copy);
	else
		XsVector_assign(copy, src->m_size, src->m_data);
}

/*! \relates XsVector
	\brief Compute and return the dot product of XsVectors \a a and \a b
*/
XsReal XsVector_dotProduct(const XsVector* a, const XsVector* b)
{
	XsSize i;
	XsReal r = XsMath_zero;
	assert(a->m_size == b->m_size);

	for (i = a->m_size; i--; )
		r += a->m_data[i]*b->m_data[i];
	return r;
}

/*! \relates XsVector
	\brief Compute and return the cartesian length
	\returns The cartesian length (square root of the dot product) of the vector
*/
XsReal XsVector_cartesianLength(const XsVector* thisPtr)
{
	return sqrt(XsVector_dotProduct(thisPtr, thisPtr));
}

/*! \relates XsVector
	\brief Normalize the vector
*/
void XsVector_normalize(XsVector* thisPtr)
{
	XsReal length = XsVector_cartesianLength(thisPtr);

	if (length < XsMath_tinyValue)
	{
		XsVector_setZero(thisPtr);
		return;
	}

	XsVector_multiplyScalar(thisPtr, XsMath_one / length, thisPtr);
}

//! \relates XsVector \brief Sets all elements of the %XsVector to 0
void XsVector_setZero(XsVector* thisPtr)
{
#if XSREAL_ALLOWS_MEMCPY
	memset(thisPtr->m_data, 0, sizeof(XsReal)*thisPtr->m_size);
#else
	XsSize i;
	for (i = 0; i < thisPtr->m_size; ++i)
		thisPtr->m_data[i] = XsMath_zero;
#endif
}

//! \relates XsVector \brief Sets all elements of the %XsVector to \a value
void XsVector_fill(struct XsVector* thisPtr, XsReal value)
{
	XsSize i;
	for (i = 0; i < thisPtr->m_size; ++i)
		thisPtr->m_data[i] = value;
}

//! \relates XsVector \brief Returns a non-zero value if the XsVector does not contain any values
int XsVector_empty(const XsVector* thisPtr)
{
	return (thisPtr->m_size == 0) || (thisPtr->m_flags & XSDF_Empty);
}

//! \relates XsVector \brief Multiplies all values in this XsVector by \a scalar and puts the result in XsVector \a dest
void XsVector_multiplyScalar(const XsVector* thisPtr, XsReal scalar, XsVector* dest)
{
	XsSize i;
	XsVector_assign(dest, thisPtr->m_size, 0);
	for (i = 0; i < thisPtr->m_size; ++i)
		dest->m_data[i] = thisPtr->m_data[i] * scalar;
}

/*! \relates XsVector
	\brief Get an effective angular velocity from the quaternion, which must represent a delta angle.
	\param deltaT The length of the time interval over which \a quat was integrated in seconds
	\param quat The orientation increment to convert to an angular velocity
	\returns A vector containing the effective angular velocity in radians around each axis.
*/
void XsVector_angularVelocityFromQuaternion(XsVector* thisPtr, XsReal deltaT, const XsQuaternion* quat)
{
	XsReal a;
	if (XsQuaternion_empty(quat))
	{
		XsVector_destruct(thisPtr);
		return;
	}

	XsVector_assign(thisPtr, 3, &quat->m_data[1]);
	a = XsVector_cartesianLength(thisPtr);
	XsVector_multiplyScalar(thisPtr, (a > XsMath_tinyValue) ? (XsMath_two*asin(a)/(a*deltaT)) : (XsMath_two/deltaT), thisPtr);
}

/*! \relates XsVector \brief Swap the contents of \a a and \a b
	\details This function swaps the internal buffers so no actual data is moved around. For unmanaged
	data an elementwise swap is done, but only if the vectors are the same size.
	\param a Object whose contents will be placed in \a b
	\param b Object whose contents will be placed in \a a
*/
void XsVector_swap(XsVector* a, XsVector* b)
{
#ifdef __ICCARM__
	#pragma diag_suppress=Pe370
#endif
	XSLISTSWAP3(XsReal, XsVector, realSwap)
}

/*! \relates XsVector
	\brief Returns non-zero when the two vectors are identical
	\param a Vector to compare against \a b
	\param b Vector to compare against \a a
	\returns non-zero when the vectors are identical
*/
int XsVector_equal(const struct XsVector* a, const struct XsVector* b)
{
	if (a == b)
		return 1;
	if (!a || !b)
		return 0;
	if (XsVector_empty(a) && XsVector_empty(b))
		return 1;
	if (a->m_size != b->m_size)
		return 0;
	return memcmp(a->m_data, b->m_data, a->m_size*sizeof(XsReal)) == 0;
}

/*! \relates XsVector
	\brief Returns non-zero if the two vectors are equal within \a epsilon

	\param thatPtr the vector to compare with
	\param epsilon the maximum difference between values
	\returns non-zero when the vectors are equal within \a epsilon
*/
int XsVector_compare(const struct XsVector* thisPtr, const struct XsVector* thatPtr, XsReal epsilon)
{
	XsSize i;

	if (thisPtr == thatPtr)
		return 1;
	if (!thisPtr || !thatPtr)
		return 0;
	if (XsVector_empty(thisPtr) && XsVector_empty(thatPtr))
		return 1;
	if (thisPtr->m_size != thatPtr->m_size)
		return 0;

	for (i = 0; i < thisPtr->m_size; i++)
	{
		if (fabs(thisPtr->m_data[i] - thatPtr->m_data[i]) > epsilon)
			return 0;
	}
	return 1;
}

/*! @} */
