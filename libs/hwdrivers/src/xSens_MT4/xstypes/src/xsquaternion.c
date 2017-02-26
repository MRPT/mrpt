/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsquaternion.h"
#include "xseuler.h"
#include "xsmatrix.h"
#include "xsvector.h"
#include <math.h>

/*! \class XsQuaternion
	\brief A class that implements a quaternion
*/	

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsQuaternion \brief Clear the quaternion by setting all values to 0 */
void XsQuaternion_destruct(XsQuaternion* thisPtr)
{
	thisPtr->m_w = XsMath_zero;
	thisPtr->m_x = XsMath_zero;
	thisPtr->m_y = XsMath_zero;
	thisPtr->m_z = XsMath_zero;
}

/*! \relates XsQuaternion \brief Test if this is a null object. */
int XsQuaternion_empty(const XsQuaternion* thisPtr)
{
	return thisPtr->m_w == XsMath_zero && thisPtr->m_x == XsMath_zero && thisPtr->m_y == XsMath_zero && thisPtr->m_z == XsMath_zero;
}

/*! \relates XsQuaternion
	\brief Invert this quaternion
	\details Where \a q = a + bi + cj + dk, this function will replace it with a - bi - cj - dk.
*/
void XsQuaternion_invert(XsQuaternion* thisPtr)
{
	XsQuaternion_inverse(thisPtr, thisPtr);
}

/*! \relates XsQuaternion
	\brief Compute the inverse/conjugate of this quaternion
	\details Where \a q = a + bi + cj + dk, this function will return a - bi - cj - dk in \a dest.
	\param dest The object to write to
*/
void XsQuaternion_inverse(const XsQuaternion* thisPtr, XsQuaternion* dest)
{
	dest->m_w = thisPtr->m_w;
	dest->m_x = -thisPtr->m_x;
	dest->m_y = -thisPtr->m_y;
	dest->m_z = -thisPtr->m_z;
}

/*! \relates XsQuaternion \brief Create a normalized version of this quaternion	
*/
XsReal XsQuaternion_normalized(const XsQuaternion* thisPtr, XsQuaternion* dest)
{
	XsReal divisor, length = sqrt(thisPtr->m_w*thisPtr->m_w + thisPtr->m_x*thisPtr->m_x + thisPtr->m_y*thisPtr->m_y + thisPtr->m_z*thisPtr->m_z);
#ifndef XSENS_MATH_MATLAB_COMPLIANT
	divisor = XsMath_one/length;
	if (thisPtr->m_w < 0)
		divisor = -divisor;
	dest->m_w = thisPtr->m_w * divisor;
	dest->m_x = thisPtr->m_x * divisor;
	dest->m_y = thisPtr->m_y * divisor;
	dest->m_z = thisPtr->m_z * divisor;
#else
	if (thisPtr->m_w < 0)
		divisor = -length;
	else
		divisor = length;
	dest->m_w = thisPtr->m_w / divisor;
	dest->m_x = thisPtr->m_x / divisor;
	dest->m_y = thisPtr->m_y / divisor;
	dest->m_z = thisPtr->m_z / divisor;
#endif
	return length;
}

/*! \relates XsQuaternion \brief Normalize this quaternion
*/
XsReal XsQuaternion_normalize(XsQuaternion* thisPtr)
{
	return XsQuaternion_normalized(thisPtr, thisPtr);
}

/*! \relates XsQuaternion \brief Create a quaternion representation from \a euler angles */
void XsQuaternion_fromEulerAngles(XsQuaternion* thisPtr, const XsEuler* src)
{
	XsReal cosX, sinX, cosY, sinY, cosZ, sinZ;

	if (XsEuler_empty(src))
	{
		*thisPtr = *XsQuaternion_identity();
		return;
	}

	cosX = cos(XsMath_pt5 * XsMath_deg2rad(src->m_x));
	sinX = sin(XsMath_pt5 * XsMath_deg2rad(src->m_x));
	cosY = cos(XsMath_pt5 * XsMath_deg2rad(src->m_y));
	sinY = sin(XsMath_pt5 * XsMath_deg2rad(src->m_y));
	cosZ = cos(XsMath_pt5 * XsMath_deg2rad(src->m_z));
	sinZ = sin(XsMath_pt5 * XsMath_deg2rad(src->m_z));

	thisPtr->m_w = cosX * cosY * cosZ + sinX * sinY * sinZ;
	thisPtr->m_x = sinX * cosY * cosZ - cosX * sinY * sinZ;
	thisPtr->m_y = cosX * sinY * cosZ + sinX * cosY * sinZ;
	thisPtr->m_z = cosX * cosY * sinZ - sinX * sinY * cosZ;
}

/*! \relates XsQuaternion
	\brief Create a quaternion representation of orientation matrix \a ori
	\details The matrix \a ori is interpreted as an orthonormal orientation matrix, which is
	translated into a quaternion representation. If \a ori is not a 3x3 matrix, a null-quaternion is
	returned.
	\param ori The source orientation matrix
*/
void XsQuaternion_fromRotationMatrix(XsQuaternion* thisPtr, const XsMatrix* ori)
{
	XsReal trace;	// Trace of matrix
	XsReal s;

	if (!XsMatrix_dimensionsMatch(ori, 3,3))
	{
		XsQuaternion_destruct(thisPtr);
		return;
	}

//	XsQuaternion result;

	// Calculate trace of matrix
	// T = 4 - 4x^2 - 4y^2 - 4z^2
	//   = 4 ( 1 - x^2 - y^2 - z^2)
	//   = 1 + fRgs[0][0] + fRgs[1][1] + fRgs[2][2] (= 4q0^2)

	trace = XsMatrix_value(ori, 0, 0) + XsMatrix_value(ori, 1, 1) + XsMatrix_value(ori, 2, 2) + XsMath_one;

	// If the trace of the matrix is greater than zero,
	// then perform an "instant" calculation.
	// Important note wrt. rounding errors:
	// Test if (T > 0.0000001) to avoid large distortions!
	// If the trace of the matrix is equal to zero, then identify
	// which major diagonal element has the greatest value
	if (trace*trace >= XsMath_tinyValue)
	{
		s = XsMath_two * sqrt(trace);
		thisPtr->m_w = XsMath_pt25 * s;

		#ifndef XSENS_MATH_MATLAB_COMPLIANT
			s = XsMath_one/s;
			thisPtr->m_x = (XsMatrix_value(ori, 1, 2) - XsMatrix_value(ori, 2, 1)) * s;
			thisPtr->m_y = (XsMatrix_value(ori, 2, 0) - XsMatrix_value(ori, 0, 2)) * s;
			thisPtr->m_z = (XsMatrix_value(ori, 0, 1) - XsMatrix_value(ori, 1, 0)) * s;	
		#else
			thisPtr->m_x = (XsMatrix_value(ori, 1, 2) - XsMatrix_value(ori, 2, 1)) / s;
			thisPtr->m_y = (XsMatrix_value(ori, 2, 0) - XsMatrix_value(ori, 0, 2)) / s;
			thisPtr->m_z = (XsMatrix_value(ori, 0, 1) - XsMatrix_value(ori, 1, 0)) / s;	
		#endif
	}
	else if ((XsMatrix_value(ori, 0, 0) > XsMatrix_value(ori, 1, 1)) && (XsMatrix_value(ori, 0, 0) > XsMatrix_value(ori, 2, 2)))
	{
		trace = XsMath_one + XsMatrix_value(ori, 0, 0) - XsMatrix_value(ori, 1, 1) - XsMatrix_value(ori, 2, 2);
		s = XsMath_two * sqrt(trace);
		thisPtr->m_x = XsMath_pt25 * s;

		#ifndef XSENS_MATH_MATLAB_COMPLIANT
			s = XsMath_one/s;
			thisPtr->m_w = (XsMatrix_value(ori, 1, 2) - XsMatrix_value(ori, 2, 1)) * s;
			thisPtr->m_y = (XsMatrix_value(ori, 0, 1) + XsMatrix_value(ori, 1, 0)) * s;
			thisPtr->m_z = (XsMatrix_value(ori, 2, 0) + XsMatrix_value(ori, 0, 2)) * s;
		#else
			thisPtr->m_w = (XsMatrix_value(ori, 1, 2) - XsMatrix_value(ori, 2, 1)) / s;
			thisPtr->m_y = (XsMatrix_value(ori, 0, 1) + XsMatrix_value(ori, 1, 0)) / s;
			thisPtr->m_z = (XsMatrix_value(ori, 2, 0) + XsMatrix_value(ori, 0, 2)) / s;
		#endif
	}
	else if (XsMatrix_value(ori, 1, 1) > XsMatrix_value(ori, 2, 2))
	{
		trace = XsMath_one + XsMatrix_value(ori, 1, 1) - XsMatrix_value(ori, 0, 0) - XsMatrix_value(ori, 2, 2);
		s = XsMath_two * sqrt(trace);
		thisPtr->m_y = XsMath_pt25 * s;

		#ifndef XSENS_MATH_MATLAB_COMPLIANT
			s = XsMath_one/s;
			thisPtr->m_w = (XsMatrix_value(ori, 2, 0) - XsMatrix_value(ori, 0, 2)) * s;
			thisPtr->m_x = (XsMatrix_value(ori, 0, 1) + XsMatrix_value(ori, 1, 0)) * s;
			thisPtr->m_z = (XsMatrix_value(ori, 1, 2) + XsMatrix_value(ori, 2, 1)) * s;
		#else
			thisPtr->m_w = (XsMatrix_value(ori, 2, 0) - XsMatrix_value(ori, 0, 2)) / s;
			thisPtr->m_x = (XsMatrix_value(ori, 0, 1) + XsMatrix_value(ori, 1, 0)) / s;
			thisPtr->m_z = (XsMatrix_value(ori, 1, 2) + XsMatrix_value(ori, 2, 1)) / s;
		#endif
	}
	else
	{
		trace = XsMath_one + XsMatrix_value(ori, 2, 2) - XsMatrix_value(ori, 0, 0) - XsMatrix_value(ori, 1, 1);
		s = XsMath_two * sqrt(trace);
		thisPtr->m_z = XsMath_pt25 * s;

		#ifndef XSENS_MATH_MATLAB_COMPLIANT
			s = XsMath_one/s;
			thisPtr->m_w = (XsMatrix_value(ori, 0, 1) - XsMatrix_value(ori, 1, 0)) * s;
			thisPtr->m_x = (XsMatrix_value(ori, 2, 0) + XsMatrix_value(ori, 0, 2)) * s;
			thisPtr->m_y = (XsMatrix_value(ori, 1, 2) + XsMatrix_value(ori, 2, 1)) * s;
		#else
			thisPtr->m_w = (XsMatrix_value(ori, 0, 1) - XsMatrix_value(ori, 1, 0)) / s;
			thisPtr->m_x = (XsMatrix_value(ori, 2, 0) + XsMatrix_value(ori, 0, 2)) / s;
			thisPtr->m_y = (XsMatrix_value(ori, 1, 2) + XsMatrix_value(ori, 2, 1)) / s;
		#endif
	}

	XsQuaternion_inverse(thisPtr, thisPtr);
}

/*! \relates XsQuaternion \brief Returns an XsQuaternion that represents the identity quaternion */
const XsQuaternion* XsQuaternion_identity(void)
{
	static const XsQuaternion sIdentity = { { { 1, 0, 0, 0 } } };
	return &sIdentity;
}

/*! \relates XsQuaternion \brief Multiply \a left quaternion with \a right quaternion and put the result in \a dest. The parameters may point to the same XsQuaternion(s). */
void XsQuaternion_multiply(const XsQuaternion* left, const XsQuaternion* right, XsQuaternion* dest)
{
	XsReal qa0 = left->m_w;
	XsReal qa1 = left->m_x;
	XsReal qa2 = left->m_y;
	XsReal qa3 = left->m_z;

	XsReal qb0 = right->m_w;
	XsReal qb1 = right->m_x;
	XsReal qb2 = right->m_y;
	XsReal qb3 = right->m_z;

	dest->m_w = qa0 * qb0 - qa1 * qb1 - qa2 * qb2 - qa3 * qb3;
	dest->m_x = qa1 * qb0 + qa0 * qb1 - qa3 * qb2 + qa2 * qb3;
	dest->m_y = qa2 * qb0 + qa3 * qb1 + qa0 * qb2 - qa1 * qb3;
	dest->m_z = qa3 * qb0 - qa2 * qb1 + qa1 * qb2 + qa0 * qb3;
}

/*! \relates XsQuaternion \brief Swap the contents of \a a and \a b
*/
void XsQuaternion_swap(XsQuaternion* a, XsQuaternion* b)
{
	XsReal t;
	int i;
	for (i = 0; i < 4; ++i)
	{
		t = a->m_data[i];
		a->m_data[i] = b->m_data[i];
		b->m_data[i] = t;
	}
}

/*! \relates XsQuaternion \brief Copies the contents of \a thisPtr into \a copy
*/
void XsQuaternion_copy(XsQuaternion* copy, XsQuaternion const* src)
{
	copy->m_w = src->m_w;
	copy->m_x = src->m_x;
	copy->m_y = src->m_y;
	copy->m_z = src->m_z;
}

/*! \relates XsQuaternion \brief returns non-zero if \a a and \a b are numerically equal
*/
int XsQuaternion_equal(XsQuaternion const* a, XsQuaternion const* b)
{
	return (a->m_w == b->m_w &&
			a->m_x == b->m_x &&
			a->m_y == b->m_y &&
			a->m_z == b->m_z);
}

/*! @} */
