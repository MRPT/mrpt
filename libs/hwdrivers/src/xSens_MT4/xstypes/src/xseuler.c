/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xseuler.h"
#include "xsquaternion.h"
#include <math.h>

/*! \class XsEuler
	\brief Contains Euler Angle data and conversion from Quaternion
	\details Euler Angles are computed as a rotation around the x-axis, followed by a rotation
	around the y-axis, followed by a rotation around the z-axis.
	In aerospace terms, x,y and z are also often referred to as roll pitch and yaw, so those names
	are also available as a convenience.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsEuler
	\brief Clears all angles in the XsEuler object by setting them to 0
*/
void XsEuler_destruct(XsEuler* thisPtr)
{
	thisPtr->m_x = XsMath_zero;
	thisPtr->m_y = XsMath_zero;
	thisPtr->m_z = XsMath_zero;
}

/*! \relates XsEuler
	\brief Returns true if all angles in this object are zero
*/
int XsEuler_empty(const XsEuler* thisPtr)
{
	return thisPtr->m_x == XsMath_zero && thisPtr->m_y == XsMath_zero && thisPtr->m_z == XsMath_zero;
}

/*! \relates XsEuler
	\brief Get an euler angle representation of the quaternion.
*/
void XsEuler_fromQuaternion(XsEuler* thisPtr, const XsQuaternion* quat)
{
	XsReal sqw, dphi, dpsi;

	if (XsQuaternion_empty(quat))
	{
		XsEuler_destruct(thisPtr);
		return;
	}

	sqw = quat->m_w * quat->m_w;
	dphi = XsMath_two * (sqw + quat->m_z * quat->m_z) - XsMath_one;
	dpsi = XsMath_two * (sqw + quat->m_x * quat->m_x) - XsMath_one;

	thisPtr->m_x =  XsMath_rad2deg(atan2(XsMath_two*(quat->m_y*quat->m_z + quat->m_w*quat->m_x), dphi));
	thisPtr->m_y = -XsMath_rad2deg(XsMath_asinClamped(XsMath_two*(quat->m_x*quat->m_z - quat->m_w*quat->m_y)));
	thisPtr->m_z =  XsMath_rad2deg(atan2(XsMath_two*(quat->m_x*quat->m_y + quat->m_w*quat->m_z), dpsi));
}

/*! @} */
