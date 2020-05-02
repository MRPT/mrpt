
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

#include "xseuler.h"
#include "xsquaternion.h"
#include <math.h>
#include "xsfloatmath.h"

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
