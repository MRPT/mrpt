/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xssdidata.h"

/*! \class XsSdiData
	\brief Contains StrapDown Integration (SDI) data.
	\details SDI data consists of a rotation and an acceleration, expressed as an orientation increment
	(also known as deltaQ) and a velocity increment (also known as deltaV).
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsSdiData
	\brief Initialize an %XsSdiData object with the optional arguments.
	\param orientationIncrement The orientation increment to initialize the object with, may be 0
	\param velocityIncrement The velocity increment to initialize the object with, may be 0
*/
void XsSdiData_construct(XsSdiData* thisPtr, const XsReal* orientationIncrement, const XsReal* velocityIncrement)
{
	if (orientationIncrement)
	{
		thisPtr->m_orientationIncrement.m_data[0] = orientationIncrement[0];
		thisPtr->m_orientationIncrement.m_data[1] = orientationIncrement[1];
		thisPtr->m_orientationIncrement.m_data[2] = orientationIncrement[2];
		thisPtr->m_orientationIncrement.m_data[3] = orientationIncrement[3];
	}
	else
		XsQuaternion_destruct(&thisPtr->m_orientationIncrement);

	if (velocityIncrement)
		XsVector3_assign(&thisPtr->m_velocityIncrement, velocityIncrement);
	else
		XsVector3_destruct(&thisPtr->m_velocityIncrement);		
}

/*! \relates XsSdiData
	\brief Destruct the object, makes the fields invalid
*/
void XsSdiData_destruct(XsSdiData* thisPtr)
{
	XsQuaternion_destruct(&thisPtr->m_orientationIncrement);
	XsVector3_destruct(&thisPtr->m_velocityIncrement);
}

/*! @} */ 
