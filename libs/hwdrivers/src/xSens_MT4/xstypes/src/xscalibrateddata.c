/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xscalibrateddata.h"

/*!	\struct XsCalibratedData
	\brief Container for combined calibrated measurement data from accelerometers, gyroscopes and
	magnetometers.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsCalibratedData
	\brief Construct an %XsCalibratedData object
*/
void XsCalibratedData_construct(XsCalibratedData* thisPtr, const XsReal* acc, const XsReal* gyr, const XsReal* mag)
{
	XsVector3_construct(&thisPtr->m_acc, acc);
	XsVector3_construct(&thisPtr->m_gyr, gyr);
	XsVector3_construct(&thisPtr->m_mag, mag);
}

/*!	\relates XsCalibratedData
	\brief Destruct an %XsCalibratedData object
*/
void XsCalibratedData_destruct(XsCalibratedData* thisPtr)
{
	XsVector3_destruct(&thisPtr->m_acc);
	XsVector3_destruct(&thisPtr->m_gyr);
	XsVector3_destruct(&thisPtr->m_mag);
}

/*! @} */
