/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSYNCROLE_H
#define XSSYNCROLE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization roles
*/
enum XsSyncRole {
	XSR_Slave,
	XSR_None,
	XSR_MasterSlave,
	XSR_Master
};
/*! @} */
typedef enum XsSyncRole XsSyncRole;

#endif // file guard
