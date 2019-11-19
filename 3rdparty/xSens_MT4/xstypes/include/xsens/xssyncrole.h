/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef XSSYNCROLE_H
#define XSSYNCROLE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization roles
 */
enum XsSyncRole
{
	XSR_Slave,
	XSR_None,
	XSR_MasterSlave,
	XSR_Master
};
/*! @} */
typedef enum XsSyncRole XsSyncRole;

#endif  // file guard
