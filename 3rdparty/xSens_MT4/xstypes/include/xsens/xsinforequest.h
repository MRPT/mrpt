/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef XSINFOREQUEST_H
#define XSINFOREQUEST_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Information request identifiers
	\details These values are used by the XsDevice::requestInfo function and
	XsCallback::onInfoResponse functions.
*/
enum XsInfoRequest
{
	XIR_BatteryLevel = 0,
	XIR_GpsSvInfo
};
/*! @} */
typedef enum XsInfoRequest XsInfoRequest;

#endif  // file guard
