/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSCONTROLLINE_H
#define XSCONTROLLINE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Serial control lines. */
enum XsControlLine {
	XCL_DCD		= 0x0001,		//!< pin 1: Carrier Detect
	XCL_RD		= 0x0002,		//!< pin 2: Received Data
	XCL_TD		= 0x0004,		//!< pin 3: Transmitted Data
	XCL_DTR		= 0x0008,		//!< pin 4: Data Terminal Ready
	XCL_GND		= 0x0010,		//!< pin 5: Common Ground
	XCL_DSR		= 0x0020,		//!< pin 6: Data Set Ready
	XCL_RTS		= 0x0040,		//!< pin 7: Request To Send
	XCL_CTS		= 0x0080,		//!< pin 8: Clear To Send
	XCL_RI		= 0x0100		//!< pin 9: Ring Indicator
};
/*! @} */
typedef enum XsControlLine XsControlLine;

#endif // file guard
