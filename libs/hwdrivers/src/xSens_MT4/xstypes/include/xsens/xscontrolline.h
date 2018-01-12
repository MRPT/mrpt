/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSCONTROLLINE_H
#define XSCONTROLLINE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Serial control lines. */
enum XsControlLine
{
	/** pin 1: Carrier Detect */
	XCL_DCD = 0x0001,
	/** pin 2: Received Data */
	XCL_RD = 0x0002,
	/** pin 3: Transmitted Data */
	XCL_TD = 0x0004,
	/** pin 4: Data Terminal Ready */
	XCL_DTR = 0x0008,
	/** pin 5: Common Ground */
	XCL_GND = 0x0010,
	/** pin 6: Data Set Ready */
	XCL_DSR = 0x0020,
	/** pin 7: Request To Send */
	XCL_RTS = 0x0040,
	/** pin 8: Clear To Send */
	XCL_CTS = 0x0080,
	/** pin 9: Ring Indicator */
	XCL_RI = 0x0100
};
/*! @} */
typedef enum XsControlLine XsControlLine;

#endif  // file guard
