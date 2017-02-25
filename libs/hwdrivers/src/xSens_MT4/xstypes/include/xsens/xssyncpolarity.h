/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSYNCPOLARITY_H
#define XSSYNCPOLARITY_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Signal polarity */
enum XsSyncPolarity
{
	XSP_None = 0,						/*!< \brief Don't generate or react to trigger level changes */
	XSP_RisingEdge = 1,					/*!< \brief React to a rising edge on input */
	XSP_PositivePulse = XSP_RisingEdge,	/*!< \brief Generate a positive pulse on output */
	XSP_FallingEdge = 2,					/*!< \brief React to a falling edge on input */
	XSP_NegativePulse = XSP_FallingEdge,	/*!< \brief Generate a falling edge on output */
	XSP_Both							/*!< \brief Toggle output or react on all toggles on input */
};
/*! @} */
typedef enum XsSyncPolarity XsSyncPolarity;

#endif // file guard
