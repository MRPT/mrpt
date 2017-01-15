/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSOUTPUTMODE_H
#define XSOUTPUTMODE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
//! Bit values for legacy output mode
enum XsOutputMode {
	XOM_None			= 0x0000,
	XOM_Temperature		= 0x0001,
	XOM_Calibrated		= 0x0002,
	XOM_Orientation		= 0x0004,
	XOM_Auxiliary		= 0x0008,
	XOM_Position		= 0x0010,
	XOM_Velocity		= 0x0020,
	XOM_Sdi				= 0x0200,
	XOM_Status			= 0x0800,
	XOM_GpsPvt_Pressure	= 0x1000,
	XOM_Reserved		= 0x2000,
	XOM_Raw				= 0x4000,
	XOM_Mt9				= 0x8000
};
/*! @} */
typedef enum XsOutputMode XsOutputMode;

// Extended (analog) Output Modes
#define XS_EXTOUTPUTMODE_DISABLED			0x0000
#define XS_EXTOUTPUTMODE_EULER				0x0001

#define XS_DEFAULT_OUTPUT_MODE			XOM_Orientation

#ifdef __cplusplus
/*! \brief Allow logical or of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator | (XsOutputMode a, XsOutputMode b)
{
	return (XsOutputMode) ((unsigned short) a | (unsigned short) b);
}

/*! \brief Allow logical and of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator & (XsOutputMode a, XsOutputMode b)
{
	return (XsOutputMode) ((unsigned short) a & (unsigned short) b);
}

/*! \brief Allow logical inversion of XsOutputMode to be a valid XsOutputMode value */
inline XsOutputMode operator ~ (XsOutputMode a)
{
	return (XsOutputMode) ~((unsigned short)a);
}
#endif

#endif // XSOUTPUTMODE_H
