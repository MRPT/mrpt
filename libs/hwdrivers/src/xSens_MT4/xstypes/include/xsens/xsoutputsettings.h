/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSOUTPUTSETTINGS_H
#define XSOUTPUTSETTINGS_H

/*!	\addtogroup enums Global enumerations
	@{
*/
//! Bit values for output settings
enum XsOutputSettings {
	XOS_Timestamp_Mask					= 0x00000003,
	XOS_Timestamp_None					= 0x00000000,
	XOS_Timestamp_PacketCounter			= 0x00000001,
	XOS_Timestamp_SampleUtc				= 0x00000002,
	XOS_OrientationMode_Mask			= 0x0000000C,
	XOS_OrientationMode_Quaternion		= 0x00000000,
	XOS_OrientationMode_Euler			= 0x00000004,
	XOS_OrientationMode_Matrix			= 0x00000008,
	XOS_CalibratedMode_All				= 0x00000000,
	XOS_CalibratedMode_AccGyrMag_Mask	= 0x00000070,
	XOS_CalibratedMode_Mask				= XOS_CalibratedMode_AccGyrMag_Mask,
	XOS_CalibratedMode_Acc_Mask			= 0x00000010,
	XOS_CalibratedMode_AccOnly			= 0x00000060,
	XOS_CalibratedMode_AccGyrOnly		= 0x00000040,
	XOS_CalibratedMode_AccMagOnly		= 0x00000020,
	XOS_CalibratedMode_Gyr_Mask			= 0x00000020,
	XOS_CalibratedMode_GyrOnly			= 0x00000050,
	XOS_CalibratedMode_GyrMagOnly		= 0x00000010,
	XOS_CalibratedMode_Mag_Mask			= 0x00000040,
	XOS_CalibratedMode_MagOnly			= 0x00000030,
	XOS_Status_Compact					= 0x00000000,
	XOS_Status_Detailed					= 0x00000080,
	XOS_Dataformat_Mask					= 0x00000300,
	XOS_Dataformat_Float				= 0x00000000,
	XOS_Dataformat_F1220				= 0x00000100,
	XOS_Dataformat_Fp1632				= 0x00000200,
	XOS_Dataformat_Double				= 0x00000300,

	XOS_AuxiliaryMode_Mask				= 0x00000C00,
	XOS_AuxiliaryMode_Ain1_Mask			= 0x00000400,
	XOS_AuxiliaryMode_Ain2_Mask			= 0x00000800,
	XOS_AuxiliaryMode_Ain1				= 0x00000800,
	XOS_AuxiliaryMode_Ain2				= 0x00000400,
	XOS_PositionMode_Mask				= 0x0001C000,
	XOS_PositionMode_Lla_Wgs84			= 0x00000000,
	XOS_VelocityMode_Mask				= 0x00060000,
	XOS_VelocityMode_Ms_Xyz				= 0x00000000,
	XOS_GpsInGpsPvt						= 0x00000000,
	XOS_NoGpsInGpsPvt					= 0x00080000,
	XOS_ExtendedTemperature_Mask		= 0x01000000,
	XOS_Coordinates_Ned					= 0x80000000

	//XOS_Uncertainty_Orient				= 0x00100000
	//XOS_Uncertainty_Pos					= 0x00200000
	//XOS_Uncertainty_Vel					= 0x00400000
	//XOS_Uncertainty_Mask				= 0x00F00000

};
/*! @} */
typedef enum XsOutputSettings XsOutputSettings;

#define XS_DEFAULT_OUTPUT_SETTINGS		(XsOutputSettings)(XOS_OrientationMode_Quaternion | XOS_Timestamp_PacketCounter)

#ifdef __cplusplus
/*! \brief Allow logical or of XsOutputSettings to be a valid XsOutputSettings value */
inline XsOutputSettings operator | (XsOutputSettings a, XsOutputSettings b)
{
	return (XsOutputSettings) ((unsigned long) a | (unsigned long) b);
}

/*! \brief Allow logical and of XsOutputSettings to be a valid XsOutputSettings value */
inline XsOutputSettings operator & (XsOutputSettings a, XsOutputSettings b)
{
	return (XsOutputSettings) ((unsigned long) a & (unsigned long) b);
}

/*! \brief Allow logical inversion of XsOutputSettings to be a valid XsOutputSettings value */
inline XsOutputSettings operator ~ (XsOutputSettings a)
{
	return (XsOutputSettings) ~((unsigned long)a);
}

/*! \brief Allow &= operator on XsOutputSettings */
inline XsOutputSettings& operator &= (XsOutputSettings& left, XsOutputSettings const & right)
{
	return left = left & right;
}

/*! \brief Allow |= operator on XsOutputSettings */
inline XsOutputSettings& operator |= (XsOutputSettings& left, XsOutputSettings const & right)
{
	return left = left | right;
}

#endif

#endif // file guard
