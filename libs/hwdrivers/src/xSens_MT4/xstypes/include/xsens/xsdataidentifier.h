/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDATAIDENTIFIER_H
#define XSDATAIDENTIFIER_H

//////////////////////////////////////////////////////////////////////////////////////////
/*!	\addtogroup enums Global enumerations
	@{
*/

/*!	\enum XsDataIdentifier
	\brief Defines the data identifiers

	\internal
	IMPORTANT! How to add new TYPE values:
	Use the bit reverse of 1, 2, 3 .. etc..
	This way we have the possibility to play with the border between TYPE and FORMAT at a later stage
	so\n
	0 = 0b000000..00 = 0x0000\n
	1 = 0b100000..00 = 0x8000\n
	2 = 0b010000..00 = 0x4000\n
	3 = 0b110000..00 = 0xC000\n
	4 = 0b001000..00 = 0x2000\n
	etc\n
	8 = 0b000100..00 = 0x1000

	So the TYPE grows from left to right
	the FORMAT grows from right to left

	Data that is output in float, fixedpoint or double (etc...) a sub format range is used (currently the lowest 4 bits).
*/
enum XsDataIdentifier
{
	XDI_None					= 0x0000,
	XDI_TypeMask				= 0xFE00,
	XDI_FullTypeMask			= 0xFFF0,
	XDI_FullMask				= 0xFFFF,
	XDI_FormatMask				= 0x01FF,
	XDI_DataFormatMask			= 0x000F,

	XDI_SubFormatMask			= 0x0003,	//determines, float, fp12.20, fp16.32, double output... (where applicable)
	XDI_SubFormatFloat			= 0x0000,
	XDI_SubFormatFp1220			= 0x0001,
	XDI_SubFormatFp1632			= 0x0002,
	XDI_SubFormatDouble			= 0x0003,

	XDI_TemperatureGroup		= 0x0800,
	XDI_Temperature				= 0x0810,

	XDI_TimestampGroup			= 0x1000,
	XDI_UtcTime					= 0x1010,
	XDI_PacketCounter			= 0x1020,
	XDI_Itow					= 0x1030,
	XDI_GpsAge					= 0x1040,
	XDI_PressureAge				= 0x1050,
	XDI_SampleTimeFine			= 0x1060,
	XDI_SampleTimeCoarse		= 0x1070,
	XDI_FrameRange				= 0x1080,	// add for MTw (if needed)
	XDI_PacketCounter8			= 0x1090,
	XDI_SampleTime64			= 0x10A0,

	XDI_OrientationGroup		= 0x2000,
	XDI_CoordSysMask			= 0x000C,
	XDI_CoordSysEnu				= 0x0000,
	XDI_CoordSysNed				= 0x0004,
	XDI_CoordSysNwu				= 0x0008,
	XDI_Quaternion				= 0x2010,
	XDI_RotationMatrix			= 0x2020,
	XDI_EulerAngles				= 0x2030,

	XDI_PressureGroup			= 0x3000,
	XDI_BaroPressure			= 0x3010,

	XDI_AccelerationGroup		= 0x4000,
	XDI_DeltaV					= 0x4010,
	XDI_Acceleration			= 0x4020,
	XDI_FreeAcceleration		= 0x4030,
#ifdef NOT_FOR_PUBLIC_RELEASE
	//XDI_TransposedAcceleration	= 0x4040,
#endif //NOT_FOR_PUBLIC_RELEASE

	XDI_PositionGroup			= 0x5000,
	XDI_AltitudeMsl				= 0x5010,
	XDI_AltitudeEllipsoid		= 0x5020,
	XDI_PositionEcef			= 0x5030,
	XDI_LatLon					= 0x5040,

	XDI_AngularVelocityGroup	= 0x8000,
	XDI_RateOfTurn				= 0x8020,
	XDI_DeltaQ					= 0x8030,

	XDI_GpsGroup				= 0x8800,
	XDI_GpsDop					= 0x8830,
	XDI_GpsSol					= 0x8840,
	XDI_GpsTimeUtc				= 0x8880,
	XDI_GpsSvInfo				= 0x88A0,

	XDI_RawSensorGroup			= 0xA000,
	XDI_RawAccGyrMagTemp		= 0xA010,
	XDI_RawGyroTemp				= 0xA020,
	XDI_RawAcc					= 0xA030,
	XDI_RawGyr					= 0xA040,
	XDI_RawMag					= 0xA050,

	XDI_AnalogInGroup			= 0xB000,
	XDI_AnalogIn1				= 0xB010,
	XDI_AnalogIn2				= 0xB020,

	XDI_MagneticGroup			= 0xC000,
	XDI_MagneticField			= 0xC020,

	XDI_VelocityGroup			= 0xD000,
	XDI_VelocityXYZ				= 0xD010,

	XDI_StatusGroup				= 0xE000,
	XDI_StatusByte				= 0xE010,
	XDI_StatusWord				= 0xE020,
	XDI_Rssi					= 0xE040,

	XDI_IndicationGroup	        = 0x4800, // 0100.1000 -> bit reverse = 0001.0010 -> type 18
	XDI_TriggerIn1				= 0x4810,
	XDI_TriggerIn2				= 0x4820,

#ifdef NOT_FOR_PUBLIC_RELEASE
	/*
	XDI_Accuracy				= 0xF000,
	XDI_GpsHAcc					= 0xF010,
	XDI_GpsVAcc					= 0xF020,
	XDI_GpsSAcc					= 0xF030,
	*/
#endif //NOT_FOR_PUBLIC_RELEASE
};
/*! @} */

typedef enum XsDataIdentifier XsDataIdentifier;

#define XDI_MAX_FREQUENCY		((uint16_t) 0xFFFF)

#ifdef __cplusplus
inline XsDataIdentifier operator | (XsDataIdentifier a, XsDataIdentifier b)
{
	return (XsDataIdentifier) ((int) a | (int) b);
}

inline XsDataIdentifier operator & (XsDataIdentifier a, XsDataIdentifier b)
{
	return (XsDataIdentifier) ((int) a & (int) b);
}

inline XsDataIdentifier operator ~ (XsDataIdentifier a)
{
	return (XsDataIdentifier) ~((unsigned short)a);
}
#endif

#endif // file guard
