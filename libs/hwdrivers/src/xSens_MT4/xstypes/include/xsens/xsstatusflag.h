/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSTATUSFLAG_H
#define XSSTATUSFLAG_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Status flags
	\details These flags define the function of specific bits in the status returned by
	XsDataPacket::status()
	\sa XsDataPacket::status()
*/

enum XsStatusFlag {
	 XSF_SelfTestOk			= 0x01		//!< Is set when the self test result was ok
	,XSF_OrientationValid	= 0x02		//!< Is set when the computed orientation is valid. The orientation may be invalid during startup or when the sensor data is clipping during violent (for the device) motion
	,XSF_GpsValid			= 0x04		//!< Is set when the device has a GPS receiver and the receiver says that there is a GPS position fix.

	,XSF_NoRotationMask				= 0x18		//!< If all of these flags are set, the No Rotation algorithm is running
	,XSF_NoRotationAborted			= 0x10		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm was aborted
	,XSF_NoRotationSamplesRejected	= 0x08		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running but has rejected samples
	,XSF_NoRotationRunningNormally	= 0x18		//!< If all these flags are set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running normally

	,XSF_ClipAccX	= 0x00000100
	,XSF_ClipAccY	= 0x00000200
	,XSF_ClipAccZ	= 0x00000400
	,XSF_ClipGyrX	= 0x00000800
	,XSF_ClipGyrY	= 0x00001000
	,XSF_ClipGyrZ	= 0x00002000
	,XSF_ClipMagX	= 0x00004000
	,XSF_ClipMagY	= 0x00008000
	,XSF_ClipMagZ	= 0x00010000

	,XSF_SyncIn		= 0x00200000		//!< When set indicates a sync-in event has been triggered
	,XSF_SyncOut	= 0x00400000		//!< When set Indicates a sync-out event has been generated

	,XSF_FilterMode	= 0x03800000		//!< Mask for the 3 bit filter mode field
};

/*! \brief Status flag bit offsets
	\details Sometimes (rarely) it is necessary to know the bit offset instead of the bit mask (ie when
	shifting to only keep a subset of flags) for the status flags. This enumeration provides these
	offsets.
	\sa XsStatusFlag
*/
enum XsStatusFlagOffset {
	 XSFO_OffsetSelfTestOk			= 0
	,XSFO_OffsetOrientationValid	= 1
	,XSFO_OffsetGpsValid			= 2
	,XSFO_OffsetNoRotation			= 3

	,XSFO_OffsetClipAccX			= 8
	,XSFO_OffsetClipAccY			= 9
	,XSFO_OffsetClipAccZ			= 10
	,XSFO_OffsetClipGyrX			= 11
	,XSFO_OffsetClipGyrY			= 12
	,XSFO_OffsetClipGyrZ			= 13
	,XSFO_OffsetClipMagX			= 14
	,XSFO_OffsetClipMagY			= 15
	,XSFO_OffsetClipMagZ			= 16
	
	,XSFO_SyncIn					= 21
	,XSFO_SyncOut					= 22

	,XSFO_FilterMode				= 23	// bits 23 -> 23 + XSFO_FilterModeNrOfBits - 1
	,XSFO_FilterModeNrOfBits		= 3		// note: bit 26 is reserved for future use		
};

/*! @} */
typedef enum XsStatusFlag XsStatusFlag;
typedef enum XsStatusFlagOffset XsStatusFlagOffset;

#endif // file guard

//
//#define XS_STATUSFLAG_SELFTEST_OK					0x01
//#define XSF_OrientationValid						0x02
//#define XS_STATUSFLAG_GPSVALID						0x04
//#define XSF_NoRotationMask				0x18
//#define XSF_NoRotationRunningNormally 					0x18
//#define XSF_NoRotationAborted 			0x10
//#define XSF_NoRotationSamplesRejected 	0x08

//#define XS_STATUSFLAG_CLIP_ACC_X		0x000100
//#define XS_STATUSFLAG_CLIP_ACC_Y		0x000200
//#define XS_STATUSFLAG_CLIP_ACC_Z		0x000400
//#define XS_STATUSFLAG_CLIP_GYR_X		0x000800
//#define XS_STATUSFLAG_CLIP_GYR_Y		0x001000
//#define XS_STATUSFLAG_CLIP_GYR_Z		0x002000
//#define XS_STATUSFLAG_CLIP_MAG_X		0x004000
//#define XS_STATUSFLAG_CLIP_MAG_Y		0x008000
//#define XS_STATUSFLAG_CLIP_MAG_Z		0x010000
//#define XS_STATUSFLAG_CLIP_ACC_OFFSET	8
