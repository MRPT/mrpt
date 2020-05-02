
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSSTATUSFLAG_H
#define XSSTATUSFLAG_H

#include "pstdint.h"

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Status flags
	\details These flags define the function of specific bits in the status returned by
	XsDataPacket::status()
	\sa XsDataPacket::status()
*/

enum XsStatusFlag
{
	 XSF_SelfTestOk					= 0x01		//!< Is set when the self test result was ok
	,XSF_OrientationValid			= 0x02		//!< Is set when the computed orientation is valid. The orientation may be invalid during startup or when the sensor data is clipping during violent (for the device) motion
	,XSF_GpsValid					= 0x04		//!< Is set when the device has a GPS receiver and the receiver says that there is a GPS position fix.

	,XSF_NoRotationMask				= 0x18		//!< If all of these flags are set, the No Rotation algorithm is running
	,XSF_NoRotationAborted			= 0x10		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm was aborted
	,XSF_NoRotationSamplesRejected	= 0x08		//!< If only this flag is set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running but has rejected samples
	,XSF_NoRotationRunningNormally	= 0x18		//!< If all these flags are set (out of the XSF_NoRotationMask) then the No Rotation algorithm is running normally

	,XSF_RepresentativeMotion		= 0x20		//!< Indicates if the In-Run Compass Calibration is doing the representative motion analysis

	,XSF_ExternalClockSynced		= 0x40		//!< Indicates whether the internal clock is synced with an external clock (Either GNNS or custom provided clock sync)

	,XSF_ClipAccX					= 0x00000100
	,XSF_ClipAccY					= 0x00000200
	,XSF_ClipAccZ					= 0x00000400
	,XSF_ClipGyrX					= 0x00000800
	,XSF_ClipGyrY					= 0x00001000
	,XSF_ClipGyrZ					= 0x00002000
	,XSF_ClipMagX					= 0x00004000
	,XSF_ClipMagY					= 0x00008000
	,XSF_ClipMagZ					= 0x00010000

	,XSF_Retransmitted				= 0x00040000	//!< When set Indicates the sample was received as a retransmission
	,XSF_ClippingDetected			= 0x00080000	//!< When set Indicates clipping has occurred
	,XSF_Interpolated				= 0x00100000	//!< When set Indicates the sample is an interpolation between other samples
	,XSF_SyncIn						= 0x00200000	//!< When set indicates a sync-in event has been triggered
	,XSF_SyncOut					= 0x00400000	//!< When set Indicates a sync-out event has been generated


	,XSF_FilterMode					= 0x03800000	//!< Mask for the 3 bit filter mode field
	,XSF_HaveGnssTimePulse			= 0x04000000	//!< Indicates that the 1PPS GNSS time pulse is present
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

	,XSFO_Retransmitted				= 19
	,XSFO_Interpolated				= 20
	,XSFO_SyncIn					= 21
	,XSFO_SyncOut					= 22

	,XSFO_FilterMode				= 23	// bits 23 -> 23 + XSFO_FilterModeNrOfBits - 1
	,XSFO_FilterModeNrOfBits		= 3		// note: bit 26 is reserved for future use
};

/*! @} */
typedef enum XsStatusFlag XsStatusFlag;
typedef enum XsStatusFlagOffset XsStatusFlagOffset;

//! Return if any acc channel clipped
inline static bool anyAccClipped(int status)
{
	return 0 != (status & (XSF_ClipAccX | XSF_ClipAccY | XSF_ClipAccZ));
}

//! Return if any gyr channel clipped
inline static bool anyGyrClipped(int status)
{
	return 0 != (status & (XSF_ClipGyrX | XSF_ClipGyrY | XSF_ClipGyrZ));
}

//! Return if any mag channel clipped
inline static bool anyMagClipped(int status)
{
	return 0 != (status & (XSF_ClipMagX | XSF_ClipMagY | XSF_ClipMagZ));
}


//! \brief Status object.
class XsStatus
{
public:
	/*! \brief Status object constructor
		\param s XsStatus object to copy from
	*/
	inline XsStatus(const XsStatus& s) : m_status(s.m_status)
	{
	}

	/*! \brief Status object constructor
		\param status status flags to set in the object
	*/
	inline XsStatus(int status) : m_status((uint32_t)status)
	{
	}

	//! \brief Status object constructor, clears all flags
	inline XsStatus() : m_status(0)
	{
	}

	//! \brief Return statusflag of status object
	inline XsStatusFlag get() const
	{
		return (XsStatusFlag) m_status;
	}

	/*! \brief Set statusflag of status object
		\param a The Status object to copy from
	*/
	inline void set(XsStatus const& a)
	{
		m_status = a.m_status;
	}

	/*! \brief Set statusflag of status object
		\param a The full status flags to set
	*/
	inline void set(uint32_t a)
	{
		m_status = a;
	}

	/*! \brief Set statusflag of status object
		\param a The full status flags to set
	*/
	inline void set(int a)
	{
		m_status = (uint32_t)(a & 0x1FFFF);
	}

	/*! \brief Assignment operator
		\param a The Status object to copy from
		\return A reference to this object
	*/
	inline XsStatus const& operator = (XsStatus const& a)
	{
		m_status = a.m_status; return *this;
	}

	/*! \brief Assignment operator
		\param a The full status flags to set
		\return A reference to this object
	*/
	inline XsStatus const& operator = (int a)
	{
		m_status = (uint32_t)(a & 0x1FFFF);
		return *this;
	}

	/*! \brief Binary OR operator
		\param a The status flags to set, leaving already set flags as they were
		\return A reference to this object
	*/
	inline XsStatus const& operator |= (XsStatus const& a)
	{
		m_status |= a.m_status;
		return *this;
	}

	/*! \brief Binary OR operator
		\param a The status flags to set, leaving already set flags as they were
		\return A reference to this object
	*/
	inline XsStatus const& operator |= (int a)
	{
		m_status |= (uint32_t)(a & 0x1FFFF);
		return *this;
	}

	/*! \brief Binary AND operator
		\param a The mask of flags to keep, clearing other flags
		\return A reference to this object
	*/
	inline XsStatus const& operator &= (XsStatus const& a)
	{
		m_status &= a.m_status;
		return *this;
	}

	/*! \brief Binary AND operator
		\param a The mask of flags to keep, clearing other flags
		\return A reference to this object
	*/
	inline XsStatus const& operator &= (int a)
	{
		m_status &= (uint32_t)(a & 0x1FFFF);
		return *this;
	}

	//! Return if any acc channel clipped
	inline bool anyAccClipped() const
	{
		return ::anyAccClipped((int)m_status);
	}

	//! Return if any gyr channel clipped
	inline bool anyGyrClipped() const
	{
		return ::anyGyrClipped((int)m_status);
	}

	//! Return if any mag channel clipped
	inline bool anyMagClipped() const
	{
		return ::anyMagClipped((int)m_status);
	}

	//! Clear status object
	inline void clear()
	{
		m_status = 0;
	}

protected:
	uint32_t m_status;	//!< Statusflag
};

#endif
