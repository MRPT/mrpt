
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

#ifndef XSGLOVEDATA_H
#define XSGLOVEDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"
#include "xsquaternion.h"

struct XsGloveData;
struct XsFingerData;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSFINGERDATA_INITIALIZER {XSQUATERNION_INITIALIZER, XSVECTOR3_INITIALIZER, XSVECTOR3_INITIALIZER, 0, 0, 0}
#define XSGLOVEDATA_INITIALIZER {0, 0, 0, 0, \
									XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, \
									XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER, XSFINGERDATA_INITIALIZER }
#endif

XSTYPES_DLL_API void XsFingerData_construct(struct XsFingerData* thisPtr);
XSTYPES_DLL_API void XsFingerData_destruct(struct XsFingerData* thisPtr);

XSTYPES_DLL_API void XsGloveData_construct(struct XsGloveData* thisPtr);
XSTYPES_DLL_API void XsGloveData_destruct(struct XsGloveData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief A container for Finger data
*/
struct XsFingerData
{
#ifdef __cplusplus
	//! \brief Construct an empty object
	inline XsFingerData()
		: m_flags(0)
		, m_ccAcc(0)
		, m_ccGyr(0)
	{
	}

	//! \brief Construct an initialized object
	inline XsFingerData(const XsQuaternion& dq, const XsVector& dv, const XsVector& mag, const uint16_t flags, const uint8_t ccAcc, const uint8_t ccGyr)
		: m_orientationIncrement(dq)
		, m_velocityIncrement(dv)
		, m_mag(mag)
		, m_flags(flags)
		, m_ccAcc(ccAcc)
		, m_ccGyr(ccGyr)
	{
	}

	//! \brief Copy constructor
	inline XsFingerData(const XsFingerData& other)
		: m_orientationIncrement(other.m_orientationIncrement)
		, m_velocityIncrement(other.m_velocityIncrement)
		, m_mag(other.m_mag)
		, m_flags(other.m_flags)
		, m_ccAcc(other.m_ccAcc)
		, m_ccGyr(other.m_ccGyr)
	{
	}

	//! \brief Assignment operator
	inline const XsFingerData& operator=(const XsFingerData& other)
	{
		if (this != &other)
		{
			m_orientationIncrement = other.m_orientationIncrement;
			m_velocityIncrement = other.m_velocityIncrement;
			m_mag = other.m_mag;
			m_flags = other.m_flags;
			m_ccAcc = other.m_ccAcc;
			m_ccGyr = other.m_ccGyr;
		}
		return *this;
	}

	//! \brief Clear the object so it contains unity data
	inline void zero()
	{
		m_orientationIncrement = XsQuaternion::identity();
		m_velocityIncrement.zero();
		m_mag.zero();
	}

	//! \brief Returns the contained orientation increment
	inline const XsQuaternion& orientationIncrement() const
	{
		return m_orientationIncrement;
	}

	//! \brief Update the contained orientation increment
	inline void setOrientationIncrement(const XsQuaternion& dq)
	{
		m_orientationIncrement = dq;
	}

	//! \brief Returns the contained velocity increment
	inline const XsVector3& velocityIncrement() const
	{
		return m_velocityIncrement;
	}

	//! \brief Update the contained velocity increment
	inline void setVelocityIncrement(const XsVector& dv)
	{
		m_velocityIncrement = dv;
	}
	//! \brief Returns the mag
	inline const XsVector3& mag() const
	{
		return m_mag;
	}

	//! \brief Update the mag
	inline void setMag(const XsVector& mag)
	{
		m_mag = mag;
	}

	//! \brief Returns the flags
	inline const uint16_t& flags() const
	{
		return m_flags;
	}

	//! \brief Update the flags
	inline void setFlags(const uint16_t& flags)
	{
		m_flags = flags;
	}

	//! \brief Returns the acc clip
	inline const uint8_t& ccAcc() const
	{
		return m_ccAcc;
	}

	//! \brief Update the acc clip
	inline void setCcAcc(const uint8_t& ccAcc)
	{
		m_ccAcc = ccAcc;
	}

	//! \brief Returns the gyr clip
	inline const uint8_t& ccGyr() const
	{
		return m_ccGyr;
	}

	//! \brief Update the gyr clip
	inline void setCcGyr(const uint8_t& ccGyr)
	{
		m_ccGyr = ccGyr;
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsFingerData& other) const
	{
		return	m_orientationIncrement == other.m_orientationIncrement &&
			m_velocityIncrement == other.m_velocityIncrement &&
			m_mag == other.m_mag &&
			m_flags == other.m_flags &&
			m_ccAcc == other.m_ccAcc &&
			m_ccGyr == other.m_ccGyr;
	}

private:
#endif

	XsQuaternion m_orientationIncrement;	//!< The orientation increment
	XsVector3    m_velocityIncrement;		//!< The velocity increment
	XsVector3	 m_mag;
	uint16_t	 m_flags;
	uint8_t		 m_ccAcc;
	uint8_t		 m_ccGyr;
};
typedef struct XsFingerData XsFingerData;

/*! \brief A container for glove data
*/
struct XsGloveData
{
#ifdef __cplusplus
	//! \brief Construct an empty object
	inline XsGloveData()
		: m_snapshotCounter(0)
		, m_validSampleFlags(0)
		, m_timestamp(0)
		, m_carpusOffset(0)
	{
	}

	//! \brief Construct an initialized object
	inline XsGloveData(const uint16_t& snapshotCounter,	const uint16_t& validSampleFlags, const uint16_t& timestamp, const uint8_t& carpusOffset, const XsFingerData *fingerData)
		: m_snapshotCounter(snapshotCounter)
		, m_validSampleFlags(validSampleFlags)
		, m_timestamp(timestamp)
		, m_carpusOffset(carpusOffset)
	{
		setFingerData(fingerData);
	}

	//! \brief Copy constructor
	inline XsGloveData(const XsGloveData& other)
		: m_snapshotCounter(other.m_snapshotCounter)
		, m_validSampleFlags(other.m_validSampleFlags)
		, m_timestamp(other.m_timestamp)
		, m_carpusOffset(other.m_carpusOffset)
	{
		setFingerData(other.m_fingerData);
	}

	//! \brief Returns the snapshot counter
	inline const uint16_t& snapshotCounter() const
	{
		return m_snapshotCounter;
	}

	//! \brief Update the snapshot counter
	inline void setSnapshotCounter(const uint16_t& snapshotCounter)
	{
		m_snapshotCounter = snapshotCounter;
	}

	//! \brief Returns the valid sample flags
	inline const uint16_t& validSampleFlags() const
	{
		return m_validSampleFlags;
	}

	//! \brief Update the valid sample flags
	inline void setValidSampleFlags(const uint16_t& validSampleFlags)
	{
		m_validSampleFlags = validSampleFlags;
	}

	//! \brief Returns the timestamp
	inline const uint16_t& timestamp() const
	{
		return m_timestamp;
	}

	//! \brief Update the timestamp
	inline void setTimestamp(const uint16_t& timestamp)
	{
		m_timestamp = timestamp;
	}

	//! \brief Returns the carpus offset
	inline const uint8_t& carpusOffset() const
	{
		return m_carpusOffset;
	}

	//! \brief Update the carpus offset
	inline void setCarpusOffset(const uint8_t& carpusOffset)
	{
		m_carpusOffset = carpusOffset;
	}

	//! \brief Returns the finger data
	inline void fingerData(XsFingerData *fingerData) const
	{
		for (int i = 0; i < 12; i++)
			fingerData[i] = m_fingerData[i];
	}

	//! \brief Update the finger data
	inline void setFingerData(const XsFingerData *fingerData)
	{
		for (int i = 0; i < 12; i++)
			m_fingerData[i] = fingerData[i];
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsGloveData& other) const
	{
		if (m_snapshotCounter != other.m_snapshotCounter ||
			m_validSampleFlags != other.m_validSampleFlags ||
			m_timestamp != other.m_timestamp ||
			m_carpusOffset != other.m_carpusOffset)
			return false;

		for (int i = 0; i < 12; i++)
		{
			if (!(m_fingerData[i] == other.m_fingerData[i]))
				return false;
		}
		return true;
	}

private:
#endif
	uint16_t m_snapshotCounter;
	uint16_t m_validSampleFlags;
	uint16_t m_timestamp; /*!< \brief The timestamp */
	uint8_t m_carpusOffset;
	XsFingerData m_fingerData[12];
};

typedef struct XsGloveData XsGloveData;

#endif	// file guard
