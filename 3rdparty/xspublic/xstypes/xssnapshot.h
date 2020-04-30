
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

#ifndef XSSNAPSHOT_H
#define XSSNAPSHOT_H

#include "xstypesconfig.h"
#include "xsdeviceid.h"

struct XsSnapshot;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSSNAPSHOT_INITIALIZER { XSDEVICEID_INITIALIZER, 0, 0, 0,0,0,0, 0,0,0, 0,0,0, 0, 0, 0, 0 }
#endif

XSTYPES_DLL_API void XsSnapshot_construct(struct XsSnapshot* thisPtr);
XSTYPES_DLL_API void XsSnapshot_destruct(struct XsSnapshot* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

enum SnapshotType
{
	ST_Awinda = 0, //
	ST_Full
};
typedef enum SnapshotType SnapshotType;
/*! \brief A container for Snapshot data
*/
struct XsSnapshot
{
	XsDeviceId m_deviceId;	/*!< \brief The ID of the device that created the data */
	uint32_t m_frameNumber;		/*!< \brief The frame */
	uint64_t m_timestamp;	/*!< \brief The timestamp */
	int32_t m_iQ[4];		/*!< \brief The integrated orientation */
	int64_t m_iV[3];		/*!< \brief The integrated velocity */
	int32_t m_mag[3];		/*!< \brief The magnetic field */
	int32_t m_baro;			/*!< \brief The barometric pressure */
	uint16_t m_status;		/*!< \brief The clipping flags of the latest interval  */
	uint8_t m_accClippingCounter;	/*!< \brief The clipping event counter for the Acc */
	uint8_t m_gyrClippingCounter;	/*!< \brief The clipping event counter for the Gyr */
	SnapshotType m_type; /*!< \brief The type of the snapshot (Awinda, Full) */
#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsSnapshot& other) const
	{
		if (m_frameNumber != other.m_frameNumber ||
			m_baro != other.m_baro ||
			m_status != other.m_status ||
			m_accClippingCounter != other.m_accClippingCounter ||
			m_gyrClippingCounter != other.m_gyrClippingCounter||
			m_type != other.m_type)
			return false;

		for (int i = 0; i < 3; ++i)
		{
			if (m_iQ[i] != other.m_iQ[i] ||
				m_iV[i] != other.m_iV[i] ||
				m_mag[i] != other.m_mag[i])
				return false;
		}
		if (m_type == ST_Full)
		{
			if (m_iQ[3] != other.m_iQ[3] || m_timestamp != other.m_timestamp  )
				return false;
		}
		return true;
	}
#endif
};
typedef struct XsSnapshot XsSnapshot;

/*! \brief Status flag definitions for XsSnapshot status field */
enum SnapshotStatusFlag
{
	FSFL_ClipAccX		= 0x0001,
	FSFL_ClipAccY		= 0x0002,
	FSFL_ClipAccZ		= 0x0004,
	FSFL_ClipAccMask	= 0x0007,
	FSFL_ClipGyrX		= 0x0008,
	FSFL_ClipGyrY		= 0x0010,
	FSFL_ClipGyrZ		= 0x0020,
	FSFL_ClipGyrMask	= 0x0038,
	FSFL_ClipMagX		= 0x0040,
	FSFL_ClipMagY		= 0x0080,
	FSFL_ClipMagZ		= 0x0100,
	FSFL_ClipMagMask	= 0x01C0,
	FSFL_MagIsNew		= 0x0200,
	FSFL_BaroIsNew		= 0x0400,
	FSFL_RotationMask	= 0x1800
};
typedef enum SnapshotStatusFlag SnapshotStatusFlag;



#define FSFL_ClipAccShift			0
#define FSFL_ClipGyrShift			04
#define FSFL_ClipMagshift			6
#define FSFL_RotationShift		11

#endif
