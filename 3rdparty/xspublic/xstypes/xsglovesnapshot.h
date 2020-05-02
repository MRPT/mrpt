
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

#ifndef XSGLOVESNAPSHOT_H
#define XSGLOVESNAPSHOT_H

#include "xstypesconfig.h"
#include "xsdeviceid.h"

struct XsGloveSnapshot;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSGLOVESNAPSHOT_INITIALIZER {0, 0, 0, 0, \
										XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, \
										XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER, XSFINGERSNAPSHOT_INITIALIZER}
#define XSFINGERSNAPSHOT_INITIALIZER {0,0,0, 0,0,0, 0,0,0, 0, 0, 0}
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef _MSC_VER
#pragma pack(push, 1)
#ifndef	PACK_POST
#define PACK_POST
#endif
#else
#ifndef	PACK_POST
#define PACK_POST __attribute__((__packed__))
#endif
#endif

#include <math.h>       /* pow */

/*! \brief int24
*/
struct int24_t {
	uint8_t m_vals[3]; /*!< \brief The data*/

#ifdef __cplusplus
	//! \brief convert int24 to double
	inline double toDouble() const
	{
		if (m_vals[2] & 0x80)
			return ((double)(int32_t)(((uint32_t)0xff000000) | (((uint32_t)m_vals[2]) << 16) | (((uint32_t)m_vals[1]) << 8) | ((uint32_t)m_vals[0])));
		else
			return (double)((((uint32_t)m_vals[2]) << 16) | (((uint32_t)m_vals[1]) << 8) | (uint32_t)m_vals[0]);
	}

#endif
#ifdef SWIG
};
#else
} PACK_POST;
#endif
typedef struct int24_t int24_t;

/*! \brief A container for Finger Snapshot data
*/
struct XsFingerSnapshot {
	int24_t m_iQ[3];		/*!< \brief The integrated orientation */
	int32_t m_iV[3];		/*!< \brief The integrated velocity */
	int16_t m_mag[3];		/*!< \brief The magnetic field */
	uint16_t m_flags;		/*!< \brief The flags */
	uint8_t m_ccacc;		/*!< \brief The acceleration cliping */
	uint8_t m_ccgyr;		/*!< \brief The gyroscope clipping */
#ifdef SWIG
};
#else
} PACK_POST;
#endif
typedef struct XsFingerSnapshot XsFingerSnapshot;

/*! \brief A container for Glove Snapshot data
*/
struct XsGloveSnapshot
{
	uint16_t m_snapshotCounter;		/*!< \brief The snapshot counter */
	uint16_t m_validSampleFlags;	/*!< \brief The valid sample flags */
	uint16_t m_timestamp;			/*!< \brief The timestamp */
	uint8_t m_carpusOffset;			/*!< \brief The carpus offset */
	XsFingerSnapshot m_fingers[12];	/*!< \brief The 12 fingers */
#if 0 // def __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsGloveSnapshot& other) const
	{
		return true;
	}
#endif
#ifdef SWIG
};
#else
} PACK_POST;
#endif
typedef struct XsGloveSnapshot XsGloveSnapshot;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif
