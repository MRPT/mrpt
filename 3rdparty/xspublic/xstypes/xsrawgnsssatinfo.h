
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

#ifndef XSRAWGNSSSATINFO_H
#define XSRAWGNSSSATINFO_H

#include "pstdint.h"

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

#ifndef __cplusplus
#define XSSATINFO_INITIALIZER	{ 0, 0, 0, 0 }
#define XSRAWGNSSSATINFO_INITIALIZER { 0, 0, 0, 0, 0,  \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, \
										XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER, XSSATINFO_INITIALIZER }
#endif

/*! \addtogroup enums Global enumerations
   @{
*/
/*! \brief Xsens Satellite Info Flags
*/
enum XsSatInfoFlags
{
	XSIF_SignalQualityIndicator_Mask				= 0x7,
	XSIF_SignalQualityIndicator_NoSignal			= 0x0,
	XSIF_SignalQualityIndicator_Searching			= 0x1,
	XSIF_SignalQualityIndicator_Acquired			= 0x2,
	XSIF_SignalQualityIndicator_Unusable			= 0x3,
	XSIF_SignalQualityIndicator_CodeTimeOk			= 0x4,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk1	= 0x5,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk2	= 0x6,
	XSIF_SignalQualityIndicator_CodeCarrierTimeOk3	= 0x7,
	XSIF_UsedForNavigation_Mask						= 0x8,
	XSIF_UsedForNavigation_Used						= 0x8,
	XSIF_HealthFlag_Mask							= 0x30,
	XSIF_HealthFlag_Unknown							= 0x00,
	XSIF_HealthFlag_Healthy							= 0x10,
	XSIF_HealthFlag_Unhealthy						= 0x20,
	XSIF_Differential_Mask							= 0x40,
	XSIF_Differential_Available						= 0x40
};
/*! @} */
typedef enum XsSatInfoFlags XsSatInfoFlags;

//! \brief A container for information of one GNSS satellite
struct XsSatInfo
{
		uint8_t		m_gnssId;	//!< GNSS identifier
		uint8_t		m_svId;		//!< Satellite identifier
		uint8_t		m_cno;		//!< Carrier to noise ratio (signals strength)
		uint8_t		m_flags;	/*!<
									bits[0..2] : Signal quality indicator
										0 = No signal
										1 = Searching signal
										2 = Signal acquired
										3 = Signal detected but unusable
										4 = Code locked and time synchronized
										5, 6, 7 = Code and carrier locked and time synchronized
									bits[3] : Is set to 1 when the SV is being used for navigation
									bits[4..5] : SV health flag
										0 = unknown
										1 = healthy
										2 = unhealthy
									bits[6] : Is set to 1 when differential correction data is available
									bits[7] : reserved
									*/
};
typedef struct XsSatInfo XsSatInfo;

/*! \brief A container for GNSS Satellite Information
*/
struct XsRawGnssSatInfo
{
	uint32_t	m_itow;			//!< GNSS time of week (ms)
	uint8_t		m_numSvs;		//!< Number of satellites
	uint8_t		m_res1;			//!< Reserved for future use (1)
	uint8_t		m_res2;			//!< Reserved for future use (2)
	uint8_t		m_res3;			//!< Reserved for future use (3)

	XsSatInfo m_satInfos[60]; //!< The information of all satellites, maximum 60

#ifdef __cplusplus
	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsRawGnssSatInfo& b) const
	{
		if (m_itow		!= b.m_itow ||
			m_numSvs	!= b.m_numSvs ||
			m_res1		!= b.m_res1 ||
			m_res2		!= b.m_res2 ||
			m_res3		!= b.m_res3)
			return false;
		for (uint8_t i = 0; i < m_numSvs; ++i)
			if (m_satInfos[i].m_gnssId	!= b.m_satInfos[i].m_gnssId ||
				m_satInfos[i].m_svId	!= b.m_satInfos[i].m_svId ||
				m_satInfos[i].m_cno		!= b.m_satInfos[i].m_cno ||
				m_satInfos[i].m_flags	!= b.m_satInfos[i].m_flags)
				return false;
		return true;
	}
#endif
};
typedef struct XsRawGnssSatInfo XsRawGnssSatInfo;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif
