
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

#ifndef XSTRIGGERINDICATIONDATA_H
#define XSTRIGGERINDICATIONDATA_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSTRIGGERINDICATIONDATA_INITIALIZER	{ 0, 0, 0, 0 }
#endif

struct XsTriggerIndicationData;

XSTYPES_DLL_API void XsTriggerIndicationData_destruct(struct XsTriggerIndicationData* thisPtr);
XSTYPES_DLL_API int XsTriggerIndicationData_valid(const struct XsTriggerIndicationData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Data for a trigger indication message */
struct XsTriggerIndicationData {
	uint8_t m_line;			//!< The line number
	uint8_t m_polarity;		//!< The polarity
	uint32_t m_timestamp;	//!< The timestamp
	uint16_t m_frameNumber;	//!< The frame number

#ifdef __cplusplus
	/*! Constructor
		\param[in] line Line
		\param[in] polarity Polarity
		\param[in] timestamp Timestamp
		\param[in] frameNumber Frame number
	*/
	explicit XsTriggerIndicationData(uint8_t line = 0, uint8_t polarity = 0, uint32_t timestamp = 0, uint16_t frameNumber = 0)
	  : m_line(line), m_polarity(polarity), m_timestamp(timestamp), m_frameNumber(frameNumber)
	{}

	/*! \brief \copybrief XsTriggerIndicationData_destruct */
	inline void clear()
	{
		XsTriggerIndicationData_destruct(this);
	}

	/*! \brief \copybrief XsTriggerIndicationData_valid */
	inline bool valid() const
	{
		return 0 != XsTriggerIndicationData_valid(this);
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (XsTriggerIndicationData const& other) const
	{
		return m_line == other.m_line &&
				m_polarity == other.m_polarity &&
				m_timestamp == other.m_timestamp &&
				m_frameNumber == other.m_frameNumber;
	}
#endif
};

typedef struct XsTriggerIndicationData XsTriggerIndicationData;

#endif
