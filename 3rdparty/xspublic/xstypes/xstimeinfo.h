
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

#ifndef XSTIMEINFO_H
#define XSTIMEINFO_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
#ifndef __cplusplus
#define XSTIMEINFO_INITIALIZER	{ 0, 0 ,0, 0, 0, 0, 0, 0, 0}
#endif
struct XsTimeInfo;

XSTYPES_DLL_API void XsTimeInfo_currentTime(struct XsTimeInfo * thisPtr);
XSTYPES_DLL_API void XsTimeInfo_currentLocalTime(struct XsTimeInfo * thisPtr);
XSTYPES_DLL_API void XsTimeInfo_makeUtc(struct XsTimeInfo * thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief A structure for storing Time values. */
struct XsTimeInfo {
	uint32_t	m_nano;			//!< \brief Nanosecond part of the time
	uint16_t	m_year;			//!< \brief The year (if date is valid)
	uint8_t		m_month;		//!< \brief The month (if date is valid)
	uint8_t		m_day;			//!< \brief The day of the month (if date is valid)
	uint8_t		m_hour;			//!< \brief The hour (if time is valid)
	uint8_t		m_minute;		//!< \brief The minute (if time is valid)
	uint8_t		m_second;		//!< \brief The second (if time is valid)
	uint8_t		m_valid;		//!< \brief Validity indicator \details When received: bit (0) - UTC Date is valid; bit (1) - UTC Time of Day is valid; bit (2) - UTC Time of Day has been fully resolved (i.e. no seconds uncertainty).
	int16_t		m_utcOffset;	//!< \brief Offset to UTC time in minutes. This value can be added to the stored time to get UTC time.

#ifdef __cplusplus
	/*! \brief Sets all values to 0 */
	void clear()
	{
		memset(this, 0, sizeof(XsTimeInfo));
	}

	/*! \copydoc XsTimeInfo_currentTime
		\return The current UTC Time
	*/
	inline static XsTimeInfo currentTime()
	{
		XsTimeInfo tmp;
		XsTimeInfo_currentTime(&tmp);
		return tmp;
	}

	/*! \copydoc XsTimeInfo_currentLocalTime
		\return The current local Time
	*/
	inline static XsTimeInfo currentLocalTime()
	{
		XsTimeInfo tmp;
		XsTimeInfo_currentLocalTime(&tmp);
		return tmp;
	}

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsTimeInfo& other) const
	{
		return	m_nano		== other.m_nano &&
				m_year		== other.m_year &&
				m_month		== other.m_month &&
				m_day		== other.m_day &&
				m_hour		== other.m_hour &&
				m_minute	== other.m_minute &&
				m_second	== other.m_second &&
				m_valid		== other.m_valid &&
				m_utcOffset	== other.m_utcOffset;
	}

	/*! \brief Removes the local time information, making the object pure UTC */
	void makeUtc()
	{
		if (m_utcOffset)
			XsTimeInfo_makeUtc(this);
	}
#endif
};
typedef struct XsTimeInfo XsTimeInfo;

#endif
