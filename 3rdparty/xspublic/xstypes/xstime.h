
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

#ifndef XSTIME_H
#define XSTIME_H

#include "xstypesconfig.h"
#ifdef _WIN32
#include <windows.h>
#endif

#include <time.h>
#include "pstdint.h"
#include "xstimestamp.h"

#ifdef __cplusplus
#include "xsstring.h"
extern "C" {
#endif

XSTYPES_DLL_API extern const XsTimeStamp XsTime_secPerDay;
XSTYPES_DLL_API extern const XsTimeStamp XsTime_milliSecPerDay;
XSTYPES_DLL_API extern const XsTimeStamp XsTime_timeStampMax;

XSTYPES_DLL_API uint32_t XsTime_getTimeOfDay(struct tm* date_, time_t* secs_);
XSTYPES_DLL_API int64_t XsTime_getDateTime(struct tm * date);
XSTYPES_DLL_API void XsTime_getDateAsString(char* dest, struct tm const* date);
XSTYPES_DLL_API void XsTime_getTimeAsString(char* dest, struct tm const* time);
XSTYPES_DLL_API void XsTime_getDateAsWString(wchar_t* dest, struct tm const* date);
XSTYPES_DLL_API void XsTime_getTimeAsWString(wchar_t* dest, struct tm const* time);
XSTYPES_DLL_API void XsTime_msleep(uint32_t ms);
XSTYPES_DLL_API void XsTime_udelay(uint64_t us);
XSTYPES_DLL_API int64_t XsTime_timeStampNow(XsTimeStamp* now);
XSTYPES_DLL_API void XsTime_initializeTime(void);
XSTYPES_DLL_API int64_t XsTime_utcToLocal();
XSTYPES_DLL_API int64_t XsTime_localToUtc();

#ifdef __cplusplus
} // extern "C"

namespace XsTime {
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
	/*! \brief The number of seconds in a day */
	static const XsTimeStamp& secPerDay = XsTime_secPerDay;
	/*! \brief The number of milliseconds in a day */
	static const XsTimeStamp& milliSecPerDay = XsTime_milliSecPerDay;
	/*! \brief The maximum possible timestamp  */
	static const XsTimeStamp& timeStampMax = XsTime_timeStampMax;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

	//! \copydoc XsTime_getTimeOfDay
	inline uint32_t getTimeOfDay(tm* date_ = NULL, time_t* secs_ = NULL)
	{
		return XsTime_getTimeOfDay(date_ , secs_);
	}

	//! \copydoc XsTime_getDateTime
	inline int64_t getDateTime(tm * date = 0)
	{
		return XsTime_getDateTime(date);
	}

	//! \copydoc XsTime_getDateAsString
	inline void getDateAsString(char* dest, tm const* date = 0)
	{
		XsTime_getDateAsString(dest, date);
	}

	//! \copydoc XsTime_getTimeAsString
	inline void getTimeAsString(char* dest, tm const* date = 0)
	{
		XsTime_getTimeAsString(dest, date);
	}

	/*! \brief Returns the date as a readable string
		\param date to convert to string
		\returns The date as a readable string
		\sa XsTime_getDateAsWString
	*/
	inline XsString getDateAsString(tm const* date = 0)
	{
		wchar_t wcharBuf[9];
		XsTime_getDateAsWString(wcharBuf, date);
		wcharBuf[8] = 0;
		return XsString(wcharBuf);
	}

	/*! \brief Returns the time as a readable string
		\param time to convert to string
		\returns The time as a readable string
		\sa XsTime_getTimeAsWString
	*/
	inline XsString getTimeAsString(tm const* time = 0)
	{
		wchar_t wcharBuf[9];
		XsTime_getTimeAsWString(wcharBuf, time);
		wcharBuf[8] = 0;
		return XsString(wcharBuf);
	}

	//! \copydoc XsTime_msleep
	inline void msleep(uint32_t ms) noexcept
	{
		XsTime_msleep(ms);
	}

	//! \copydoc XsTime_udelay
	inline void udelay(uint64_t us) noexcept
	{
		XsTime_udelay(us);
	}

	//! \copydoc XsTime_initializeTime
	inline void initializeTime()
	{
		XsTime_initializeTime();
	}

	//! \copydoc XsTime_timeStampNow
	inline int64_t timeStampNow(XsTimeStamp* now = 0)
	{
		return XsTime_timeStampNow(now);
	}
}
#endif

#endif
