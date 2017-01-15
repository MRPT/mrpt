/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
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
XSTYPES_DLL_API void XsTime_udelay(uint32_t us);
XSTYPES_DLL_API int64_t XsTime_timeStampNow(XsTimeStamp* now);
XSTYPES_DLL_API void XsTime_initializeTime(void);

#ifdef __cplusplus
} // extern "C"

namespace XsTime {
	/*! \brief The number of seconds in a day */
	static const XsTimeStamp& secPerDay = XsTime_secPerDay;
	/*! \brief The number of milliseconds in a day */
	static const XsTimeStamp& milliSecPerDay = XsTime_milliSecPerDay;
	/*! \brief The maximum possible timestamp  */
	static const XsTimeStamp& timeStampMax = XsTime_timeStampMax;

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
	inline void msleep(uint32_t ms)
	{
		XsTime_msleep(ms);
	}
	
	//! \copydoc XsTime_udelay
	inline void udelay(uint32_t us)
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

#endif	// file guard
