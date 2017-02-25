/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// Note, this function requires compiler option "-lrt" to be set when compiling with gcc

#include "xsens_time.h"
#include <sys/timeb.h>

#ifdef _WIN32
#	include <windows.h>
#else
#	include <unistd.h>
#   include <sys/time.h>
#endif

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Other  functions ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// A platform-independent clock.
uint32_t getTimeOfDay(tm* date_, time_t* secs_)
{
#ifdef _WIN32
	_timeb tp;  //__timeb32 tp;

	_ftime(&tp); //_ftime32_s(&tp);

	if (date_ != NULL)
	{
		time_t tin = tp.time; //__time32_t tin = tp.time;
		*date_ = *localtime(&tin); // _localtime32_s(date_,&tin);
	}
	if (secs_ != NULL)
		secs_[0] = tp.time;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (1000 * ((uint32_t) tp.time % XSENS_SEC_PER_DAY)) + tp.millitm;
	
/* Jerome Monceaux : 2011/03/08
 * Add a special case for apple 
 * because librt is not available
 * so clock_gettime as well
 */
#else
# ifdef __APPLE__
	struct timeval tv;
 	timespec  tp;
 
  gettimeofday(&tv, NULL);
  tp.tv_sec = tv.tv_sec;
  tp.tv_nsec = tv.tv_usec*1000;
# else
	timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt
# endif

	if (date_ != NULL)
		localtime_r(&tp.tv_sec,date_);

	if (secs_ != NULL)
		secs_[0] = tp.tv_sec;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (1000 * (tp.tv_sec % XSENS_SEC_PER_DAY)) + (tp.tv_nsec/1000000);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
// A platform-independent sleep routine.
void msleep(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	clock_t end = clock() + (CLOCKS_PER_SEC/1000) * ms;
	clock_t diff;

	while ((diff = end - clock()) > 0)
	{
		diff = (1000 * diff) / CLOCKS_PER_SEC;
		if (diff > 1000)
			sleep(diff / 1000);
		else
			usleep(diff * 1000);
	}
#endif
}

TimeStamp timeStampNow(void)
{
	TimeStamp ms;
	time_t s;
	ms = (TimeStamp) getTimeOfDay(NULL,&s);
	ms = (ms % 1000) + (((TimeStamp)s)*1000);

	return ms;
}

}	// end of xsens namespace
