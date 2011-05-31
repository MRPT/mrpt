/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
// Note, this function requires compiler option "-lrt" to be set when compiling with gcc

#include "xsens_time.h"
#include <sys/timeb.h>

#ifdef WIN32
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
#ifdef WIN32
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
