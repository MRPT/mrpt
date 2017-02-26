/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

#ifndef HAVE_TIMEGM
#   include <mrpt/synch/CCriticalSection.h>
#endif // HAVE_TIMEGM

#include <cstring>
#include <float.h>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <ctime>
#include <cstdio>

#ifdef MRPT_OS_WINDOWS
    #include <conio.h>
	#include <windows.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
    #include <pthread.h>
    #include <termios.h>
	#include <poll.h>
    #include <unistd.h>
    #include <sys/select.h>
    #include <sys/time.h>
    #include <time.h>
	#include <unistd.h>
	#include <utime.h>
	#include <errno.h>
//	#include <signal.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>

#ifdef MRPT_OS_LINUX
	#define _access access
	#define _rmdir rmdir
	#define _stat stat
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

#ifndef MRPT_OS_WINDOWS
    /** By ninjalj in http://stackoverflow.com/questions/3962263/checking-if-a-key-was-pressed
      */
	void my_aux_sighandler(int ) {}

    int myKbhit(void)
    {
		struct termios oldtio, curtio;
//		struct sigaction sa;

		/* Save stdin terminal attributes */
		tcgetattr(0, &oldtio);

//		memset(&sa, 0, sizeof(struct sigaction));

		/* Set non-canonical no-echo for stdin */
		tcgetattr(0, &curtio);
		curtio.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(0, TCSANOW, &curtio);

		struct pollfd pfds[1];

		/* See if there is data available */
		pfds[0].fd = 0;
		pfds[0].events = POLLIN;
		const int ret = poll(pfds, 1, 0);

		/* restore terminal attributes */
		tcsetattr(0, TCSANOW, &oldtio);

		return (ret > 0);
    }

#endif

/*---------------------------------------------------------------
					timegm
  ---------------------------------------------------------------*/
#ifdef HAVE_TIMEGM
	time_t mrpt::system::os::timegm(struct tm *tm)
	{
		return ::timegm(tm);
	}
#else
	// Version for MSVC>=2005, which lacks "timegm"
	#ifdef HAVE_MKGMTIME
		time_t mrpt::system::os::timegm(struct tm *tm)
		{
			return ::_mkgmtime(tm);
		}
	#else
		// generic version, slower but probably not used in any modern compiler!
		time_t mrpt::system::os::timegm(struct tm *tm)
		{
			static mrpt::synch::CCriticalSection cs;
			mrpt::synch::CCriticalSectionLocker locker(&cs);

			time_t ret;
			char tz[256];

			/* save current timezone and set UTC */
			char *org_tz = getenv("TZ");
			if (org_tz) os::strcpy(tz,sizeof(tz),org_tz);

			putenv("TZ=UTC");   /* use Coordinated Universal Time (i.e. zero offset) */
			tzset();

			ret = mktime(tm);
			if(org_tz)
			{
				char buf[256];
				mrpt::system::os::sprintf(buf, sizeof(buf), "TZ=%s", tz);
				putenv(buf);
			} else
				putenv("TZ=");
			tzset();

			return ret;
		}

	#endif
#endif	// HAVE_TIMEGM

/*---------------------------------------------------------------
					mrpt::system::MRPT_getCompilationDate
---------------------------------------------------------------*/
#include <mrpt/version.h>
#include <errno.h>
#include <limits>
#include <climits>
#include <ctime>
#include <cstdlib>

string mrpt::system::MRPT_getCompilationDate()
{
	time_t now;
	char *endptr;
	const char *source_date_epoch = MRPT_SOURCE_DATE_EPOCH;
	
	errno = 0;
	unsigned long epoch = strtoul(source_date_epoch, &endptr, 10);
	if (epoch==0 || ((errno == ERANGE && (epoch == std::numeric_limits<unsigned long>::max() || epoch == 0)) || (errno != 0 && epoch == 0))) {
		// Last resort:
 		now = time(NULL);
	}
	else {
		now = epoch;
	}
	struct tm *build_time = gmtime(&now);
	const int year  = build_time->tm_year + 1900;
	const int month = build_time->tm_mon + 1;
	const int day   = build_time->tm_mday;

	return mrpt::format("%i-%02i-%02i",year,month,day);
}

/*---------------------------------------------------------------
					mrpt::system::MRPT_getVersion
---------------------------------------------------------------*/
string mrpt::system::MRPT_getVersion()
{
	return string( ::MRPT_version_str );
}

/*---------------------------------------------------------------
						sprintf
---------------------------------------------------------------*/
int os::sprintf(char *buf, size_t bufSize, const char *format, ...) MRPT_NO_THROWS
{
	MRPT_UNUSED_PARAM(bufSize);

	int			result;
	va_list		ap;
	va_start (ap, format);

#if defined(_MSC_VER) && (_MSC_VER>=1400)
	// Use a secure version in Visual Studio 2005:
	result = ::vsprintf_s (buf, bufSize, format, ap);
#else
	// Use standard version:
	result = ::vsprintf (buf, format, ap);
#endif

	va_end (ap);
	return result;
}

/*---------------------------------------------------------------
					vsprintf
---------------------------------------------------------------*/
int os::vsprintf(char *buf, size_t bufSize, const char *format, va_list args) MRPT_NO_THROWS
{
	MRPT_UNUSED_PARAM(bufSize);
#if defined(_MSC_VER) && (_MSC_VER>=1400)
	// Use a secure version in Visual Studio 2005:
	return ::vsprintf_s (buf, bufSize, format, args);
#else
	// Use standard version:
	return ::vsprintf (buf, format, args);
#endif
}

/*---------------------------------------------------------------
				vsnprintf
---------------------------------------------------------------*/
int os::vsnprintf(char *buf, size_t bufSize, const char *format, va_list args) MRPT_NO_THROWS
{
#if defined(_MSC_VER)
	#if (_MSC_VER>=1400)
		// Use a secure version in Visual Studio 2005:
		return ::vsnprintf_s (buf, bufSize, _TRUNCATE, format, args);
	#else
		return ::vsprintf(buf,format, args);
	#endif
#else
	// Use standard version:
	return ::vsnprintf(buf, bufSize,format, args);
#endif
}

/*---------------------------------------------------------------
					fopen
---------------------------------------------------------------*/
FILE * os::fopen(const std::string &fileName,const char *mode) MRPT_NO_THROWS
{
	return fopen(fileName.c_str(),mode);
}

/*---------------------------------------------------------------
					fopen
---------------------------------------------------------------*/
FILE * os::fopen(const char *fileName,const char *mode) MRPT_NO_THROWS
{
#if defined(_MSC_VER) && (_MSC_VER>=1400)
	// Use a secure version in Visual Studio 2005:
	FILE	*f;
	if ( 0 != ::fopen_s(&f,fileName,mode) )
			return NULL;
	else	return f;
#else
	// Use standard version:
	return ::fopen(fileName,mode);
#endif
}

/*---------------------------------------------------------------
					fclose
---------------------------------------------------------------*/
void os::fclose(FILE *f)
{
	if (!f) THROW_EXCEPTION("Trying to close a NULL 'FILE*' descriptor")
	::fclose(f);
}

/*---------------------------------------------------------------
						strcat
---------------------------------------------------------------*/
char * os::strcat(char *dest, size_t destSize, const char *source) MRPT_NO_THROWS
{
	MRPT_UNUSED_PARAM(destSize);

#if defined(_MSC_VER) && (_MSC_VER>=1400)
	::strcat_s(dest,destSize,source);
#else
	::strcat(dest,source);
#endif
	return dest;
}

/*---------------------------------------------------------------
						strcpy
---------------------------------------------------------------*/
char * os::strcpy(char *dest, size_t destSize, const char *source) MRPT_NO_THROWS
{
	MRPT_UNUSED_PARAM(destSize);

#if defined(_MSC_VER) && (_MSC_VER>=1400)
	::strcpy_s(dest,destSize,source);
#else
	::strcpy(dest,source);
#endif
	return dest;
}


/*---------------------------------------------------------------
						strcmp
---------------------------------------------------------------*/
int os::_strcmp(const char*str1,const char*str2) MRPT_NO_THROWS
{
	return ::strcmp( str1,str2 );
}

/*---------------------------------------------------------------
						strcmpi
---------------------------------------------------------------*/
int os::_strcmpi(const char*str1,const char*str2) MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
    #if defined(_MSC_VER) && (_MSC_VER>=1400)
        return ::_strcmpi( str1,str2 );
    #else
        return ::strcmpi( str1,str2 );
    #endif
#else
    return ::strcasecmp( str1,str2 );
#endif
}

/** An OS-independent version of strncmp.
* \return It will return 0 when both strings are equal, casi sensitive.
*/
int os::_strncmp(const char*str1,const char*str2,size_t count) MRPT_NO_THROWS
{
    return ::strncmp( str1,str2,count);
}

/** An OS-independent version of strnicmp.
* \return It will return 0 when both strings are equal, casi insensitive.
*/
int os::_strnicmp(const char*str1,const char*str2,size_t count) MRPT_NO_THROWS
{
#if defined(_MSC_VER)
	return ::_strnicmp( str1,str2,count );
#else
    return ::strncasecmp( str1,str2,count );
#endif
}



/*---------------------------------------------------------------
						memcpy
---------------------------------------------------------------*/
void os::memcpy(
	void		*dest,
	size_t		destSize,
	const void	*src,
	size_t		copyCount ) MRPT_NO_THROWS
{
#if defined(_MSC_VER) && (_MSC_VER>=1400)
	::memcpy_s(dest,destSize,src,copyCount);
#else
	MRPT_UNUSED_PARAM(destSize);
	::memcpy( dest,src,copyCount );
#endif
}

/*---------------------------------------------------------------
						getch
---------------------------------------------------------------*/
int os::getch() MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
	return ::getch(); // cin.get();
#else
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
#endif
}

/*---------------------------------------------------------------
						kbhit
---------------------------------------------------------------*/
bool os::kbhit() MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
    #if defined(_MSC_VER) && (_MSC_VER>=1400)
        return ::_kbhit() != 0;
    #else
        return ::kbhit() != 0;
    #endif
#else
    return myKbhit();
#endif
}

/*---------------------------------------------------------------
						os::fprintf
---------------------------------------------------------------*/
int os::fprintf(FILE *fil, const char *frm, ...) MRPT_NO_THROWS
{
	int			result;
	va_list		ap;
	va_start(ap, frm);

#if defined(_MSC_VER) && (_MSC_VER>=1400)
	// Use a secure version in Visual Studio 2005:
	result = ::vfprintf_s(fil, frm, ap);

#else
	// Use standard version:
	result = ::vfprintf(fil, frm, ap);
#endif

	va_end (ap);
	return result;
}


/*---------------------------------------------------------------
					mrpt::system::pause
---------------------------------------------------------------*/
void mrpt::system::pause(const std::string &msg ) MRPT_NO_THROWS
{
	std::cout << msg << std::endl;
	os::getch();
}

/*---------------------------------------------------------------
					clearConsole
---------------------------------------------------------------*/
void mrpt::system::clearConsole()
{
#ifdef MRPT_OS_WINDOWS
	int ret=::system("cls");
#else
	int ret=::system("clear");
#endif
	if (ret)
		cerr << "[mrpt::system::clearConsole] Error invoking 'clear screen' " << endl;
}


/*---------------------------------------------------------------
					_strtoll
  ---------------------------------------------------------------*/
int64_t mrpt::system::os::_strtoll(const char *nptr, char **endptr, int base)
{
#ifdef MRPT_OS_WINDOWS
	return (int64_t) ::strtol( nptr, endptr, base );
#else
	return (int64_t) ::strtoll( nptr, endptr, base );
#endif

}

/*---------------------------------------------------------------
					_strtoull
  ---------------------------------------------------------------*/
uint64_t mrpt::system::os::_strtoull(const char *nptr, char **endptr, int base)
{
#ifdef MRPT_OS_WINDOWS
	return (uint64_t) ::strtoul( nptr, endptr, base );
#else
	return (uint64_t) ::strtoull( nptr, endptr, base );
#endif

}

/** Changes the text color in the console for the text written from now on.
  * The parameter "color" can be:
  *  - 0 : Normal text color
  *  - 1 : Blue text color
  *  - 2 : Green text color
  *  - 4 : Red text color
  */
void mrpt::system::setConsoleColor( TConsoleColor color,bool changeStdErr )
{
	static const int TS_NORMAL = 0;
	static const int TS_BLUE   = 1;
	static const int TS_GREEN  = 2;
	static const int TS_RED    = 4;
#ifdef MRPT_OS_WINDOWS
    static int normal_attributes = -1;
    HANDLE hstdout = GetStdHandle( changeStdErr ? STD_ERROR_HANDLE : STD_OUTPUT_HANDLE );
	fflush(changeStdErr ? stderr: stdout);

    if( normal_attributes < 0 )
    {
        CONSOLE_SCREEN_BUFFER_INFO info;
        GetConsoleScreenBufferInfo( hstdout, &info );
        normal_attributes = info.wAttributes;
    }

    SetConsoleTextAttribute( hstdout,
        (WORD)(color == TS_NORMAL ? normal_attributes :
        ((color & TS_BLUE ? FOREGROUND_BLUE : 0)|
        (color & TS_GREEN ? FOREGROUND_GREEN : 0)|
        (color & TS_RED ? FOREGROUND_RED : 0)|FOREGROUND_INTENSITY)) );
#else
	// *nix:
    static const uint8_t ansi_tab[] = { 30, 34, 32, 36, 31, 35, 33, 37 };
    int code = 0;
	fflush( changeStdErr ? stdout:stderr );
    if( color != TS_NORMAL )
        code = ansi_tab[color & (TS_BLUE|TS_GREEN|TS_RED)];
    fprintf(changeStdErr ? stdout:stderr, "\x1b[%dm", code );
#endif
}

const char* sLicenseTextF =
"                     Mobile Robot Programming Toolkit (MRPT)                \n"
"                          http://www.mrpt.org/                              \n"
"                                                                            \n"
" Copyright (c) 2005-%Y, Individual contributors, see AUTHORS file         \n"
" See: http://www.mrpt.org/Authors - All rights reserved.                   \n"
" Released under BSD License. See details in http://www.mrpt.org/License    \n";

const std::string & mrpt::system::getMRPTLicense()
{
	static bool sLicenseTextReady = false;
	static std::string sLicenseText;

	if (!sLicenseTextReady)
	{
		// Automatically update the last year of the copyright to the compilation date:
		time_t rawtime;
		struct tm * timeinfo;
		time (&rawtime);
		timeinfo = localtime (&rawtime);

		char buf[1024];
		::strftime(buf,sizeof(buf),sLicenseTextF,timeinfo);
		sLicenseText = std::string(buf);
		sLicenseTextReady=true;
	}
	return sLicenseText;
}

#include <mrpt/mrpt_paths_config.h>
std::string mrpt::system::find_mrpt_shared_dir()
{
	static bool mrpt_shared_first_call = true;
	static std::string found_mrpt_shared_dir;

	if (mrpt_shared_first_call)
	{
		mrpt_shared_first_call = false;

		for (int attempt = 0; ; attempt++)
		{
			std::string dir;
			switch (attempt)
			{
			case 0:
				dir = string(MRPT_SOURCE_BASE_DIRECTORY) + string("/share/mrpt/");
				break;
			case 1:
				dir = string(MRPT_INSTALL_PREFIX_DIRECTORY) + string("/share/mrpt/");
				break;
#ifdef _WIN32
			case 2:
			{
				char curExe[4096];
				GetModuleFileNameA(NULL,curExe, sizeof(curExe));
				
				dir = mrpt::system::extractFileDirectory(std::string(curExe)) + "/../share/mrpt/";
			}
				break;
#endif

			default:
				found_mrpt_shared_dir = ".";
				break;
			};
			if (!dir.empty() && mrpt::system::directoryExists(dir))
				found_mrpt_shared_dir = dir;

			if (!found_mrpt_shared_dir.empty())
				break;
		}
	}

	return found_mrpt_shared_dir;
}


