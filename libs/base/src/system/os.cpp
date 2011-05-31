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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/os.h>

#include <cstdlib>
#include <cstdarg>
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
	#include <signal.h>
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
	void my_aux_sighandler(int signo) {}

    int myKbhit(void)
    {
		struct termios oldtio, curtio;
		struct sigaction sa;

		/* Save stdin terminal attributes */
		tcgetattr(0, &oldtio);

		memset(&sa, 0, sizeof(struct sigaction));

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


// --------------------------------------------------------------------------------------------------
// For UNIX only: If a fatal signal is caught, throw a MRPT exception to inform about the event:
//   Based on code from wxWidgets (utilsunx.cpp)
// --------------------------------------------------------------------------------------------------

// Use the wonderful wxWidgets stack walker!
#if MRPT_HAS_WXWIDGETS && 0

#include <wx/string.h>
#include <wx/stackwalk.h>
#if wxUSE_STACKWALKER

#include <mrpt/gui/WxSubsystem.h>
//#include <wx/string.h>
//#include <wx/log.h>
//#include <wx/app.h>


/** A custom class that build a string representation of the stack frames
  */
class CMRPTStackWalker : public wxStackWalker
{
   private:
	   std::string  m_stackDescription;

   public:
	CMRPTStackWalker() : m_stackDescription()
    {
    	// We need wx subsystem running for this class!
		mrpt::gui::WxSubsystem::createOneInstanceMainThread();
    }

    virtual ~CMRPTStackWalker()
    {
    }

	std::string getAsString() const
	{
		if (m_stackDescription.empty())
		{
			return std::string();
		}
		else
		{
			// Under Windows, we only have a stack trace in debug:
#if defined(MRPT_OS_WINDOWS) && !defined(_DEBUG)
			return std::string();
#else
			return std::string("==== MRPT stack trace ====\n")+m_stackDescription;
#endif
		}
	}

    void OnStackFrame(const wxStackFrame& frame)
    {
        //cerr << format("%u\n",(unsigned int)frame.GetLevel());
        string filename(
            mrpt::system::extractFileName( string(frame.GetFileName().mb_str()) ) +
            string(".") +
            mrpt::system::extractFileExtension( string(frame.GetFileName().mb_str()) ) );

        m_stackDescription += format(
          "[%4u] 0x%p -> %s File: %s Function: %s Line: %u\n",
          (unsigned int)frame.GetLevel(),
          frame.GetAddress(),
          string(frame.GetModule().mb_str()).c_str(),
          filename.c_str(),
          string(frame.GetName().mb_str()).c_str(),
          (unsigned int)frame.GetLine()
          );
    }

};

#endif  // stack walker
#endif  // wxWidgets

extern "C" void MRPT_SIGNAL_HANDLER_SIG( int )
{
#if MRPT_HAS_WXWIDGETS && wxUSE_STACKWALKER
    CMRPTStackWalker    sw;
    sw.Walk();
    cerr << sw.getAsString(); cerr.flush();
    //THROW_EXCEPTION( "*FATAL*: Signal SIGSEGV caught!" );
    abort();
#else
	cerr << "*FATAL*: Signal SIGSEGV caught!" << endl;
    abort();
#endif
}

/** Dumps the current program stack with detailed information of source files and lines.
  *  This function requires MRPT linked against wxWidgets. Otherwise, an empty string is returned.
  *  File names and lines won't be available in release builds.
  */
std::string mrpt::system::stack_trace(bool calling_from_exception )
{
#if MRPT_HAS_WXWIDGETS && wxUSE_STACKWALKER
    CMRPTStackWalker    sw;
	/*if (calling_from_exception)
		 sw.WalkFromException();
	else sw.Walk();*/
    return sw.getAsString();
#else
	return std::string();
#endif
}


/*---------------------------------------------------------------
            registerFatalExceptionHandlers
  ---------------------------------------------------------------*/
void mrpt::system::registerFatalExceptionHandlers()
{
	static bool done = false;
	if (done) return;
	done = true;

#ifndef MRPT_OS_WINDOWS
    // install the signal handler
    struct sigaction act;
    // some systems extend it with non std fields, so zero everything
    memset(&act, 0, sizeof(act));

    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;

    act.sa_handler = MRPT_SIGNAL_HANDLER_SIG; //MRPT_SIGNAL_HANDLER_SIGFPE;
    if (0!=sigaction(SIGFPE, &act, NULL) ) cerr << "[registerFatalExceptionHandlers] Cannot install signal handler!!" << endl;

    act.sa_handler = MRPT_SIGNAL_HANDLER_SIG; //MRPT_SIGNAL_HANDLER_SIGILL;
    if (0!=sigaction(SIGILL, &act, NULL) ) cerr << "[registerFatalExceptionHandlers] Cannot install signal handler!!" << endl;

    act.sa_handler = MRPT_SIGNAL_HANDLER_SIG; //MRPT_SIGNAL_HANDLER_SIGBUS;
    if (0!=sigaction(SIGBUS, &act, NULL) ) cerr << "[registerFatalExceptionHandlers] Cannot install signal handler!!" << endl;

    act.sa_handler = MRPT_SIGNAL_HANDLER_SIG; //MRPT_SIGNAL_HANDLER_SIGSEGV;
    if (0!=sigaction(SIGSEGV, &act, NULL) ) cerr << "[registerFatalExceptionHandlers] Cannot install signal handler!!" << endl;
#endif

}

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
						vectorToTextFile
 ---------------------------------------------------------------*/
bool  mrpt::system::vectorToTextFile( const vector<float> &vec, const string &fileName, bool append, bool byRows )
{
	FILE	*f=os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (vector<float>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%e ":"%e\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

/*---------------------------------------------------------------
						vectorToTextFile
 ---------------------------------------------------------------*/
bool  mrpt::system::vectorToTextFile( const vector<double> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<double>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%e ":"%e\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

/*---------------------------------------------------------------
						vectorToTextFile
 ---------------------------------------------------------------*/
bool  mrpt::system::vectorToTextFile( const vector<int> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<int>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%i ":"%i\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

/*---------------------------------------------------------------
						vectorToTextFile
 ---------------------------------------------------------------*/
bool  mrpt::system::vectorToTextFile( const vector<size_t> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<size_t>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%u ":"%u\n",static_cast<unsigned int>(*it));

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

/*---------------------------------------------------------------
						vectorFromTextFile
 ---------------------------------------------------------------*/
bool  mrpt::system::vectorFromTextFile( std::vector<double> &vec, const std::string &fileName, bool byRows )
{
	FILE	*f = os::fopen( fileName.c_str(), "r" );
	if (!f) return false;

	double number = 0;

	while ( !feof(f) )
	{
		size_t readed = fscanf( f, byRows ? "%lf" : "%lf\n", &number );
		if ( (!byRows) || (readed == 1) )
			vec.push_back( number );
	}

	return true;
}

/*---------------------------------------------------------------
					mrpt::system::MRPT_getCompilationDate
---------------------------------------------------------------*/
string mrpt::system::MRPT_getCompilationDate()
{
	return string(__DATE__);
}

/*---------------------------------------------------------------
					mrpt::system::MRPT_getVersion
---------------------------------------------------------------*/
#include <mrpt/version.h>
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
					loadBinaryFile
  ---------------------------------------------------------------*/
bool mrpt::system::loadBinaryFile( vector_byte &out_data, const std::string &fileName )
{
	try
	{
		CFileInputStream	fi(fileName);
		size_t  N = fi.getTotalBytesCount();

		out_data.resize(N);
		if (N)
		{
			size_t NN = fi.ReadBuffer( &out_data[0], N);
			return NN==N;
		}
		else return true;
	}
	catch(...) { return false; }
}

/*---------------------------------------------------------------
					vectorToBinaryFile
  ---------------------------------------------------------------*/
bool mrpt::system::vectorToBinaryFile( const vector_byte &vec, const std::string &fileName )
{
	try
	{
		mrpt::utils::CFileOutputStream	of(fileName);
		if (!vec.empty())
			of.WriteBuffer( &vec[0], sizeof(vec[0])*vec.size() );
		return true;
	}
	catch(...) { return false; }
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
