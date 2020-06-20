/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers
//

#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#ifndef HAVE_TIMEGM
#endif  // HAVE_TIMEGM

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <mutex>

#ifdef _WIN32
#include <windows.h>
//
#include <conio.h>
#include <direct.h>
#include <io.h>
#include <sys/utime.h>
#include <tlhelp32.h>
#else
#include <poll.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <utime.h>

#include <cerrno>
#include <ctime>
//	#include <signal.h>
#endif

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
#include <dlfcn.h>
#endif

#include <sys/stat.h>
#include <sys/types.h>

#include <map>

#ifdef MRPT_OS_LINUX
#define _access access
#define _rmdir rmdir
#define _stat stat
#endif

#include <sstream>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

#ifndef _WIN32
/** By ninjalj in
 * http://stackoverflow.com/questions/3962263/checking-if-a-key-was-pressed
 */
void my_aux_sighandler(int) {}
int myKbhit()
{
	struct termios oldtio
	{
	}, curtio{};
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
time_t mrpt::system::os::timegm(struct tm* tm) { return ::timegm(tm); }
#else
// Version for MSVC>=2005, which lacks "timegm"
#ifdef HAVE_MKGMTIME
time_t mrpt::system::os::timegm(struct tm* tm) { return ::_mkgmtime(tm); }
#else
// generic version, slower but probably not used in any modern compiler!
time_t mrpt::system::os::timegm(struct tm* tm)
{
	static std::mutex cs;
	std::lock_guard<std::mutex> lock(cs);

	time_t ret;
	char tz[256];

	/* save current timezone and set UTC */
	char* org_tz = getenv("TZ");
	if (org_tz) os::strcpy(tz, sizeof(tz), org_tz);

	putenv("TZ=UTC"); /* use Coordinated Universal Time (i.e. zero offset) */
	tzset();

	ret = mktime(tm);
	if (org_tz)
	{
		char buf[256];
		mrpt::system::os::sprintf(buf, sizeof(buf), "TZ=%s", tz);
		putenv(buf);
	}
	else
		putenv("TZ=");
	tzset();

	return ret;
}

#endif
#endif  // HAVE_TIMEGM

/*---------------------------------------------------------------
					mrpt::system::MRPT_getCompilationDate
---------------------------------------------------------------*/
#include <mrpt/version.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <limits>

string mrpt::system::MRPT_getCompilationDate()
{
	time_t now;
	char* endptr;
	const char* source_date_epoch = MRPT_SOURCE_DATE_EPOCH;

	errno = 0;
	unsigned long epoch = strtoul(source_date_epoch, &endptr, 10);
	if (epoch == 0 ||
		((errno == ERANGE &&
		  (epoch == std::numeric_limits<unsigned long>::max() || epoch == 0)) ||
		 (errno != 0 && epoch == 0)))
	{
		// Last resort:
		now = time(nullptr);
	}
	else
	{
		now = epoch;
	}
	struct tm* build_time = gmtime(&now);
	const int year = build_time->tm_year + 1900;
	const int month = build_time->tm_mon + 1;
	const int day = build_time->tm_mday;

	return mrpt::format(
		"%i-%02i-%02i %02i:%02i:%02i UTC", year, month, day,
		build_time->tm_hour, build_time->tm_min, build_time->tm_sec);
}

/*---------------------------------------------------------------
					mrpt::system::MRPT_getVersion
---------------------------------------------------------------*/
string mrpt::system::MRPT_getVersion() { return string(::MRPT_version_str); }
/*---------------------------------------------------------------
						sprintf
---------------------------------------------------------------*/
int os::sprintf(
	char* buf, [[maybe_unused]] size_t bufSize, const char* format,
	...) noexcept
{
	int result;
	va_list ap;
	va_start(ap, format);

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	// Use a secure version in Visual Studio 2005:
	result = ::vsprintf_s(buf, bufSize, format, ap);
#else
	// Use standard version:
	result = ::vsprintf(buf, format, ap);
#endif

	va_end(ap);
	return result;
}

/*---------------------------------------------------------------
					vsprintf
---------------------------------------------------------------*/
int os::vsprintf(
	char* buf, [[maybe_unused]] size_t bufSize, const char* format,
	va_list args) noexcept
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	// Use a secure version in Visual Studio 2005:
	return ::vsprintf_s(buf, bufSize, format, args);
#else
	// Use standard version:
	return ::vsprintf(buf, format, args);
#endif
}

/*---------------------------------------------------------------
				vsnprintf
---------------------------------------------------------------*/
int os::vsnprintf(
	char* buf, size_t bufSize, const char* format, va_list args) noexcept
{
#if defined(_MSC_VER)
#if (_MSC_VER >= 1400)
	// Use a secure version in Visual Studio 2005:
	return ::vsnprintf_s(buf, bufSize, _TRUNCATE, format, args);
#else
	return ::vsprintf(buf, format, args);
#endif
#else
	// Use standard version:
	return ::vsnprintf(buf, bufSize, format, args);
#endif
}

/*---------------------------------------------------------------
					fopen
---------------------------------------------------------------*/
FILE* os::fopen(const std::string& fileName, const char* mode) noexcept
{
	return fopen(fileName.c_str(), mode);
}

/*---------------------------------------------------------------
					fopen
---------------------------------------------------------------*/
FILE* os::fopen(const char* fileName, const char* mode) noexcept
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	// Use a secure version in Visual Studio 2005:
	FILE* f;
	if (0 != ::fopen_s(&f, fileName, mode))
		return NULL;
	else
		return f;
#else
	// Use standard version:
	return ::fopen(fileName, mode);
#endif
}

/*---------------------------------------------------------------
					fclose
---------------------------------------------------------------*/
void os::fclose(FILE* f)
{
	if (!f) THROW_EXCEPTION("Trying to close a nullptr 'FILE*' descriptor");
	::fclose(f);
}

/*---------------------------------------------------------------
						strcat
---------------------------------------------------------------*/
char* os::strcat(
	char* dest, [[maybe_unused]] size_t destSize, const char* source) noexcept
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	::strcat_s(dest, destSize, source);
#else
	::strcat(dest, source);
#endif
	return dest;
}

/*---------------------------------------------------------------
						strcpy
---------------------------------------------------------------*/
char* os::strcpy(
	char* dest, [[maybe_unused]] size_t destSize, const char* source) noexcept
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	::strcpy_s(dest, destSize, source);
#else
	::strcpy(dest, source);
#endif
	return dest;
}

/*---------------------------------------------------------------
						strcmp
---------------------------------------------------------------*/
int os::_strcmp(const char* str1, const char* str2) noexcept
{
	return ::strcmp(str1, str2);
}

/*---------------------------------------------------------------
						strcmpi
---------------------------------------------------------------*/
int os::_strcmpi(const char* str1, const char* str2) noexcept
{
#ifdef _WIN32
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	return ::_strcmpi(str1, str2);
#else
	return ::strcmpi(str1, str2);
#endif
#else
	return ::strcasecmp(str1, str2);
#endif
}

/** An OS-independent version of strncmp.
 * \return It will return 0 when both strings are equal, casi sensitive.
 */
int os::_strncmp(const char* str1, const char* str2, size_t count) noexcept
{
	return ::strncmp(str1, str2, count);
}

/** An OS-independent version of strnicmp.
 * \return It will return 0 when both strings are equal, casi insensitive.
 */
int os::_strnicmp(const char* str1, const char* str2, size_t count) noexcept
{
#if defined(_MSC_VER)
	return ::_strnicmp(str1, str2, count);
#else
	return ::strncasecmp(str1, str2, count);
#endif
}

/*---------------------------------------------------------------
						memcpy
---------------------------------------------------------------*/
void os::memcpy(
	void* dest, [[maybe_unused]] size_t destSize, const void* src,
	size_t copyCount) noexcept
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	::memcpy_s(dest, destSize, src, copyCount);
#else
	::memcpy(dest, src, copyCount);
#endif
}

/*---------------------------------------------------------------
						getch
---------------------------------------------------------------*/
int os::getch() noexcept
{
#ifdef _WIN32
	return ::getch();  // cin.get();
#else
	struct termios oldt
	{
	}, newt{};
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#endif
}

/*---------------------------------------------------------------
						kbhit
---------------------------------------------------------------*/
bool os::kbhit() noexcept
{
#ifdef _WIN32
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
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
int os::fprintf(FILE* fil, const char* frm, ...) noexcept
{
	int result;
	va_list ap;
	va_start(ap, frm);

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	// Use a secure version in Visual Studio 2005:
	result = ::vfprintf_s(fil, frm, ap);

#else
	// Use standard version:
	result = ::vfprintf(fil, frm, ap);
#endif

	va_end(ap);
	return result;
}

/*---------------------------------------------------------------
					mrpt::system::pause
---------------------------------------------------------------*/
void mrpt::system::pause(const std::string& msg) noexcept
{
	std::cout << msg << std::endl;
	os::getch();
}

/*---------------------------------------------------------------
					clearConsole
---------------------------------------------------------------*/
void mrpt::system::clearConsole()
{
#ifdef _WIN32
	int ret = ::system("cls");
#else
	int ret = ::system("clear");
#endif
	if (ret)
		cerr << "[mrpt::system::clearConsole] Error invoking 'clear screen' "
			 << endl;
}

/*---------------------------------------------------------------
					_strtoll
  ---------------------------------------------------------------*/
int64_t mrpt::system::os::_strtoll(const char* nptr, char** endptr, int base)
{
#ifdef _WIN32
	return (int64_t)::strtol(nptr, endptr, base);
#else
	return (int64_t)::strtoll(nptr, endptr, base);
#endif
}

/*---------------------------------------------------------------
					_strtoull
  ---------------------------------------------------------------*/
uint64_t mrpt::system::os::_strtoull(const char* nptr, char** endptr, int base)
{
#ifdef _WIN32
	return (uint64_t)::strtoul(nptr, endptr, base);
#else
	return (uint64_t)::strtoull(nptr, endptr, base);
#endif
}

void mrpt::system::setConsoleColor(TConsoleColor color, bool changeStdErr)
{
	static const int TS_NORMAL = 0;
	static const int TS_BLUE = 1;
	static const int TS_GREEN = 2;
	static const int TS_RED = 4;
#ifdef _WIN32
	static int normal_attributes = -1;
	HANDLE hstdout =
		GetStdHandle(changeStdErr ? STD_ERROR_HANDLE : STD_OUTPUT_HANDLE);
	fflush(changeStdErr ? stderr : stdout);

	if (normal_attributes < 0)
	{
		CONSOLE_SCREEN_BUFFER_INFO info;
		GetConsoleScreenBufferInfo(hstdout, &info);
		normal_attributes = info.wAttributes;
	}

	SetConsoleTextAttribute(
		hstdout,
		(WORD)(
			color == TS_NORMAL ? normal_attributes
							   : ((color & TS_BLUE ? FOREGROUND_BLUE : 0) |
								  (color & TS_GREEN ? FOREGROUND_GREEN : 0) |
								  (color & TS_RED ? FOREGROUND_RED : 0) |
								  FOREGROUND_INTENSITY)));
#else
	// *nix:
	FILE* f = changeStdErr ? stdout : stderr;
	const int fd = changeStdErr ? STDOUT_FILENO : STDERR_FILENO;

	// No color support if it is not a real console:
	if (!isatty(fd)) return;

	static TConsoleColor last_color = mrpt::system::CONCOL_NORMAL;
	if (color == last_color) return;
	last_color = color;

	static const uint8_t ansi_tab[] = {30, 34, 32, 36, 31, 35, 33, 37};
	int code = 0;
	fflush(f);
	if (color != TS_NORMAL)
		code = ansi_tab[color & (TS_BLUE | TS_GREEN | TS_RED)];
	fprintf(f, "\x1b[%dm", code);
#endif
}

const char* sLicenseTextF =
	"                     Mobile Robot Programming Toolkit (MRPT)              "
	"  \n"
	"                          https://www.mrpt.org/                           "
	" "
	"  \n"
	"                                                                          "
	"  \n"
	" Copyright (c) 2005-%Y, Individual contributors, see AUTHORS file         "
	"\n"
	" See: https://www.mrpt.org/Authors - All rights reserved.                 "
	" "
	" \n"
	" Released under BSD License. See details in https://www.mrpt.org/License  "
	" "
	" \n";

const std::string& mrpt::system::getMRPTLicense()
{
	static bool sLicenseTextReady = false;
	static std::string sLicenseText;

	if (!sLicenseTextReady)
	{
		// Automatically update the last year of the copyright to the
		// compilation date:
		time_t rawtime;
		struct tm* timeinfo;
		time(&rawtime);
		timeinfo = localtime(&rawtime);

		char buf[1024];
		::strftime(buf, sizeof(buf), sLicenseTextF, timeinfo);
		sLicenseText = std::string(buf);
		sLicenseTextReady = true;
	}
	return sLicenseText;
}

#ifdef _WIN32
std::string winerror2str(const char* errorPlaceName)
{
	char str[700];
	DWORD e = GetLastError();
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, e, 0, str, sizeof(str), NULL);
	std::string s;
	s = "[";
	s += errorPlaceName;
	s += "] Error: ";
	s += str;
	return s;
}
#endif

/*---------------------------------------------------------------
launchProcess
---------------------------------------------------------------*/
bool mrpt::system::launchProcess(const std::string& command)
{
#ifdef _WIN32
	STARTUPINFOA SI;
	PROCESS_INFORMATION PI;
	memset(&SI, 0, sizeof(STARTUPINFOA));
	SI.cb = sizeof(STARTUPINFOA);
	if (CreateProcessA(
			NULL, (LPSTR)command.c_str(), NULL, NULL, true, 0, NULL, NULL, &SI,
			&PI))
	{
		// Wait:
		WaitForSingleObject(PI.hProcess, INFINITE);
		return true;
	}  // End of process executed OK
	else
	{
		std::cerr << winerror2str("launchProcess");
		return false;
	}

#else

	return 0 == ::system(command.c_str());

#endif

}  // end launchProcess

#include <mrpt/mrpt_paths_config.h>
std::string mrpt::system::find_mrpt_shared_dir()
{
	static bool mrpt_shared_first_call = true;
	static std::string found_mrpt_shared_dir;

	if (mrpt_shared_first_call)
	{
		mrpt_shared_first_call = false;

		for (int attempt = 0;; attempt++)
		{
			std::string dir;
			switch (attempt)
			{
				case 0:
					dir = string(MRPT_SOURCE_BASE_DIRECTORY) +
						  string("/share/mrpt/");
					break;
				case 1:
					dir = string(MRPT_INSTALL_PREFIX_DIRECTORY) +
						  string("/share/mrpt/");
					break;
#ifdef _WIN32
				case 2:
				{
					char curExe[4096];
					GetModuleFileNameA(nullptr, curExe, sizeof(curExe));

					dir = mrpt::system::extractFileDirectory(
							  std::string(curExe)) +
						  "/../share/mrpt/";
				}
				break;
#endif

				default:
					found_mrpt_shared_dir = ".";
					break;
			};
			if (!dir.empty() && mrpt::system::directoryExists(dir))
				found_mrpt_shared_dir = dir;

			if (!found_mrpt_shared_dir.empty()) break;
		}
	}

	return found_mrpt_shared_dir;
}  // end of find_mrpt_shared_dir

int mrpt::system::executeCommand(
	const std::string& command, std::string* output /*=NULL*/,
	const std::string& mode /*="r"*/)
{
	using namespace std;

	// Create the stringstream
	stringstream sout;
	int exit_code = -1;

#ifndef _WIN32
	// Run Popen
	FILE* in;
	char buff[512];

	// Test output
	if (!(in = popen(command.c_str(), mode.c_str())))
	{
		sout << "Popen Execution failed!" << endl;
		*output = sout.str();

		return -1;
	}

	// Parse output
	while (fgets(buff, sizeof(buff), in) != nullptr)
	{
		sout << buff;
	}

	// Close
	exit_code = pclose(in);
#else
	try
	{
		exit_code = -1;

		HANDLE g_hChildStd_IN_Rd = NULL;
		HANDLE g_hChildStd_IN_Wr = NULL;
		HANDLE g_hChildStd_OUT_Rd = NULL;
		HANDLE g_hChildStd_OUT_Wr = NULL;

		HANDLE g_hInputFile = NULL;
		SECURITY_ATTRIBUTES saAttr;
		// Set the bInheritHandle flag so pipe handles are inherited.
		saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
		saAttr.bInheritHandle = TRUE;
		saAttr.lpSecurityDescriptor = NULL;
		// Create a pipe for the child process's STDOUT.
		if (!CreatePipe(&g_hChildStd_OUT_Rd, &g_hChildStd_OUT_Wr, &saAttr, 0))
			throw winerror2str("StdoutRd CreatePipe");

		// Ensure the read handle to the pipe for STDOUT is not inherited.

		if (!SetHandleInformation(g_hChildStd_OUT_Rd, HANDLE_FLAG_INHERIT, 0))
			throw winerror2str("Stdout SetHandleInformation");

		// Create a pipe for the child process's STDIN.

		if (!CreatePipe(&g_hChildStd_IN_Rd, &g_hChildStd_IN_Wr, &saAttr, 0))
			throw winerror2str("Stdin CreatePipe");

		// Ensure the write handle to the pipe for STDIN is not inherited.

		if (!SetHandleInformation(g_hChildStd_IN_Wr, HANDLE_FLAG_INHERIT, 0))
			throw winerror2str("Stdin SetHandleInformation");

		// Create the child process:
		PROCESS_INFORMATION piProcInfo;
		STARTUPINFOA siStartInfo;
		BOOL bSuccess = FALSE;

		// Set up members of the PROCESS_INFORMATION structure.

		ZeroMemory(&piProcInfo, sizeof(PROCESS_INFORMATION));

		// Set up members of the STARTUPINFO structure.
		// This structure specifies the STDIN and STDOUT handles for
		// redirection.

		ZeroMemory(&siStartInfo, sizeof(STARTUPINFO));
		siStartInfo.cb = sizeof(STARTUPINFO);
		siStartInfo.hStdError = g_hChildStd_OUT_Wr;
		siStartInfo.hStdOutput = g_hChildStd_OUT_Wr;
		siStartInfo.hStdInput = g_hChildStd_IN_Rd;
		siStartInfo.dwFlags |= STARTF_USESTDHANDLES;

		// Create the child process.
		bSuccess = CreateProcessA(
			NULL,
			(LPSTR)command.c_str(),  // command line
			NULL,  // process security attributes
			NULL,  // primary thread security attributes
			TRUE,  // handles are inherited
			0,  // creation flags
			NULL,  // use parent's environment
			NULL,  // use parent's current directory
			&siStartInfo,  // STARTUPINFO pointer
			&piProcInfo);  // receives PROCESS_INFORMATION

		// If an error occurs, exit the application.
		if (!bSuccess) throw winerror2str("CreateProcess");

		// Read from pipe that is the standard output for child process.
		DWORD dwRead;
		CHAR chBuf[4096];
		bSuccess = FALSE;
		DWORD exitval = 0;
		exit_code = 0;
		for (;;)
		{
			DWORD dwAvailable = 0;
			PeekNamedPipe(
				g_hChildStd_OUT_Rd, NULL, NULL, NULL, &dwAvailable, NULL);
			if (dwAvailable)
			{
				bSuccess = ReadFile(
					g_hChildStd_OUT_Rd, chBuf, sizeof(chBuf), &dwRead, NULL);
				if (!bSuccess || dwRead == 0) break;
				sout.write(chBuf, dwRead);
			}
			else
			{
				// process ended?
				if (GetExitCodeProcess(piProcInfo.hProcess, &exitval))
				{
					if (exitval != STILL_ACTIVE)
					{
						exit_code = exitval;
						break;
					}
				}
			}
		}

		// Close handles to the child process and its primary thread.
		CloseHandle(piProcInfo.hProcess);
		CloseHandle(piProcInfo.hThread);
	}
	catch (std::string& errStr)
	{
		std::cerr << errStr;
		return 1;  // !=0 means error
	}
#endif
	// set output - if valid pointer given
	if (output)
	{
		*output = sout.str();
	}

	// Return exit code
	return exit_code;
}  // end of executeCommand

struct LoadedModuleInfo
{
#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	void* handle = nullptr;
#endif
#ifdef MRPT_OS_WINDOWS
	HMODULE handle;
#endif
};

struct ModulesRegistry
{
	static ModulesRegistry& Instance()
	{
		static ModulesRegistry obj;
		return obj;
	}

	ModulesRegistry() = default;
	~ModulesRegistry()
	{
		try
		{
			// Make a copy, since each unload() call would invalidate iterators
			// to the changing list "loadedModules":
			loadedModules_mtx.lock();
			auto lstCopy = loadedModules;
			loadedModules_mtx.unlock();

			for (auto& ent : lstCopy) unloadPluginModule(ent.first);
		}
		catch (const std::exception& e)
		{
			std::cerr << "[~ModulesRegistry] Exception: "
					  << mrpt::exception_to_str(e);
		}
	}

	using full_path_t = std::string;
	std::map<full_path_t, LoadedModuleInfo> loadedModules;
	std::mutex loadedModules_mtx;
};

bool mrpt::system::loadPluginModule(
	const std::string& moduleFileName,
	mrpt::optional_ref<std::string> outErrorMsgs)
{
	auto& md = ModulesRegistry::Instance();
	std::lock_guard<std::mutex> lck(md.loadedModules_mtx);

	std::string sError;

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	void* handle = dlopen(moduleFileName.c_str(), RTLD_LAZY);
#elif defined(MRPT_OS_WINDOWS)
	HMODULE handle = LoadLibraryA(moduleFileName.c_str());
#else
	void* handle = nullptr;
	sError = "Unsupported OS";
#endif

	if (handle)
	{
		// register for future dlclose()
		md.loadedModules[moduleFileName].handle = handle;
		return true;
	}

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	sError = mrpt::format(
		"Error loading '%s':\n%s\n", moduleFileName.c_str(), dlerror());
#elif defined(MRPT_OS_WINDOWS)
	sError = mrpt::format(
		"Error loading '%s':\nWindows error: %u\n", moduleFileName.c_str(),
		GetLastError());
#endif

	if (outErrorMsgs)
		outErrorMsgs.value().get() += sError;
	else
		std::cerr << sError;
	return false;
}

bool mrpt::system::unloadPluginModule(
	const std::string& moduleFileName,
	mrpt::optional_ref<std::string> outErrorMsgs)
{
	auto& md = ModulesRegistry::Instance();
	std::lock_guard<std::mutex> lck(md.loadedModules_mtx);

	std::string sError;

	const auto it = md.loadedModules.find(moduleFileName);
	if (it == md.loadedModules.end())
	{
		sError = "Module filename '" + moduleFileName +
				 "' not found in previous calls to loadPluginModule().";
	}
	else
	{
#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
		if (0 != dlclose(it->second.handle))
		{
			sError = mrpt::format(
				"Error unloading '%s':\n%s\n", moduleFileName.c_str(),
				dlerror());
		}
#elif defined(MRPT_OS_WINDOWS)
		if (!FreeLibrary(it->second.handle))
		{
			sError = mrpt::format(
				"Error unloading '%s':\nWindows error: %u\n",
				moduleFileName.c_str(), GetLastError());
		}
#endif
	}

	if (!sError.empty())
	{
		if (outErrorMsgs)
			outErrorMsgs.value().get() += sError;
		else
			std::cerr << sError;
		return false;
	}
	else
	{
		md.loadedModules.erase(it);
		return true;
	}
}

bool mrpt::system::loadPluginModules(
	const std::string& moduleFileNames,
	mrpt::optional_ref<std::string> outErrorMsgs)
{
	std::vector<std::string> lstModules;
	mrpt::system::tokenize(moduleFileNames, ",", lstModules);

	bool allOk = true;
	for (const auto& sLib : lstModules)
		if (!loadPluginModule(sLib, outErrorMsgs)) allOk = false;

	return allOk;
}

bool mrpt::system::unloadPluginModules(
	const std::string& moduleFileNames,
	mrpt::optional_ref<std::string> outErrorMsgs)
{
	std::vector<std::string> lstModules;
	mrpt::system::tokenize(moduleFileNames, ",", lstModules);

	bool allOk = true;
	for (const auto& sLib : lstModules)
		if (!unloadPluginModule(sLib, outErrorMsgs)) allOk = false;

	return allOk;
}
