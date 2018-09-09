/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>

#include <cstdlib>
#include <cstdarg>
#include <cstdint>
#include <cstdio>  // FILE
#include <string>
#include <mrpt/core/common.h>

namespace mrpt::system
{
/** \defgroup mrpt_system_os OS and compiler abstraction
 * Header: `#include <mrpt/system/os.h>`.
 * Library: \ref mrpt_system_grp
 * \ingroup mrpt_system_grp */

namespace os
{
/** \addtogroup mrpt_system_os
 * @{ */

/** An OS-independent version of sprintf (Notice the bufSize param, which may be
 * ignored in some compilers)
 *  \sa mrpt::format
 */
int sprintf(
	char* buf, size_t bufSize, const char* format,
	...) noexcept MRPT_printf_format_check(3, 4);

/** An OS-independent version of vsprintf (Notice the bufSize param, which may
 * be ignored in some compilers)
 */
int vsprintf(
	char* buf, size_t bufSize, const char* format, va_list args) noexcept;

/** An OS-independent version of vsnprintf (Notice the bufSize param, which may
 * be ignored in some compilers)
 */
int vsnprintf(
	char* buf, size_t bufSize, const char* format, va_list args) noexcept;

/** An OS-independent version of fopen.
 * \return It will always return nullptr on any error.
 */
FILE* fopen(const char* fileName, const char* mode) noexcept;

/** An OS-independent version of fopen (std::string version)
 * \return It will always return nullptr on any error.
 */
FILE* fopen(const std::string& fileName, const char* mode) noexcept;

/** An OS-independent version of fprintf
 */
int fprintf(
	FILE* fil, const char* format, ...) noexcept MRPT_printf_format_check(2, 3);

/** An OS-independent version of fclose.
 * \exception std::exception On trying to close a nullptr file descriptor.
 */
void fclose(FILE* f);

/** An OS-independent version of strcat.
 * \return It will always return the "dest" pointer.
 */
char* strcat(char* dest, size_t destSize, const char* source) noexcept;

/** An OS-independent version of strcpy.
 * \return It will always return the "dest" pointer.
 */
char* strcpy(char* dest, size_t destSize, const char* source) noexcept;

/** An OS-independent version of strcmp.
 * \return It will return 0 when both strings are equal, casi sensitive.
 */
int _strcmp(const char* str1, const char* str2) noexcept;

/** An OS-independent version of strcmpi.
 * \return It will return 0 when both strings are equal, casi insensitive.
 */
int _strcmpi(const char* str1, const char* str2) noexcept;

/** An OS-independent version of strncmp.
 * \return It will return 0 when both strings are equal, casi sensitive.
 */
int _strncmp(const char* str, const char* subStr, size_t count) noexcept;

/** An OS-independent version of strnicmp.
 * \return It will return 0 when both strings are equal, casi insensitive.
 */
int _strnicmp(const char* str, const char* subStr, size_t count) noexcept;

/** An OS-independent version of strtoll.
 */
int64_t _strtoll(const char* nptr, char** endptr, int base);

/** An OS-independent version of strtoull.
 */
uint64_t _strtoull(const char* nptr, char** endptr, int base);

/** An OS-independent version of timegm (which is not present in all compilers):
 * converts a time structure into an UTM time_t */
time_t timegm(struct tm* tm);

/** An OS and compiler independent version of "memcpy"
 */
void memcpy(
	void* dest, size_t destSize, const void* src, size_t copyCount) noexcept;

/** An OS-independent version of getch, which waits until a key is pushed.
 * \return The pushed key code
 */
int getch() noexcept;

/** An OS-independent version of kbhit, which returns true if a key has been
 * pushed.
 */
bool kbhit() noexcept;

/** @} */

}  // namespace os

/** \addtogroup mrpt_system_os
 * @{ */

/** Shows the message "Press any key to continue" (or other custom message) to
 * the current standard output and returns when a key is pressed */
void pause(
	const std::string& msg =
		std::string("Press any key to continue...")) noexcept;

/** Clears the console window */
void clearConsole();

/** Returns the MRPT source code timestamp, according to the Reproducible-Builds
 * specifications: https://reproducible-builds.org/specs/source-date-epoch/  */
std::string MRPT_getCompilationDate();

/** Returns a string describing the MRPT version */
std::string MRPT_getVersion();

/** Returns a const ref to a text with the same text that appears at the
 * beginning of each MRPT file (useful for displaying the License text in GUIs)
 */
const std::string& getMRPTLicense();

/** Finds the "[MRPT]/share/mrpt/" directory, if available in the system. This
 * searches in (1) source code tree, (2) install target paths. */
std::string find_mrpt_shared_dir();

/** For use in  setConsoleColor */
enum TConsoleColor
{
	CONCOL_NORMAL = 0,
	CONCOL_BLUE = 1,
	CONCOL_GREEN = 2,
	CONCOL_RED = 4
};

/** Changes the text color in the console for the text written from now on.
 * The parameter "color" can be any value in TConsoleColor.
 *
 * By default the color of "cout" is changed, unless changeStdErr=true, in
 * which case "cerr" is changed.
 */
void setConsoleColor(TConsoleColor color, bool changeStdErr = false);

/** @brief Execute Generic Shell Command
 *
 * @param[in]   command Command to execute
 * @param[out]  output  Pointer to string containing the shell output
 * @param[in]   mode read/write access
 *
 * @return 0 for success, -1 otherwise.
 *
 * \note Original code snippet found in http://stackoverflow.com/a/30357710
 */
int executeCommand(
	const std::string& command, std::string* output = nullptr,
	const std::string& mode = "r");

/** Executes the given command (which may contain a program + arguments), and
waits until it finishes.

* \return false on any error, true otherwise

*/
bool launchProcess(const std::string& command);

/** @} */

}  // namespace mrpt::system
