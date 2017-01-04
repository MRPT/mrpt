/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  FILE_MRPT_OS_H
#define  FILE_MRPT_OS_H

#include <mrpt/config.h>

#include <cstdlib>
#include <cstdarg>

// Duplicated here since <mrpt/system/os.h> is the only header that cannot include "utils_defs.h"
#include <mrpt/base/link_pragmas.h>  // DLL import/export definitions

#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>

namespace mrpt
{
	/** This namespace provides a OS-independent interface to many useful functions: filenames manipulation, time and date, string parsing, file I/O, threading, memory allocation, etc.
	 *  \sa mrpt::system::os \ingroup mrpt_base_grp
	 */
	namespace system
	{
		/** \defgroup mrpt_system_os OS and compiler abstraction (in #include <mrpt/system/os.h>)
		  * \ingroup mrpt_base_grp */

		/** This namespace provides a OS-independent interface to low-level functions.
		 *   Most of these functions are converted into calls to standard functions, unless we are into Visual Studio 2005 (or newer). In that case the secure version
		 *     of the standard library functions (prefix "_s") are used instead. \ingroup mrpt_base_grp mrpt_system_os
		 */
		namespace os
		{
			/** \addtogroup mrpt_system_os
			  * @{ */

			/** An OS-independent version of sprintf (Notice the bufSize param, which may be ignored in some compilers)
			  *  \sa utils::format
			  */
			int BASE_IMPEXP sprintf(char *buf, size_t bufSize, const char *format, ...) MRPT_NO_THROWS MRPT_printf_format_check(3,4);

			/** An OS-independent version of vsprintf (Notice the bufSize param, which may be ignored in some compilers)
			  */
			int BASE_IMPEXP vsprintf(char *buf, size_t bufSize, const char *format, va_list args) MRPT_NO_THROWS;

			/** An OS-independent version of vsnprintf (Notice the bufSize param, which may be ignored in some compilers)
			  */
			int BASE_IMPEXP vsnprintf(char *buf, size_t bufSize, const char *format, va_list args) MRPT_NO_THROWS;

			/** An OS-independent version of fopen.
			  * \return It will always return NULL on any error.
			  */
			FILE BASE_IMPEXP *fopen(const char *fileName,const char *mode) MRPT_NO_THROWS;

			/** An OS-independent version of fopen (std::string version)
			  * \return It will always return NULL on any error.
			  */
			FILE BASE_IMPEXP *fopen(const std::string &fileName,const char *mode) MRPT_NO_THROWS;

			/** An OS-independent version of fprintf
			  */
			int BASE_IMPEXP fprintf(FILE *fil, const char *format, ...)  MRPT_NO_THROWS MRPT_printf_format_check(2,3);

			/** An OS-independent version of fclose.
			  * \exception std::exception On trying to close a NULL file descriptor.
			  */
			void BASE_IMPEXP fclose(FILE *f);

			/** An OS-independent version of strcat.
			  * \return It will always return the "dest" pointer.
			  */
			char BASE_IMPEXP * strcat(char *dest, size_t destSize, const char *source) MRPT_NO_THROWS;

			/** An OS-independent version of strcpy.
			  * \return It will always return the "dest" pointer.
			  */
			char  BASE_IMPEXP *strcpy(char *dest, size_t destSize, const char *source) MRPT_NO_THROWS;

			/** An OS-independent version of strcmp.
			  * \return It will return 0 when both strings are equal, casi sensitive.
			  */
			int BASE_IMPEXP _strcmp(const char*str1,const char*str2) MRPT_NO_THROWS;

			/** An OS-independent version of strcmpi.
			  * \return It will return 0 when both strings are equal, casi insensitive.
			  */
			int BASE_IMPEXP _strcmpi(const char*str1,const char*str2) MRPT_NO_THROWS;

			/** An OS-independent version of strncmp.
			  * \return It will return 0 when both strings are equal, casi sensitive.
			  */
			int BASE_IMPEXP _strncmp(const char*str,const char*subStr,size_t count) MRPT_NO_THROWS;

			/** An OS-independent version of strnicmp.
			  * \return It will return 0 when both strings are equal, casi insensitive.
			  */
			int BASE_IMPEXP _strnicmp(const char*str,const char*subStr,size_t count) MRPT_NO_THROWS;

			/** An OS-independent version of strtoll.
			  */
			int64_t BASE_IMPEXP _strtoll(const char *nptr, char **endptr, int base);

			/** An OS-independent version of strtoull.
			  */
			uint64_t BASE_IMPEXP _strtoull(const char *nptr, char **endptr, int base);

			/** An OS-independent version of timegm (which is not present in all compilers): converts a time structure into an UTM time_t */
			time_t BASE_IMPEXP timegm(struct tm *tm);

			/** An OS and compiler independent version of "memcpy"
			  */
			void BASE_IMPEXP memcpy(
				void		*dest,
				size_t		destSize,
				const void	*src,
				size_t		copyCount ) MRPT_NO_THROWS;

			/** An OS-independent version of getch, which waits until a key is pushed.
			  * \return The pushed key code
			  */
			int BASE_IMPEXP getch() MRPT_NO_THROWS;

			/** An OS-independent version of kbhit, which returns true if a key has been pushed.
			  */
			bool BASE_IMPEXP kbhit() MRPT_NO_THROWS;

			/** @} */

		}	// end namespace "os"

		/** \addtogroup mrpt_system_os
		  * @{ */

		/** Shows the message "Press any key to continue" (or other custom message) to the current standard output and returns when a key is pressed */
		void BASE_IMPEXP pause(const std::string &msg = std::string("Press any key to continue...") ) MRPT_NO_THROWS;

		/** Clears the console window */
		void BASE_IMPEXP clearConsole();

		/** Returns the MRPT source code timestamp, according to the Reproducible-Builds specifications: https://reproducible-builds.org/specs/source-date-epoch/  */
		std::string BASE_IMPEXP MRPT_getCompilationDate();

		/** Returns a string describing the MRPT version */
		std::string BASE_IMPEXP MRPT_getVersion();

		/** Returns a const ref to a text with the same text that appears at the beginning of each MRPT file (useful for displaying the License text in GUIs) */
		const std::string BASE_IMPEXP & getMRPTLicense();

		/** Finds the "[MRPT]/share/mrpt/" directory, if available in the system. This searches in (1) source code tree, (2) install target paths. */
		std::string BASE_IMPEXP find_mrpt_shared_dir();

		/** For use in  setConsoleColor */
		enum TConsoleColor
		{
			CONCOL_NORMAL = 0,
			CONCOL_BLUE   = 1,
			CONCOL_GREEN  = 2,
			CONCOL_RED    = 4
		};

		/** Changes the text color in the console for the text written from now on.
		  * The parameter "color" can be any value in TConsoleColor.
		  *
		  * By default the color of "cout" is changed, unless changeStdErr=true, in which case "cerr" is changed.
		  */
		void BASE_IMPEXP setConsoleColor( TConsoleColor color, bool changeStdErr=false );

		/** @} */

	} // End of namespace

} // End of namespace

#endif
