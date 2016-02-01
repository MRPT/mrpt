/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CDebugOutputCapable_H
#define  CDebugOutputCapable_H

#include <mrpt/base/link_pragmas.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		/** This base class provides a common printf-like method to send debug information to std::cout, with the purpose of allowing its redirection to other streams if desired.
		  *  By default, messages sent to "printf_debug" will be shown in the console (cout) and also in
		  *   the stream passed to debugOutputSetStream, but the console output can be
		  *   switched off with debugOutputEnableConsole(false).
		  *
		  *  In addition, this class will send all the messages from "printf_debug" to the Visual Studio "Output Window" in debug mode (obviously, this is a feature only enabled under Windows).
		  *
		  *  See CDebugOutputCapable::printf_debug.
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CDebugOutputCapable
		{
		public:
			/** Default initialization */
			CDebugOutputCapable() { }
			virtual ~CDebugOutputCapable() { }

			/** Sends a formated text to "debugOut" if not NULL, or to cout otherwise. */
			static void printf_debug( const char *frmt, ... );

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
