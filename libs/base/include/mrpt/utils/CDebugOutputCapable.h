/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  CDebugOutputCapable_H
#define  CDebugOutputCapable_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>

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
