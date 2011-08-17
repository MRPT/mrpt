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
