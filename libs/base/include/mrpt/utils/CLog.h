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

/***************************************************************
 * Name:      Log.h
 * Purpose:   Defines the Log class
 * Author:    Vicente Arévalo (varevalo@ctima.uma.es)
 * Created:   2009-09-23
 * Copyright: mapir (http://babel.isa.uma.es/mapir)
 * License:
 **************************************************************/

#ifndef CLog_H
#define CLog_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/synch/CCriticalSection.h>

namespace mrpt
{
	namespace utils
	{
		/** A decorator of CStringList special for keeping logs.
		  * \note Class written by Vicente Arevalo
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CLog : protected mrpt::utils::CStringList
		{
		public:
			CLog();
			virtual ~CLog();

			/** push a message
			*/
			void pushMessages(std::string message);

			/** push a list of messages
			*/
			void pushMessages(mrpt::utils::CStringList messages);

			/** pop the current unpublished message (clear the content of "message")
			*/
			void popMessages(std::string& message);

			/** pop all unpublished messages (clear the content of "messages")
			*/
			void popMessages(mrpt::utils::CStringList& messages);

			/** get messages from "begin" to "end" (clear the content of "messages")
			*/
			void getMessages(size_t begin, size_t end, mrpt::utils::CStringList& messages);

			/** save the current log
			*/
			void saveLog(std::string name);

			/** load a log (clear the previous content)
			*/
			void loadLog(std::string name);

			/** clear the log content
			*/
			void clearLog();

			/** change the last unpublished message. IMPORTANT: this function should
				not be used directly.
			*/
			void setLastMessageIndex(size_t index);

			/** get the current unpublished message index.
			*/
			size_t getLastMessageIndex();

		protected:

			mrpt::synch::CCriticalSection semaphore;

			size_t last;
		};
	}
}

#endif // CLog_H

