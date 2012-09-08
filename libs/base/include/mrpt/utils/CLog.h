/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

