/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/***************************************************************
 * Name:      Log.h
 * Purpose:   Defines the Log class
 * Author:    Vicente Arevalo (varevalo@ctima.uma.es)
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

