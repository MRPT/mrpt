/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CMessageQueue_H
#define  CMessageQueue_H

#include <mrpt/utils/CThreadSafeQueue.h>

namespace mrpt
{
	namespace utils
	{

		/** A thread-safe class for message passing between threads.   
		  * \sa CThreadSafeQueue 
		  * \ingroup mrpt_base_grp
		  */
		typedef CThreadSafeQueue<CMessage> CMessageQueue;

	} // End of namespace
} // end of namespace
#endif
