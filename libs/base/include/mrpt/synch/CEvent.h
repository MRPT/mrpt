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
#ifndef  mrpt_synch_event_H
#define  mrpt_synch_event_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CReferencedMemBlock.h>

/*---------------------------------------------------------------
        Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace synch
	{
		/** This class provides a simple way of waiting for and signaling events (NOT IMPLEMENTED YET!).
		 * \ingroup synch_grp
		  */
		class BASE_IMPEXP CEvent
		{
		private:
				utils::CReferencedMemBlock              m_data;

		public:
				/** Constructor: set the initial signaled state of the event.
				  */
				CEvent( bool initialSignaled );

				/** Destructor
				  */
				~CEvent();

				/** Signal the event: the first waiting thread resumes execution (if no thread is waiting, the object keeps signaled)
				  */
				void  signal();

				/** Waits for the event to be signaled.
				  */
				void  wait();

				/** Manual reset of the event, without waiting to a signaled state (without effect if it is currently not signaled)
				  */
				void  reset();
		};

	} // End of namespace

} // End of namespace

#endif
