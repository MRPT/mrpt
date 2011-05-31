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
#ifndef  internal_class_registry_H
#define  internal_class_registry_H

#include <mrpt/synch/atomic_incr.h>
#include <mrpt/utils/CThreadSafeQueue.h>

namespace mrpt
{
	namespace utils
	{
		/** Register all pending classes - to be called just before de-serializing an object, for example.
		  * After calling this method, pending_class_registers_modified is set to false until pending_class_registers() is invoked.
 		  */
		void BASE_IMPEXP registerAllPendingClasses();

		typedef void (*TRegisterFunction)(); // A void(void) function

		// Use a queue for the pending register issues, but also an atomic counter, which is much faster to check than a CS.
		mrpt::synch::CAtomicCounter	BASE_IMPEXP &           pending_class_registers_count();
		CThreadSafeQueue<TRegisterFunction> BASE_IMPEXP &   pending_class_registers();
		extern volatile bool BASE_IMPEXP                           pending_class_registers_modified; //!< Set to true if pending_class_registers() has been called after registerAllPendingClasses(). Startup value is false.

	} // End of namespace
} // End of namespace

#endif
