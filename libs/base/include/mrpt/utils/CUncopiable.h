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
#ifndef  CUNCOPIABLE_H
#define  CUNCOPIABLE_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** The base class of classes that cannot be copied: compile-time errors will be issued on any copy operation.
		 *   An example:
		 *
		 *  \code
		 *   class MyFancyClass : public mrpt::utils::CUncopiable
		 *   {
		 *    public:
		 *     ...
		 *   };
		 *  \endcode
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CUncopiable
		{
		private:
			CUncopiable(const CUncopiable &);  // This doesn't need to be implemented anywhere
			const CUncopiable& operator =(const CUncopiable &);   // This doesn't need to be implemented anywhere
		public:
			CUncopiable() {  }
		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
