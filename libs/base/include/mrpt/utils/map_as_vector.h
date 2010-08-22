/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef  mrpt_map_as_vector_H
#define  mrpt_map_as_vector_H

// Note: This file is included from "stl_extensions.h"

#include <mrpt/utils/utils_defs.h>
#include <map>
#include <vector>

namespace mrpt
{
	namespace utils
	{
		/** A STL-like container which looks like a std::map<> but is implemented as a linear std::vector<> indexed by KEY.
		  *  Note that KEY must be integer types only (size_t, uint32_t, etc.)
		  *  This implementation is much more efficient than std::map<> when the most common operation is accesing elements
		  *   by KEY with find() or [], and the range of KEY values starts at 0 (or a reasonable low number).
		  *
		  */
		template <typename KEY,typename VALUE>
		class map_as_vector
		{
		public:
			typedef std::vector<VALUE> vec_t;

		private:
			vec_t  m_vec; //!< The actual container

		public:
			/** Default constructor - does nothing */
			inline map_as_vector() { }

			inline size_t size() const { return m_vec.size(); }
			inline bool empty() const { return m_vec.empty(); }

			/** Return a read-only reference to the internal vector */
			inline const vec_t &getVector() const { return m_vec; }

			/** Clear the contents of this container */
			inline void clear() { m_vec.clear(); }


		};  // end class map_as_vector

	} // End of namespace
} // End of namespace
#endif
