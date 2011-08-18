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
#ifndef  mrpt_traits_maps_H
#define  mrpt_traits_maps_H

// File to be included from stl_extensions.h
#include <mrpt/utils/map_as_vector.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp
		  *  @{ */

		/** @name Trait helper classes for templatized selection of a std::map implementation
		    @{ */

		/**  Traits for using a std::map<> (sparse representation) \sa map_traits_map_as_vector */
		struct map_traits_stdmap {
			template <class KEY,class VALUE,class _LessPred = std::less<KEY>, class _Alloc = Eigen::aligned_allocator<std::pair<const KEY, VALUE> > >
			struct map : public std::map<KEY,VALUE,_LessPred,_Alloc> {
			};
		};

		/**  Traits for using a mrpt::utils::map_as_vector<> (dense, fastest representation) \sa map_traits_stdmap  */
		struct map_traits_map_as_vector	{
			template <class KEY,class VALUE,class _LessPred = std::less<KEY>, class _Alloc = Eigen::aligned_allocator<std::pair<const KEY, VALUE> > >
			struct map : public mrpt::utils::map_as_vector<KEY,VALUE> { };
		};

		/** @} */
		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace

#endif
