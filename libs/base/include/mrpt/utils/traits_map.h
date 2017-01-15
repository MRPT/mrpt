/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_traits_maps_H
#define  mrpt_traits_maps_H

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
