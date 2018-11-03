/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/map_as_vector.h>
#include <mrpt/core/aligned_allocator.h>

namespace mrpt::containers
{
/** \addtogroup stlext_grp
 *  @{ */

/** @name Trait helper classes for templatized selection of a std::map
   implementation
	@{ */

/**  Traits for using a std::map<> (sparse representation) \sa
 * map_traits_map_as_vector */
struct map_traits_stdmap
{
	template <
		class KEY, class VALUE, class _LessPred = std::less<KEY>,
		class _Alloc =
			mrpt::aligned_allocator_cpp11<std::pair<const KEY, VALUE>>>
	using map = std::map<KEY, VALUE, _LessPred, _Alloc>;
};

/**  Traits for using a mrpt::utils::map_as_vector<> (dense, fastest
 * representation) \sa map_traits_stdmap  */
struct map_traits_map_as_vector
{
	template <
		class KEY, class VALUE, class _LessPred = std::less<KEY>,
		class _Alloc =
			mrpt::aligned_allocator_cpp11<std::pair<const KEY, VALUE>>>
	using map = mrpt::containers::map_as_vector<KEY, VALUE>;
};

/** @} */
/** @} */  // end of grouping

}  // namespace mrpt::containers
