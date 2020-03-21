/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/map_as_vector.h>

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
	template <class KEY, class VALUE>
	using map = std::map<KEY, VALUE>;
};

/**  Traits for using a mrpt::containers::map_as_vector<> (dense, fastest
 * representation) \sa map_traits_stdmap  */
struct map_traits_map_as_vector
{
	template <class KEY, class VALUE>
	using map = mrpt::containers::map_as_vector<KEY, VALUE>;
};

/** @} */
/** @} */  // end of grouping

}  // namespace mrpt::containers
