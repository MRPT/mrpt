/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef VARIANT_H
#define VARIANT_H

#include <mrpt/otherlibs/mapbox/variant.hpp>
namespace mrpt
{
namespace utils
{
template <typename... T>
using variant = mapbox::util::variant<T...>;
}
}
#endif
