/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <utility>	// std::move()

namespace mrpt::containers
{
/** A wrapper for a piece of data of type `T` which should not be copied
 *  or moved by default `operator =()`.
 *
 *  Useful for instance to hold a std::mutex or alike within a class or
 * structure with other regular data fields for which the default `operator =()`
 * is desired.
 *
 * \ingroup mrpt_containers_grp
 */
template <class T>
class NonCopiableData
{
   public:
	NonCopiableData() = default;
	~NonCopiableData() = default;

	T data;

	NonCopiableData(const NonCopiableData&) {}
	NonCopiableData& operator=(const NonCopiableData&) { return *this; }

	NonCopiableData(NonCopiableData&&) {}
	NonCopiableData& operator=(NonCopiableData&&) { return *this; }
};

}  // namespace mrpt::containers
