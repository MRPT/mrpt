/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <list>
#include <algorithm>

namespace mrpt::containers
{
/** This class implements a STL container with features of both, a std::set and
 * a std::list.
 * \note Defined in #include <mrpt/containers/list_searchable.h>
 * \ingroup mrpt_containers_grp
 */
template <class T>
class list_searchable : public std::list<T>
{
   public:
	void insert(const T& o) { std::list<T>::push_back(o); }
	typename std::list<T>::iterator find(const T& i)
	{
		return std::find(std::list<T>::begin(), std::list<T>::end(), i);
	}

	typename std::list<T>::const_iterator find(const T& i) const
	{
		return std::find(std::list<T>::begin(), std::list<T>::end(), i);
	}

	/** Finds an element in a list of smart pointers, having "->pointer()", such
	 * as it matches a given plain pointer "ptr". */
	template <typename PTR>
	typename std::list<T>::iterator find_ptr_to(const PTR ptr)
	{
		for (auto it = std::list<T>::begin(); it != std::list<T>::end(); it++)
			if (it->get() == ptr) return it;
		return std::list<T>::end();
	}

	/** Finds an element in a list of smart pointers, having "->pointer()", such
	 * as it matches a given plain pointer "ptr". */
	template <typename PTR>
	typename std::list<T>::const_iterator find_ptr_to(const PTR ptr) const
	{
		for (typename std::list<T>::const_iterator it = std::list<T>::begin();
			 it != std::list<T>::end(); it++)
			if (it->pointer() == ptr) return it;
		return std::list<T>::end();
	}
};
}  // namespace mrpt::containers
