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
#include <map>
#include <set>
#include <iostream>

namespace mrpt::containers
{
/** \addtogroup stlext_grp
 * @{ */

/** Returns the index of the value "T" in the container "vect"
 * (std::vector,std::deque,etc), or string::npos if not found.
 */
template <class T, class CONTAINER>
size_t find_in_vector(const T& value, const CONTAINER& vect)
{
	auto last = vect.end();
	for (auto i = vect.begin(); i != last; ++i)
		if (*i == value) return std::distance(vect.begin(), i);
	return std::string::npos;
}

/** Calls the standard "erase" method of a STL container, but also returns an
 * iterator to the next element in the container (or ::end if none) */
template <class T>
inline typename std::list<T>::iterator erase_return_next(
	std::list<T>& cont, typename std::list<T>::iterator& it)
{
	return cont.erase(it);
}
//! \overload
template <class K, class V>
inline typename std::map<K, V>::iterator erase_return_next(
	std::map<K, V>& cont, typename std::map<K, V>::iterator& it)
{
	typename std::map<K, V>::iterator itRet = it;
	++itRet;
	cont.erase(it);
	return itRet;
}
//! \overload
template <class K, class V>
inline typename std::multimap<K, V>::iterator erase_return_next(
	std::multimap<K, V>& cont, typename std::multimap<K, V>::iterator& it)
{
	typename std::multimap<K, V>::iterator itRet = it;
	++itRet;
	cont.erase(it);
	return itRet;
}
//! \overload
template <class T>
inline typename std::set<T>::iterator erase_return_next(
	std::set<T>& cont, typename std::set<T>::iterator& it)
{
	auto itRet = it;
	++itRet;
	cont.erase(it);
	return itRet;
}

/**\brief Return a STL container in std::string form.
 *
 * \param[in] t Template STL container (e.g. vector)
 * \return String form of given STL container
 */
template <class T>
std::string getSTLContainerAsString(const T& t)
{
	using namespace std;
	stringstream ss;
	for (auto it = t.begin(); it != t.end(); ++it)
	{
		ss << *it << ", ";
	}
	return ss.str();
}
/**\brief Print the given vector t.
 *
 * \param[in] t Template vector
 */
template <class T>
void printSTLContainer(const T& t)
{
	using namespace std;
	cout << getSTLContainerAsString(t) << endl;
}
/**\brief Print the given STL container of containers t.
 *
 * \param[in] t Template STL container (containing other containers)
 */
template <class T>
void printSTLContainerOfContainers(const T& t)
{
	using namespace std;

	int i = 0;
	for (typename T::const_iterator it = t.begin(); it != t.end(); ++i, ++it)
	{
		cout << "List " << i + 1 << "/" << t.size() << endl << "\t";
		printSTLContainer(*it);
	}
}
/**\brief Return contents of map in a string representation
 *
 * \param[in] m Template map
 * \param[in] sep String that seperates visually each key and its value.
 * Defaults to " => "
 * \return std::string representation of map
 * */
template <class T1, class T2>
std::string getMapAsString(
	const std::map<T1, T2>& m, const std::string& sep = " => ")
{
	using namespace std;
	stringstream ss("");

	for (typename map<T1, T2>::const_iterator it = m.begin(); it != m.end();
		 ++it)
	{
		ss << it->first << " => " << it->second << endl;
	}

	return ss.str();
}
/**\brief Print the given map m
 *
 * \param[in] m Template map
 */
template <class T1, class T2>
void printMap(const std::map<T1, T2>& m)
{
	std::cout << getMapAsString(m) << std::endl;
}

/**\brief Deep clear for a std vector container
 */
template <class CONTAINER>
void deep_clear(CONTAINER& c)
{
	CONTAINER empty;
	c.swap(empty);
}

/** @} */  // end of grouping
}  // namespace mrpt::containers
