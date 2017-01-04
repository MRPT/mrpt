/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <list>
#include <map>
#include <set>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp 
		  * @{ */

		/** Returns the index of the value "T" in the container "vect" (std::vector,std::deque,etc), or string::npos if not found.
		  */
		template <class T,class CONTAINER>
		size_t find_in_vector(const T &value, const CONTAINER &vect)
		{
			typename CONTAINER::const_iterator last = vect.end();
			for (typename CONTAINER::const_iterator i=vect.begin();i!=last;++i)
				if (*i==value) return std::distance(vect.begin(),i);
			return std::string::npos;
		}

		/** Calls the standard "erase" method of a STL container, but also returns an iterator to the next element in the container (or ::end if none) */
		template <class T> inline typename std::list<T>::iterator erase_return_next(std::list<T> &cont, typename std::list<T>::iterator &it)
		{
			return cont.erase(it);
		}
		//! \overload
		template <class K,class V> inline typename std::map<K,V>::iterator erase_return_next(std::map<K,V> &cont, typename std::map<K,V>::iterator &it)
		{
			typename std::map<K,V>::iterator itRet = it; ++itRet;
			cont.erase(it);
			return itRet;
		}
		//! \overload
		template <class K,class V> inline typename std::multimap<K,V>::iterator erase_return_next(std::multimap<K,V> &cont, typename std::multimap<K,V>::iterator &it)
		{
			typename std::multimap<K,V>::iterator itRet = it; ++itRet;
			cont.erase(it);
			return itRet;
		}
		//! \overload
		template <class T> inline typename std::set<T>::iterator erase_return_next(std::set<T> &cont, typename std::set<T>::iterator &it)
		{
			typename std::set<T>::iterator itRet = it; ++itRet;
			cont.erase(it);
			return itRet;
		}


		/** @} */  // end of grouping
	}
}

