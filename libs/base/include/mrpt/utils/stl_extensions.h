/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  stl_extensions_H
#define  stl_extensions_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/TTypeName_impl.h> // TTypeName<> for STL templates, needed for serialization of STL templates
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/metaprogramming.h>

#include <set>
#include <map>
#include <list>
#include <cctype>  // tolower

// These are "STL extensions" but have their own files for clarity
#include <mrpt/utils/circular_buffer.h>
#include <mrpt/utils/list_searchable.h>
#include <mrpt/utils/bimap.h>
#include <mrpt/utils/map_as_vector.h>
#include <mrpt/utils/traits_map.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/utils/printf_vector.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp STL extensions and metaprogramming
		  * \ingroup mrpt_base_grp
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

		/** A case-insensitive comparator struct for usage within STL containers, eg: map<string,string,ci_less>
		  */
		struct ci_less : std::binary_function<std::string,std::string,bool>
		{
			// case-independent (ci) compare_less binary function
			struct nocase_compare : public std::binary_function<char,char,bool> {
				bool operator()(const char c1, const char c2) const { return tolower(c1)<tolower(c2); }
			};
			bool operator() (const std::string & s1, const std::string & s2) const {
				return std::lexicographical_compare(s1.begin(),s1.end(), s2.begin(),s2.end(), nocase_compare());
			}
		}; // end of ci_less

		/** @} */  // end of grouping

	} // End of namespace
} // End of namespace
#endif
