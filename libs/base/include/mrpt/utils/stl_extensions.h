/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef  stl_extensions_H
#define  stl_extensions_H

#include <mrpt/utils/CSerializable.h>
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


namespace mrpt
{
	namespace utils
	{
		using namespace mrpt::utils::metaprogramming;
		using std::for_each;
		using std::string;


		#define MRPTSTL_SERIALIZABLE_SEQ_CONTAINER( CONTAINER )  \
			/** Template method to serialize a sequential STL container  */ \
			template <class T> \
			CStream& operator << (CStream& out, const CONTAINER<T> &obj) \
			{ \
				out << string(#CONTAINER) << TTypeName<T>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				for_each( obj.begin(), obj.end(), ObjectWriteToStream(&out) ); \
				return out; \
			} \
			/** Template method to deserialize a sequential STL container */ \
			template <class T>  \
			CStream& operator >> (CStream& in, CONTAINER<T> &obj) \
			{ \
				obj.clear(); \
				string pref,stored_T; \
				in >> pref; \
				if (pref!=#CONTAINER) THROW_EXCEPTION(format("Error: serialized container %s<%s>'s preambles is wrong: '%s'",#CONTAINER,TTypeName<T>::get().c_str(),pref.c_str() )) \
				in >> stored_T; \
				if (stored_T != TTypeName<T>::get() ) THROW_EXCEPTION(format("Error: serialized container %s< %s != %s >",#CONTAINER,stored_T.c_str(),TTypeName<T>::get().c_str() )) \
				uint32_t n; \
				in >> n; \
				obj.resize(n); \
				for_each( obj.begin(), obj.end(), ObjectReadFromStream(&in) ); \
				return in; \
			}


		#define MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER( CONTAINER )  \
			/** Template method to serialize an associative STL container  */ \
			template <class K,class V> \
			CStream& operator << (CStream& out, const CONTAINER<K,V> &obj) \
			{ \
				out << string(#CONTAINER) << TTypeName<K>::get() << TTypeName<V>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				for (typename CONTAINER<K,V>::const_iterator it=obj.begin();it!=obj.end();++it) \
					out << it->first << it->second; \
				return out; \
			} \
			/** Template method to deserialize an associative STL container */ \
			template <class K,class V>  \
			CStream& operator >> (CStream& in, CONTAINER<K,V> &obj) \
			{ \
				obj.clear(); \
				string pref,stored_K,stored_V; \
				in >> pref; \
				if (pref!=#CONTAINER) THROW_EXCEPTION(format("Error: serialized container %s<%s,%s>'s preamble is wrong: '%s'",#CONTAINER, TTypeName<K>::get().c_str(), TTypeName<V>::get().c_str() ,pref.c_str())) \
				in >> stored_K; \
				if (stored_K != TTypeName<K>::get()) THROW_EXCEPTION(format("Error: serialized container %s key type %s != %s",#CONTAINER,stored_K.c_str(), TTypeName<K>::get().c_str())) \
				in >> stored_V; \
				if (stored_V != TTypeName<V>::get()) THROW_EXCEPTION(format("Error: serialized container %s value type %s != %s",#CONTAINER,stored_V.c_str(), TTypeName<V>::get().c_str())) \
				uint32_t n; \
				in >> n; \
				for (uint32_t i=0;i<n;i++) \
				{ \
					K 	key_obj; \
					in >> key_obj; \
					/* Create an pair (Key, empty), then read directly into the ".second": */ \
					typename CONTAINER<K,V>::iterator it_new = obj.insert(obj.begin(), std::make_pair(key_obj, V()) ); \
					in >> it_new->second; \
				} \
				return in; \
			}

		MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::vector)		// Serialization for std::vector
		MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::deque)		// Serialization for std::deque
		MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::list)		// Serialization for std::list

		MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER(std::map)		// Serialization for std::map
		MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER(std::multimap)	// Serialization for std::multimap


		#define MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER( CONTAINER )  \
			/** Template method to serialize an associative STL container  */ \
			template <class K> \
			CStream& operator << (CStream& out, const CONTAINER<K> &obj) \
			{ \
				out << string(#CONTAINER) << TTypeName<K>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				for (typename CONTAINER<K>::const_iterator it=obj.begin();it!=obj.end();++it) \
					out << *it; \
				return out; \
			} \
			/** Template method to deserialize an associative STL container */ \
			template <class K>  \
			CStream& operator >> (CStream& in, CONTAINER<K> &obj) \
			{ \
				obj.clear(); \
				string pref,stored_K; \
				in >> pref; \
				if (pref!=#CONTAINER) THROW_EXCEPTION(format("Error: serialized container %s<%s>'s preamble is wrong: '%s'",#CONTAINER, TTypeName<K>::get().c_str(),pref.c_str())) \
				in >> stored_K; \
				if (stored_K != TTypeName<K>::get()) THROW_EXCEPTION(format("Error: serialized container %s key type %s != %s",#CONTAINER,stored_K.c_str(), TTypeName<K>::get().c_str())) \
				uint32_t n; \
				in >> n; \
				for (uint32_t i=0;i<n;i++) \
				{ \
					K 	key_obj; \
					in >> key_obj; \
					obj.insert(key_obj); \
				} \
				return in; \
			}

		MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER(std::set)		// Serialization for std::set
		MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER(std::multiset)	// Serialization for std::multiset


		/** Template method to serialize a STL pair */
		template <class T1,class T2>
		CStream& operator << (CStream& out, const std::pair<T1,T2> &obj)
		{
			out << string("std::pair") << TTypeName<T1>::get() << TTypeName<T2>::get();
			out << obj.first << obj.second;
			return out;
		}
		/** Template method to deserialize a STL pair */
		template <class T1,class T2>
		CStream& operator >> (CStream& in, std::pair<T1,T2> &obj)
		{
			string pref,stored_K,stored_V;
			in >> pref;
			if (pref!="std::pair") THROW_EXCEPTION(format("Error: serialized std::pair<%s,%s>'s preamble is wrong: '%s'", TTypeName<T1>::get().c_str(), TTypeName<T2>::get().c_str() ,pref.c_str()))
			in >> stored_K;
			if (stored_K != TTypeName<T1>::get()) THROW_EXCEPTION(format("Error: serialized std::pair first type %s != %s",stored_K.c_str(), TTypeName<T1>::get().c_str()))
			in >> stored_V;
			if (stored_V != TTypeName<T2>::get()) THROW_EXCEPTION(format("Error: serialized std::pair second type %s != %s",stored_V.c_str(), TTypeName<T2>::get().c_str()))
			in >> obj.first >> obj.second;
			return in;
		}


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

		/** Calls the standard "erase" method of a STL container, but also returns an iterator to the next element in the container (or ::end if none)
		  */
		template <class CONTAINER>
		typename CONTAINER::iterator erase_return_next(CONTAINER &cont, typename CONTAINER::iterator &it)
		{
			typename CONTAINER::iterator itRet = it;
			++itRet;
			cont.erase(it);
			return itRet;
		}

		/** Generates a string for a vector in the format [A,B,C,...] to std::cout, and the fmt string for <b>each</b> vector element. */
		template <typename T>
		std::string sprintf_vector(const char *fmt, const std::vector<T> &V )
		{
			std::string ret = "[";
			size_t N = V.size();
			for (size_t i=0;i<N;i++)
			{
				ret+= format(fmt,V[i]);
				if (i!=(N-1)) ret+= ",";
			}
			ret+="]";
			return ret;
		}

		/** Prints a vector in the format [A,B,C,...] to std::cout, and the fmt string for <b>each</b> vector element. */
		template <typename T>
		void printf_vector(const char *fmt, const std::vector<T> &V ) {
			std::cout << sprintf_vector(fmt, V);
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

	} // End of namespace
} // End of namespace
#endif
