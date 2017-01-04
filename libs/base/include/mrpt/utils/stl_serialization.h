/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/TTypeName_impl.h> // TTypeName<> for STL templates, needed for serialization of STL templates
#include <mrpt/utils/metaprogramming_serialization.h>
#include <mrpt/utils/CStream.h>
#include <vector>
#include <deque>
#include <set>
#include <map>
#include <list>
#include <algorithm> // for_each()

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp
		  * @{ */

		#define MRPTSTL_SERIALIZABLE_SEQ_CONTAINER( CONTAINER )  \
			/** Template method to serialize a sequential STL container  */ \
			template <class T,class _Ax> \
			CStream& operator << (mrpt::utils::CStream& out, const CONTAINER<T,_Ax> &obj) \
			{ \
				out << std::string(#CONTAINER) << mrpt::utils::TTypeName<T>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				std::for_each( obj.begin(), obj.end(), mrpt::utils::metaprogramming::ObjectWriteToStream(&out) ); \
				return out; \
			} \
			/** Template method to deserialize a sequential STL container */ \
			template <class T,class _Ax>  \
			CStream& operator >> (mrpt::utils::CStream& in, CONTAINER<T,_Ax> &obj) \
			{ \
				obj.clear(); \
				std::string pref,stored_T; \
				in >> pref; \
				if (pref!=#CONTAINER) THROW_EXCEPTION(mrpt::format("Error: serialized container %s<%s>'s preambles is wrong: '%s'",#CONTAINER,TTypeName<T>::get().c_str(),pref.c_str() )) \
				in >> stored_T; \
				if (stored_T != mrpt::utils::TTypeName<T>::get() ) THROW_EXCEPTION(mrpt::format("Error: serialized container %s< %s != %s >",#CONTAINER,stored_T.c_str(),TTypeName<T>::get().c_str() )) \
				uint32_t n; \
				in >> n; \
				obj.resize(n); \
				std::for_each( obj.begin(), obj.end(), mrpt::utils::metaprogramming::ObjectReadFromStream(&in) ); \
				return in; \
			}


		#define MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER( CONTAINER )  \
			/** Template method to serialize an associative STL container  */ \
			template <class K,class V, class _Pr, class _Alloc> \
			CStream& operator << (mrpt::utils::CStream& out, const CONTAINER<K,V,_Pr,_Alloc> &obj) \
			{ \
				out << std::string(#CONTAINER) << TTypeName<K>::get() << TTypeName<V>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				for (typename CONTAINER<K,V,_Pr,_Alloc>::const_iterator it=obj.begin();it!=obj.end();++it) \
					out << it->first << it->second; \
				return out; \
			} \
			/** Template method to deserialize an associative STL container */ \
			template <class K,class V, class _Pr, class _Alloc>  \
			CStream& operator >> (mrpt::utils::CStream& in, CONTAINER<K,V,_Pr,_Alloc> &obj) \
			{ \
				obj.clear(); \
				std::string pref,stored_K,stored_V; \
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
					typename CONTAINER<K,V,_Pr,_Alloc>::iterator it_new = obj.insert(obj.end(), std::make_pair(key_obj, V()) ); \
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
			template <class K,class _Pr,class _Alloc> \
			CStream& operator << (mrpt::utils::CStream& out, const CONTAINER<K,_Pr,_Alloc> &obj) \
			{ \
				out << std::string(#CONTAINER) << TTypeName<K>::get(); \
				out << static_cast<uint32_t>(obj.size()); \
				for (typename CONTAINER<K,_Pr,_Alloc>::const_iterator it=obj.begin();it!=obj.end();++it) \
					out << *it; \
				return out; \
			} \
			/** Template method to deserialize an associative STL container */ \
			template <class K,class _Pr,class _Alloc>  \
			CStream& operator >> (mrpt::utils::CStream& in, CONTAINER<K,_Pr,_Alloc> &obj) \
			{ \
				obj.clear(); \
				std::string pref,stored_K; \
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
		CStream& operator << (mrpt::utils::CStream& out, const std::pair<T1,T2> &obj)
		{
			out << std::string("std::pair") << TTypeName<T1>::get() << TTypeName<T2>::get();
			out << obj.first << obj.second;
			return out;
		}
		/** Template method to deserialize a STL pair */
		template <class T1,class T2>
		CStream& operator >> (mrpt::utils::CStream& in, std::pair<T1,T2> &obj)
		{
			std::string pref,stored_K,stored_V;
			in >> pref;
			if (pref!="std::pair") THROW_EXCEPTION(format("Error: serialized std::pair<%s,%s>'s preamble is wrong: '%s'", TTypeName<T1>::get().c_str(), TTypeName<T2>::get().c_str() ,pref.c_str()))
			in >> stored_K;
			if (stored_K != TTypeName<T1>::get()) THROW_EXCEPTION(format("Error: serialized std::pair first type %s != %s",stored_K.c_str(), TTypeName<T1>::get().c_str()))
			in >> stored_V;
			if (stored_V != TTypeName<T2>::get()) THROW_EXCEPTION(format("Error: serialized std::pair second type %s != %s",stored_V.c_str(), TTypeName<T2>::get().c_str()))
			in >> obj.first >> obj.second;
			return in;
		}

		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace
