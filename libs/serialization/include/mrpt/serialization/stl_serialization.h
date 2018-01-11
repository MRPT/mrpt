/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/typemeta/TTypeName_stl.h>  // TTypeName<> for STL templates, needed for serialization of STL templates
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <vector>
#include <array>
#include <deque>
#include <set>
#include <map>
#include <list>
#include <algorithm>  // for_each()

namespace mrpt
{
namespace serialization
{
/** \addtogroup stlext_grp
  * @{ */

#define MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(CONTAINER)                       \
	/** Template method to serialize a sequential STL container  */         \
	template <class T, class _Ax>                                           \
	CArchive& operator<<(                                                   \
		CArchive& out, const CONTAINER<T, _Ax>& obj)                        \
	{                                                                       \
		out << std::string(#CONTAINER) << mrpt::typemeta::TTypeName<T>::get(); \
		out << static_cast<uint32_t>(obj.size());                           \
		std::for_each(                                                      \
			obj.begin(), obj.end(),                                         \
			metaprogramming::ObjectWriteToStream(&out));       \
		return out;                                                         \
	}                                                                       \
	/** Template method to deserialize a sequential STL container */        \
	template <class T, class _Ax>                                           \
	CArchive& operator>>(CArchive& in, CONTAINER<T, _Ax>& obj)              \
	{                                                                       \
		obj.clear();                                                        \
		std::string pref, stored_T;                                         \
		in >> pref;                                                         \
		if (pref != #CONTAINER)                                             \
			THROW_EXCEPTION_FMT(                                            \
				"Error: serialized container %s<%s>'s preambles is wrong: " \
				"'%s'",                                                     \
				#CONTAINER, mrpt::typemeta::TTypeName<T>::get().c_str(), pref.c_str())      \
		in >> stored_T;                                                     \
		if (stored_T != std::string(mrpt::typemeta::TTypeName<T>::get().c_str())) \
			THROW_EXCEPTION_FMT(                                            \
				"Error: serialized container %s< %s != %s >", #CONTAINER,   \
				stored_T.c_str(), mrpt::typemeta::TTypeName<T>::get().c_str())              \
		uint32_t n;                                                         \
		in >> n;                                                            \
		obj.resize(n);                                                      \
		std::for_each(                                                      \
			obj.begin(), obj.end(),                                         \
			metaprogramming::ObjectReadFromStream(&in));       \
		return in;                                                          \
	}

#define MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER(CONTAINER)                     \
	/** Template method to serialize an associative STL container  */       \
	template <class K, class V, class _Pr, class _Alloc>                    \
	CArchive& operator<<(                                                   \
		CArchive& out, const CONTAINER<K, V, _Pr, _Alloc>& obj)             \
	{                                                                       \
		out << std::string(#CONTAINER) << mrpt::typemeta::TTypeName<K>::get()\
			<< mrpt::typemeta::TTypeName<V>::get();                         \
		out << static_cast<uint32_t>(obj.size());                           \
		for (typename CONTAINER<K, V, _Pr, _Alloc>::const_iterator it =     \
				 obj.begin();                                               \
			 it != obj.end(); ++it)                                         \
			out << it->first << it->second;                                 \
		return out;                                                         \
	}                                                                       \
	/** Template method to deserialize an associative STL container */      \
	template <class K, class V, class _Pr, class _Alloc>                    \
	CArchive& operator>>(                                                   \
		CArchive& in, CONTAINER<K, V, _Pr, _Alloc>& obj)                    \
	{                                                                       \
		obj.clear();                                                        \
		std::string pref, stored_K, stored_V;                               \
		in >> pref;                                                         \
		if (pref != #CONTAINER)                                             \
			THROW_EXCEPTION(                                                \
				format(                                                     \
					"Error: serialized container %s<%s,%s>'s preamble is "  \
					"wrong: '%s'",                                          \
					#CONTAINER, mrpt::typemeta::TTypeName<K>::get().c_str(),\
					mrpt::typemeta::TTypeName<V>::get().c_str(), pref.c_str()))             \
		in >> stored_K;                                                     \
		if (stored_K != std::string(mrpt::typemeta::TTypeName<K>::get().c_str())) \
			THROW_EXCEPTION(                                                \
				format(                                                     \
					"Error: serialized container %s key type %s != %s",     \
					#CONTAINER, stored_K.c_str(),                           \
					mrpt::typemeta::TTypeName<K>::get().c_str()))           \
		in >> stored_V;                                                     \
		if (stored_V != std::string(mrpt::typemeta::TTypeName<V>::get().c_str()))  \
			THROW_EXCEPTION(                                                \
				format(                                                     \
					"Error: serialized container %s value type %s != %s",   \
					#CONTAINER, stored_V.c_str(),                           \
					mrpt::typemeta::TTypeName<V>::get().c_str()))           \
		uint32_t n;                                                         \
		in >> n;                                                            \
		for (uint32_t i = 0; i < n; i++)                                    \
		{                                                                   \
			K key_obj;                                                      \
			in >> key_obj;                                                  \
			/* Create an pair (Key, empty), then read directly into the     \
			 * ".second": */                                                \
			typename CONTAINER<K, V, _Pr, _Alloc>::iterator it_new =        \
				obj.insert(obj.end(), std::make_pair(key_obj, V()));        \
			in >> it_new->second;                                           \
		}                                                                   \
		return in;                                                          \
	}

#define MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER(CONTAINER)                 \
	/** Template method to serialize an associative STL container  */          \
	template <class K, class _Pr, class _Alloc>                                \
	CArchive& operator<<(                                                      \
		CArchive& out, const CONTAINER<K, _Pr, _Alloc>& obj)                   \
	{                                                                          \
		out << std::string(#CONTAINER) << mrpt::typemeta::TTypeName<K>::get(); \
		out << static_cast<uint32_t>(obj.size());                              \
		for (typename CONTAINER<K, _Pr, _Alloc>::const_iterator it =           \
				 obj.begin();                                                  \
			 it != obj.end(); ++it)                                            \
			out << *it;                                                        \
		return out;                                                            \
	}                                                                          \
	/** Template method to deserialize an associative STL container */         \
	template <class K, class _Pr, class _Alloc>                                \
	CArchive& operator>>(                                                      \
		CArchive& in, CONTAINER<K, _Pr, _Alloc>& obj)                          \
	{                                                                          \
		obj.clear();                                                           \
		std::string pref, stored_K;                                            \
		in >> pref;                                                            \
		if (pref != #CONTAINER)                                                \
			THROW_EXCEPTION(                                                   \
				format(                                                        \
					"Error: serialized container %s<%s>'s preamble is wrong: " \
					"'%s'",                                                    \
					#CONTAINER, mrpt::typemeta::TTypeName<K>::get().c_str(), pref.c_str()))    \
		in >> stored_K;                                                        \
		if (stored_K != std::string(mrpt::typemeta::TTypeName<K>::get().c_str()))\
			THROW_EXCEPTION(                                                   \
				format(                                                        \
					"Error: serialized container %s key type %s != %s",        \
					#CONTAINER, stored_K.c_str(),                              \
					mrpt::typemeta::TTypeName<K>::get().c_str()))              \
		uint32_t n;                                                            \
		in >> n;                                                               \
		for (uint32_t i = 0; i < n; i++)                                       \
		{                                                                      \
			K key_obj;                                                         \
			in >> key_obj;                                                     \
			obj.insert(key_obj);                                               \
		}                                                                      \
		return in;                                                             \
	}

MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::vector)
MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::deque)
MRPTSTL_SERIALIZABLE_SEQ_CONTAINER(std::list)

MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER(std::map)
MRPTSTL_SERIALIZABLE_ASSOC_CONTAINER(std::multimap)

MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER(std::set)
MRPTSTL_SERIALIZABLE_SIMPLE_ASSOC_CONTAINER(std::multiset)

/** Template method to serialize a std::array<T,N> */
template <class T, size_t N>
CArchive& operator<<(CArchive& out, const std::array<T,N>& obj)
{
	out << std::string("std::array") << static_cast<uint32_t>(N) << mrpt::typemeta::TTypeName<T>::get();
	std::for_each(
		obj.begin(), obj.end(),
		metaprogramming::ObjectWriteToStream(&out));
	return out;
}

/** Template method to deserialize a std::array<T,N>  */
template <class T, size_t N>
CArchive& operator>>(CArchive& in, std::array<T, N>& obj)
{
	std::string pref, stored_T;
	uint32_t stored_N;
	in >> pref >> stored_N;
	if (pref != "std::array" || stored_N != N)
		THROW_EXCEPTION_FMT(\
			"Error: serialized container %s's preambles is wrong: " \
			"'%s'", \
			mrpt::typemeta::TTypeName<std::array<T, N>>::get().c_str(), pref.c_str());
	in >> stored_T;
	if (stored_T != std::string(mrpt::typemeta::TTypeName<T>::get().c_str()))
		THROW_EXCEPTION_FMT(\
			"Error: serialized container std::array< %s != %s >", \
			stored_T.c_str(), mrpt::typemeta::TTypeName<T>::get().c_str());
	std::for_each(
		obj.begin(), obj.end(),
		metaprogramming::ObjectReadFromStream(&in));
	return in;
}


/** Template method to serialize a STL pair */
template <class T1, class T2>
CArchive& operator<<(CArchive& out, const std::pair<T1, T2>& obj)
{
	out << std::string("std::pair") << mrpt::typemeta::TTypeName<T1>::get()
		<< mrpt::typemeta::TTypeName<T2>::get();
	out << obj.first << obj.second;
	return out;
}
/** Template method to deserialize a STL pair */
template <class T1, class T2>
CArchive& operator>>(CArchive& in, std::pair<T1, T2>& obj)
{
	std::string pref, stored_K, stored_V;
	in >> pref;
	if (pref != "std::pair")
		THROW_EXCEPTION(
			format(
				"Error: serialized std::pair<%s,%s>'s preamble is wrong: '%s'",
				mrpt::typemeta::TTypeName<T1>::get().c_str(), mrpt::typemeta::TTypeName<T2>::get().c_str(),
				pref.c_str()))
	in >> stored_K;
	if (stored_K != std::string(mrpt::typemeta::TTypeName<T1>::get().c_str()))
		THROW_EXCEPTION(
			format(
				"Error: serialized std::pair first type %s != %s",
				stored_K.c_str(), mrpt::typemeta::TTypeName<T1>::get().c_str()))
	in >> stored_V;
	if (stored_V != std::string(mrpt::typemeta::TTypeName<T2>::get().c_str()))
		THROW_EXCEPTION(
			format(
				"Error: serialized std::pair second type %s != %s",
				stored_V.c_str(), mrpt::typemeta::TTypeName<T2>::get().c_str()))
	in >> obj.first >> obj.second;
	return in;
}

/** @} */  // end of grouping
}  // End of namespace
}  // End of namespace
