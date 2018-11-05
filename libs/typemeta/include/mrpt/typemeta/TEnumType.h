/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <map>
#include <stdexcept>

namespace mrpt
{
namespace typemeta
{
namespace internal
{
template <typename KEY, typename VALUE>
struct bimap
{
	std::map<KEY, VALUE> m_k2v;
	std::map<VALUE, KEY> m_v2k;

	using const_iterator = typename std::map<KEY, VALUE>::const_iterator;
	const_iterator begin() const { return m_k2v.begin(); }
	const_iterator end() const { return m_k2v.end(); }
	bool direct(const KEY& k, VALUE& out_v) const
	{
		auto i = m_k2v.find(k);
		if (i == m_k2v.end()) return false;
		out_v = i->second;
		return true;
	}
	bool inverse(const VALUE& v, KEY& out_k) const
	{
		auto i = m_v2k.find(v);
		if (i == m_v2k.end()) return false;
		out_k = i->second;
		return true;
	}
	void insert(const KEY& k, const VALUE& v)
	{
		m_k2v[k] = v;
		m_v2k[v] = k;
	}
};  // end bimap
}  // namespace internal

/** Only specializations of this class are defined for each enum type of
 * interest
 * \sa TEnumType \ingroup mrpt_io_grp
 */
template <typename ENUMTYPE>
struct TEnumTypeFiller
{
	static void fill(internal::bimap<ENUMTYPE, std::string>& m_map);
};

#define MRPT_ENUM_TYPE_BEGIN(_ENUM_TYPE_WITH_NS)                              \
	namespace mrpt                                                            \
	{                                                                         \
	namespace typemeta                                                        \
	{                                                                         \
	template <>                                                               \
	struct TEnumTypeFiller<_ENUM_TYPE_WITH_NS>                                \
	{                                                                         \
		static void fill(                                                     \
			mrpt::typemeta::internal::bimap<_ENUM_TYPE_WITH_NS, std::string>& \
				m_map)                                                        \
		{
#define MRPT_ENUM_TYPE_BEGIN_NAMESPACE(_NAMESPACE, _ENUM_TYPE_WITH_NS) \
	MRPT_ENUM_TYPE_BEGIN(_ENUM_TYPE_WITH_NS)                           \
	using namespace _NAMESPACE;

#define MRPT_ENUM_TYPE_END() \
	}                        \
	}                        \
	;                        \
	}                        \
	}

/** For use in specializations of TEnumTypeFiller */
#define MRPT_FILL_ENUM(_X) m_map.insert(_X, #_X)
#define MRPT_FILL_ENUM_CUSTOM_NAME(_X, _NAME) m_map.insert(_X, _NAME)
#define MRPT_FILL_ENUM_MEMBER(_CLASS, _VALUE) \
	m_map.insert(_CLASS::_VALUE, #_VALUE)

/** A helper class that can convert an enum value into its textual
 * representation, and viceversa. \ingroup mrpt_typename_grp */
template <typename ENUMTYPE>
struct TEnumType
{
#define _MRPT_AUXTOSTR(__AA) #__AA

	/** Gives the numerical name for a given enum text name \exception
	 * std::exception on unknown enum name */
	static ENUMTYPE name2value(const std::string& name)
	{
		ENUMTYPE val;
		if (!getBimap().inverse(name, val))
		{
			throw std::runtime_error(
				std::string("TEnumType<" _MRPT_AUXTOSTR(
					TEnumType) ">::name2value(): Unknown name: ") +
				name);
		}
		return val;
	}

	/** Gives the textual name for a given enum value \exception std::exception
	 * on unknown enum value name */
	static std::string value2name(const ENUMTYPE val)
	{
		std::string s;
		if (!getBimap().direct(val, s))
		{
			throw std::runtime_error(
				std::string("TEnumType<" _MRPT_AUXTOSTR(
					TEnumType) ">::value2name(): Unknown value: ") +
				std::to_string(static_cast<int>(val)));
		}
		return s;
	}

	/** Singleton access */
	static inline internal::bimap<ENUMTYPE, std::string>& getBimap()
	{
		static internal::bimap<ENUMTYPE, std::string> data;
		if (data.m_k2v.empty()) TEnumTypeFiller<ENUMTYPE>::fill(data);
		return data;
	}
#undef _MRPT_AUXTOSTR
};

}  // namespace typemeta
}  // namespace mrpt
