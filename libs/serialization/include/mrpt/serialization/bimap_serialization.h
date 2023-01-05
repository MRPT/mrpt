/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/bimap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/typemeta/TTypeName_stl.h>  // TTypeName<> for STL templates, needed for serialization of STL templates

namespace mrpt::serialization
{
/** \addtogroup mrpt_serialization_stlext_grp
 * @{ */

/** Template method to serialize an associative STL container  */
template <typename KEY, typename VALUE>
CArchive& operator<<(
	CArchive& out, const mrpt::containers::bimap<KEY, VALUE>& obj)
{
	out << "mrpt::containers::bimap" << mrpt::typemeta::TTypeName<KEY>::get()
		<< mrpt::typemeta::TTypeName<VALUE>::get();
	out.WriteAs<uint32_t>(obj.size());
	for (const auto& kv : obj.getDirectMap())
		out << kv.first << kv.second;
	return out;
}
/** Template method to deserialize an associative STL container */
template <typename KEY, typename VALUE>
CArchive& operator>>(CArchive& in, mrpt::containers::bimap<KEY, VALUE>& obj)
{
	obj.clear();
	std::string pref, stored_K, stored_V;
	in >> pref;
	const auto kType = mrpt::typemeta::TTypeName<KEY>::get();
	const auto mTp = mrpt::typemeta::TTypeName<VALUE>::get();
	if (pref != "mrpt::containers::bimap")
		THROW_EXCEPTION_FMT(
			"Error: serialized container %s<%s,%s>'s preamble is "
			"wrong: '%s'",
			"mrpt::containers::bimap", kType.c_str(), mTp.c_str(),
			pref.c_str());
	in >> stored_K;
	if (stored_K != std::string(kType.c_str()))
		THROW_EXCEPTION_FMT(
			"Error: serialized container %s key type %s != %s",
			"mrpt::containers::bimap", stored_K.c_str(), kType.c_str());
	in >> stored_V;
	if (stored_V != std::string(mTp.c_str()))
		THROW_EXCEPTION_FMT(
			"Error: serialized container %s value type %s != %s",
			"mrpt::containers::bimap", stored_V.c_str(), mTp.c_str());
	const uint32_t n = in.ReadAs<uint32_t>();
	for (uint32_t i = 0; i < n; i++)
	{
		KEY key_obj;
		in >> key_obj;
		VALUE value_obj;
		in >> value_obj;
		obj.insert(key_obj, value_obj);
	}
	return in;
}
/** @} */  // end of grouping
}  // namespace mrpt::serialization
