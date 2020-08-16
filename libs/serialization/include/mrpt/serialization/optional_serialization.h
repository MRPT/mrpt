/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CArchive.h>
#include <mrpt/typemeta/TTypeName_stl.h>  // TTypeName<> for STL templates, needed for serialization of STL templates
#include <optional>

namespace mrpt::serialization
{
/** \addtogroup mrpt_serialization_stlext_grp
 * @{ */

/** Template to serialize a std::optional<T> */
template <class T>
CArchive& operator<<(CArchive& out, const std::optional<T>& obj)
{
	out << std::string("std::optional") << mrpt::typemeta::TTypeName<T>::get();
	out << obj.has_value();
	if (obj.has_value()) out << *obj;
	return out;
}

/** Template to deserialize a std::optional<T> */
template <class T>
CArchive& operator>>(CArchive& in, std::optional<T>& obj)
{
	std::string pref, stored_T;
	in >> pref;
	if (pref != "std::optional")
		THROW_EXCEPTION(format(
			"Error: serialized std::optional<%s>'s preamble is wrong: '%s'",
			mrpt::typemeta::TTypeName<T>::get().c_str(), pref.c_str()));
	in >> stored_T;
	if (stored_T != std::string(mrpt::typemeta::TTypeName<T>::get().c_str()))
		THROW_EXCEPTION(format(
			"Error: serialized std::optional type %s != %s", stored_T.c_str(),
			mrpt::typemeta::TTypeName<T>::get().c_str()));

	bool has_value;
	in >> has_value;
	if (has_value)
	{
		T val;
		in >> val;
		obj = std::move(val);
	}
	else
	{
		obj.reset();
	}
	return in;
}

/** @} */  // end of grouping
}  // namespace mrpt::serialization
