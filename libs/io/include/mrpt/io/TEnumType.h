/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

//#include <mrpt/utils/bimap.h>

namespace mrpt
{
namespace io
{
/** Only specializations of this class are defined for each enum type of
 * interest
  * \sa TEnumType \ingroup mrpt_io_grp
  */
template <typename ENUMTYPE>
struct TEnumTypeFiller
{
	typedef ENUMTYPE enum_t;
	static void fill(mrpt::utils::bimap<enum_t, std::string>& m_map);
};

/** For use in specializations of TEnumTypeFiller */
#define MRPT_FILL_ENUM(_X) m_map.insert(_X, #_X)
#define MRPT_FILL_ENUM_MEMBER(_CLASS, _VALUE) \
	m_map.insert(_CLASS::_VALUE, #_VALUE)

/** A helper class that can convert an enum value into its textual
 * representation, and viceversa. \ingroup mrpt_io_grp */
template <typename ENUMTYPE>
struct TEnumType
{
	/** Gives the numerical name for a given enum text name \exception
	 * std::exception on unknown enum name */
	static ENUMTYPE name2value(const std::string& name)
	{
		return getBimap().inverse(name);
	}

	/** Gives the textual name for a given enum value \exception std::exception
	 * on unknown enum value name */
	static std::string value2name(const ENUMTYPE val)
	{
		return getBimap().direct(val);
	}

	/** Singleton access */
	static inline bimap<ENUMTYPE, std::string>& getBimap()
	{
		static bimap<ENUMTYPE, std::string> data;
		if (data.empty()) TEnumTypeFiller<ENUMTYPE>::fill(data);
		return data;
	}
};

}  // End of namespace
}  // end of namespace
#endif
