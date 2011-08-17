/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef  mrpt_TEnumType_H
#define  mrpt_TEnumType_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace utils
	{
		/** Only specializations of this class are defined for each enum type of interest
		  * \sa TEnumType \ingroup mrpt_base_grp
		  */
		template <typename ENUMTYPE>
		struct TEnumTypeFiller
		{
			typedef ENUMTYPE enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map);
		};


		/** A helper class that can convert an enum value into its textual representation, and viceversa. \ingroup mrpt_base_grp */
		template <typename ENUMTYPE>
		struct TEnumType
		{
			/** Gives the numerical name for a given enum text name \exception std::exception on unknown enum name */
			static ENUMTYPE    name2value(const std::string &name)
			{
				if (getBimap().empty()) TEnumTypeFiller<ENUMTYPE>::fill(getBimap());
				return getBimap().inverse(name);
			}

			/** Gives the textual name for a given enum value \exception std::exception on unknown enum value name */
			static std::string value2name(const ENUMTYPE val)
			{
				if (getBimap().empty()) TEnumTypeFiller<ENUMTYPE>::fill(getBimap());
				return getBimap().direct(val);
			}

			/** Singleton access */
			static inline bimap<ENUMTYPE,std::string> &getBimap()
			{
				static bimap<ENUMTYPE,std::string> data;
				return data;
			}
		};

	} // End of namespace
} // end of namespace
#endif
