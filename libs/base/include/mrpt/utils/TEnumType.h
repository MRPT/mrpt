/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
