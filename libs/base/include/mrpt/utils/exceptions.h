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
#ifndef  exceptions_H
#define  exceptions_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** The base for MRPT-especific exceptions \ingroup mrpt_base_grp */
		class CMRPTException: public std::logic_error
		{
		public:
			CMRPTException(const std::string &s) : std::logic_error(s.c_str()) {  }
		};

		/** Used in mrpt::utils::CImage */
		class CExceptionExternalImageNotFound : public mrpt::utils::CMRPTException
		{
		public:
			CExceptionExternalImageNotFound(const std::string &s) : CMRPTException(s) {  }
		};

		/** Used in mrpt::utils::CStream */
		class CExceptionEOF : public mrpt::utils::CMRPTException
		{
		public:
			CExceptionEOF(const std::string &s) : CMRPTException(s) {  }
		};

	} // End of namespace
} // End of namespace
#endif
