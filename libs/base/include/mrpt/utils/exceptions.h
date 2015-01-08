/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  exceptions_H
#define  exceptions_H

#include <stdexcept>
#include <string>

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
