/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <vector>
#include <string>
#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"

namespace mrpt
{
	/** \addtogroup mrpt_simpletypes Simple mrpt types (in #include <mrpt/utils/types_simple.h>)
		*  \ingroup mrpt_base_grp
		* @{ */
	typedef std::vector<int8_t>      vector_signed_byte;
	typedef std::vector<int16_t>     vector_signed_word;
	typedef std::vector<int32_t>     vector_int;
	typedef std::vector<int64_t>     vector_long;
	typedef std::vector<size_t>      vector_size_t;
	typedef std::vector<uint8_t>     vector_byte;
	typedef std::vector<uint16_t>    vector_word;
	typedef std::vector<uint32_t>	 vector_uint;
	typedef std::vector<bool>        vector_bool;	//!<  A type for passing a vector of bools.
	typedef std::vector<std::string> vector_string;	//!<  A type for passing a vector of strings.
	/** @} */

	namespace utils
	{
		/** \addtogroup mrpt_simpletypes
		  * @{ */
		/** For performing type casting from a pointer to its numeric value.
		*/
		#if defined(_MSC_VER) && (_MSC_VER>=1300)
			typedef unsigned long long POINTER_TYPE;
		#else
			typedef unsigned long POINTER_TYPE;
		#endif

		typedef uint64_t TNodeID;  //!< The type for node IDs in graphs of different types.
		typedef std::pair<TNodeID,TNodeID> TPairNodeIDs; //!< A pair of node IDs.
		#define INVALID_NODEID  static_cast<mrpt::utils::TNodeID>(-1)
		/** @} */
	} // end namespace
}
