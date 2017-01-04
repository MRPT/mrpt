/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CMemoryChunk_H
#define  CMemoryChunk_H

#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace utils
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CMemoryChunk, mrpt::utils::CSerializable )

		/** A memory buffer (implements CStream) which can be itself serialized.
		 *
		 * \sa CStream
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CMemoryChunk : public CSerializable, public CMemoryStream
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CMemoryChunk )

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CMemoryChunk, mrpt::utils::CSerializable )

	} // End of namespace
} // end of namespace
#endif
