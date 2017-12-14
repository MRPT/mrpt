/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CMemoryChunk_H
#define CMemoryChunk_H

#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt
{
namespace utils
{
/** A memory buffer (implements CStream) which can be itself serialized.
 *
 * \sa CStream
 * \ingroup mrpt_base_grp
 */
class CMemoryChunk : public CSerializable, public CMemoryStream
{
	DEFINE_SERIALIZABLE(CMemoryChunk)

};  // End of class def.

}  // End of namespace
}  // end of namespace
#endif
