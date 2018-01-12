/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/CArchive.h>

namespace mrpt
{
namespace io
{
class CMemoryStream;
}
namespace serialization
{
template <class Derived>
class CMemoryChunkBase;

using CMemoryChunk = CMemoryChunkBase<mrpt::io::CMemoryStream>;

/** A memory buffer (implements CStream) which can be itself serialized.
 *
 * \sa CStream
 * \ingroup mrpt_base_grp
 */
template <class Derived>
class CMemoryChunkBase : public CSerializable, public Derived
{
	DEFINE_SERIALIZABLE(CMemoryChunk)
   protected:
	Derived& derived() { return static_cast<Derived&>(*this); }
	const Derived& derived() const
	{
		return static_cast<const Derived&>(*this);
	}
};  // End of class def.

template <class Derived>
uint8_t CMemoryChunkBase<Derived>::serializeGetVersion() const
{
	return 0;
}
template <class Derived>
void CMemoryChunkBase<Derived>::serializeTo(CArchive& out) const
{
	out << static_cast<uint64_t>(derived().getTotalBytesCount());
	if (derived().getTotalBytesCount())
	{
		ASSERT_(derived().getRawBufferData());
		out.WriteBuffer(
			derived().getRawBufferData(), derived().getTotalBytesCount());
	}
}
template <class Derived>
void CMemoryChunkBase<Derived>::serializeFrom(CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint64_t N;
			in >> N;
			derived().changeSize(N);
			derived().Seek(0);
			if (N) in.ReadBuffer(derived().getRawBufferData(), N);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

}  // namespace serialization
}  // namespace mrpt
