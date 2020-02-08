/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/TTriangle.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::opengl;

template <typename T>
void TTriangle_<T>::writeTo(mrpt::serialization::CArchive& o) const
{
	o.WriteBufferFixEndianness(x, 3);
	o.WriteBufferFixEndianness(y, 3);
	o.WriteBufferFixEndianness(z, 3);

	o.WriteBufferFixEndianness(r, 3);
	o.WriteBufferFixEndianness(g, 3);
	o.WriteBufferFixEndianness(b, 3);
	o.WriteBufferFixEndianness(a, 3);
}
template <typename T>
void TTriangle_<T>::readFrom(mrpt::serialization::CArchive& i)
{
	i.ReadBufferFixEndianness(x, 3);
	i.ReadBufferFixEndianness(y, 3);
	i.ReadBufferFixEndianness(z, 3);

	i.ReadBufferFixEndianness(r, 3);
	i.ReadBufferFixEndianness(g, 3);
	i.ReadBufferFixEndianness(b, 3);
	i.ReadBufferFixEndianness(a, 3);
}

namespace mrpt::opengl
{
// Explicit instantiations:
template class TTriangle_<float>;
template class TTriangle_<double>;
}  // namespace mrpt::opengl
