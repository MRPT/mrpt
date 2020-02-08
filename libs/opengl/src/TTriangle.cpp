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

static_assert(sizeof(TTriangle) == sizeof(float) * 3 * 7, "pack(1) test");

void TTriangle::writeTo(mrpt::serialization::CArchive& o) const
{
	for (int i = 0; i < 3; i++)
		o << vertex[i].pt.x << vertex[i].pt.y << vertex[i].pt.z << vertex[i].R
		  << vertex[i].G << vertex[i].B << vertex[i].A;
}
void TTriangle::readFrom(mrpt::serialization::CArchive& in)
{
	for (int i = 0; i < 3; i++)
		in >> vertex[i].pt.x >> vertex[i].pt.y >> vertex[i].pt.z >>
			vertex[i].R >> vertex[i].G >> vertex[i].B >> vertex[i].A;
}
