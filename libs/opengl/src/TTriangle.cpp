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

// packet size= 3 vertices, each XYZ (float) + RGBA (u8)
static_assert(sizeof(TTriangle) == (sizeof(float) * 3 + 4) * 3, "pack(1) test");

void TTriangle::writeTo(mrpt::serialization::CArchive& o) const
{
	for (int i = 0; i < 3; i++)
		o << vertex[i].pt.x << vertex[i].pt.y << vertex[i].pt.z << vertex[i].r
		  << vertex[i].g << vertex[i].b << vertex[i].a;
}
void TTriangle::readFrom(mrpt::serialization::CArchive& in)
{
	for (int i = 0; i < 3; i++)
		in >> vertex[i].pt.x >> vertex[i].pt.y >> vertex[i].pt.z >>
			vertex[i].r >> vertex[i].g >> vertex[i].b >> vertex[i].a;
}
