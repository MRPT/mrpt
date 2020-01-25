/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#include <mrpt/img/TColor.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt::img;
using mrpt::serialization::CArchive;

// Check "#pragma pack(push, 1)" is doing its job:
static_assert(sizeof(TColor) == 4 * sizeof(uint8_t));
static_assert(sizeof(TColorf) == 4 * sizeof(float));

TColor mrpt::img::operator+(const TColor& first, const TColor& second)
{
	TColor ret;
	ret.R = first.R + second.R;
	ret.G = first.G + second.G;
	ret.B = first.B + second.B;
	ret.A = first.A + second.A;

	return ret;
}

TColor mrpt::img::operator-(const TColor& first, const TColor& second)
{
	TColor ret;
	ret.R = first.R - second.R;
	ret.G = first.G - second.G;
	ret.B = first.B - second.B;
	ret.A = first.A - second.A;

	return ret;
}

TColor& TColor::operator+=(const TColor& other)
{
	this->R += other.R;
	this->G += other.G;
	this->B += other.B;
	this->A += other.A;

	return *this;
}

TColor& TColor::operator-=(const TColor& other)
{
	this->R -= other.R;
	this->G -= other.G;
	this->B -= other.B;
	this->A -= other.A;

	return *this;
}

TColor& TColor::operator=(const TColor& other) = default;

bool mrpt::img::operator==(const TColor& first, const TColor& second)
{
	bool ret = first.R == second.R && first.G == second.G &&
			   first.B == second.B && first.A == second.A;

	return ret;
}

// bool operator!=(const TColor& first, const TColor& second) {
// return (!(first == second));
//}

// Text streaming:
std::ostream& mrpt::img::operator<<(std::ostream& o, const TColor& c)
{
	char buf[200];
	mrpt::system::os::sprintf(
		buf, sizeof(buf), "RGBA=[%u,%u,%u,%u]", static_cast<unsigned int>(c.R),
		static_cast<unsigned int>(c.G), static_cast<unsigned int>(c.B),
		static_cast<unsigned int>(c.A));
	o << buf;
	return o;
}

// Binary streaming:
CArchive& mrpt::img::operator<<(CArchive& o, const TColor& c)
{
	o << c.R << c.G << c.B << c.A;
	return o;
}

CArchive& mrpt::img::operator>>(CArchive& i, TColor& c)
{
	i >> c.R >> c.G >> c.B >> c.A;
	return i;
}

// Text streaming:
std::ostream& mrpt::img::operator<<(std::ostream& o, const TColorf& c)
{
	char buf[200];
	mrpt::system::os::sprintf(
		buf, sizeof(buf), "RGBAf=[%f,%f,%f,%f]", c.R, c.G, c.B, c.A);
	o << buf;
	return o;
}

// Binary streaming:
CArchive& mrpt::img::operator<<(CArchive& o, const TColorf& c)
{
	o << c.R << c.G << c.B << c.A;
	return o;
}

CArchive& mrpt::img::operator>>(CArchive& i, TColorf& c)
{
	i >> c.R >> c.G >> c.B >> c.A;
	return i;
}
