/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef mrpt_utils_tcolor_H
#define mrpt_utils_tcolor_H

#include <cstdint>
#include <iosfwd>
#include <iostream>

namespace mrpt
{
namespace img
{
class CStream;

/** A RGB color - 8bit
 * \ingroup mrpt_img_grp */
struct TColor
{
	constexpr inline TColor() : R(0), G(0), B(0), A(255) {}
	constexpr inline TColor(uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255)
		: R(r), G(g), B(b), A(alpha)
	{
	}

	constexpr inline explicit TColor(const unsigned int color_RGB_24bit)
		: R(uint8_t(color_RGB_24bit >> 16)),
		  G(uint8_t(color_RGB_24bit >> 8)),
		  B(uint8_t(color_RGB_24bit)),
		  A(255)
	{
	}

	constexpr inline TColor(const unsigned int color_RGB_24bit, const uint8_t alpha)
		: R(uint8_t(color_RGB_24bit >> 16)),
		  G(uint8_t(color_RGB_24bit >> 8)),
		  B(uint8_t(color_RGB_24bit)),
		  A(alpha)
	{
	}

	uint8_t R, G, B, A;

	/** Operator for implicit conversion into an int binary representation
	 * 0xRRGGBB */
	inline operator unsigned int(void) const
	{
		return (((unsigned int)R) << 16) | (((unsigned int)G) << 8) | B;
	}

	TColor& operator=(const TColor& other);
	TColor& operator+=(const TColor& other);
	TColor& operator-=(const TColor& other);

	/** Predefined colors */
	static constexpr TColor red() { return TColor(255, 0, 0); }
	static constexpr TColor green() { return TColor(0, 255, 0); }
	static constexpr TColor blue() { return TColor(0, 0, 255); }
	static constexpr TColor black() { return TColor(0, 0, 0); }
	static constexpr TColor white() { return TColor(255, 255, 255); }
	static constexpr TColor gray() { return TColor(127, 127, 127); }
};
// Text streaming:
std::ostream& operator<<(std::ostream& o, const TColor& c);
// Binary streaming:
CStream& operator<<(mrpt::utils::CStream& o, const TColor& c);
CStream& operator>>(mrpt::utils::CStream& i, TColor& c);

/** A RGB color - floats in the range [0,1]
 * \ingroup mrpt_img_grp */
struct TColorf
{
	TColorf(float r = 0, float g = 0, float b = 0, float alpha = 1.0f)
		: R(r), G(g), B(b), A(alpha)
	{
	}

	explicit TColorf(const TColor& col)
		: R(col.R * (1.f / 255)),
		  G(col.G * (1.f / 255)),
		  B(col.B * (1.f / 255)),
		  A(col.A * (1.f / 255))
	{
	}

	float R, G, B, A;
};

/**\brief Pairwise addition of their corresponding RGBA members
 */
TColor operator+(const TColor& first, const TColor& second);
/**\brief Pairwise substraction of their corresponding RGBA members
 */
TColor operator-(const TColor& first, const TColor& second);
bool operator==(const TColor& first, const TColor& second);
// bool operator!=(const TColor& first, const TColor& second);

// Text streaming:
std::ostream& operator<<(std::ostream& o, const TColorf& c);
// Binary streaming:
CStream& operator<<(mrpt::utils::CStream& o, const TColorf& c);
CStream& operator>>(mrpt::utils::CStream& i, TColorf& c);

}  // end namespace
}  // end of namespace

#endif
