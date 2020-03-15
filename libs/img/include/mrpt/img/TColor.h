/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>
#include <cstdint>
#include <iosfwd>
#include <iostream>

namespace mrpt::img
{
// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** A RGB color - 8bit. Struct pack=1 is ensured.
 * \ingroup mrpt_img_grp */
struct TColor
{
	constexpr inline TColor() = default;
	constexpr inline TColor(
		uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255)
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

	constexpr inline TColor(
		const unsigned int color_RGB_24bit, const uint8_t alpha)
		: R(uint8_t(color_RGB_24bit >> 16)),
		  G(uint8_t(color_RGB_24bit >> 8)),
		  B(uint8_t(color_RGB_24bit)),
		  A(alpha)
	{
	}

	uint8_t R{0}, G{0}, B{0}, A{255};

	/** Operator for implicit conversion into an int binary representation
	 * 0xRRGGBB */
	inline operator unsigned int() const
	{
		return (((unsigned int)R) << 16) | (((unsigned int)G) << 8) | B;
	}

	TColor(const TColor& other) { *this = other; }
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
#pragma pack(pop)

// Text streaming:
std::ostream& operator<<(std::ostream& o, const TColor& c);
// Binary streaming:
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& o, const TColor& c);
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& i, TColor& c);

// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** An RGBA color - floats in the range [0,1]
 * \ingroup mrpt_img_grp */
struct TColorf
{
	TColorf(float r = 0, float g = 0, float b = 0, float alpha = 1.0f)
		: R(r), G(g), B(b), A(alpha)
	{
	}

	explicit TColorf(const TColor& col)
		: R(u8tof(col.R)), G(u8tof(col.G)), B(u8tof(col.B)), A(u8tof(col.A))
	{
	}

	/** Returns the 0-255 integer version of this color: RGBA_u8  */
	TColor asTColor() const
	{
		return TColor(
			mrpt::f2u8(R), mrpt::f2u8(G), mrpt::f2u8(B), mrpt::f2u8(A));
	}

	float R, G, B, A;
};
#pragma pack(pop)

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
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& o, const TColorf& c);
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& i, TColorf& c);

}  // namespace mrpt::img

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TColor, mrpt::img)
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TColorf, mrpt::img)
}  // namespace mrpt::typemeta
