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
#include <mrpt/core/format.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <cmath>  // sqrt

namespace mrpt::math
{
/** Lightweight 3D point (float version).
 * \sa mrpt::poses::CPoint3D, mrpt::math::TPoint3D
 */
struct TPoint3Df : public TPoseOrPoint,
				   public internal::ProvideStaticResize<TPoint3Df>
{
	enum
	{
		static_size = 3
	};
	float x{.0f}, y{.0f}, z{.0f};

	TPoint3Df() = default;
	constexpr TPoint3Df(const float xx, const float yy, const float zz)
		: x(xx), y(yy), z(zz)
	{
	}
	TPoint3Df& operator+=(const TPoint3Df& p)
	{
		x += p.x;
		y += p.y;
		z += p.z;
		return *this;
	}
	TPoint3Df operator*(const float s)
	{
		return TPoint3Df(x * s, y * s, z * s);
	}
	/** Coordinate access using operator[]. Order: x,y,z */
	float& operator[](size_t i)
	{
		switch (i)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			default:
				throw std::out_of_range("index out of range");
		}
	}

	/** Coordinate access using operator[]. Order: x,y,z */
	constexpr const float& operator[](size_t i) const
	{
		switch (i)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			default:
				throw std::out_of_range("index out of range");
		}
	}
};

/** Trivially copiable underlying data for TPoint3D */
struct TPoint3D_data
{
	/** X,Y,Z coordinates */
	double x, y, z;
};

/**
 * Lightweight 3D point. Allows coordinate access using [] operator.
 * \sa mrpt::poses::CPoint3D, mrpt::math::TPoint3Df
 */
struct TPoint3D : public TPoseOrPoint,
				  public TPoint3D_data,
				  public internal::ProvideStaticResize<TPoint3D>
{
	enum
	{
		static_size = 3
	};

	/** Default constructor. Initializes to zeros. */
	constexpr TPoint3D() : TPoint3D_data{0, 0, 0} {}
	/** Constructor from coordinates.  */
	constexpr TPoint3D(double xx, double yy, double zz)
		: TPoint3D_data{xx, yy, zz}
	{
	}
	constexpr TPoint3D(const TPoint3D_data& d) : TPoint3D_data{d.x, d.y, d.z} {}

	/** Explicit constructor from coordinates.  */
	explicit TPoint3D(const TPoint3Df& p)
	{
		x = static_cast<double>(p.x);
		y = static_cast<double>(p.y);
		z = static_cast<double>(p.z);
	}
	/** Implicit constructor from TPoint2D. Zeroes the z.
	 * \sa TPoint2D
	 */
	TPoint3D(const TPoint2D& p);
	/**
	 * Constructor from TPose2D, losing information. Zeroes the z.
	 * \sa TPose2D
	 */
	explicit TPoint3D(const TPose2D& p);
	/**
	 * Constructor from TPose3D, losing information.
	 * \sa TPose3D
	 */
	explicit TPoint3D(const TPose3D& p);
	/** Coordinate access using operator[]. Order: x,y,z */
	double& operator[](size_t i)
	{
		switch (i)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/** Coordinate access using operator[]. Order: x,y,z */
	constexpr const double& operator[](size_t i) const
	{
		switch (i)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/**
	 * Point-to-point distance.
	 */
	double distanceTo(const TPoint3D& p) const
	{
		return sqrt(square(p.x - x) + square(p.y - y) + square(p.z - z));
	}
	/**
	 * Point-to-point distance, squared.
	 */
	double sqrDistanceTo(const TPoint3D& p) const
	{
		return square(p.x - x) + square(p.y - y) + square(p.z - z);
	}
	/** Squared norm: |v|^2 = x^2+y^2+z^2 */
	double sqrNorm() const { return x * x + y * y + z * z; }

	/** Point norm: |v| = sqrt(x^2+y^2+z^2) */
	double norm() const { return std::sqrt(sqrNorm()); }

	/** Scale point/vector */
	TPoint3D& operator*=(const double f)
	{
		x *= f;
		y *= f;
		z *= f;
		return *this;
	}
	/**
	 * Transformation into vector.
	 */
	template <class VECTORLIKE>
	void asVector(VECTORLIKE& v) const
	{
		v.resize(3);
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}
	/**
	 * Translation.
	 */
	TPoint3D& operator+=(const TPoint3D& p)
	{
		x += p.x;
		y += p.y;
		z += p.z;
		return *this;
	}
	/**
	 * Difference between points.
	 */
	TPoint3D& operator-=(const TPoint3D& p)
	{
		x -= p.x;
		y -= p.y;
		z -= p.z;
		return *this;
	}
	/**
	 * Points addition.
	 */
	constexpr TPoint3D operator+(const TPoint3D& p) const
	{
		return {x + p.x, y + p.y, z + p.z};
	}
	/**
	 * Points substraction.
	 */
	constexpr TPoint3D operator-(const TPoint3D& p) const
	{
		return {x - p.x, y - p.y, z - p.z};
	}

	constexpr TPoint3D operator*(double d) const
	{
		return {x * d, y * d, z * d};
	}

	constexpr TPoint3D operator/(double d) const
	{
		return {x / d, y / d, z / d};
	}

	bool operator<(const TPoint3D& p) const;

	/** Returns a human-readable textual representation of the object (eg:
	 * "[0.02 1.04 -0.8]" )
	 * \sa fromString
	 */
	void asString(std::string& s) const
	{
		s = mrpt::format("[%f %f %f]", x, y, z);
	}
	std::string asString() const
	{
		std::string s;
		asString(s);
		return s;
	}

	/** Set the current object value from a string generated by 'asString' (eg:
	 * "[0.02 1.04 -0.8]" )
	 * \sa asString
	 * \exception std::exception On invalid format
	 */
	void fromString(const std::string& s);

	static TPoint3D FromString(const std::string& s)
	{
		TPoint3D o;
		o.fromString(s);
		return o;
	}
};

/** Useful type alias for 3-vectors */
using TVector3D = TPoint3D;

/** Useful type alias for 2-vectors */
using TVector2D = TPoint2D;

/** XYZ point (double) + Intensity(u8) \sa mrpt::math::TPoint3D */
struct TPointXYZIu8
{
	mrpt::math::TPoint3D pt;
	uint8_t intensity{0};
	TPointXYZIu8() : pt() {}
	constexpr TPointXYZIu8(double x, double y, double z, uint8_t intensity_val)
		: pt(x, y, z), intensity(intensity_val)
	{
	}
};
/** XYZ point (double) + RGB(u8) \sa mrpt::math::TPoint3D */
struct TPointXYZRGBu8
{
	mrpt::math::TPoint3D pt;
	uint8_t R{0}, G{0}, B{0};
	TPointXYZRGBu8() = default;
	constexpr TPointXYZRGBu8(
		double x, double y, double z, uint8_t R_val, uint8_t G_val,
		uint8_t B_val)
		: pt(x, y, z), R(R_val), G(G_val), B(B_val)
	{
	}
};
/** XYZ point (float) + Intensity(u8) \sa mrpt::math::TPoint3D */
struct TPointXYZfIu8
{
	mrpt::math::TPoint3Df pt;
	uint8_t intensity{0};
	TPointXYZfIu8() = default;
	constexpr TPointXYZfIu8(float x, float y, float z, uint8_t intensity_val)
		: pt(x, y, z), intensity(intensity_val)
	{
	}
};
/** XYZ point (float) + RGB(u8) \sa mrpt::math::TPoint3D */
struct TPointXYZfRGBu8
{
	mrpt::math::TPoint3Df pt;
	uint8_t R{0}, G{0}, B{0};
	TPointXYZfRGBu8() : pt() {}
	constexpr TPointXYZfRGBu8(
		float x, float y, float z, uint8_t R_val, uint8_t G_val, uint8_t B_val)
		: pt(x, y, z), R(R_val), G(G_val), B(B_val)
	{
	}
};

/** Unary minus operator for 3D points. */
constexpr TPoint3D operator-(const TPoint3D& p1)
{
	return {-p1.x, -p1.y, -p1.z};
}

/** Exact comparison between 3D points */
constexpr bool operator==(const TPoint3D& p1, const TPoint3D& p2)
{
	return (p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z);  //-V550
}
/** Exact comparison between 3D points */
constexpr bool operator!=(const TPoint3D& p1, const TPoint3D& p2)
{
	return (p1.x != p2.x) || (p1.y != p2.y) || (p1.z != p2.z);  //-V550
}

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPoint3D, mrpt::math)
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPoint3Df, mrpt::math)
}  // namespace mrpt::typemeta
