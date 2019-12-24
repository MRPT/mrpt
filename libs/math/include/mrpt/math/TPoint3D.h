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
/** Trivially copiable underlying data for TPoint3D */
template <typename T>
struct TPoint3D_data
{
	/** X,Y,Z coordinates */
	T x, y, z;
};

/** Base template for TPoint3D and TPoint3Df
 * \ingroup geometry_grp
 */
template <typename T>
struct TPoint3D_ : public TPoseOrPoint,
				   public TPoint3D_data<T>,
				   public internal::ProvideStaticResize<TPoint3D_<T>>
{
	enum
	{
		static_size = 3
	};

	/** Default constructor. Initializes to zeros. */
	constexpr TPoint3D_() : TPoint3D_data<T>{0, 0, 0} {}
	/** Constructor from coordinates.  */
	constexpr TPoint3D_(T xx, T yy, T zz) : TPoint3D_data<T>{xx, yy, zz} {}

	/** Explicit constructor from coordinates.  */
	template <typename U>
	explicit TPoint3D_(const TPoint3D_data<U>& p)
	{
		TPoint3D_data<T>::x = static_cast<T>(p.x);
		TPoint3D_data<T>::y = static_cast<T>(p.y);
		TPoint3D_data<T>::z = static_cast<T>(p.z);
	}

	/** Implicit constructor from TPoint2D. Zeroes the z.
	 * \sa TPoint2D
	 */
	TPoint3D_(const TPoint2D_<T>& p);
	/**
	 * Constructor from TPose2D, losing information. Zeroes the z.
	 * \sa TPose2D
	 */
	explicit TPoint3D_(const TPose2D& p);
	/**
	 * Constructor from TPose3D, losing information.
	 * \sa TPose3D
	 */
	explicit TPoint3D_(const TPose3D& p);

	/** Return a copy of this object using type U for coordinates */
	template <typename U>
	TPoint3D_<U> cast() const
	{
		return TPoint3D_<U>(
			static_cast<U>(this->x), static_cast<U>(this->y),
			static_cast<U>(this->z));
	}

	/** Coordinate access using operator[]. Order: x,y,z */
	T& operator[](size_t i)
	{
		switch (i)
		{
			case 0:
				return TPoint3D_data<T>::x;
			case 1:
				return TPoint3D_data<T>::y;
			case 2:
				return TPoint3D_data<T>::z;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/** Coordinate access using operator[]. Order: x,y,z */
	constexpr T operator[](size_t i) const
	{
		switch (i)
		{
			case 0:
				return TPoint3D_data<T>::x;
			case 1:
				return TPoint3D_data<T>::y;
			case 2:
				return TPoint3D_data<T>::z;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/**
	 * Point-to-point distance.
	 */
	T distanceTo(const TPoint3D_<T>& p) const
	{
		return std::sqrt(
			square(p.x - TPoint3D_data<T>::x) +
			square(p.y - TPoint3D_data<T>::y) +
			square(p.z - TPoint3D_data<T>::z));
	}
	/**
	 * Point-to-point distance, squared.
	 */
	T sqrDistanceTo(const TPoint3D_<T>& p) const
	{
		return square(p.x - TPoint3D_data<T>::x) +
			   square(p.y - TPoint3D_data<T>::y) +
			   square(p.z - TPoint3D_data<T>::z);
	}
	/** Squared norm: |v|^2 = x^2+y^2+z^2 */
	T sqrNorm() const
	{
		return square(TPoint3D_data<T>::x) + square(TPoint3D_data<T>::y) +
			   square(TPoint3D_data<T>::z);
	}

	/** Point norm: |v| = sqrt(x^2+y^2+z^2) */
	T norm() const { return std::sqrt(sqrNorm()); }

	/** Scale point/vector */
	TPoint3D_<T>& operator*=(const T f)
	{
		TPoint3D_data<T>::x *= f;
		TPoint3D_data<T>::y *= f;
		TPoint3D_data<T>::z *= f;
		return *this;
	}
	/**
	 * Transformation into vector.
	 */
	template <class VECTORLIKE>
	void asVector(VECTORLIKE& v) const
	{
		v.resize(3);
		v[0] = TPoint3D_data<T>::x;
		v[1] = TPoint3D_data<T>::y;
		v[2] = TPoint3D_data<T>::z;
	}
	/**
	 * Translation.
	 */
	TPoint3D_<T>& operator+=(const TPoint3D_<T>& p)
	{
		TPoint3D_data<T>::x += p.x;
		TPoint3D_data<T>::y += p.y;
		TPoint3D_data<T>::z += p.z;
		return *this;
	}
	/**
	 * Difference between points.
	 */
	TPoint3D_<T>& operator-=(const TPoint3D_<T>& p)
	{
		TPoint3D_data<T>::x -= p.x;
		TPoint3D_data<T>::y -= p.y;
		TPoint3D_data<T>::z -= p.z;
		return *this;
	}
	/**
	 * Points addition.
	 */
	constexpr TPoint3D_<T> operator+(const TPoint3D_<T>& p) const
	{
		return {TPoint3D_data<T>::x + p.x, TPoint3D_data<T>::y + p.y,
				TPoint3D_data<T>::z + p.z};
	}
	/**
	 * Points substraction.
	 */
	constexpr TPoint3D_<T> operator-(const TPoint3D_<T>& p) const
	{
		return {TPoint3D_data<T>::x - p.x, TPoint3D_data<T>::y - p.y,
				TPoint3D_data<T>::z - p.z};
	}

	constexpr TPoint3D_<T> operator*(T d) const
	{
		return {TPoint3D_data<T>::x * d, TPoint3D_data<T>::y * d,
				TPoint3D_data<T>::z * d};
	}

	constexpr TPoint3D_<T> operator/(T d) const
	{
		return {TPoint3D_data<T>::x / d, TPoint3D_data<T>::y / d,
				TPoint3D_data<T>::z / d};
	}

	bool operator<(const TPoint3D_<T>& p) const;

	/** Returns a human-readable textual representation of the object (eg:
	 * "[0.02 1.04 -0.8]" )
	 * \sa fromString
	 */
	void asString(std::string& s) const
	{
		s = mrpt::format(
			"[%f %f %f]", TPoint3D_data<T>::x, TPoint3D_data<T>::y,
			TPoint3D_data<T>::z);
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

	static TPoint3D_<T> FromString(const std::string& s)
	{
		TPoint3D_<T> o;
		o.fromString(s);
		return o;
	}
};

/**
 * Lightweight 3D point. Allows coordinate access using [] operator.
 * \sa mrpt::poses::CPoint3D, mrpt::math::TPoint3Df
 */
using TPoint3D = TPoint3D_<double>;
using TPoint3Df = TPoint3D_<float>;

/** Useful type alias for 3-vectors */
using TVector3D = TPoint3D;
using TVector3Df = TPoint3Df;

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
template <typename T>
constexpr TPoint3D_<T> operator-(const TPoint3D_<T>& p1)
{
	return {-p1.x, -p1.y, -p1.z};
}

/** Exact comparison between 3D points */
template <typename T>
constexpr bool operator==(const TPoint3D_<T>& p1, const TPoint3D_<T>& p2)
{
	return (p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z);  //-V550
}
/** Exact comparison between 3D points */
template <typename T>
constexpr bool operator!=(const TPoint3D_<T>& p1, const TPoint3D_<T>& p2)
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
