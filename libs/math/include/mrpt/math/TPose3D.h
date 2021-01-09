/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/wrap2pi.h>

namespace mrpt::math
{
/**
 * Lightweight 3D pose (three spatial coordinates, plus three angular
 * coordinates). Allows coordinate access using [] operator.
 * \sa mrpt::poses::CPose3D
 * \ingroup geometry_grp
 */
struct TPose3D : public TPoseOrPoint,
				 public internal::ProvideStaticResize<TPose3D>
{
	enum
	{
		static_size = 6
	};
	/** X,Y,Z, coords */
	double x{.0}, y{.0}, z{.0};
	/** Yaw coordinate (rotation angle over Z axis). */
	double yaw{.0};
	/** Pitch coordinate (rotation angle over Y axis). */
	double pitch{.0};
	/** Roll coordinate (rotation angle over X coordinate). */
	double roll{.0};

	/** Returns the identity transformation, T=eye(4) */
	static constexpr TPose3D Identity() { return TPose3D(); }

	/** Implicit constructor from TPoint2D. Zeroes all the unprovided
	 * information.
	 * \sa TPoint2D
	 */
	TPose3D(const TPoint2D& p);
	/**
	 * Implicit constructor from TPose2D. Gets the yaw from the 2D pose's phi,
	 * zeroing all the unprovided information.
	 * \sa TPose2D
	 */
	TPose3D(const TPose2D& p);
	/**
	 * Implicit constructor from TPoint3D. Zeroes angular information.
	 * \sa TPoint3D
	 */
	TPose3D(const TPoint3D& p);
	/**
	 * Constructor from coordinates.
	 */
	constexpr TPose3D(
		double _x, double _y, double _z, double _yaw, double _pitch,
		double _roll)
		: x(_x), y(_y), z(_z), yaw(_yaw), pitch(_pitch), roll(_roll)
	{
	}
	/**
	 * Default fast constructor. Initializes to zeros.
	 */
	constexpr TPose3D() = default;
	/** Coordinate access using operator[]. Order: x,y,z,yaw,pitch,roll */
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
			case 3:
				return yaw;
			case 4:
				return pitch;
			case 5:
				return roll;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/** Coordinate access using operator[]. Order: x,y,z,yaw,pitch,roll */
	constexpr double operator[](size_t i) const
	{
		switch (i)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			case 3:
				return yaw;
			case 4:
				return pitch;
			case 5:
				return roll;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/**
	 * Pose's spatial coordinates norm.
	 */
	double norm() const { return std::sqrt(square(x) + square(y) + square(z)); }
	/**
	 * Gets the pose as a vector of doubles.
	 */
	void asVector(std::vector<double>& v) const
	{
		v.resize(6);
		v[0] = x;
		v[1] = y;
		v[2] = z;
		v[3] = yaw;
		v[4] = pitch;
		v[5] = roll;
	}
	/** Returns a human-readable textual representation of the object (eg: "[x y
	 * z yaw pitch roll]", angles in degrees.)
	 * \sa fromString
	 */
	void asString(std::string& s) const;
	std::string asString() const
	{
		std::string s;
		asString(s);
		return s;
	}

	/** Returns the quaternion associated to the rotation of this object (NOTE:
	 * XYZ translation is ignored)
	 * \f[ \mathbf{q} = \left( \begin{array}{c} \cos (\phi /2) \cos (\theta /2)
	 * \cos (\psi /2) +  \sin (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \sin
	 * (\phi /2) \cos (\theta /2) \cos (\psi /2) -  \cos (\phi /2) \sin (\theta
	 * /2) \sin (\psi /2) \\ \cos (\phi /2) \sin (\theta /2) \cos (\psi /2) +
	 * \sin (\phi /2) \cos (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \cos
	 * (\theta /2) \sin (\psi /2) -  \sin (\phi /2) \sin (\theta /2) \cos (\psi
	 * /2) \\ \end{array}\right) \f]
	 * With : \f$ \phi = roll \f$,  \f$ \theta = pitch \f$ and \f$ \psi = yaw
	 * \f$.
	 * \param out_dq_dr  If provided, the 4x3 Jacobian of the transformation
	 * will be computed and stored here. It's the Jacobian of the transformation
	 * from (yaw pitch roll) to (qr qx qy qz).
	 */
	void getAsQuaternion(
		mrpt::math::CQuaternion<double>& q,
		mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 4, 3>> out_dq_dr =
			std::nullopt) const;

	void composePoint(const TPoint3D& l, TPoint3D& g) const;
	TPoint3D composePoint(const TPoint3D& l) const;
	void inverseComposePoint(const TPoint3D& g, TPoint3D& l) const;
	TPoint3D inverseComposePoint(const TPoint3D& g) const;
	void composePose(const TPose3D other, TPose3D& result) const;

	/** Operator "oplus" pose composition: "ret=this \oplus b"  \sa CPose3D
	 * \note [Added in MRPT 2.1.5] */
	mrpt::math::TPose3D operator+(const mrpt::math::TPose3D& b) const
	{
		mrpt::math::TPose3D ret;
		this->composePose(b, ret);
		return ret;
	}

	void getRotationMatrix(mrpt::math::CMatrixDouble33& R) const;
	inline mrpt::math::CMatrixDouble33 getRotationMatrix() const
	{
		mrpt::math::CMatrixDouble33 R;
		getRotationMatrix(R);
		return R;
	}
	void getHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const;
	inline mrpt::math::CMatrixDouble44 getHomogeneousMatrix() const
	{
		mrpt::math::CMatrixDouble44 H;
		getHomogeneousMatrix(H);
		return H;
	}
	void getInverseHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const;
	mrpt::math::CMatrixDouble44 getInverseHomogeneousMatrix() const
	{
		mrpt::math::CMatrixDouble44 H;
		getInverseHomogeneousMatrix(H);
		return H;
	}
	void fromHomogeneousMatrix(const mrpt::math::CMatrixDouble44& HG);
	static void SO3_to_yaw_pitch_roll(
		const mrpt::math::CMatrixDouble33& R, double& yaw, double& pitch,
		double& roll);
	/** Set the current object value from a string generated by 'asString' (eg:
	 * "[0.02 1.04 -0.8]" )
	 * \sa asString
	 * \exception std::exception On invalid format
	 */
	void fromString(const std::string& s);

	static TPose3D FromString(const std::string& s)
	{
		TPose3D o;
		o.fromString(s);
		return o;
	}
};

/** Unary $\ominus\$ operator: computes inverse SE(3) element */
TPose3D operator-(const TPose3D& p);
/** Binary $\ominus\$ operator: \$b \ominus a\$ computes the relative SE(3) pose
 * of `b` "as seen from" `a` */
TPose3D operator-(const TPose3D& b, const TPose3D& a);

/** Exact comparison between 3D poses, taking possible cycles into account */
inline bool operator==(const TPose3D& p1, const TPose3D& p2)
{
	return (p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z) &&
		   (mrpt::math::wrapTo2Pi(p1.yaw) == mrpt::math::wrapTo2Pi(p2.yaw)) &&
		   (mrpt::math::wrapTo2Pi(p1.pitch) ==
			mrpt::math::wrapTo2Pi(p2.pitch)) &&
		   (mrpt::math::wrapTo2Pi(p1.roll) ==
			mrpt::math::wrapTo2Pi(p2.roll));  //-V550
}
/** Exact comparison between 3D poses, taking possible cycles into account */
inline bool operator!=(const TPose3D& p1, const TPose3D& p2)
{
	return (p1.x != p2.x) || (p1.y != p2.y) || (p1.z != p2.z) ||
		   (mrpt::math::wrapTo2Pi(p1.yaw) != mrpt::math::wrapTo2Pi(p2.yaw)) ||
		   (mrpt::math::wrapTo2Pi(p1.pitch) !=
			mrpt::math::wrapTo2Pi(p2.pitch)) ||
		   (mrpt::math::wrapTo2Pi(p1.roll) !=
			mrpt::math::wrapTo2Pi(p2.roll));  //-V550
}

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPose3D, mrpt::math)
}  // namespace mrpt::typemeta
