/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CVectorFixed.h>
#include <mrpt/poses/CPose.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
class CPose3D;
/** A class used to store a 2D pose, including the 2D coordinate point and a
 * heading (phi) angle.
 *  Use this class instead of lightweight mrpt::math::TPose2D when pose/point
 * composition is to be called
 *  multiple times with the same pose, since this class caches calls to
 * expensive trigronometric functions.
 *
 * For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,
 * or refer to [this documentation page]
 *(http://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/)
 *
 *  <div align=center>
 *   <img src="CPose2D.gif">
 *  </div>
 *
 * \note Read also: "A tutorial on SE(3) transformation parameterizations and
 * on-manifold optimization", Jose-Luis Blanco.
 * http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
 * \sa CPoseOrPoint,CPoint2D
 * \ingroup poses_grp
 */
class CPose2D : public CPose<CPose2D, 3>,
				public mrpt::serialization::CSerializable
{
   public:
	DEFINE_SERIALIZABLE(CPose2D)
	DEFINE_SCHEMA_SERIALIZABLE()

   public:
	/** [x,y] */
	mrpt::math::CVectorFixedDouble<2> m_coords;

   protected:
	/** The orientation of the pose, in radians. */
	double m_phi{.0};
	/** Precomputed cos() & sin() of phi. */
	mutable double m_cosphi{1.0}, m_sinphi{.0};
	mutable bool m_cossin_uptodate{false};
	void update_cached_cos_sin() const;

   public:
	/** Default constructor (all coordinates to 0) */
	CPose2D();

	/** Constructor from an initial value of the pose.*/
	CPose2D(const double x, const double y, const double phi);

	/** Returns the identity transformation */
	static CPose2D Identity() { return CPose2D(); }

	/** Constructor from a CPoint2D object. */
	explicit CPose2D(const CPoint2D&);

	/** Aproximation!! Avoid its use, since information is lost. */
	explicit CPose2D(const CPose3D&);

	/** Constructor from lightweight object. */
	explicit CPose2D(const mrpt::math::TPose2D&);

	mrpt::math::TPose2D asTPose() const;

	/** Constructor from CPoint3D with information loss. */
	explicit CPose2D(const CPoint3D&);

	/** Fast constructor that leaves all the data uninitialized - call with
	 * UNINITIALIZED_POSE as argument */
	inline CPose2D(TConstructorFlags_Poses) : m_cossin_uptodate(false) {}
	/** Get the phi angle of the 2D pose (in radians) */
	inline const double& phi() const { return m_phi; }
	//! \overload
	inline double& phi()
	{
		m_cossin_uptodate = false;
		return m_phi;
	}  //-V659

	/** Get a (cached) value of cos(phi), recomputing it only once when phi
	 * changes. */
	inline double phi_cos() const
	{
		update_cached_cos_sin();
		return m_cosphi;
	}
	/** Get a (cached) value of sin(phi), recomputing it only once when phi
	 * changes. */
	inline double phi_sin() const
	{
		update_cached_cos_sin();
		return m_sinphi;
	}

	/** Set the phi angle of the 2D pose (in radians) */
	inline void phi(double angle)
	{
		m_phi = angle;
		m_cossin_uptodate = false;
	}

	/** Increment the PHI angle (without checking the 2 PI range, call
	 * normalizePhi is needed) */
	inline void phi_incr(const double Aphi)
	{
		m_phi += Aphi;
		m_cossin_uptodate = false;
	}

	/** Returns a 1x3 vector with [x y phi] */
	void asVector(vector_t& v) const;

	/** Returns the corresponding 4x4 homogeneous transformation matrix for the
	 * point(translation) or pose (translation+orientation).
	 * \sa getInverseHomogeneousMatrix
	 */
	void getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const;

	/** Returns the SE(2) 2x2 rotation matrix */
	void getRotationMatrix(mrpt::math::CMatrixDouble22& R) const;
	/** Returns the equivalent SE(3) 3x3 rotation matrix, with (2,2)=1. */
	void getRotationMatrix(mrpt::math::CMatrixDouble33& R) const;

	template <class MATRIX22>
	inline MATRIX22 getRotationMatrix() const
	{
		MATRIX22 R;
		getRotationMatrix(R);
		return R;
	}

	/** The operator \f$ a = this \oplus D \f$ is the pose compounding operator.
	 */
	CPose2D operator+(const CPose2D& D) const;

	/** Makes \f$ this = A \oplus B \f$
	 *  \note A or B can be "this" without problems.  */
	void composeFrom(const CPose2D& A, const CPose2D& B);

	/** The operator \f$ a = this \oplus D \f$ is the pose compounding operator.
	 */
	CPose3D operator+(const CPose3D& D) const;

	/** The operator \f$ u' = this \oplus u \f$ is the pose/point compounding
	 * operator.  */
	CPoint2D operator+(const CPoint2D& u) const;

	/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L
	 * \f$ with G and L being 2D points and P this 2D pose.  */
	void composePoint(double lx, double ly, double& gx, double& gy) const;

	/** overload \f$ G = P \oplus L \f$ with G and L being 2D points and P this
	 * 2D pose */
	void composePoint(
		const mrpt::math::TPoint2D& l, mrpt::math::TPoint2D& g) const;

	/// \overload
	mrpt::math::TPoint3D composePoint(const mrpt::math::TPoint3D& l) const;

	/** overload \f$ G = P \oplus L \f$ with G and L being 3D points and P this
	 * 2D pose (the "z" coordinate remains unmodified) */
	void composePoint(
		const mrpt::math::TPoint3D& l, mrpt::math::TPoint3D& g) const;
	/** overload (the "z" coordinate remains unmodified) */
	void composePoint(
		double lx, double ly, double lz, double& gx, double& gy,
		double& gz) const;

	/**  Computes the 2D point L such as \f$ L = G \ominus this \f$. \sa
	 * composePoint, composeFrom */
	void inverseComposePoint(
		const double gx, const double gy, double& lx, double& ly) const;
	/** \overload */
	void inverseComposePoint(
		const mrpt::math::TPoint2D& g, mrpt::math::TPoint2D& l) const;
	/** \overload */
	mrpt::math::TPoint2D inverseComposePoint(
		const mrpt::math::TPoint2D& g) const;

	/** The operator \f$ u' = this \oplus u \f$ is the pose/point compounding
	 * operator. */
	CPoint3D operator+(const CPoint3D& u) const;

	/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient
	 * than "this= A - B;" since it avoids the temporary object.
	 *  \note A or B can be "this" without problems.
	 * \sa composeFrom, composePoint
	 */
	void inverseComposeFrom(const CPose2D& A, const CPose2D& B);

	/** Convert this pose into its inverse, saving the result in itself. \sa
	 * operator- */
	void inverse();

	/** Compute \f$ RET = this \ominus b \f$  */
	inline CPose2D operator-(const CPose2D& b) const
	{
		CPose2D ret(UNINITIALIZED_POSE);
		ret.inverseComposeFrom(*this, b);
		return ret;
	}

	/** The operator \f$ a \ominus b \f$ is the pose inverse compounding
	 * operator. */
	CPose3D operator-(const CPose3D& b) const;

	/** Scalar sum of components: This is diferent from poses
	 *    composition, which is implemented as "+" operators in "CPose" derived
	 * classes.
	 */
	void AddComponents(const CPose2D& p);

	/** Scalar multiplication.
	 */
	void operator*=(const double s);

	/** Make \f$ this = this \oplus b \f$  */
	CPose2D& operator+=(const CPose2D& b);

	/** Forces "phi" to be in the range [-pi,pi];
	 */
	void normalizePhi();

	/** Return the opposite of the current pose instance by taking the negative
	 * of all its components \a individually
	 */
	CPose2D getOppositeScalar() const;

	/** Returns a human-readable textual representation of the object (eg: "[x y
	 * yaw]", yaw in degrees)
	 * \sa fromString
	 */
	void asString(std::string& s) const;
	inline std::string asString() const
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
	/** Same as fromString, but without requiring the square brackets in the
	 * string */
	void fromStringRaw(const std::string& s);

	inline const double& operator[](unsigned int i) const
	{
		switch (i)
		{
			case 0:
				return m_coords[0];
			case 1:
				return m_coords[1];
			case 2:
				return m_phi;
			default:
				throw std::runtime_error(
					"CPose2D::operator[]: Index of bounds.");
		}
	}
	inline double& operator[](unsigned int i)
	{
		switch (i)
		{
			case 0:
				return m_coords[0];
			case 1:
				return m_coords[1];
			case 2:
				return m_phi;
			default:
				throw std::runtime_error(
					"CPose2D::operator[]: Index of bounds.");
		}
	}

	/** makes: this = p (+) this */
	inline void changeCoordinatesReference(const CPose2D& p)
	{
		composeFrom(p, CPose2D(*this));
	}

	/** Returns the 2D distance from this pose/point to a 2D pose using the
	 * Frobenius distance. */
	double distance2DFrobeniusTo(const CPose2D& p) const;

	/** Used to emulate CPosePDF types, for example, in
	 * mrpt::graphs::CNetworkOfPoses */
	using type_value = CPose2D;
	enum
	{
		is_3D_val = 0
	};
	static constexpr bool is_3D() { return is_3D_val != 0; }
	enum
	{
		rotation_dimensions = 2
	};
	enum
	{
		is_PDF_val = 0
	};
	static constexpr bool is_PDF() { return is_PDF_val != 0; }
	inline const type_value& getPoseMean() const { return *this; }
	inline type_value& getPoseMean() { return *this; }
	void setToNaN() override;

	/** @name STL-like methods and typedefs
	   @{   */
	/** The type of the elements */
	using value_type = double;
	using reference = double&;
	using const_reference = const double&;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;

	// size is constant
	enum
	{
		static_size = 3
	};
	static constexpr size_type size() { return static_size; }
	static constexpr bool empty() { return false; }
	static constexpr size_type max_size() { return static_size; }
	static inline void resize(const size_t n)
	{
		if (n != static_size)
			throw std::logic_error(format(
				"Try to change the size of CPose2D to %u.",
				static_cast<unsigned>(n)));
	}

	/** @} */

};  // End of class def.

std::ostream& operator<<(std::ostream& o, const CPose2D& p);

/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same
 * than a pose with negative x y phi) \sa CPose2D::inverse() */
CPose2D operator-(const CPose2D& p);

/** Compose a 2D point from a new coordinate base given by a 2D pose. */
mrpt::math::TPoint2D operator+(
	const CPose2D& pose, const mrpt::math::TPoint2D& pnt);

bool operator==(const CPose2D& p1, const CPose2D& p2);
bool operator!=(const CPose2D& p1, const CPose2D& p2);

}  // namespace mrpt::poses
