/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose.h>
#include <mrpt/system/string_utils.h>

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::poses::CPose3D)

namespace mrpt::poses
{
class CPose3DQuat;

/** A class used to store a 3D pose (a 3D translation + a rotation in 3D).
 *   The 6D transformation in SE(3) stored in this class is kept in two
 *   separate containers: a 3-array for the translation, and a 3x3 rotation
 * matrix.
 *
 *  This class allows parameterizing 6D poses as a 6-vector: [x y z yaw pitch
 * roll] (read below
 *   for the angles convention). Note however,
 *   that the yaw/pitch/roll angles are only computed (on-demand and
 * transparently)
 *   when the user requests them. Normally, rotations and transformations are
 * always handled
 *   via the 3x3 rotation matrix.
 *
 *  Yaw/Pitch/Roll angles are defined as successive rotations around *local*
 * (dynamic) axes in the Z/Y/X order:
 *
 *  <div align=center>
 *   <img src="CPose3D.gif">
 *  </div>
 *
 * It may be extremely confusing and annoying to find a different criterion also
 * involving
 * the names "yaw, pitch, roll" but regarding rotations around *global* (static)
 * axes.
 * Fortunately, it's very easy to see (by writing down the product of the three
 * rotation matrices) that both conventions lead to exactly the same numbers.
 * Only, that it's conventional to write the numbers in reverse order.
 * That is, the same rotation can be described equivalently with any of these
 * two
 * parameterizations:
 *
 * - In local axes Z/Y/X convention: [yaw pitch roll]   (This is the convention
 * used in mrpt::poses::CPose3D)
 * - In global axes X/Y/Z convention: [roll pitch yaw] (One of the Euler angles
 * conventions)
 *
 * For further descriptions of point & pose classes, see
 * mrpt::poses::CPoseOrPoint or refer
 * to the [2D/3D Geometry tutorial](http://www.mrpt.org/2D_3D_Geometry) online.
 *
 * To change the individual components of the pose, use CPose3D::setFromValues.
 * This class assures that the internal
 * 3x3 rotation matrix is always up-to-date with the "yaw pitch roll" members.
 *
 * Rotations in 3D can be also represented by quaternions. See
 * mrpt::math::CQuaternion, and method CPose3D::getAsQuaternion.
 *
 * This class and CPose3DQuat are very similar, and they can be converted to the
 * each other automatically via transformation constructors.
 *
 * For Lie algebra methods, see mrpt::poses::Lie.
 *
 * \note Read also: "A tutorial on SE(3) transformation parameterizations and
 * on-manifold optimization", in \cite blanco_se3_tutorial
 *
 * \ingroup poses_grp
 * \sa CPoseOrPoint,CPoint3D, mrpt::math::CQuaternion
 */
class CPose3D : public CPose<CPose3D, 6>,
				public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CPose3D, mrpt::poses)
	DEFINE_SCHEMA_SERIALIZABLE()

	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	/** The translation vector [x,y,z] access directly or with x(), y(), z()
	 * setter/getter methods. */
	mrpt::math::CVectorFixedDouble<3> m_coords;

   protected:
	/** The 3x3 rotation matrix, access with getRotationMatrix(),
	 * setRotationMatrix() (It's not safe to set this field as public) */
	mrpt::math::CMatrixDouble33 m_ROT;

	/** Whether yaw/pitch/roll members are up-to-date since the last rotation
	 * matrix update. */
	mutable bool m_ypr_uptodate{false};
	/** These variables are updated every time that the object rotation matrix
	 * is modified (construction, loading from values, pose composition, etc )
	 */
	mutable double m_yaw{0}, m_pitch{0}, m_roll{0};

	/** Rebuild the homog matrix from the angles. */
	void rebuildRotationMatrix();

	/** Updates Yaw/pitch/roll members from the m_ROT  */
	inline void updateYawPitchRoll() const
	{
		if (!m_ypr_uptodate)
		{
			m_ypr_uptodate = true;
			getYawPitchRoll(m_yaw, m_pitch, m_roll);
		}
	}

   public:
	/** @name Constructors
		@{ */

	/** Default constructor, with all the coordinates set to zero. */
	CPose3D();

	/** Constructor with initilization of the pose; (remember that angles are
	 * always given in radians!)  */
	CPose3D(
		const double x, const double y, const double z, const double yaw = 0,
		const double pitch = 0, const double roll = 0);

	/** Returns the identity transformation */
	static CPose3D Identity() { return CPose3D(); }

	/** Constructor from a 4x4 homogeneous matrix - the passed matrix can be
	 * actually of any size larger than or equal 3x4, since only those first
	 * values are used (the last row of a homogeneous 4x4 matrix are always
	 * fixed). */
	explicit CPose3D(const math::CMatrixDouble& m);

	/** Constructor from a 4x4 homogeneous matrix: */
	explicit CPose3D(const math::CMatrixDouble44& m);

	/** Constructor from a 3x3 rotation matrix and a the translation given as a
	 * 3-vector, a 3-array, a CPoint3D or a mrpt::math::TPoint3D */
	template <class MATRIX33, class VECTOR3>
	inline CPose3D(const MATRIX33& rot, const VECTOR3& xyz)
		: m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
	{
		ASSERT_EQUAL_(rot.rows(), 3);
		ASSERT_EQUAL_(rot.cols(), 3);
		ASSERT_EQUAL_(xyz.size(), 3);
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++) m_ROT(r, c) = rot(r, c);
		for (int r = 0; r < 3; r++) m_coords[r] = xyz[r];
	}
	//! \overload
	inline CPose3D(
		const mrpt::math::CMatrixDouble33& rot,
		const mrpt::math::CVectorFixedDouble<3>& xyz)
		: m_coords(xyz), m_ROT(rot), m_ypr_uptodate(false)
	{
	}

	/** Constructor from a CPose2D object.
	 */
	explicit CPose3D(const CPose2D&);

	/** Constructor from a CPoint3D object.
	 */
	explicit CPose3D(const CPoint3D&);

	/** Constructor from lightweight object.
	 */
	explicit CPose3D(const mrpt::math::TPose3D&);

	mrpt::math::TPose3D asTPose() const;

	/** Constructor from a quaternion (which only represents the 3D rotation
	 * part) and a 3D displacement. */
	CPose3D(
		const mrpt::math::CQuaternionDouble& q, const double x, const double y,
		const double z);

	/** Constructor from a CPose3DQuat. */
	explicit CPose3D(const CPose3DQuat&);

	/** Fast constructor that leaves all the data uninitialized - call with
	 * UNINITIALIZED_POSE as argument */
	inline CPose3D(TConstructorFlags_Poses)
		: m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
	{
	}

	/** Constructor from an array with these 12 elements: [r11 r21 r31 r12 r22
	 * r32 r13 r23 r33 tx ty tz]
	 *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
	 * the 3D translation of the pose
	 *  \sa setFrom12Vector, getAs12Vector
	 */
	inline explicit CPose3D(const mrpt::math::CVectorFixedDouble<12>& vec12)
		: m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
	{
		setFrom12Vector(vec12);
	}

	/** @} */  // end Constructors

	/** @name Access 3x3 rotation and 4x4 homogeneous matrices
		@{ */

	/** Returns the corresponding 4x4 homogeneous transformation matrix for the
	 * point(translation) or pose (translation+orientation).
	 * \sa getInverseHomogeneousMatrix, getRotationMatrix
	 */
	void getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const;

	/** Get the 3x3 rotation matrix \sa getHomogeneousMatrix  */
	inline void getRotationMatrix(mrpt::math::CMatrixDouble33& ROT) const
	{
		ROT = m_ROT;
	}
	//! \overload
	inline const mrpt::math::CMatrixDouble33& getRotationMatrix() const
	{
		return m_ROT;
	}

	/** Sets the 3x3 rotation matrix \sa getRotationMatrix, getHomogeneousMatrix
	 */
	inline void setRotationMatrix(const mrpt::math::CMatrixDouble33& ROT)
	{
		m_ROT = ROT;
		m_ypr_uptodate = false;
	}

	/** @} */  // end rot and HM

	/** @name Pose-pose and pose-point compositions and operators
		@{ */

	/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
	inline CPose3D operator+(const CPose3D& b) const
	{
		CPose3D ret(UNINITIALIZED_POSE);
		ret.composeFrom(*this, b);
		return ret;
	}

	/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
	CPoint3D operator+(const CPoint3D& b) const;

	/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
	CPoint3D operator+(const CPoint2D& b) const;

	/** Computes the spherical coordinates of a 3D point as seen from the 6D
	 * pose specified by this object. For the coordinate system see the top of
	 * this page. */
	void sphericalCoordinates(
		const mrpt::math::TPoint3D& point, double& out_range, double& out_yaw,
		double& out_pitch) const;

	/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L
	 * \f$ with G and L being 3D points and P this 6D pose.
	 *  If pointers are provided, the corresponding Jacobians are returned.
	 *  "out_jacobian_df_dse3" stands for the Jacobian with respect to the 6D
	 * locally Euclidean vector in the tangent space of SE(3).
	 *  See [this
	 * report](http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf)
	 * for mathematical details.
	 *  \param  If set to true, the Jacobian "out_jacobian_df_dpose" uses a
	 * fastest linearized appoximation (valid only for small rotations!).
	 */
	void composePoint(
		double lx, double ly, double lz, double& gx, double& gy, double& gz,
		mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint =
			std::nullopt,
		mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose =
			std::nullopt,
		mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3 =
			std::nullopt,
		bool use_small_rot_approx = false) const;

	/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L
	 * \f$ with G and L being 3D points and P this 6D pose.
	 * \note local_point is passed by value to allow global and local point to
	 * be the same variable
	 */
	inline void composePoint(
		const mrpt::math::TPoint3D& local_point,
		mrpt::math::TPoint3D& global_point) const
	{
		composePoint(
			local_point.x, local_point.y, local_point.z, global_point.x,
			global_point.y, global_point.z);
	}
	/** \overload Returns global point: "this \oplus l" */
	inline mrpt::math::TPoint3D composePoint(
		const mrpt::math::TPoint3D& l) const
	{
		mrpt::math::TPoint3D g;
		composePoint(l, g);
		return g;
	}

	/** This version of the method assumes that the resulting point has no Z
	 * component (use with caution!) */
	inline void composePoint(
		const mrpt::math::TPoint3D& local_point,
		mrpt::math::TPoint2D& global_point) const
	{
		double dummy_z;
		composePoint(
			local_point.x, local_point.y, local_point.z, global_point.x,
			global_point.y, dummy_z);
	}

	/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L
	 * \f$ with G and L being 3D points and P this 6D pose.  */
	inline void composePoint(
		double lx, double ly, double lz, float& gx, float& gy, float& gz) const
	{
		double ggx, ggy, ggz;
		composePoint(lx, ly, lz, ggx, ggy, ggz);
		gx = d2f(ggx);
		gy = d2f(ggy);
		gz = d2f(ggz);
	}

	/** Rotates a vector (i.e. like composePoint(), but ignoring translation) */
	mrpt::math::TVector3D rotateVector(
		const mrpt::math::TVector3D& local) const;

	/** Inverse of rotateVector(), i.e. using the inverse rotation matrix */
	mrpt::math::TVector3D inverseRotateVector(
		const mrpt::math::TVector3D& global) const;

	/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
	 *  If pointers are provided, the corresponding Jacobians are returned.
	 *  "out_jacobian_df_dse3" stands for the Jacobian with respect to the 6D
	 * locally Euclidean vector in the tangent space of SE(3).
	 *  See [this
	 * report](http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf)
	 * for mathematical details.
	 * \sa composePoint, composeFrom
	 */
	void inverseComposePoint(
		const double gx, const double gy, const double gz, double& lx,
		double& ly, double& lz,
		mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint =
			std::nullopt,
		mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose =
			std::nullopt,
		mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3 =
			std::nullopt) const;

	/** \overload */
	inline void inverseComposePoint(
		const mrpt::math::TPoint3D& g, mrpt::math::TPoint3D& l) const
	{
		inverseComposePoint(g.x, g.y, g.z, l.x, l.y, l.z);
	}
	/** \overload Returns local point: `g` as seen from `this` pose */
	inline mrpt::math::TPoint3D inverseComposePoint(
		const mrpt::math::TPoint3D& g) const
	{
		mrpt::math::TPoint3D l;
		inverseComposePoint(g, l);
		return l;
	}

	/** overload for 2D points \exception If the z component of the result is
	 * greater than some epsilon */
	inline void inverseComposePoint(
		const mrpt::math::TPoint2D& g, mrpt::math::TPoint2D& l,
		const double eps = 1e-6) const
	{
		double lz;
		inverseComposePoint(g.x, g.y, 0, l.x, l.y, lz);
		ASSERT_LT_(std::abs(lz), eps);
	}

	/**  Makes "this = A (+) B"; this method is slightly more efficient than
	 * "this= A + B;" since it avoids the temporary object.
	 *  \note A or B can be "this" without problems.
	 */
	void composeFrom(const CPose3D& A, const CPose3D& B);

	/** Make \f$ this = this \oplus b \f$  (\a b can be "this" without problems)
	 */
	inline CPose3D& operator+=(const CPose3D& b)
	{
		composeFrom(*this, b);
		return *this;
	}

	/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient
	 * than "this= A - B;" since it avoids the temporary object.
	 *  \note A or B can be "this" without problems.
	 * \sa composeFrom, composePoint
	 */
	void inverseComposeFrom(const CPose3D& A, const CPose3D& B);

	/** Compute \f$ RET = this \oplus b \f$  */
	inline CPose3D operator-(const CPose3D& b) const
	{
		CPose3D ret(UNINITIALIZED_POSE);
		ret.inverseComposeFrom(*this, b);
		return ret;
	}

	/** Convert this pose into its inverse, saving the result in itself. \sa
	 * operator- */
	void inverse();

	/** makes: this = p (+) this */
	inline void changeCoordinatesReference(const CPose3D& p)
	{
		composeFrom(p, CPose3D(*this));
	}

	/** @} */  // compositions

	/** Return the opposite of the current pose instance by taking the negative
	 * of all its components \a individually
	 */
	CPose3D getOppositeScalar() const;

	/** @name Access and modify contents
		@{ */

	/** Scalar sum of all 6 components: This is diferent from poses composition,
	 * which is implemented as "+" operators.
	 * \sa normalizeAngles
	 */
	void addComponents(const CPose3D& p);

	/** Rebuild the internal matrix & update the yaw/pitch/roll angles within
	 * the ]-PI,PI] range (Must be called after using addComponents)
	 * \sa addComponents
	 */
	void normalizeAngles();

	/** Scalar multiplication of x,y,z,yaw,pitch & roll (angles will be wrapped
	 * to the ]-pi,pi] interval). */
	void operator*=(const double s);

	/** Set the pose from a 3D position (meters) and yaw/pitch/roll angles
	 * (radians) - This method recomputes the internal rotation matrix.
	 * \sa getYawPitchRoll, setYawPitchRoll
	 */
	void setFromValues(
		const double x0, const double y0, const double z0, const double yaw = 0,
		const double pitch = 0, const double roll = 0);

	/** Set the pose from a 3D position (meters) and a quaternion, stored as [x
	 * y z qr qx qy qz] in a 7-element vector.
	 * \sa setFromValues, getYawPitchRoll, setYawPitchRoll, CQuaternion,
	 * getAsQuaternion
	 */
	template <typename VECTORLIKE>
	inline void setFromXYZQ(const VECTORLIKE& v, const size_t index_offset = 0)
	{
		ASSERT_GE_(v.size(), 7 + index_offset);
		// The 3x3 rotation part:
		mrpt::math::CQuaternion<typename VECTORLIKE::value_type> q(
			v[index_offset + 3], v[index_offset + 4], v[index_offset + 5],
			v[index_offset + 6]);
		q.rotationMatrixNoResize(m_ROT);
		m_ypr_uptodate = false;
		m_coords[0] = v[index_offset + 0];
		m_coords[1] = v[index_offset + 1];
		m_coords[2] = v[index_offset + 2];
	}

	/** Set the 3 angles of the 3D pose (in radians) - This method recomputes
	 * the internal rotation coordinates matrix.
	 * \sa getYawPitchRoll, setFromValues
	 */
	inline void setYawPitchRoll(
		const double yaw_, const double pitch_, const double roll_)
	{
		setFromValues(x(), y(), z(), yaw_, pitch_, roll_);
	}

	/** Set pose from an array with these 12 elements: [r11 r21 r31 r12 r22 r32
	 * r13 r23 r33 tx ty tz]
	 *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
	 * the 3D translation of the pose
	 *  \sa getAs12Vector
	 */
	template <class ARRAYORVECTOR>
	inline void setFrom12Vector(const ARRAYORVECTOR& vec12)
	{
		m_ROT(0, 0) = vec12[0];
		m_ROT(0, 1) = vec12[3];
		m_ROT(0, 2) = vec12[6];
		m_ROT(1, 0) = vec12[1];
		m_ROT(1, 1) = vec12[4];
		m_ROT(1, 2) = vec12[7];
		m_ROT(2, 0) = vec12[2];
		m_ROT(2, 1) = vec12[5];
		m_ROT(2, 2) = vec12[8];
		m_ypr_uptodate = false;
		m_coords[0] = vec12[9];
		m_coords[1] = vec12[10];
		m_coords[2] = vec12[11];
	}

	/** Get the pose representation as an array with these 12 elements: [r11 r21
	 * r31 r12 r22 r32 r13 r23 r33 tx ty tz]
	 *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
	 * the 3D translation of the pose
	 *  \sa setFrom12Vector
	 */
	template <class ARRAYORVECTOR>
	inline void getAs12Vector(ARRAYORVECTOR& vec12) const
	{
		vec12[0] = m_ROT(0, 0);
		vec12[3] = m_ROT(0, 1);
		vec12[6] = m_ROT(0, 2);
		vec12[1] = m_ROT(1, 0);
		vec12[4] = m_ROT(1, 1);
		vec12[7] = m_ROT(1, 2);
		vec12[2] = m_ROT(2, 0);
		vec12[5] = m_ROT(2, 1);
		vec12[8] = m_ROT(2, 2);
		vec12[9] = m_coords[0];
		vec12[10] = m_coords[1];
		vec12[11] = m_coords[2];
	}

	/** Returns the three angles (yaw, pitch, roll), in radians, from the
	 * rotation matrix.
	 * \sa setFromValues, yaw, pitch, roll
	 */
	void getYawPitchRoll(double& yaw, double& pitch, double& roll) const;

	/** Get the YAW angle (in radians)  \sa setFromValues */
	inline double yaw() const
	{
		updateYawPitchRoll();
		return m_yaw;
	}
	/** Get the PITCH angle (in radians) \sa setFromValues */
	inline double pitch() const
	{
		updateYawPitchRoll();
		return m_pitch;
	}
	/** Get the ROLL angle (in radians) \sa setFromValues */
	inline double roll() const
	{
		updateYawPitchRoll();
		return m_roll;
	}

	/** Returns a 6x1 vector with [x y z yaw pitch roll]' */
	void asVector(vector_t& v) const;

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
		mrpt::math::CQuaternionDouble& q,
		mrpt::optional_ref<mrpt::math::CMatrixDouble43> out_dq_dr =
			std::nullopt) const;

	inline double operator[](unsigned int i) const
	{
		updateYawPitchRoll();
		switch (i)
		{
			case 0:
				return m_coords[0];
			case 1:
				return m_coords[1];
			case 2:
				return m_coords[2];
			case 3:
				return m_yaw;
			case 4:
				return m_pitch;
			case 5:
				return m_roll;
			default:
				throw std::runtime_error(
					"CPose3D::operator[]: Index of bounds.");
		}
	}
	// CPose3D CANNOT have a write [] operator, since it'd leave the object in
	// an inconsistent state (outdated rotation matrix).
	// Use setFromValues() instead.
	// inline double &operator[](unsigned int i)

	/** Returns a human-readable textual representation of the object (eg: "[x y
	 * z yaw pitch roll]", angles in degrees.)
	 * \sa fromString
	 */
	void asString(std::string& s) const
	{
		using mrpt::RAD2DEG;
		updateYawPitchRoll();
		s = mrpt::format(
			"[%f %f %f %f %f %f]", m_coords[0], m_coords[1], m_coords[2],
			RAD2DEG(m_yaw), RAD2DEG(m_pitch), RAD2DEG(m_roll));
	}
	inline std::string asString() const
	{
		std::string s;
		asString(s);
		return s;
	}

	/** Set the current object value from a string generated by 'asString' (eg:
	 * "[x y z yaw pitch roll]", angles in deg. )
	 * \sa asString
	 * \exception std::exception On invalid format
	 */
	void fromString(const std::string& s);

	/** Same as fromString, but without requiring the square brackets in the
	 * string */
	void fromStringRaw(const std::string& s);

	static CPose3D FromString(const std::string& s)
	{
		CPose3D o;
		o.fromString(s);
		return o;
	}

	/** Return true if the 6D pose represents a Z axis almost exactly vertical
	 * (upwards or downwards), with a given tolerance (if set to 0 exact
	 * horizontality is tested). */
	bool isHorizontal(const double tolerance = 0) const;

	/** The euclidean distance between two poses taken as two 6-length vectors
	 * (angles in radians). */
	double distanceEuclidean6D(const CPose3D& o) const;

	/** @} */  // modif. components

	void setToNaN() override;

	/** Used to emulate CPosePDF types, for example, in
	 * mrpt::graphs::CNetworkOfPoses */
	using type_value = CPose3D;
	enum
	{
		is_3D_val = 1
	};
	static constexpr bool is_3D() { return is_3D_val != 0; }
	enum
	{
		rotation_dimensions = 3
	};
	enum
	{
		is_PDF_val = 0
	};
	static constexpr bool is_PDF() { return is_PDF_val != 0; }
	inline const type_value& getPoseMean() const { return *this; }
	inline type_value& getPoseMean() { return *this; }
	/** @name STL-like methods and typedefs
	   @{   */
	/** The type of the elements */
	using value_type = double;
	using reference = double&;
	using const_reference = double;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;

	// size is constant
	enum
	{
		static_size = 6
	};
	static constexpr size_type size() { return static_size; }
	static constexpr bool empty() { return false; }
	static constexpr size_type max_size() { return static_size; }
	static inline void resize(const size_t n)
	{
		if (n != static_size)
			throw std::logic_error(format(
				"Try to change the size of CPose3D to %u.",
				static_cast<unsigned>(n)));
	}
	/** @} */

};  // End of class def.

std::ostream& operator<<(std::ostream& o, const CPose3D& p);

/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same
 * than a pose with negative x y z yaw pitch roll) */
CPose3D operator-(const CPose3D& p);

bool operator==(const CPose3D& p1, const CPose3D& p2);
bool operator!=(const CPose3D& p1, const CPose3D& p2);

}  // namespace mrpt::poses
