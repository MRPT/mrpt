/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/config.h>  // for HAVE_SINCOS
#include <mrpt/core/bits_math.h>  // for square
#include <mrpt/math/CMatrixDynamic.h>  // for CMatrixD...
#include <mrpt/math/CMatrixF.h>  // for CMatrixF
#include <mrpt/math/CMatrixFixed.h>  // for CMatrixF...
#include <mrpt/math/CQuaternion.h>  // for CQuatern...
#include <mrpt/math/CVectorFixed.h>  // for CArrayDo...
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/geometry.h>  // for skew_sym...
#include <mrpt/math/homog_matrices.h>  // for homogene...
#include <mrpt/math/matrix_serialization.h>  // for operator>>
#include <mrpt/math/ops_containers.h>  // for dotProduct
#include <mrpt/math/utils_matlab.h>
#include <mrpt/math/wrap2pi.h>  // for wrapToPi
#include <mrpt/poses/CPoint2D.h>  // for CPoint2D
#include <mrpt/poses/CPoint3D.h>  // for CPoint3D
#include <mrpt/poses/CPose2D.h>  // for CPose2D
#include <mrpt/poses/CPose3D.h>  // for CPose3D
#include <mrpt/poses/CPose3DQuat.h>  // for CPose3DQuat
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/CSerializable.h>  // for CSeriali...
#include <Eigen/Dense>
#include <algorithm>  // for move
#include <cmath>  // for fabs
#include <iomanip>  // for operator<<
#include <limits>  // for numeric_...
#include <ostream>  // for operator<<
#include <string>  // for allocator

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CPose3D, CSerializable, mrpt::poses)

/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPose3D::CPose3D()
{
	m_coords[0] = m_coords[1] = m_coords[2] = 0;
	m_ROT.setIdentity();
}

CPose3D::CPose3D(
	const double x, const double y, const double z, const double yaw,
	const double pitch, const double roll)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	setFromValues(x, y, z, yaw, pitch, roll);
}

CPose3D::CPose3D(const mrpt::math::TPose3D& o) : m_ypr_uptodate(false)
{
	setFromValues(o.x, o.y, o.z, o.yaw, o.pitch, o.roll);
}

CPose3D::CPose3D(const CPose2D& p) : m_ypr_uptodate(false)
{
	setFromValues(p.x(), p.y(), 0, p.phi(), 0, 0);
}

CPose3D::CPose3D(const CPoint3D& p)
	: m_ypr_uptodate(false), m_yaw(), m_pitch(), m_roll()
{
	setFromValues(p.x(), p.y(), p.z());
}

CPose3D::CPose3D(const math::CMatrixDouble& m)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	ASSERT_GE_(m.rows(), 3);
	ASSERT_GE_(m.cols(), 4);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) m_ROT(r, c) = m(r, c);
	for (int r = 0; r < 3; r++) m_coords[r] = m(r, 3);
}

CPose3D::CPose3D(const math::CMatrixDouble44& m)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) m_ROT(r, c) = m(r, c);
	for (int r = 0; r < 3; r++) m_coords[r] = m(r, 3);
}

/** Constructor from a quaternion (which only represents the 3D rotation part)
 * and a 3D displacement. */
CPose3D::CPose3D(
	const mrpt::math::CQuaternionDouble& q, const double _x, const double _y,
	const double _z)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	double yaw, pitch, roll;
	q.rpy(roll, pitch, yaw);
	this->setFromValues(_x, _y, _z, yaw, pitch, roll);
}

/** Constructor from a quaternion-based full pose. */
CPose3D::CPose3D(const CPose3DQuat& p)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	// Extract XYZ + ROT from quaternion:
	m_coords[0] = p.x();
	m_coords[1] = p.y();
	m_coords[2] = p.z();
	p.quat().rotationMatrixNoResize(m_ROT);
}

uint8_t CPose3D::serializeGetVersion() const { return 3; }
void CPose3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	// v2 serialized the equivalent CPose3DQuat representation.
	// But this led to (**really** tiny) numerical differences between the
	// original and reconstructed poses. To ensure bit-by-bit equivalence before
	// and after serialization, let's get back to serializing the actual SO(3)
	// matrix in serialization v3:
	for (int i = 0; i < 3; i++) out << m_coords[i];
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) out << m_ROT(r, c);
}
void CPose3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// The coordinates:
			CMatrixF HM2;
			in >> HM2;
			ASSERT_(HM2.rows() == 4 && HM2.isSquare());

			m_ROT = HM2.block<3, 3>(0, 0).cast<double>();

			m_coords[0] = HM2(0, 3);
			m_coords[1] = HM2(1, 3);
			m_coords[2] = HM2(2, 3);
		}
		break;
		case 1:
		{
			// The coordinates:
			CMatrixDouble44 HM;
			in >> HM;

			m_ROT = HM.block<3, 3>(0, 0);

			m_coords[0] = HM(0, 3);
			m_coords[1] = HM(1, 3);
			m_coords[2] = HM(2, 3);
		}
		break;
		case 2:
		{
			// An equivalent CPose3DQuat
			CPose3DQuat p(UNINITIALIZED_QUATERNION);
			in >> p[0] >> p[1] >> p[2] >> p[3] >> p[4] >> p[5] >> p[6];

			// Extract XYZ + ROT from quaternion:
			m_coords[0] = p.x();
			m_coords[1] = p.y();
			m_coords[2] = p.z();
			p.quat().rotationMatrixNoResize(m_ROT);
		}
		break;
		case 3:
		{
			for (int i = 0; i < 3; i++) in >> m_coords[i];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++) in >> m_ROT(r, c);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	m_ypr_uptodate = false;
}

void CPose3D::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["x"] = m_coords[0];
	out["y"] = m_coords[1];
	out["z"] = m_coords[2];
	out["rot"] = CMatrixD(m_ROT);
}
void CPose3D::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			m_coords[0] = static_cast<double>(in["x"]);
			m_coords[1] = static_cast<double>(in["y"]);
			m_coords[2] = static_cast<double>(in["z"]);
			CMatrixD m;
			in["rot"].readTo(m);
			m_ROT = m;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator<<(std::ostream& o, const CPose3D& p)
{
	const std::streamsize old_pre = o.precision();
	const std::ios_base::fmtflags old_flags = o.flags();
	o << "(x,y,z,yaw,pitch,roll)=(" << std::fixed << std::setprecision(4)
	  << p.m_coords[0] << "," << p.m_coords[1] << "," << p.m_coords[2] << ","
	  << std::setprecision(2) << RAD2DEG(p.yaw()) << "deg,"
	  << RAD2DEG(p.pitch()) << "deg," << RAD2DEG(p.roll()) << "deg)";
	o.flags(old_flags);
	o.precision(old_pre);
	return o;
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM(mrpt::poses::CPose3D)
#endif

mxArray* CPose3D::writeToMatlab() const
{
#if MRPT_HAS_MATLAB
	const char* fields[] = {"R", "t"};
	mexplus::MxArray pose_struct(
		mexplus::MxArray::Struct(sizeof(fields) / sizeof(fields[0]), fields));
	pose_struct.set("R", mrpt::math::convertToMatlab(this->m_ROT));
	pose_struct.set("t", mrpt::math::convertToMatlab(this->m_coords));
	return pose_struct.release();
#else
	THROW_EXCEPTION("MRPT was built without MEX (Matlab) support!");
#endif
}

/*---------------------------------------------------------------
				normalizeAngles
---------------------------------------------------------------*/
void CPose3D::normalizeAngles() { updateYawPitchRoll(); }
/*---------------------------------------------------------------
 Set the pose from 3D point and yaw/pitch/roll angles, in radians.
---------------------------------------------------------------*/
void CPose3D::setFromValues(
	const double x0, const double y0, const double z0, const double yaw,
	const double pitch, const double roll)
{
	m_coords[0] = x0;
	m_coords[1] = y0;
	m_coords[2] = z0;
	this->m_yaw = mrpt::math::wrapToPi(yaw);
	this->m_pitch = mrpt::math::wrapToPi(pitch);
	this->m_roll = mrpt::math::wrapToPi(roll);

	m_ypr_uptodate = true;

	rebuildRotationMatrix();
}

void CPose3D::rebuildRotationMatrix()
{
	m_ROT = Lie::SO<3>::fromYPR(m_yaw, m_pitch, m_roll);
}

/*---------------------------------------------------------------
		Scalar multiplication.
---------------------------------------------------------------*/
void CPose3D::operator*=(const double s)
{
	updateYawPitchRoll();
	m_coords[0] *= s;
	m_coords[1] *= s;
	m_coords[2] *= s;
	m_yaw *= s;
	m_pitch *= s;
	m_roll *= s;
	rebuildRotationMatrix();
}

/*---------------------------------------------------------------
		getYawPitchRoll
---------------------------------------------------------------*/
void CPose3D::getYawPitchRoll(double& yaw, double& pitch, double& roll) const
{
	TPose3D::SO3_to_yaw_pitch_roll(m_ROT, yaw, pitch, roll);
}

/*---------------------------------------------------------------
		sphericalCoordinates
---------------------------------------------------------------*/
void CPose3D::sphericalCoordinates(
	const TPoint3D& point, double& out_range, double& out_yaw,
	double& out_pitch) const
{
	// Pass to coordinates as seen from this 6D pose:
	TPoint3D local;
	this->inverseComposePoint(
		point.x, point.y, point.z, local.x, local.y, local.z);

	// Range:
	out_range = local.norm();

	// Yaw:
	if (local.y != 0 || local.x != 0)
		out_yaw = atan2(local.y, local.x);
	else
		out_yaw = 0;

	// Pitch:
	if (out_range != 0)
		out_pitch = -asin(local.z / out_range);
	else
		out_pitch = 0;
}

CPose3D CPose3D::getOppositeScalar() const
{
	return CPose3D(
		-m_coords[0], -m_coords[1], -m_coords[2], -m_yaw, -m_pitch, -m_roll);
}

/*---------------------------------------------------------------
		addComponents
---------------------------------------------------------------*/
void CPose3D::addComponents(const CPose3D& p)
{
	updateYawPitchRoll();
	m_coords[0] += p.m_coords[0];
	m_coords[1] += p.m_coords[1];
	m_coords[2] += p.m_coords[2];
	m_yaw += p.m_yaw;
	m_pitch += p.m_pitch;
	m_roll += p.m_roll;
	rebuildRotationMatrix();
}

/*---------------------------------------------------------------
		distanceEuclidean6D
---------------------------------------------------------------*/
double CPose3D::distanceEuclidean6D(const CPose3D& o) const
{
	updateYawPitchRoll();
	o.updateYawPitchRoll();
	return sqrt(
		square(o.m_coords[0] - m_coords[0]) +
		square(o.m_coords[1] - m_coords[1]) +
		square(o.m_coords[2] - m_coords[2]) +
		square(wrapToPi(o.m_yaw - m_yaw)) +
		square(wrapToPi(o.m_pitch - m_pitch)) +
		square(wrapToPi(o.m_roll - m_roll)));
}

/*---------------------------------------------------------------
		composePoint
---------------------------------------------------------------*/
void CPose3D::composePoint(
	double lx, double ly, double lz, double& gx, double& gy, double& gz,
	mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint,
	mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose,
	mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3,
	bool use_small_rot_approx) const
{
	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint) out_jacobian_df_dpoint.value().get() = m_ROT;

	// Jacob: df/dpose
	if (out_jacobian_df_dpose)
	{
		if (use_small_rot_approx)
		{
			// Linearized Jacobians around (yaw,pitch,roll)=(0,0,0):
			alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[3 * 6] = {
				1, 0, 0, -ly, lz, 0, 0, 1, 0, lx, 0, -lz, 0, 0, 1, 0, -lx, ly};
			out_jacobian_df_dpose.value().get().loadFromArray(nums);
		}
		else
		{
			// Exact Jacobians:
			updateYawPitchRoll();
#ifdef HAVE_SINCOS
			double cy, sy;
			::sincos(m_yaw, &sy, &cy);
			double cp, sp;
			::sincos(m_pitch, &sp, &cp);
			double cr, sr;
			::sincos(m_roll, &sr, &cr);
#else
			const double cy = cos(m_yaw);
			const double sy = sin(m_yaw);
			const double cp = cos(m_pitch);
			const double sp = sin(m_pitch);
			const double cr = cos(m_roll);
			const double sr = sin(m_roll);
#endif

			alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[3 * 6] = {
				1,
				0,
				0,
				-lx * sy * cp + ly * (-sy * sp * sr - cy * cr) +
					lz * (-sy * sp * cr + cy * sr),  // d_x'/d_yaw
				-lx * cy * sp + ly * (cy * cp * sr) +
					lz * (cy * cp * cr),  // d_x'/d_pitch
				ly * (cy * sp * cr + sy * sr) +
					lz * (-cy * sp * sr + sy * cr),  // d_x'/d_roll
				0,
				1,
				0,
				lx * cy * cp + ly * (cy * sp * sr - sy * cr) +
					lz * (cy * sp * cr + sy * sr),  // d_y'/d_yaw
				-lx * sy * sp + ly * (sy * cp * sr) +
					lz * (sy * cp * cr),  // d_y'/d_pitch
				ly * (sy * sp * cr - cy * sr) +
					lz * (-sy * sp * sr - cy * cr),  // d_y'/d_roll
				0,
				0,
				1,
				0,  // d_z' / d_yaw
				-lx * cp - ly * sp * sr - lz * sp * cr,  // d_z' / d_pitch
				ly * cp * cr - lz * cp * sr  // d_z' / d_roll
			};
			out_jacobian_df_dpose.value().get().loadFromArray(nums);
		}
	}

	gx = m_ROT(0, 0) * lx + m_ROT(0, 1) * ly + m_ROT(0, 2) * lz + m_coords[0];
	gy = m_ROT(1, 0) * lx + m_ROT(1, 1) * ly + m_ROT(1, 2) * lz + m_coords[1];
	gz = m_ROT(2, 0) * lx + m_ROT(2, 1) * ly + m_ROT(2, 2) * lz + m_coords[2];

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[3 * 6] = {
			1, 0, 0, 0, gz, -gy, 0, 1, 0, -gz, 0, gx, 0, 0, 1, gy, -gx, 0};
		out_jacobian_df_dse3.value().get().loadFromArray(nums);
	}
}

mrpt::math::TVector3D CPose3D::rotateVector(
	const mrpt::math::TVector3D& l) const
{
	mrpt::math::TVector3D g;
	g.x = m_ROT(0, 0) * l.x + m_ROT(0, 1) * l.y + m_ROT(0, 2) * l.z;
	g.y = m_ROT(1, 0) * l.x + m_ROT(1, 1) * l.y + m_ROT(1, 2) * l.z;
	g.z = m_ROT(2, 0) * l.x + m_ROT(2, 1) * l.y + m_ROT(2, 2) * l.z;
	return g;
}

mrpt::math::TVector3D CPose3D::inverseRotateVector(
	const mrpt::math::TVector3D& g) const
{
	mrpt::math::TVector3D l;
	l.x = m_ROT(0, 0) * g.x + m_ROT(1, 0) * g.y + m_ROT(2, 0) * g.z;
	l.y = m_ROT(0, 1) * g.x + m_ROT(1, 1) * g.y + m_ROT(2, 1) * g.z;
	l.z = m_ROT(0, 2) * g.x + m_ROT(1, 2) * g.y + m_ROT(2, 2) * g.z;
	return l;
}

void CPose3D::asVector(vector_t& r) const
{
	updateYawPitchRoll();
	r[0] = m_coords[0];
	r[1] = m_coords[1];
	r[2] = m_coords[2];
	r[3] = m_yaw;
	r[4] = m_pitch;
	r[5] = m_roll;
}

/*---------------------------------------------------------------
		unary -
---------------------------------------------------------------*/
CPose3D mrpt::poses::operator-(const CPose3D& b)
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix(B_INV);
	return CPose3D(B_INV);
}

void CPose3D::getAsQuaternion(
	mrpt::math::CQuaternionDouble& q,
	mrpt::optional_ref<mrpt::math::CMatrixDouble43> out_dq_dr) const
{
	updateYawPitchRoll();
	mrpt::math::TPose3D(0, 0, 0, m_yaw, m_pitch, m_roll)
		.getAsQuaternion(q, out_dq_dr);
}

bool mrpt::poses::operator==(const CPose3D& p1, const CPose3D& p2)
{
	return (p1.m_coords == p2.m_coords) &&
		   ((p1.getRotationMatrix() - p2.getRotationMatrix())
				.array()
				.abs()
				.maxCoeff() < 1e-6);
}

bool mrpt::poses::operator!=(const CPose3D& p1, const CPose3D& p2)
{
	return (p1.m_coords != p2.m_coords) ||
		   ((p1.getRotationMatrix() - p2.getRotationMatrix())
				.array()
				.abs()
				.maxCoeff() >= 1e-6);
}

/*---------------------------------------------------------------
				point3D = pose3D + point3D
  ---------------------------------------------------------------*/
CPoint3D CPose3D::operator+(const CPoint3D& b) const
{
	return CPoint3D(
		m_coords[0] + m_ROT(0, 0) * b.x() + m_ROT(0, 1) * b.y() +
			m_ROT(0, 2) * b.z(),
		m_coords[1] + m_ROT(1, 0) * b.x() + m_ROT(1, 1) * b.y() +
			m_ROT(1, 2) * b.z(),
		m_coords[2] + m_ROT(2, 0) * b.x() + m_ROT(2, 1) * b.y() +
			m_ROT(2, 2) * b.z());
}

/*---------------------------------------------------------------
				point3D = pose3D + point2D
  ---------------------------------------------------------------*/
CPoint3D CPose3D::operator+(const CPoint2D& b) const
{
	return CPoint3D(
		m_coords[0] + m_ROT(0, 0) * b.x() + m_ROT(0, 1) * b.y(),
		m_coords[1] + m_ROT(1, 0) * b.x() + m_ROT(1, 1) * b.y(),
		m_coords[2] + m_ROT(2, 0) * b.x() + m_ROT(2, 1) * b.y());
}

/*---------------------------------------------------------------
				this = A + B
  ---------------------------------------------------------------*/
void CPose3D::composeFrom(const CPose3D& A, const CPose3D& B)
{
	// The translation part HM(0:3,3)
	if (this == &B)
	{
		// we need to make a temporary copy of the vector:
		const CVectorFixedDouble<3> B_coords = B.m_coords;
		for (int r = 0; r < 3; r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r, 0) * B_coords[0] +
						  A.m_ROT(r, 1) * B_coords[1] +
						  A.m_ROT(r, 2) * B_coords[2];
	}
	else
	{
		for (int r = 0; r < 3; r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r, 0) * B.m_coords[0] +
						  A.m_ROT(r, 1) * B.m_coords[1] +
						  A.m_ROT(r, 2) * B.m_coords[2];
	}

	// Important: Make this multiplication AFTER the translational part, to cope
	// with the case when A==this
	m_ROT = A.m_ROT * B.m_ROT;

	m_ypr_uptodate = false;
}

/** Convert this pose into its inverse, saving the result in itself. */
void CPose3D::inverse()
{
	CMatrixDouble33 inv_rot(UNINITIALIZED_MATRIX);
	CVectorFixedDouble<3> inv_xyz;

	mrpt::math::homogeneousMatrixInverse(m_ROT, m_coords, inv_rot, inv_xyz);

	m_ROT = inv_rot;
	m_coords = inv_xyz;
	m_ypr_uptodate = false;
}

/*---------------------------------------------------------------
						isHorizontal
 ---------------------------------------------------------------*/
bool CPose3D::isHorizontal(const double tolerance) const
{
	updateYawPitchRoll();
	return (fabs(m_pitch) <= tolerance || M_PI - fabs(m_pitch) <= tolerance) &&
		   (fabs(m_roll) <= tolerance ||
			fabs(mrpt::math::wrapToPi(m_roll - M_PI)) <= tolerance);
}

/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient
 * than "this= A - B;" since it avoids the temporary object.
 *  \note A or B can be "this" without problems.
 * \sa composeFrom, composePoint
 */
void CPose3D::inverseComposeFrom(const CPose3D& A, const CPose3D& B)
{
	// this    =    A  (-)  B
	// HM_this = inv(HM_B) * HM_A
	//
	// [  R_b  | t_b ] -1   [  R_a  | t_a ]    [ R_b^t * Ra |    ..    ]
	// [ ------+-----]    * [ ------+-----]  = [ ---------- +----------]
	// [ 0 0 0 |  1  ]      [ 0 0 0 |  1  ]    [  0  0   0  |      1   ]
	//

	// XYZ part:
	CMatrixDouble33 R_b_inv(UNINITIALIZED_MATRIX);
	CVectorFixedDouble<3> t_b_inv;
	mrpt::math::homogeneousMatrixInverse(B.m_ROT, B.m_coords, R_b_inv, t_b_inv);

	for (int i = 0; i < 3; i++)
		m_coords[i] = t_b_inv[i] + R_b_inv(i, 0) * A.m_coords[0] +
					  R_b_inv(i, 1) * A.m_coords[1] +
					  R_b_inv(i, 2) * A.m_coords[2];

	// Rot part:
	m_ROT = R_b_inv * A.m_ROT;
	m_ypr_uptodate = false;
}

/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
 * \sa composePoint, composeFrom
 */
void CPose3D::inverseComposePoint(
	const double gx, const double gy, const double gz, double& lx, double& ly,
	double& lz,
	mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint,
	mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose,
	mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3) const
{
	CMatrixDouble33 R_inv(UNINITIALIZED_MATRIX);
	CVectorFixedDouble<3> t_inv;
	mrpt::math::homogeneousMatrixInverse(m_ROT, m_coords, R_inv, t_inv);

	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint) out_jacobian_df_dpoint.value().get() = R_inv;

	// Jacob: df/dpose
	if (out_jacobian_df_dpose)
	{
		// TODO: Perhaps this and the sin/cos's can be avoided if all needed
		// terms are already in m_ROT ???
		updateYawPitchRoll();

#ifdef HAVE_SINCOS
		double cy, sy;
		::sincos(m_yaw, &sy, &cy);
		double cp, sp;
		::sincos(m_pitch, &sp, &cp);
		double cr, sr;
		::sincos(m_roll, &sr, &cr);
#else
		const double cy = cos(m_yaw);
		const double sy = sin(m_yaw);
		const double cp = cos(m_pitch);
		const double sp = sin(m_pitch);
		const double cr = cos(m_roll);
		const double sr = sin(m_roll);
#endif

		const double m11_dy = -sy * cp;
		const double m12_dy = cy * cp;
		const double m13_dy = 0;
		const double m11_dp = -cy * sp;
		const double m12_dp = -sy * sp;
		const double m13_dp = -cp;
		const double m11_dr = 0;
		const double m12_dr = 0;
		const double m13_dr = 0;

		const double m21_dy = (-sy * sp * sr - cy * cr);
		const double m22_dy = (cy * sp * sr - sy * cr);
		const double m23_dy = 0;
		const double m21_dp = (cy * cp * sr);
		const double m22_dp = (sy * cp * sr);
		const double m23_dp = -sp * sr;
		const double m21_dr = (cy * sp * cr + sy * sr);
		const double m22_dr = (sy * sp * cr - cy * sr);
		const double m23_dr = cp * cr;

		const double m31_dy = (-sy * sp * cr + cy * sr);
		const double m32_dy = (cy * sp * cr + sy * sr);
		const double m33_dy = 0;
		const double m31_dp = (cy * cp * cr);
		const double m32_dp = (sy * cp * cr);
		const double m33_dp = -sp * cr;
		const double m31_dr = (-cy * sp * sr + sy * cr);
		const double m32_dr = (-sy * sp * sr - cy * cr);
		const double m33_dr = -cp * sr;

		const double Ax = gx - m_coords[0];
		const double Ay = gy - m_coords[1];
		const double Az = gz - m_coords[2];

		alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[3 * 6] = {
			-m_ROT(0, 0),
			-m_ROT(1, 0),
			-m_ROT(2, 0),
			Ax * m11_dy + Ay * m12_dy + Az * m13_dy,  // d_x'/d_yaw
			Ax * m11_dp + Ay * m12_dp + Az * m13_dp,  // d_x'/d_pitch
			Ax * m11_dr + Ay * m12_dr + Az * m13_dr,  // d_x'/d_roll

			-m_ROT(0, 1),
			-m_ROT(1, 1),
			-m_ROT(2, 1),
			Ax * m21_dy + Ay * m22_dy + Az * m23_dy,  // d_x'/d_yaw
			Ax * m21_dp + Ay * m22_dp + Az * m23_dp,  // d_x'/d_pitch
			Ax * m21_dr + Ay * m22_dr + Az * m23_dr,  // d_x'/d_roll

			-m_ROT(0, 2),
			-m_ROT(1, 2),
			-m_ROT(2, 2),
			Ax * m31_dy + Ay * m32_dy + Az * m33_dy,  // d_x'/d_yaw
			Ax * m31_dp + Ay * m32_dp + Az * m33_dp,  // d_x'/d_pitch
			Ax * m31_dr + Ay * m32_dr + Az * m33_dr,  // d_x'/d_roll
		};
		out_jacobian_df_dpose.value().get().loadFromArray(nums);
	}

	lx = t_inv[0] + R_inv(0, 0) * gx + R_inv(0, 1) * gy + R_inv(0, 2) * gz;
	ly = t_inv[1] + R_inv(1, 0) * gx + R_inv(1, 1) * gy + R_inv(1, 2) * gz;
	lz = t_inv[2] + R_inv(2, 0) * gx + R_inv(2, 1) * gy + R_inv(2, 2) * gz;

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[3 * 6] = {
			-1, 0, 0, 0, -lz, ly, 0, -1, 0, lz, 0, -lx, 0, 0, -1, -ly, lx, 0};
		out_jacobian_df_dse3.value().get().loadFromArray(nums);
	}
}

void CPose3D::setToNaN()
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m_ROT(i, j) = std::numeric_limits<double>::quiet_NaN();

	for (int i = 0; i < 3; i++)
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();
}

mrpt::math::TPose3D CPose3D::asTPose() const
{
	return mrpt::math::TPose3D(x(), y(), z(), yaw(), pitch(), roll());
}

void CPose3D::fromString(const std::string& s)
{
	using mrpt::DEG2RAD;
	mrpt::math::CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(m.rows() == 1 && m.cols() == 6, "Expected vector length=6");
	this->setFromValues(
		m(0, 0), m(0, 1), m(0, 2), DEG2RAD(m(0, 3)), DEG2RAD(m(0, 4)),
		DEG2RAD(m(0, 5)));
}

void CPose3D::fromStringRaw(const std::string& s)
{
	this->fromString("[" + s + "]");
}

void CPose3D::getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const
{
	auto M = out_HM.asEigen();
	M.block<3, 3>(0, 0) = m_ROT.asEigen();
	for (int i = 0; i < 3; i++) out_HM(i, 3) = m_coords[i];
	out_HM(3, 0) = out_HM(3, 1) = out_HM(3, 2) = 0.;
	out_HM(3, 3) = 1.;
}
