/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/config.h>  // for HAVE_SINCOS
#include <mrpt/math/types_math.h>  // for CVectorD...
#include <mrpt/math/CMatrix.h>  // for CMatrix
#include <mrpt/math/geometry.h>  // for skew_sym...
#include <mrpt/math/matrix_serialization.h>  // for operator>>
#include <mrpt/math/wrap2pi.h>  // for wrapToPi
#include <mrpt/poses/CPoint2D.h>  // for CPoint2D
#include <mrpt/poses/CPoint3D.h>  // for CPoint3D
#include <mrpt/poses/CPose2D.h>  // for CPose2D
#include <mrpt/poses/CPose3D.h>  // for CPose3D
#include <mrpt/poses/CPose3DQuat.h>  // for CPose3DQuat
#include <mrpt/poses/CPose3DRotVec.h>  // for CPose3DR...
#include <mrpt/serialization/CArchive.h>
#include <algorithm>  // for move
#include <cmath>  // for fabs
#include <iomanip>  // for operator<<
#include <limits>  // for numeric_...
#include <ostream>  // for operator<<
#include <string>  // for allocator
#include <mrpt/math/CArrayNumeric.h>  // for CArrayDo...
#include <mrpt/math/CMatrixFixedNumeric.h>  // for CMatrixF...
#include <mrpt/math/CMatrixTemplateNumeric.h>  // for CMatrixD...
#include <mrpt/math/CQuaternion.h>  // for CQuatern...
#include <mrpt/math/homog_matrices.h>  // for homogene...
#include <mrpt/math/lightweight_geom_data.h>  // for TPoint3D
#include <mrpt/math/ops_containers.h>  // for dotProduct
#include <mrpt/serialization/CSerializable.h>  // for CSeriali...
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/core/bits_math.h>  // for square
#include <mrpt/math/utils_matlab.h>
#include <mrpt/otherlibs/sophus/so3.hpp>
#include <mrpt/otherlibs/sophus/se3.hpp>

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
	m_ROT.unit(3, 1.0);
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
	ASSERT_ABOVEEQ_(m.rows(), 3);
	ASSERT_ABOVEEQ_(m.cols(), 4);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) m_ROT(r, c) = m.get_unsafe(r, c);
	for (int r = 0; r < 3; r++) m_coords[r] = m.get_unsafe(r, 3);
}

CPose3D::CPose3D(const math::CMatrixDouble44& m)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) m_ROT(r, c) = m.get_unsafe(r, c);
	for (int r = 0; r < 3; r++) m_coords[r] = m.get_unsafe(r, 3);
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

/** Constructor from a rotation vector-based full pose. */
CPose3D::CPose3D(const CPose3DRotVec& p)
	: m_ROT(UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
{
	m_coords[0] = p.m_coords[0];
	m_coords[1] = p.m_coords[1];
	m_coords[2] = p.m_coords[2];

	this->setRotationMatrix(this->exp_rotation(p.m_rotvec));
}

uint8_t CPose3D::serializeGetVersion() const { return 2; }
void CPose3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	const CPose3DQuat q(*this);
	// The coordinates:
	out << q[0] << q[1] << q[2] << q[3] << q[4] << q[5] << q[6];
}
void CPose3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// The coordinates:
			CMatrix HM2;
			in >> HM2;
			ASSERT_(HM2.rows() == 4 && HM2.isSquare());

			m_ROT = HM2.block(0, 0, 3, 3).cast<double>();

			m_coords[0] = HM2.get_unsafe(0, 3);
			m_coords[1] = HM2.get_unsafe(1, 3);
			m_coords[2] = HM2.get_unsafe(2, 3);
			m_ypr_uptodate = false;
		}
		break;
		case 1:
		{
			// The coordinates:
			CMatrixDouble44 HM;
			in >> HM;

			m_ROT = HM.block(0, 0, 3, 3);

			m_coords[0] = HM.get_unsafe(0, 3);
			m_coords[1] = HM.get_unsafe(1, 3);
			m_coords[2] = HM.get_unsafe(2, 3);
			m_ypr_uptodate = false;
		}
		break;
		case 2:
		{
			// An equivalent CPose3DQuat
			CPose3DQuat p(UNINITIALIZED_QUATERNION);
			in >> p[0] >> p[1] >> p[2] >> p[3] >> p[4] >> p[5] >> p[6];

			// Extract XYZ + ROT from quaternion:
			m_ypr_uptodate = false;
			m_coords[0] = p.x();
			m_coords[1] = p.y();
			m_coords[2] = p.z();
			p.quat().rotationMatrixNoResize(m_ROT);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
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
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
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

/*---------------------------------------------------------------
 Set the pose from 3D point and yaw/pitch/roll angles, in radians.
---------------------------------------------------------------*/
void CPose3D::rebuildRotationMatrix()
{
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

	alignas(MRPT_MAX_ALIGN_BYTES)
		const double rot_vals[] = {cy * cp,
								   cy * sp * sr - sy * cr,
								   cy * sp * cr + sy * sr,
								   sy * cp,
								   sy * sp * sr + cy * cr,
								   sy * sp * cr - cy * sr,
								   -sp,
								   cp * sr,
								   cp * cr};
	m_ROT.loadFromArray(rot_vals);
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
	mrpt::math::CMatrixFixedNumeric<double, 3, 3>* out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double, 3, 6>* out_jacobian_df_dpose,
	mrpt::math::CMatrixFixedNumeric<double, 3, 6>* out_jacobian_df_dse3,
	bool use_small_rot_approx) const
{
	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint) *out_jacobian_df_dpoint = m_ROT;

	// Jacob: df/dpose
	if (out_jacobian_df_dpose)
	{
		if (use_small_rot_approx)
		{
			// Linearized Jacobians around (yaw,pitch,roll)=(0,0,0):
			alignas(MRPT_MAX_ALIGN_BYTES) const double nums[3 * 6] = {
				1, 0, 0, -ly, lz, 0, 0, 1, 0, lx, 0, -lz, 0, 0, 1, 0, -lx, ly};
			out_jacobian_df_dpose->loadFromArray(nums);
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

			alignas(MRPT_MAX_ALIGN_BYTES) const double nums[3 * 6] = {
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
			out_jacobian_df_dpose->loadFromArray(nums);
		}
	}

	gx = m_ROT(0, 0) * lx + m_ROT(0, 1) * ly + m_ROT(0, 2) * lz + m_coords[0];
	gy = m_ROT(1, 0) * lx + m_ROT(1, 1) * ly + m_ROT(1, 2) * lz + m_coords[1];
	gz = m_ROT(2, 0) * lx + m_ROT(2, 1) * ly + m_ROT(2, 2) * lz + m_coords[2];

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		alignas(MRPT_MAX_ALIGN_BYTES) const double nums[3 * 6] = {
			1, 0, 0, 0, gz, -gy, 0, 1, 0, -gz, 0, gx, 0, 0, 1, gy, -gx, 0};
		out_jacobian_df_dse3->loadFromArray(nums);
	}
}

// TODO: Use SSE2? OTOH, this forces mem align...
#if MRPT_HAS_SSE2 && defined(MRPT_USE_SSE2)
/*static inline __m128 transformSSE(const __m128* matrix, const __m128& in)
{
	ASSERT_(((size_t)matrix & 15) == 0);
	__m128 a0 = _mm_mul_ps(_mm_load_ps((float*)(matrix+0)),
_mm_shuffle_ps(in,in,_MM_SHUFFLE(0,0,0,0)));
	__m128 a1 = _mm_mul_ps(_mm_load_ps((float*)(matrix+1)),
_mm_shuffle_ps(in,in,_MM_SHUFFLE(1,1,1,1)));
	__m128 a2 = _mm_mul_ps(_mm_load_ps((float*)(matrix+2)),
_mm_shuffle_ps(in,in,_MM_SHUFFLE(2,2,2,2)));

	return _mm_add_ps(_mm_add_ps(a0,a1),a2);
}*/
#endif  // SSE2

/*---------------------------------------------------------------
		getAsVector
---------------------------------------------------------------*/
void CPose3D::getAsVector(CVectorDouble& r) const
{
	updateYawPitchRoll();
	r.resize(6);
	r[0] = m_coords[0];
	r[1] = m_coords[1];
	r[2] = m_coords[2];
	r[3] = m_yaw;
	r[4] = m_pitch;
	r[5] = m_roll;
}

void CPose3D::getAsVector(mrpt::math::CArrayDouble<6>& r) const
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
	mrpt::math::CMatrixFixedNumeric<double, 4, 3>* out_dq_dr) const
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
		const CArrayDouble<3> B_coords = B.m_coords;
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
	m_ROT.multiply_AB(A.m_ROT, B.m_ROT);

	m_ypr_uptodate = false;
}

/** Convert this pose into its inverse, saving the result in itself. */
void CPose3D::inverse()
{
	CMatrixDouble33 inv_rot(UNINITIALIZED_MATRIX);
	CArrayDouble<3> inv_xyz;

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
	CArrayDouble<3> t_b_inv;
	mrpt::math::homogeneousMatrixInverse(B.m_ROT, B.m_coords, R_b_inv, t_b_inv);

	for (int i = 0; i < 3; i++)
		m_coords[i] = t_b_inv[i] + R_b_inv(i, 0) * A.m_coords[0] +
					  R_b_inv(i, 1) * A.m_coords[1] +
					  R_b_inv(i, 2) * A.m_coords[2];

	// Rot part:
	m_ROT.multiply_AB(R_b_inv, A.m_ROT);
	m_ypr_uptodate = false;
}

/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
 * \sa composePoint, composeFrom
 */
void CPose3D::inverseComposePoint(
	const double gx, const double gy, const double gz, double& lx, double& ly,
	double& lz,
	mrpt::math::CMatrixFixedNumeric<double, 3, 3>* out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double, 3, 6>* out_jacobian_df_dpose,
	mrpt::math::CMatrixFixedNumeric<double, 3, 6>* out_jacobian_df_dse3) const
{
	CMatrixDouble33 R_inv(UNINITIALIZED_MATRIX);
	CArrayDouble<3> t_inv;
	mrpt::math::homogeneousMatrixInverse(m_ROT, m_coords, R_inv, t_inv);

	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint) *out_jacobian_df_dpoint = R_inv;

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

		alignas(MRPT_MAX_ALIGN_BYTES) const double nums[3 * 6] = {
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
		out_jacobian_df_dpose->loadFromArray(nums);
	}

	lx = t_inv[0] + R_inv(0, 0) * gx + R_inv(0, 1) * gy + R_inv(0, 2) * gz;
	ly = t_inv[1] + R_inv(1, 0) * gx + R_inv(1, 1) * gy + R_inv(1, 2) * gz;
	lz = t_inv[2] + R_inv(2, 0) * gx + R_inv(2, 1) * gy + R_inv(2, 2) * gz;

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		alignas(MRPT_MAX_ALIGN_BYTES) const double nums[3 * 6] = {
			-1, 0, 0, 0, -lz, ly, 0, -1, 0, lz, 0, -lx, 0, 0, -1, -ly, lx, 0};
		out_jacobian_df_dse3->loadFromArray(nums);
	}
}

CPose3D CPose3D::exp(
	const mrpt::math::CArrayNumeric<double, 6>& mu, bool pseudo_exponential)
{
	CPose3D P(UNINITIALIZED_POSE);
	CPose3D::exp(mu, P, pseudo_exponential);
	return P;
}

void CPose3D::exp(
	const mrpt::math::CArrayNumeric<double, 6>& mu, CPose3D& out_pose,
	bool pseudo_exponential)
{
	if (pseudo_exponential)
	{
		auto R = Sophus::SO3<double>::exp(mu.block<3, 1>(3, 0));
		out_pose.setRotationMatrix(R.matrix());
		out_pose.x(mu[0]);
		out_pose.y(mu[1]);
		out_pose.z(mu[2]);
	}
	else
	{
		auto R = Sophus::SE3<double>::exp(mu);
		out_pose = CPose3D(CMatrixDouble44(R.matrix()));
	}
}

CArrayDouble<3> CPose3D::ln_rotation() const
{
	Sophus::SO3<double> R(this->m_ROT);
	const auto& r = R.log();
	CArrayDouble<3> ret;
	for (int i = 0; i < 3; i++) ret[i] = r[i];
	return ret;
}

CMatrixDouble33 CPose3D::exp_rotation(
	const mrpt::math::CArrayNumeric<double, 3>& w)
{
	auto R = Sophus::SO3<double>::exp(w);
	return R.matrix();
}

void CPose3D::ln(CArrayDouble<6>& result) const
{
	const Sophus::SE3<double> RT(m_ROT, m_coords);
	result = RT.log();
}

/* The following code fragments are based on formulas originally reported in the
 * TooN and RobotVision packages */
namespace mrpt::poses
{
template <class VEC3, class MAT33>
inline void deltaR(const MAT33& R, VEC3& v)
{
	v[0] = R(2, 1) - R(1, 2);
	v[1] = R(0, 2) - R(2, 0);
	v[2] = R(1, 0) - R(0, 1);
}

template <typename VEC3, typename MAT3x3, typename MAT3x9>
inline void M3x9(const VEC3& a, const MAT3x3& B, MAT3x9& RES)
{
	alignas(MRPT_MAX_ALIGN_BYTES) const double vals[] = {
		a[0],	 -B(0, 2), B(0, 1),  B(0, 2),  a[0],	-B(0, 0), -B(0, 1),
		B(0, 0),  a[0],		a[1],	 -B(1, 2), B(1, 1), B(1, 2),  a[1],
		-B(1, 0), -B(1, 1), B(1, 0),  a[1],		a[2],	-B(2, 2), B(2, 1),
		B(2, 2),  a[2],		-B(2, 0), -B(2, 1), B(2, 0), a[2]};
	RES.loadFromArray(vals);
}

inline CMatrixDouble33 ddeltaRt_dR(const CPose3D& P)
{
	const CMatrixDouble33& R = P.getRotationMatrix();
	const CArrayDouble<3>& t = P.m_coords;

	CArrayDouble<3> abc;
	deltaR(R, abc);
	double a = abc[0];
	double b = abc[1];
	double c = abc[2];

	alignas(MRPT_MAX_ALIGN_BYTES) const double vals[] = {
		-b * t[1] - c * t[2],	 2 * b * t[0] - a * t[1],
		2 * c * t[0] - a * t[2],  -b * t[0] + 2 * a * t[1],
		-a * t[0] - c * t[2],	 2 * c * t[1] - b * t[2],
		-c * t[0] + 2 * a * t[2], -c * t[1] + 2 * b * t[2],
		-a * t[0] - b * t[1]};
	return CMatrixDouble33(vals);
}

inline void dVinvt_dR(const CPose3D& P, CMatrixFixedNumeric<double, 3, 9>& J)
{
	CArrayDouble<3> a;
	CMatrixDouble33 B(UNINITIALIZED_MATRIX);

	const CMatrixDouble33& R = P.getRotationMatrix();
	const CArrayDouble<3>& t = P.m_coords;

	const double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);

	if (d > 0.9999)
	{
		a[0] = a[1] = a[2] = 0;
		B.zeros();
	}
	else
	{
		const double theta = acos(d);
		const double theta2 = square(theta);
		const double oned2 = (1 - square(d));
		const double sq = std::sqrt(oned2);
		const double cot = 1. / tan(0.5 * theta);
		const double csc2 = square(1. / sin(0.5 * theta));

		CMatrixDouble33 skewR(UNINITIALIZED_MATRIX);
		CArrayDouble<3> vr;
		deltaR(R, vr);
		mrpt::math::skew_symmetric3(vr, skewR);

		CArrayDouble<3> skewR_t;
		skewR.multiply_Ab(t, skewR_t);

		skewR_t *= -(d * theta - sq) / (8 * pow(sq, 3));
		a = skewR_t;

		CMatrixDouble33 skewR2(UNINITIALIZED_MATRIX);
		skewR2.multiply_AB(skewR, skewR);

		CArrayDouble<3> skewR2_t;
		skewR2.multiply_Ab(t, skewR2_t);
		skewR2_t *=
			(((theta * sq - d * theta2) * (0.5 * theta * cot - 1)) -
			 theta * sq * ((0.25 * theta * cot) + 0.125 * theta2 * csc2 - 1)) /
			(4 * theta2 * square(oned2));
		a += skewR2_t;

		mrpt::math::skew_symmetric3(t, B);
		B *= -0.5 * theta / (2 * sq);

		B += -(theta * cot - 2) / (8 * oned2) * ddeltaRt_dR(P);
	}
	M3x9(a, B, J);
}
}  // namespace mrpt::poses

void CPose3D::ln_jacob(mrpt::math::CMatrixFixedNumeric<double, 6, 12>& J) const
{
	J.zeros();
	// Jacobian structure 6x12:
	// (3rows, for t)       [       d_Vinvt_dR (3x9)    |  Vinv (3x3)  ]
	//                      [  -------------------------+------------- ]
	// (3rows, for \omega)  [       d_lnR_dR   (3x9)    |    0 (3x3)   ]
	//
	//          derivs wrt:     R_col1 R_col2  R_col3   |       t
	//
	// (Will be explained better in:
	// http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty )
	//
	{
		CMatrixFixedNumeric<double, 3, 9> M(UNINITIALIZED_MATRIX);
		ln_rot_jacob(m_ROT, M);
		J.insertMatrix(3, 0, M);
	}
	{
		CMatrixFixedNumeric<double, 3, 9> M(UNINITIALIZED_MATRIX);
		dVinvt_dR(*this, M);
		J.insertMatrix(0, 0, M);
	}

	const CMatrixDouble33& R = m_ROT;
	CArrayDouble<3> omega;
	CMatrixDouble33 Omega(UNINITIALIZED_MATRIX);

	CMatrixDouble33 V_inv(UNINITIALIZED_MATRIX);
	V_inv.unit(3, 1.0);  // Start with the identity_3

	const double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
	if (d > 0.99999)
	{
		mrpt::poses::deltaR(R, omega);
		omega *= 0.5;
		mrpt::math::skew_symmetric3(omega, Omega);
		CMatrixDouble33 Omega2(UNINITIALIZED_MATRIX);
		Omega2.multiply_AAt(Omega);
		Omega2 *= 1.0 / 12.0;

		Omega *= 0.5;

		V_inv -= Omega;
		V_inv -= Omega2;
	}
	else
	{
		mrpt::poses::deltaR(R, omega);

		const double theta = acos(d);
		omega *= theta / (2 * std::sqrt(1 - d * d));

		mrpt::math::skew_symmetric3(omega, Omega);

		CMatrixDouble33 Omega2(UNINITIALIZED_MATRIX);
		Omega2.multiply_AAt(Omega);

		Omega2 *= (1 - theta / (2 * std::tan(theta * 0.5))) / square(theta);
		Omega *= 0.5;

		V_inv -= Omega;
		V_inv += Omega2;
	}
	J.insertMatrix(0, 9, V_inv);
}

void CPose3D::ln_rot_jacob(
	const CMatrixDouble33& R, CMatrixFixedNumeric<double, 3, 9>& M)
{
	const double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
	CArrayDouble<3> a;
	CMatrixDouble33 B(UNINITIALIZED_MATRIX);
	if (d > 0.99999)
	{
		a[0] = a[1] = a[2] = 0;
		B.unit(3, -0.5);
	}
	else
	{
		const double theta = acos(d);
		const double d2 = square(d);
		const double sq = std::sqrt(1 - d2);
		deltaR(R, a);
		a *= (d * theta - sq) / (4 * (sq * sq * sq));
		B.unit(3, -theta / (2 * sq));
	}
	M3x9(a, B, M);
}

// Eq. 10.3.5 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
void CPose3D::jacob_dexpeD_de(
	const CPose3D& D, Eigen::Matrix<double, 12, 6>& jacob)
{
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0).setIdentity();
	for (int i = 0; i < 3; i++)
	{
		Eigen::Block<Eigen::Matrix<double, 12, 6>, 3, 3, false> trg_blc =
			jacob.block<3, 3>(3 * i, 3);
		mrpt::math::skew_symmetric3_neg(D.m_ROT.block<3, 1>(0, i), trg_blc);
	}
	{
		Eigen::Block<Eigen::Matrix<double, 12, 6>, 3, 3, false> trg_blc =
			jacob.block<3, 3>(9, 3);
		mrpt::math::skew_symmetric3_neg(D.m_coords, trg_blc);
	}
}

// Eq. 10.3.7 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
void CPose3D::jacob_dAexpeD_de(
	const CPose3D& A, const CPose3D& D, Eigen::Matrix<double, 12, 6>& jacob)
{
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0) = A.getRotationMatrix();
	Eigen::Matrix<double, 3, 3> aux;
	for (int i = 0; i < 3; i++)
	{
		mrpt::math::skew_symmetric3_neg(
			D.getRotationMatrix().block<3, 1>(0, i), aux);
		jacob.block<3, 3>(3 * i, 3) = A.m_ROT * aux;
	}
	mrpt::math::skew_symmetric3_neg(D.m_coords, aux);
	jacob.block<3, 3>(9, 3) = A.m_ROT * aux;
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
