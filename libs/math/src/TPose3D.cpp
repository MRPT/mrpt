/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/homog_matrices.h>  // homogeneousMatrixInverse()
#include <Eigen/Dense>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TPose3D>);

TPose3D::TPose3D(const TPoint2D& p)
	: x(p.x), y(p.y), z(0.0), yaw(0.0), pitch(0.0), roll(0.0)
{
}
TPose3D::TPose3D(const TPose2D& p)
	: x(p.x), y(p.y), z(0.0), yaw(p.phi), pitch(0.0), roll(0.0)
{
}
TPose3D::TPose3D(const TPoint3D& p)
	: x(p.x), y(p.y), z(p.z), yaw(0.0), pitch(0.0), roll(0.0)
{
}
void TPose3D::asString(std::string& s) const
{
	s = mrpt::format(
		"[%f %f %f %f %f %f]", x, y, z, RAD2DEG(yaw), RAD2DEG(pitch),
		RAD2DEG(roll));
}
void TPose3D::getAsQuaternion(
	mrpt::math::CQuaternion<double>& q,
	mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 4, 3>> out_dq_dr) const
{
	// See:
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	const double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
	const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
	const double cr = cos(roll * 0.5), sr = sin(roll * 0.5);

	const double ccc = cr * cp * cy;
	const double ccs = cr * cp * sy;
	const double css = cr * sp * sy;
	const double sss = sr * sp * sy;
	const double scc = sr * cp * cy;
	const double ssc = sr * sp * cy;
	const double csc = cr * sp * cy;
	const double scs = sr * cp * sy;

	q.w(ccc + sss);
	q.x(scc - css);
	q.y(csc + scs);
	q.z(ccs - ssc);

	// Compute 4x3 Jacobian: for details, see technical report:
	//   Parameterizations of SE(3) transformations: equivalences, compositions
	//   and uncertainty, J.L. Blanco (2010).
	//   https://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty
	if (out_dq_dr)
	{
		alignas(MRPT_MAX_STATIC_ALIGN_BYTES) const double nums[4 * 3] = {
			-0.5 * q[3], 0.5 * (-csc + scs), -0.5 * q[1],
			-0.5 * q[2], 0.5 * (-ssc - ccs), 0.5 * q[0],
			0.5 * q[1],  0.5 * (ccc - sss),  0.5 * q[3],
			0.5 * q[0],  0.5 * (-css - scc), -0.5 * q[2]};
		out_dq_dr.value().get().loadFromArray(nums);
	}
}
void TPose3D::composePoint(const TPoint3D& l, TPoint3D& g) const
{
	CMatrixDouble33 R;
	this->getRotationMatrix(R);
	TPoint3D res;
	res.x = R(0, 0) * l.x + R(0, 1) * l.y + R(0, 2) * l.z + this->x;
	res.y = R(1, 0) * l.x + R(1, 1) * l.y + R(1, 2) * l.z + this->y;
	res.z = R(2, 0) * l.x + R(2, 1) * l.y + R(2, 2) * l.z + this->z;

	g = res;
}
TPoint3D TPose3D::composePoint(const TPoint3D& l) const
{
	TPoint3D g;
	composePoint(l, g);
	return g;
}

void TPose3D::inverseComposePoint(const TPoint3D& g, TPoint3D& l) const
{
	CMatrixDouble44 H;
	this->getInverseHomogeneousMatrix(H);
	TPoint3D res;
	res.x = H(0, 0) * g.x + H(0, 1) * g.y + H(0, 2) * g.z + H(0, 3);
	res.y = H(1, 0) * g.x + H(1, 1) * g.y + H(1, 2) * g.z + H(1, 3);
	res.z = H(2, 0) * g.x + H(2, 1) * g.y + H(2, 2) * g.z + H(2, 3);

	l = res;
}
TPoint3D TPose3D::inverseComposePoint(const TPoint3D& g) const
{
	TPoint3D l;
	inverseComposePoint(g, l);
	return l;
}

void TPose3D::getRotationMatrix(mrpt::math::CMatrixDouble33& R) const
{
	const double cy = cos(yaw);
	const double sy = sin(yaw);
	const double cp = cos(pitch);
	const double sp = sin(pitch);
	const double cr = cos(roll);
	const double sr = sin(roll);

	alignas(MRPT_MAX_STATIC_ALIGN_BYTES)
		const double rot_vals[] = {cy * cp,
								   cy * sp * sr - sy * cr,
								   cy * sp * cr + sy * sr,
								   sy * cp,
								   sy * sp * sr + cy * cr,
								   sy * sp * cr - cy * sr,
								   -sp,
								   cp * sr,
								   cp * cr};
	R.loadFromArray(rot_vals);
}
void TPose3D::SO3_to_yaw_pitch_roll(
	const mrpt::math::CMatrixDouble33& R, double& yaw, double& pitch,
	double& roll)
{
	ASSERTDEBMSG_(
		std::abs(
			sqrt(square(R(0, 0)) + square(R(1, 0)) + square(R(2, 0))) - 1) <
			3e-3,
		"Homogeneous matrix is not orthogonal & normalized!: " +
			R.inMatlabFormat());
	ASSERTDEBMSG_(
		std::abs(
			sqrt(square(R(0, 1)) + square(R(1, 1)) + square(R(2, 1))) - 1) <
			3e-3,
		"Homogeneous matrix is not orthogonal & normalized!: " +
			R.inMatlabFormat());
	ASSERTDEBMSG_(
		std::abs(
			sqrt(square(R(0, 2)) + square(R(1, 2)) + square(R(2, 2))) - 1) <
			3e-3,
		"Homogeneous matrix is not orthogonal & normalized!: " +
			R.inMatlabFormat());

	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch = atan2(-R(2, 0), hypot(R(0, 0), R(1, 0)));

	// Roll:
	if ((fabs(R(2, 1)) + fabs(R(2, 2))) <
		10 * std::numeric_limits<double>::epsilon())
	{
		// Gimbal lock between yaw and roll. This one is arbitrarily forced to
		// be zero.
		// Check
		// https://reference.mrpt.org/devel/classmrpt_1_1poses_1_1_c_pose3_d.html.
		// If cos(pitch)==0, the homogeneous matrix is:
		// When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		// And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		// Both cases are in a "gimbal lock" status. This happens because pitch
		// is vertical.

		roll = 0.0;
		if (pitch > 0)
			yaw = atan2(R(1, 2), R(0, 2));
		else
			yaw = atan2(-R(1, 2), -R(0, 2));
	}
	else
	{
		roll = atan2(R(2, 1), R(2, 2));
		// Yaw:
		yaw = atan2(R(1, 0), R(0, 0));
	}
}

void TPose3D::fromHomogeneousMatrix(const mrpt::math::CMatrixDouble44& HG)
{
	SO3_to_yaw_pitch_roll(
		CMatrixDouble33(HG.blockCopy<3, 3>(0, 0)), yaw, pitch, roll);
	x = HG(0, 3);
	y = HG(1, 3);
	z = HG(2, 3);
}
void TPose3D::composePose(const TPose3D other, TPose3D& result) const
{
	CMatrixDouble44 me_H, o_H;
	this->getHomogeneousMatrix(me_H);
	other.getHomogeneousMatrix(o_H);
	result.fromHomogeneousMatrix(
		CMatrixDouble44(me_H.asEigen() * o_H.asEigen()));
}
void TPose3D::getHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const
{
	CMatrixDouble33 R;
	getRotationMatrix(R);
	HG.block<3, 3>(0, 0) = R.asEigen();
	HG(0, 3) = x;
	HG(1, 3) = y;
	HG(2, 3) = z;
	HG(3, 0) = HG(3, 1) = HG(3, 2) = 0.;
	HG(3, 3) = 1.;
}
void TPose3D::getInverseHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const
{  // Get current HM & inverse in-place:
	this->getHomogeneousMatrix(HG);
	mrpt::math::homogeneousMatrixInverse(HG);
}

void TPose3D::fromString(const std::string& s)
{
	CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 6, "Wrong size of vector in ::fromString");
	x = m(0, 0);
	y = m(0, 1);
	z = m(0, 2);
	yaw = DEG2RAD(m(0, 3));
	pitch = DEG2RAD(m(0, 4));
	roll = DEG2RAD(m(0, 5));
}

TPose3D mrpt::math::operator-(const TPose3D& p)
{
	CMatrixDouble44 H;
	p.getInverseHomogeneousMatrix(H);
	TPose3D ret;
	ret.fromHomogeneousMatrix(H);
	return ret;
}
TPose3D mrpt::math::operator-(const TPose3D& b, const TPose3D& a)
{
	// b - a = A^{-1} * B
	CMatrixDouble44 Hainv, Hb;
	a.getInverseHomogeneousMatrix(Hainv);
	b.getHomogeneousMatrix(Hb);
	TPose3D ret;
	ret.fromHomogeneousMatrix(CMatrixDouble44(Hainv.asEigen() * Hb.asEigen()));
	return ret;
}
