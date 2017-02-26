/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/SO_SE_average.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

// -----------   SO_average<2> --------------------
SO_average<2>::SO_average() :
	enable_exception_on_undeterminate(false),
	m_count(0),
	m_accum_x(0),m_accum_y(0)
{
}
void SO_average<2>::clear() {
	m_count=.0;
	m_accum_x=m_accum_y=.0;
}
void SO_average<2>::append(const double orientation_rad) {
	append(orientation_rad,1.0);
}
void SO_average<2>::append(const double orientation_rad, const double weight) {
	m_count += weight;
	m_accum_x += cos(orientation_rad) * weight;
	m_accum_y += sin(orientation_rad) * weight;
}
double SO_average<2>::get_average() const
{
	ASSERT_ABOVE_(m_count,0);
	const double x = m_accum_x / m_count;
	const double y = m_accum_y / m_count;
	errno=0;
	double ang = atan2(y,x);
	if (errno==EDOM) 
	{
		if (enable_exception_on_undeterminate)
		     throw std::runtime_error("[SO_average<2>::get_average()] Undetermined average value");
		else ang = 0;
	}
	return ang;
}

// -----------   SO_average<3> --------------------
SO_average<3>::SO_average() :
	enable_exception_on_undeterminate(false),
	m_count(0),
	m_accum_rot()
{
	clear();
}
void SO_average<3>::clear() {
	m_count=.0;
	m_accum_rot.setZero();
}
void SO_average<3>::append(const Eigen::Matrix3d &M) {
	append(M,1.0);
}
void SO_average<3>::append(const Eigen::Matrix3d &M, const double weight) {
	m_count += weight;
	m_accum_rot += weight*M;
}
// See: eq. (3.7) in "MEANS AND AVERAGING IN THE GROUP OF ROTATIONS", MAHER MOAKHER, 2002.
Eigen::Matrix3d SO_average<3>::get_average() const
{
	ASSERT_ABOVE_(m_count,0);
	const Eigen::Matrix3d MtM = m_accum_rot.transpose() * m_accum_rot;
	
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(MtM, Eigen::ComputeFullU);
	const Eigen::Vector3d vs = svd.singularValues();

	errno=0;
	const double d1 = 1.0/sqrt(vs[0]);
	const double d2 = 1.0/sqrt(vs[1]);
	const double d3 = mrpt::utils::sign( m_accum_rot.determinant() )/sqrt(vs[2]);
	if (errno!=0)
	{
		if (enable_exception_on_undeterminate)
		     throw std::runtime_error("[SO_average<3>::get_average()] Undetermined average value");
		else return Eigen::Matrix3d::Identity();
	}

	Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
	D(0,0)=d1; D(1,1)=d2; D(2,2)=d3;
	Eigen::Matrix3d ret = m_accum_rot * svd.matrixU() * D * svd.matrixU().transpose();
	return ret;
}

// -----------   SE_average<2> --------------------
SE_average<2>::SE_average() :
	enable_exception_on_undeterminate(false),
	m_count(0),
	m_accum_x(0),m_accum_y(0),
	m_rot_part()
{
	clear();
}
void SE_average<2>::clear() {
	m_count=.0;
	m_accum_x=m_accum_y=.0;
	m_rot_part.clear();
}
void SE_average<2>::append(const mrpt::poses::CPose2D &p) {
	append(p,1.0);
}
void SE_average<2>::append(const mrpt::poses::CPose2D &p, const double weight) {
	m_count += weight;
	m_accum_x += weight * p.x();
	m_accum_y += weight * p.y();
	m_rot_part.append(p.phi(), weight);
}
void SE_average<2>::get_average(mrpt::poses::CPose2D &ret_mean) const
{
	ASSERT_ABOVE_(m_count,0);
	ret_mean.x( m_accum_x / m_count );
	ret_mean.y( m_accum_y / m_count );
	const_cast<SO_average<2>*>(&m_rot_part)->enable_exception_on_undeterminate = this->enable_exception_on_undeterminate;
	ret_mean.phi ( m_rot_part.get_average() );
}

// -----------   SE_average<3> --------------------
SE_average<3>::SE_average() :
	enable_exception_on_undeterminate(false),
	m_count(0),
	m_accum_x(0),m_accum_y(0),m_accum_z(0),
	m_rot_part()
{
	clear();
}
void SE_average<3>::clear() {
	m_count=.0;
	m_accum_x=m_accum_y=m_accum_z=.0;
	m_rot_part.clear();
}
void SE_average<3>::append(const mrpt::poses::CPose3D &p) {
	append(p,1.0);
}
void SE_average<3>::append(const mrpt::poses::CPose3D &p, const double weight) {
	m_count += weight;
	m_accum_x += weight * p.x();
	m_accum_y += weight * p.y();
	m_accum_z += weight * p.z();
	m_rot_part.append(p.getRotationMatrix(), weight);
}
void SE_average<3>::get_average(mrpt::poses::CPose3D &ret_mean) const
{
	ASSERT_ABOVE_(m_count,0);
	ret_mean.x( m_accum_x / m_count );
	ret_mean.y( m_accum_y / m_count );
	ret_mean.z( m_accum_z / m_count );
	const_cast<SO_average<3>*>(&m_rot_part)->enable_exception_on_undeterminate = this->enable_exception_on_undeterminate;
	ret_mean.setRotationMatrix( m_rot_part.get_average() );
}
