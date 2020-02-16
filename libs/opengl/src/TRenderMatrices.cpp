/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/geometry.h>  // crossProduct3D()
#include <mrpt/math/ops_containers.h>  // dotProduct()
#include <mrpt/opengl/TRenderMatrices.h>
#include <Eigen/Dense>

using namespace mrpt::opengl;

// Replacement for obsolete: gluPerspective() and glOrtho()
void TRenderMatrices::computeProjectionMatrix(float znear, float zfar)
{
	ASSERT_ABOVE_(FOV, .0f);
	ASSERT_ABOVE_(zfar, znear);
	ASSERT_ABOVE_(zfar, .0f);
	ASSERT_ABOVEEQ_(znear, .0f);

	if (is_projective)
	{
		// Was: gluPerspective()
		// Based on GLM's perspective (MIT license).

		const float aspect = viewport_width / (1.0f * viewport_height);
		ASSERT_ABOVE_(
			std::abs(aspect - std::numeric_limits<float>::epsilon()), .0f);

		const float f = 1.0f / std::tan(mrpt::DEG2RAD(FOV) / 2.0f);

		p_matrix.setZero();

		p_matrix(0, 0) = f / aspect;
		p_matrix(1, 1) = f;
		p_matrix(2, 2) = -(zfar + znear) / (zfar - znear);
		p_matrix(3, 2) = -1.0f;
		p_matrix(2, 3) = -(2.0f * zfar * znear) / (zfar - znear);
	}
	else
	{
		// Was:
		// glOrtho(-Ax, Ax, -Ay, Ay, -0.5 * m_clip_max, 0.5 * m_clip_max);

		const float ratio = viewport_width / (1.0f * viewport_height);
		float Ax = eyeDistance * 0.5f;
		float Ay = eyeDistance * 0.5f;

		if (ratio > 1)
			Ax *= ratio;
		else
		{
			if (ratio != 0) Ay /= ratio;
		}

		p_matrix.setIdentity();
		const auto left = -.5f * Ax, right = .5f * Ax;
		const auto bottom = -.5f * Ay, top = .5f * Ay;

		p_matrix(0, 0) = 2.0f / (right - left);
		p_matrix(1, 1) = 2.0f / (top - bottom);
		p_matrix(2, 2) = 1.0f / (zfar - znear);
		p_matrix(3, 0) = -(right + left) / (right - left);
		p_matrix(3, 1) = -(top + bottom) / (top - bottom);
		p_matrix(3, 2) = -znear / (zfar - znear);
	}
}

// Replacement for deprecated OpenGL gluLookAt():
void TRenderMatrices::applyLookAt()
{
	using mrpt::math::TVector3Df;

	TVector3Df forward = TVector3Df(pointing - eye);
	const float fn = forward.norm();
	ASSERT_(fn != 0);
	forward *= 1.0f / fn;

	// Side = forward x up
	TVector3Df side = mrpt::math::crossProduct3D(forward, up);
	const float sn = side.norm();
	ASSERT_(sn != 0);
	side *= 1.0f / sn;

	// Recompute up as: up = side x forward
	const TVector3Df up2 = mrpt::math::crossProduct3D(side, forward);

	//  s.x   s.y   s.z  -dot(s, eye)
	//  u.x   u.y   u.z  -dot(u, eye)
	// -f.x  -f.y  -f.z  dot(up, eye)
	//   0     0     0      1

	mrpt::math::CMatrixFloat44 m(mrpt::math::UNINITIALIZED_MATRIX);
	// Axis X:
	m(0, 0) = side[0];
	m(0, 1) = side[1];
	m(0, 2) = side[2];
	// Axis Y:
	m(1, 0) = up2[0];
	m(1, 1) = up2[1];
	m(1, 2) = up2[2];
	// Axis Z:
	m(2, 0) = -forward[0];
	m(2, 1) = -forward[1];
	m(2, 2) = -forward[2];
	// Translation:
	m(0, 3) = -mrpt::math::dotProduct<3, float>(side, eye);
	m(1, 3) = -mrpt::math::dotProduct<3, float>(up2, eye);
	m(2, 3) = mrpt::math::dotProduct<3, float>(forward, eye);
	// Last row:
	m(3, 0) = .0f;
	m(3, 1) = .0f;
	m(3, 2) = .0f;
	m(3, 3) = 1.f;

	// Homogeneous matrices composition:
	// Overwrite projection matrix:
	p_matrix.asEigen() = p_matrix.asEigen() * m.asEigen();
}

void TRenderMatrices::projectPoint(
	float x, float y, float z, float& proj_u, float& proj_v,
	float& proj_z_depth) const
{
	const Eigen::Matrix<float, 4, 1, Eigen::ColMajor> proj =
		pmv_matrix.asEigen() *
		Eigen::Matrix<float, 4, 1, Eigen::ColMajor>(x, y, z, 1);
	proj_u = proj[3] ? proj[0] / proj[3] : 0;
	proj_v = proj[3] ? proj[1] / proj[3] : 0;
	proj_z_depth = proj[2];
}

void TRenderMatrices::projectPointPixels(
	float x, float y, float z, float& proj_u_px, float& proj_v_px,
	float& proj_depth) const
{
	projectPoint(x, y, z, proj_u_px, proj_v_px, proj_depth);
	proj_u_px = (proj_u_px + 1.0f) * (viewport_width * 0.5f);
	proj_v_px = (proj_v_px + 1.0f) * (viewport_height * 0.5f);
}
