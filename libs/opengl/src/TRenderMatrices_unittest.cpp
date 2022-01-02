/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/opengl/TRenderMatrices.h>

#include <Eigen/Dense>

//#define USE_GLM_GROUND_TRUTH

#if defined(USE_GLM_GROUND_TRUTH)
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>	 // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>	 // glm::value_ptr
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>  // glm::mat4
#include <glm/vec3.hpp>	 // glm::vec3
#include <glm/vec4.hpp>	 // glm::vec4, glm::ivec4
#endif

TEST(OpenGL, perspectiveMatrix)
{
	float zmin = 0.1f, zmax = 100.0f;
	float fovy_deg = 45.0f;
	size_t view_w = 640, view_h = 480;

	mrpt::opengl::TRenderMatrices rm;
	rm.viewport_width = view_w;
	rm.viewport_height = view_h;
	rm.FOV = fovy_deg;
	rm.is_projective = true;

	rm.computeProjectionMatrix(zmin, zmax);

	// Expected value:

#if defined(USE_GLM_GROUND_TRUTH)
	glm::mat4 Projection = glm::perspective(
		mrpt::DEG2RAD(fovy_deg), view_w / float(view_h), zmin, zmax);
	std::cout << glm::to_string(Projection) << "\n";
#endif

	mrpt::math::CMatrixFloat44 P_GT;
	P_GT.fromMatlabStringFormat(
		"[1.810660 0.000000 0.000000 0.000000;"
		"0.000000 2.414213 0.000000 0.000000;"
		"0.000000 0.000000 -1.002002 -1.000000;"
		"0.000000 0.000000 -0.200200 0.000000"
		"]");
	P_GT = P_GT.transpose().eval();	 // GT was in column major

	EXPECT_NEAR((P_GT - rm.p_matrix).array().abs().maxCoeff(), 0, 0.001f)
		<< "P=\n"
		<< rm.p_matrix << "\nExpected=\n"
		<< P_GT << "\n";
}

TEST(OpenGL, orthoMatrix)
{
	float left = -4.0f, right = 6.0f;
	float top = 10.0f, bottom = -2.0f;
	float zmin = 0.1f, zmax = 100.0f;

	mrpt::opengl::TRenderMatrices rm;
	rm.computeOrthoProjectionMatrix(left, right, bottom, top, zmin, zmax);

	// Expected value:

#if defined(USE_GLM_GROUND_TRUTH)
	glm::mat4 Projection = glm::ortho(left, right, bottom, top, zmin, zmax);
	std::cout << glm::to_string(Projection) << "\n";
#endif

	mrpt::math::CMatrixFloat44 P_GT;
	P_GT.fromMatlabStringFormat(
		"[0.200000 0.000000 0.000000 0.000000;"
		"0.000000 0.166667 0.000000 0.000000;"
		"0.000000 0.000000 -0.020020 0.000000;"
		"-0.200000 -0.666667 -1.002002 1.000000"
		"]");
	P_GT = P_GT.transpose().eval();	 // GT was in column major

	EXPECT_NEAR((P_GT - rm.p_matrix).array().abs().maxCoeff(), 0, 0.001f)
		<< "P=\n"
		<< rm.p_matrix << "\nExpected=\n"
		<< P_GT << "\n";
}

TEST(OpenGL, perspectiveMatrixFromPinhole)
{
	float zmin = 0.1f, zmax = 100.0f;

	mrpt::img::TCamera c1;
	c1.ncols = 800;
	c1.nrows = 600;
	c1.fx(350);
	c1.fy(340);
	c1.cx(400);
	c1.cy(300);

	mrpt::opengl::TRenderMatrices rm;
	rm.viewport_width = c1.ncols;
	rm.viewport_height = c1.nrows;
	rm.pinhole_model = c1;
	rm.is_projective = true;

	rm.computeProjectionMatrix(zmin, zmax);

	// Camera axes: +Z forward, +X right, +Y down.
	{
		// Project point test:
		mrpt::math::CMatrixFixed<float, 4, 1> pt1;
		pt1.fromMatlabStringFormat("[0 ; 0; 10.0; 1.0]");
		const Eigen::Vector4f pt1h = (rm.p_matrix * pt1);
		const Eigen::Vector3f pt1v = pt1h.head<3>() * (1.0f / pt1h[3]);

		EXPECT_NEAR(pt1v[0], 0, 1e-3f);
		EXPECT_NEAR(pt1v[1], 0, 1e-3f);
	}

	{
		// Project point test:
		mrpt::math::CMatrixFixed<float, 4, 1> pt1;
		const float fov_x = 2 * atan(c1.ncols / (2 * c1.fx()));
		const float fov_y = 2 * atan(c1.nrows / (2 * c1.fy()));

		const float z = 10.0f;
		pt1[0] = tan(fov_x / 2) * z;
		pt1[1] = tan(fov_y / 2) * z;
		pt1[2] = z;
		pt1[3] = 1.0;
		const Eigen::Vector4f pt1h = (rm.p_matrix * pt1);
		const Eigen::Vector3f pt1v = pt1h.head<3>() * (1.0f / pt1h[3]);

		EXPECT_NEAR(pt1v[0], -1, 1e-3f);
		EXPECT_NEAR(pt1v[1], -1, 1e-3f);
	}
}