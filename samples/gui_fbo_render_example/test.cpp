/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/CTimeLogger.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace mrpt::literals;	 // _deg
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace std::literals;

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	COpenGLScene scene;

	// Modify the scene:
	// ------------------------------------------------------
	{
		auto obj = mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
		obj->setColor(0.4f, 0.4f, 0.4f);
		scene.insert(obj);
	}
	{
		auto obj = mrpt::opengl::CBox::Create(
			mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(.1, .1, .1));
		obj->setColor(1.0f, 0.f, 0.f);
		obj->setName("x");
		obj->enableShowName(true);
		obj->setLocation(1.0, 0, 0);
		scene.insert(obj);
	}
	{
		auto obj = mrpt::opengl::CBox::Create(
			mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(.1, .1, .1));
		obj->setColor(0.0f, 1.f, 0.f);
		obj->setName("y");
		obj->enableShowName(true);
		obj->setLocation(0, 1.0, 0);
		scene.insert(obj);
	}
	{
		auto obj = mrpt::opengl::CTexturedPlane::Create();
		obj->setPlaneCorners(-10, 10, -10, 10);
		obj->setColor_u8(0x00, 0xff, 0xff, 0xff);
		obj->setLocation(0, 0, -14);
		scene.insert(obj);
	}
	{
		auto obj = mrpt::opengl::CSphere::Create();
		obj->setColor(0, 0, 1);
		obj->setRadius(1.0f);
		obj->setLocation(0, 1, 0);
		obj->setName("ball_1");
		scene.insert(obj);
	}
	{
		auto obj = mrpt::opengl::CSphere::Create();
		obj->setColor(1, 0, 0);
		obj->setRadius(2.f);
		obj->setLocation(-3, -1, 1);
		obj->setName("ball_2");
		scene.insert(obj);
	}

	CDisplayWindow win("RGB"), winDepth("Depth");

	mrpt::system::CTimeLogger tl;

	int width = 500, height = 400;
	const double cameraFOVdeg = 90.0;

	mrpt::opengl::CFBORender renderer(width, height);
	CImage frame(width, height, CH_RGB);
	mrpt::math::CMatrixFloat depth;

	scene.getViewport()->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	const float clipMax = 25.0f;
	scene.getViewport()->setViewportClipDistances(0.1, clipMax);

	{
		CCamera& camera = renderer.getCamera(scene);

#if 0
		mrpt::img::TCamera c1;
		c1.ncols = width;
		c1.nrows = height;
		c1.fx(width * 0.5);
		c1.fy(width * 0.5);
		c1.cx(width / 2);
		c1.cy(height / 2);

		camera.setProjectiveFromPinhole(c1);
#else
		camera.setProjectiveFOVdeg(cameraFOVdeg);
#endif

#if 0
		// Define camera by orbit
		camera.setZoomDistance(10);
		camera.setAzimuthDegrees(0);
		camera.setElevationDegrees(90);
#else

		// Defined by setPose() instead of orbit values:
		camera.set6DOFMode(true);

		// Reference camera pose:
		auto robotPose = mrpt::poses::CPose3D(
			0, 0, 10.0, 0.0_deg /*yaw*/, 90.0_deg /*pitch*/, 0.0_deg /*roll*/);

		// Convert to +Z pointing forward camera axes:
		auto camPose =
			robotPose +
			mrpt::poses::CPose3D::FromYawPitchRoll(
				90.0_deg /*yaw*/, 0.0_deg /*pitch*/, 90.0_deg /*roll*/);

		// std::cout << "Camera SE(3) HM:\n" <<
		// camPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>() <<
		// "\n";

		camera.setPose(camPose);

#endif
	}

	while (win.isOpen())
	{
		CRenderizable::Ptr obj = scene.getByName("ball_1");

		const double t = mrpt::Clock::nowDouble();
		obj->setLocation(
			1 + cos(t + 0.2) * 2, -2 + sin(t + 0.9) * 4, sin(t + 1.2) * 5);

		obj = scene.getByName("ball_2");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY() / 2) * 0.01,
			obj->getPoseY() - sin(obj->getPoseX() / 2) * 0.09,
			obj->getPoseZ() - sin(obj->getPoseX() / 2) * 0.08);

		tl.enter("render_RGBD");
		renderer.render_RGBD(scene, frame, depth);
		tl.leave("render_RGBD");

		// show the rendered RGB image
		win.showImage(frame);

		// Show depth:
		if (!depth.empty())
		{
			std::cout << "minDepth (0=no echo): " << depth.minCoeff()
					  << " maxDepth: " << depth.maxCoeff() << std::endl;

			mrpt::img::CImage imDepth;
			depth *= (1.0f / clipMax);
			imDepth.setFromMatrix(depth, true);
			winDepth.showImage(imDepth);
		}

		std::this_thread::sleep_for(25ms);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
{
	try
	{
		TestDisplay3D();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
