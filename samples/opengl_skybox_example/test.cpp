/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CSkyBox.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/filesystem.h>

#include <iostream>
#include <thread>

/* Download and extract the sky box textures, so we dont need to add them
  to the already too large MRPT git repo:

CloudyLightRays/
├── CloudyLightRaysBack.jpg
├── CloudyLightRaysDown.jpg
├── CloudyLightRaysFront.jpg
├── CloudyLightRaysLeft.jpg
├── CloudyLightRaysRight.jpg
├── CloudyLightRaysUp.jpg
├── LICENSE.txt
└── README.txt
 */
const char* urlZip = "https://mrpt.github.io/mvsim-models/skyboxes/SunSet.zip";
const char* textureDir = "./SunSet/";
const char* textureFilePattern = "./SunSet/SunSet%s.jpg";

void downloadTextures()
{
	if (mrpt::system::directoryExists(textureDir)) return;	// already there

	int ret = ::system(mrpt::format("wget %s -O texture.zip", urlZip).c_str());
	if (ret)
		THROW_EXCEPTION("Error invoking wget to download the texture package.");

	ret = ::system("unzip texture.zip");
	if (ret)
		THROW_EXCEPTION("Error invoking unzip to extract the texture package.");
}

// ------------------------------------------------------
//				TestSkyBox
// ------------------------------------------------------
void TestSkyBox()
{
	downloadTextures();

	mrpt::gui::CDisplayWindow3D win("Example of MRPT skybox", 800, 600);

	mrpt::opengl::Scene::Ptr& theScene = win.get3DSceneAndLock();

	// Create the 3D scene:
	// ------------------------------------------------------

	// Create the SkyBox:
	{
		using mrpt::opengl::CUBE_TEXTURE_FACE;

		auto sb = mrpt::opengl::CSkyBox::Create();

		std::vector<std::pair<CUBE_TEXTURE_FACE, const char*>> faceImages = {
			{CUBE_TEXTURE_FACE::FRONT, "Front"},
			{CUBE_TEXTURE_FACE::BACK, "Back"},
			{CUBE_TEXTURE_FACE::BOTTOM, "Down"},
			{CUBE_TEXTURE_FACE::TOP, "Up"},
			{CUBE_TEXTURE_FACE::LEFT, "Left"},
			{CUBE_TEXTURE_FACE::RIGHT, "Right"},
		};

		for (const auto& p : faceImages)
		{
			const auto fil = mrpt::format(textureFilePattern, p.second);
			std::cout << "Loading face texture: " << fil << std::endl;

			sb->assignImage(p.first, mrpt::img::CImage::LoadFromFile(fil));
		}

		theScene->insert(sb);
	}

	{
		auto obj = mrpt::opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		theScene->insert(obj);
	}

	theScene->insert(mrpt::opengl::stock_objects::CornerXYZ(3.0));

	{
		auto obj = mrpt::opengl::CBox::Create();
		obj->setWireframe(false);
		obj->setColor(1, 0, 0);
		obj->setLineWidth(3.0);
		obj->setPose(mrpt::math::TPose3D(1, 2, 3, 0.2, 0.3, 0.1));
		theScene->insert(obj);
	}

	{
		auto obj = mrpt::opengl::CSphere::Create();
		obj->setColor(0, 0, 1);
		obj->setRadius(0.3f);
		obj->setLocation(0, -2, 0);
		obj->setName("ball_1");
		theScene->insert(obj);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	// change camera parasm:
	win.setCameraAzimuthDeg(40.0f);
	win.setCameraElevationDeg(10.0f);
	win.setCameraZoom(4.0f);
	win.setFOV(70.0f);

	// Update window:
	win.forceRepaint();

	win.waitForKey();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestSkyBox();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
