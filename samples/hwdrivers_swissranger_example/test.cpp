/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/CTicTac.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::img;
using namespace std;

// ------------------------------------------------------
//				Test_SwissRanger
// ------------------------------------------------------
void Test_SwissRanger()
{
	CSwissRanger3DCamera cam;

	// Set params:
	cam.setOpenFromUSB(true);

	cam.setSave3D(true);
	cam.setSaveRangeImage(true);
	cam.setSaveIntensityImage(true);
	cam.setSaveConfidenceImage(false);

	// cam.enablePreviewWindow(true);

	// Open:
	cam.initialize();

	if (cam.isOpen())
		cout << "[Test_SwissRanger] Camera open, serial #"
			 << cam.getCameraSerialNumber() << " resolution: " << cam.cols()
			 << "x" << cam.rows() << " max. range: " << cam.getMaxRange()
			 << endl;

	const double aspect_ratio = cam.rows() / double(cam.cols());

	{
		std::string ver;
		cam.getMesaLibVersion(ver);
		cout << "[Test_SwissRanger] Version: " << ver << "\n";
	}

	CObservation3DRangeScan obs;
	bool there_is_obs = true, hard_error;

	mrpt::gui::CDisplayWindow3D win3D("3D camera view", 800, 600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5, 0, 0);

	// mrpt::gui::CDisplayWindow  win2D("2D range image",200,200);
	// mrpt::gui::CDisplayWindow  winInt("Intensity range image",200,200);
	//	win2D.setPos(10,10);
	//	winInt.setPos(350,10);
	//	win3D.setPos(10,290);
	//	win3D.resize(400,200);

	// mrpt::opengl::CPointCloud::Ptr gl_points =
	// mrpt::opengl::CPointCloud::Create();
	mrpt::opengl::CPointCloudColoured::Ptr gl_points =
		mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(4.5);

	mrpt::opengl::CTexturedPlane::Ptr gl_img_range =
		mrpt::opengl::CTexturedPlane::Create(
			0.5, -0.5, -0.5 * aspect_ratio, 0.5 * aspect_ratio);
	mrpt::opengl::CTexturedPlane::Ptr gl_img_intensity =
		mrpt::opengl::CTexturedPlane::Create(
			0.5, -0.5, -0.5 * aspect_ratio, 0.5 * aspect_ratio);
	mrpt::opengl::CTexturedPlane::Ptr gl_img_intensity_rect =
		mrpt::opengl::CTexturedPlane::Create(
			0.5, -0.5, -0.5 * aspect_ratio, 0.5 * aspect_ratio);

	{
		mrpt::opengl::COpenGLScene::Ptr& scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert(gl_points);
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());
		scene->insert(mrpt::opengl::stock_objects::CornerXYZ());

		const int VW_WIDTH = 200;
		const int VW_HEIGHT = 150;
		const int VW_GAP = 10;

		// Create the Opengl objects for the planar images, as textured planes,
		// each in a separate viewport:
		win3D.addTextMessage(
			30, -10 - 1 * (VW_GAP + VW_HEIGHT), "Range data", 1);
		opengl::COpenGLViewport::Ptr viewRange =
			scene->createViewport("view2d_range");
		scene->insert(gl_img_range, "view2d_range");
		viewRange->setViewportPosition(
			5, -10 - 1 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);
		viewRange->setTransparent(true);
		viewRange->getCamera().setOrthogonal(true);
		viewRange->getCamera().setAzimuthDegrees(90);
		viewRange->getCamera().setElevationDegrees(90);
		viewRange->getCamera().setZoomDistance(1.0);

		win3D.addTextMessage(
			30, -10 - 2 * (VW_GAP + VW_HEIGHT), "Intensity data", 2);
		opengl::COpenGLViewport::Ptr viewInt =
			scene->createViewport("view2d_int");
		scene->insert(gl_img_intensity, "view2d_int");
		viewInt->setViewportPosition(
			5, -10 - 2 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);
		viewInt->setTransparent(true);
		viewInt->getCamera().setOrthogonal(true);
		viewInt->getCamera().setAzimuthDegrees(90);
		viewInt->getCamera().setElevationDegrees(90);
		viewInt->getCamera().setZoomDistance(1.0);

		win3D.addTextMessage(
			30, -10 - 3 * (VW_GAP + VW_HEIGHT), "Intensity data (undistorted)",
			3);
		opengl::COpenGLViewport::Ptr viewIntRect =
			scene->createViewport("view2d_intrect");
		scene->insert(gl_img_intensity_rect, "view2d_intrect");
		viewIntRect->setViewportPosition(
			5, -10 - 3 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);
		viewIntRect->setTransparent(true);
		viewIntRect->getCamera().setOrthogonal(true);
		viewIntRect->getCamera().setAzimuthDegrees(90);
		viewIntRect->getCamera().setElevationDegrees(90);
		viewIntRect->getCamera().setZoomDistance(1.0);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CTicTac tictac;
	size_t nImgs = 0;

	bool endLoop = false;

	while (there_is_obs && !endLoop && win3D.isOpen())
	{
		// Grab new observation from the camera:
		cam.getNextObservation(obs, there_is_obs, hard_error);

		// Show ranges as 2D:
		if (there_is_obs && obs.hasRangeImage)
		{
			mrpt::img::CImage img = obs.rangeImage_getAsImage();

			win3D.get3DSceneAndLock();
			gl_img_range->assignImage_fast(img);
			win3D.unlockAccess3DScene();
		}

		// Show intensity image:
		if (there_is_obs && obs.hasIntensityImage)
		{
			win3D.get3DSceneAndLock();
			gl_img_intensity->assignImage(obs.intensityImage);

			CImage undistortImg;
			obs.intensityImage.undistort(undistortImg, obs.cameraParams);
			gl_img_intensity_rect->assignImage(undistortImg);
			win3D.unlockAccess3DScene();
		}

		// Show 3D points:
		if (there_is_obs && obs.hasPoints3D)
		{
			// mrpt::maps::CSimplePointsMap  pntsMap;
			CColouredPointsMap pntsMap;
			pntsMap.colorScheme.scheme =
				CColouredPointsMap::cmFromIntensityImage;
			pntsMap.loadFromRangeScan(obs);

			win3D.get3DSceneAndLock();
			gl_points->loadFromPointsMap(&pntsMap);
			win3D.unlockAccess3DScene();
			win3D.repaint();
		}

		nImgs++;
		if (nImgs > 10)
		{
			win3D.get3DSceneAndLock();
			win3D.addTextMessage(
				0.01, 0.01, format("%.02f Hz", nImgs / tictac.Tac()), 100);
			win3D.unlockAccess3DScene();
			nImgs = 0;
			tictac.Tic();
		}

		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit())
		{
			const int key = tolower(win3D.getPushedKey());
			// cout << "key: " << key << endl;

			switch (key)
			{
				case 'h':
					cam.enableImageHistEqualization(
						!cam.isEnabledImageHistEqualization());
					break;
				case 'g':
					cam.enableConvGray(!cam.isEnabledConvGray());
					break;
				case 'd':
					cam.enableDenoiseANF(!cam.isEnabledDenoiseANF());
					break;
				case 'f':
					cam.enableMedianFilter(!cam.isEnabledMedianFilter());
					break;
				case 27:
					endLoop = true;
					break;
			}
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(
			0.08, 0.02,
			format(
				"Keyboard switches: H (hist.equal: %s) | G (convGray: %s) | D "
				"(denoise: %s) | F (medianFilter: %s)",
				cam.isEnabledImageHistEqualization() ? "ON" : "OFF",
				cam.isEnabledConvGray() ? "ON" : "OFF",
				cam.isEnabledDenoiseANF() ? "ON" : "OFF",
				cam.isEnabledMedianFilter() ? "ON" : "OFF"),
			110);
		win3D.unlockAccess3DScene();

		std::this_thread::sleep_for(1ms);
	}
}

int main(int argc, char** argv)
{
	try
	{
		Test_SwissRanger();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
