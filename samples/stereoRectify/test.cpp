/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/filesystem.h> // for ASSERT_FILE_EXISTS_
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/opengl/COpenGLScene.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace mrpt::obs;
using namespace std;


// ------------------------------------------------------
//				TestStereoRectify
// ------------------------------------------------------
void TestStereoRectify(int argc, char** argv)
{
	CTimeLogger  timlog;
	mrpt::vision::CStereoRectifyMap  rectifyMap;

	// Parse optional arguments:
	if (argc!=1 && argc!=2)
	{
		cout<< "Usage:\n"
			<< argv[0] << " ==> Run with default camera parameters (from rawlog file)\n"
			<< argv[0] << "[params.cfg] ==> Load stereo camera parameters from cfg file\n";
	}
	if (argc==2)
	{
		const string sCfgFile = argv[1];
		ASSERT_FILE_EXISTS_(sCfgFile)

		// Load params from file:
		mrpt::utils::TStereoCamera params;
		params.loadFromConfigFile("CAMERA_PARAMS", CConfigFile(sCfgFile) );
	
		// Prepare rectify map:
		timlog.enter("rectifyMap.setFromCamParams");
		rectifyMap.setFromCamParams(params);
		timlog.leave("rectifyMap.setFromCamParams");
	}

	// Show to the user a list of possible camera drivers and creates and open the selected camera.
	cout << "Please, select the input stereo camera or rawlog file (with stereo images)...\n";

	mrpt::hwdrivers::CCameraSensorPtr cam = mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
	if (!cam) return;

	cout << "Video stream open OK\n";

	// Create 3D window:
	CDisplayWindow3D	win("Demo of stereo rectification",1280,600);

	// Create 2 viewports, one for each image:
	std::vector<COpenGLViewportPtr>  gl_views(2);
	{
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();
		gl_views[0]  = theScene->getViewport("main");
		ASSERT_(gl_views[0])
		gl_views[1] = theScene->createViewport("right_image");

		// Assign sizes:
		gl_views[0]->setViewportPosition(0,0, .5,1.);
		gl_views[1]->setViewportPosition(.5,0, .5,1.);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}

	win.setPos(10,10);
//	win.addTextMessage(...

	bool enable_rectify = true;
	bool enable_draw_epipolar_lines = true;
	CImage img_left_rectified, img_right_rectified; // Declared here to serve as a memory buffer (avoid deallocating/allocating)

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage (5,5,
			format("%.02fFPS",win.getRenderingFPS()),
			TColorf(1,1,1),"sans",15, mrpt::opengl::FILL, 0
			);
		win.addTextMessage (5,25,
			format("'r': Switch rectify (Now is: %s) | '+'/'-': Modify alpha (Now is: %.02f)",
				enable_rectify ? "ON":"OFF",
				rectifyMap.getAlpha()
				),
			TColorf(1,1,1),"sans",15, mrpt::opengl::FILL, 10
			);
		win.addTextMessage (5,50,
			format("'s': Switch resize output to 320x240 (Now is: %s) | 'c': Switch no-disparity (Now is: %s) | 'e': Switch epipolar lines",
				rectifyMap.isEnabledResizeOutput() ? "ON":"OFF",
				rectifyMap.isEnabledBothCentersCoincide() ? "ON":"OFF"
				),
			TColorf(1,1,1),"sans",15, mrpt::opengl::FILL, 11
			);

		mrpt::system::sleep(1);

		// Grab new video frame:
		CObservationPtr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs,CObservationStereoImages))
			{
				// Get the observation object:
				CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);

				// If the rectification maps are still not ready, prepare them now:
				if (!rectifyMap.isSet())
				{
					timlog.enter("rectifyMap.setFromCamParams");
					rectifyMap.setFromCamParams(*o);
					timlog.leave("rectifyMap.setFromCamParams");

					/*mrpt::utils::TStereoCamera params;
					o->getStereoCameraParams(params);
					cout << params.dumpAsText() << endl;*/
				}

				win.get3DSceneAndLock();

				if (enable_rectify)
				{
					// Rectify:
					timlog.enter("rectifyMap.rectify()");
					
					rectifyMap.rectify(
						o->imageLeft, 
						o->imageRight,
						img_left_rectified,
						img_right_rectified);

					timlog.leave("rectifyMap.rectify()");
				}
				else
				{
					// Don't rectify:
					img_left_rectified = o->imageLeft;
					img_right_rectified = o->imageRight;
				}

				// Draw lines:
				if (enable_draw_epipolar_lines)
				{
					const unsigned int LINES_SEP = 40;
					const unsigned int w = img_left_rectified.getWidth();
					const unsigned int h = img_left_rectified.getHeight();
					for (unsigned int y=0;y<h;y+=LINES_SEP)
					{
						img_left_rectified.line(0,y,w-1,y, mrpt::utils::TColor::red, 2 );
						img_right_rectified.line(0,y,w-1,y, mrpt::utils::TColor::red, 2 );
					}
				}

				gl_views[0]->setImageView(img_left_rectified);
				gl_views[1]->setImageView(img_right_rectified);

				win.addTextMessage(
					150,5,
					mrpt::system::timeToString(o->timestamp),
					TColorf(1,1,1),"sans",15, mrpt::opengl::FILL, 2
					);

				win.unlockAccess3DScene();
				win.repaint();
			}

			if (win.keyHit())
			{
				mrptKeyModifier kmods;
				int key = win.getPushedKey(&kmods);

				if (key==MRPTK_ESCAPE) break;
				if (key=='r' || key=='R') enable_rectify= !enable_rectify;
				if (key=='e' || key=='E') enable_draw_epipolar_lines=!enable_draw_epipolar_lines;
				if (key=='+' || key=='-') 
				{
					double alpha = rectifyMap.getAlpha() + (key=='-' ? -0.1:0.1);
					alpha=std::min(1.,std::max(0.,alpha));
					rectifyMap.setAlpha(alpha);
				}
				if (key=='s' || key=='S') {
					rectifyMap.enableResizeOutput( 
						!rectifyMap.isEnabledResizeOutput(),
						320,240);
				}
				if (key=='c' || key=='C') {
					rectifyMap.enableBothCentersCoincide(!rectifyMap.isEnabledBothCentersCoincide() );
				}
			}

		}
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		TestStereoRectify(argc,argv);
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
