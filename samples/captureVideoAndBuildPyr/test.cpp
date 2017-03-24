/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/TSimpleFeature.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::obs;
using namespace mrpt::vision;


// ------------------------------------------------------
//				TestVideoBuildPyr
// ------------------------------------------------------
void TestVideoBuildPyr()
{
	size_t N_OCTAVES  = 4;
	bool do_smooth    = false;
	bool do_grayscale = false;
	bool do_features  = false;

	// Ask for a different number of octaves:
	cout << "Number of octaves to use [4]: ";
	{
		std::string s;
		std::getline(cin,s);
		int i= atoi(s.c_str());
		if (i>0) N_OCTAVES = i;
	}

	// Show to the user a list of possible camera drivers and creates and open the selected camera.
	cout << "Please, select the input video file or camera...\n";

	mrpt::hwdrivers::CCameraSensorPtr cam = mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
	if (!cam) return;

	cout << "Video stream open OK\n";

	// Create 3D window:
	CDisplayWindow3D	win ("Demo of pyramid building from live video",800,600);

	//  Get the smart pointer to the main viewport object in this window,
	//   and create other viewports for the smaller images:
	std::vector<COpenGLViewportPtr>  gl_views(N_OCTAVES);
	{
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();
		gl_views[0]  = theScene->getViewport("main");
		ASSERT_(gl_views[0])

		// Create the other viewports:
		for (size_t i=1;i<N_OCTAVES;i++)
			gl_views[i] = theScene->createViewport(format("view_%i",(int)i));

		// Assign sizes:
		//  It can be shown mathematically than if we want all viewports to be one next to each other
		//  horizontally so they fit to the viewport width (="1") and each is the half the previous one,
		//  the first one must have a width of 2^(n-1)/(2^n - 1)
		const double W0 = (double(1<<(N_OCTAVES-1)))/((1<<N_OCTAVES)-1);

		double X = 0;
		double W = W0;
		for (size_t i=0;i<N_OCTAVES;i++)
		{
			COpenGLViewport *vw = gl_views[i].pointer();
			vw->setViewportPosition(X,.0, W,1.);
			//cout << "Created viewport " << i << " at X=" << X << " with Width=" << W << endl;
			X+=W;
			W*=0.5;
		}

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}

	win.setPos(10,10);

	win.addTextMessage(0.51,5,  // X,Y<=1 means coordinates are factors over the entire viewport area.
		"Keys: 's'=Smoothing, 'g': Grayscale 'f': Features",
		TColorf(.8,.8,0),
		"sans",10,  // font name & size
		mrpt::opengl::FILL,
		10 // An arbitrary ID to always overwrite the same, previous 2D text message
		);


	// The image pyramid: Initially empty
	CImagePyramid  imgpyr;

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage (5,5, format("%.02fFPS", win.getRenderingFPS()));
		mrpt::system::sleep(1);

		// Grab new video frame:
		CObservationPtr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs,CObservationImage))
			{
				// Get the observation object:
				CObservationImagePtr o = CObservationImagePtr(obs);

				// Update pyramid:
				imgpyr.buildPyramidFast(
					o->image,  // This image is destroyed since we are calling the *Fast() version
					N_OCTAVES,
					do_smooth,
					do_grayscale
					);

				// Also detect features?
				if (do_features)
				{
					static const int threshold = 20;

					for (unsigned int level=0;level<N_OCTAVES;++level)
					{
						CImage gray_img(imgpyr.images[level], FAST_REF_OR_CONVERT_TO_GRAY);

						TSimpleFeatureList feats;
						CFeatureExtraction::detectFeatures_SSE2_FASTER12( gray_img, feats, threshold );

						imgpyr.images[level].drawFeaturesSimple(feats, TColor::blue);
					}
				}

				win.get3DSceneAndLock();

				for (size_t i=0;i<N_OCTAVES;i++)
				{
					COpenGLViewport *vw = gl_views[i].pointer();
					vw->setImageView(imgpyr.images[i]);
				}


				win.addTextMessage(0.51,25,  // X,Y<=1 means coordinates are factors over the entire viewport area.
					format("Smooth=%i Grayscale=%i Features=%i",int(do_smooth ? 1:0), int(do_grayscale ? 1:0), int(do_features ? 1:0)),
					TColorf(.8,.8,0),
					"sans",10,  // font name & size
					mrpt::opengl::FILL,
					11 // An arbitrary ID to always overwrite the same, previous 2D text message
					);


				win.unlockAccess3DScene();
				win.repaint();
			}


			if (win.keyHit())
			{
				mrptKeyModifier kmods;
				int key = win.getPushedKey(&kmods);

				if (key==MRPTK_ESCAPE) break;

				if (key=='s' || key=='S')
					do_smooth = !do_smooth;
				if (key=='g' || key=='G')
					do_grayscale = !do_grayscale;
				if (key=='f' || key=='F')
					do_features = !do_features;
			}

		}
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestVideoBuildPyr();

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
