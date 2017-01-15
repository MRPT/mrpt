/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CSphere.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::utils;

// This is my custom class to handle the pre/post render events:
struct TMyExtraRenderingStuff : public mrpt::utils::CObserver
{
	opengl::CSpherePtr    ball_obj;  // The ball moving in the scene
	bool                  showing_help, hiding_help;
	mrpt::utils::CTicTac  tim_show_start, tim_show_end;


	TMyExtraRenderingStuff() : showing_help(false), hiding_help(false)
	{
	}

	virtual void OnEvent (const mrptEvent &e)
	{
		if (e.isOfType<mrptEventGLPreRender>())
		{
			//const mrptEventGLPreRender* ev = e.getAs<mrptEventGLPreRender>();
			//ev-> ...
		}
		else
		if (e.isOfType<mrptEventGLPostRender>())
		{
			//const mrptEventGLPostRender* ev = e.getAs<mrptEventGLPostRender>();

			// Show small message in the corner:
			mrpt::opengl::gl_utils::renderMessageBox(
				0.7f,  0.9f,  // x,y (in screen "ratios")
				0.29f, 0.09f, // width, height (in screen "ratios")
				"Press 'h' for help",
				0.02f  // text size
				);

			// Also showing help?
			if (showing_help || hiding_help)
			{
				static const double TRANSP_ANIMATION_TIME_SEC = 0.5;

				const double show_tim = tim_show_start.Tac();
				const double hide_tim = tim_show_end.Tac();

				const double tranparency = hiding_help ?
					1.0-std::min(1.0,hide_tim/TRANSP_ANIMATION_TIME_SEC)
					:
					std::min(1.0,show_tim/TRANSP_ANIMATION_TIME_SEC);

				mrpt::opengl::gl_utils::renderMessageBox(
					0.05f,  0.05f,  // x,y (in screen "ratios")
					0.90f, 0.90f, // width, height (in screen "ratios")
					"These are the supported commands:\n"
					" - 'h': Toogle help view\n"
					" - '<-' and '->': Rotate camera\n"
					" - 'Alt+Enter': Toogle fullscreen\n"
					" - 'ESC': Quit",
					0.05f,  // text size
					mrpt::utils::TColor(190,190,190, 200*tranparency),   // background
					mrpt::utils::TColor(0,0,0, 200*tranparency),  // border
					mrpt::utils::TColor(200,0,0, 150*tranparency), // text
					6.0f, // border width
					"serif", // text font
					mrpt::opengl::NICE // text style
					);

				if (hide_tim>TRANSP_ANIMATION_TIME_SEC && hiding_help)
					hiding_help = false;
			}




		}
	}
};



// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	CDisplayWindow3D	win("Example of 3D Scene Visualization - MRPT",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	// The unique instance of the observer class:
	TMyExtraRenderingStuff   my_extra_rendering;

	// And start subscribing to the viewport events:
	opengl::COpenGLViewportPtr the_main_view = theScene->getViewport("main");
	my_extra_rendering.observeBegin( *the_main_view );


	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1);
		obj->setColor(0.8,0.8,0.8);
		theScene->insert( obj );
	}

	theScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

	if (1)
	{
		opengl::CAxisPtr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10,-10,-10, 10,10,10);
		theScene->insert( obj );
	}

	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,0,1);
		obj->setRadius(0.3);
		obj->setLocation(0,0,1);
		obj->setName( "ball_1" );
		theScene->insert( obj );

		// And also let my rendering object access this ball properties:
		my_extra_rendering.ball_obj = obj;
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	// Texts:
	win.addTextMessage(0.01,0.85, "This is a 2D message", TColorf(1,1,1),0,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );

	win.setCameraElevationDeg( 25.0f );
	//win.setCameraProjective(false);

	cout << endl;
	cout << "Control with mouse or keyboard. Valid keys:" << endl;
	cout << "  ESC                        -> Exit" << endl;
	cout << "  Left/right cursor arrow    -> Camera azimuth" << endl;
	cout << endl;

	bool end = false;

	CTicTac  timer;
	timer.Tic();

	while (!end && win.isOpen() )
	{
		// Move the scene:
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();

		opengl::CRenderizablePtr obj1 = theScene->getByName("ball_1");
		const double t = timer.Tac();
		const double R = 8;
		const double W = 5.0, Q = 3.3;
		obj1->setLocation(
			R* cos(W*t)*sin(Q*t),
			R* sin(W*t),
			R* cos(W*t)*cos(Q*t) );


		// Update the texts on the gl display:
		win.addTextMessage(
			5,5,
			mrpt::format("FPS=%5.02f", win.getRenderingFPS() ),
			TColorf(1,1,1),0, MRPT_GLUT_BITMAP_HELVETICA_18 );

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		mrpt::system::sleep(1);

		if (mrpt::system::os::kbhit()) end = true;
		if (win.keyHit())
		{
			mrptKeyModifier kmods;
			int key = win.getPushedKey(&kmods);
			//printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);

			if (key==MRPTK_ESCAPE) end = true;

			if (key=='h' || key=='H')
			{
				if (!my_extra_rendering.showing_help)
				{
					my_extra_rendering.tim_show_start.Tic();
					my_extra_rendering.showing_help = true;
				}
				else
				{
					my_extra_rendering.tim_show_end.Tic();
					my_extra_rendering.showing_help = false;
					my_extra_rendering.hiding_help = true;
				}
			}

			if (key==MRPTK_RIGHT) win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() + 5 );
			if (key==MRPTK_LEFT)  win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() - 5 );
		}

	};
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestDisplay3D();

		mrpt::system::sleep(50); // leave time for the window to close
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
