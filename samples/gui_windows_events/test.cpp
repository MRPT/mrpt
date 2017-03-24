/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/system/threads.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myExampleImage( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageBasics/frame_color.jpg") );

class MyObserver : public mrpt::utils::CObserver
{
protected:
	void OnEvent(const mrptEvent &e)
	{
		if (e.isOfType<mrptEventOnDestroy>())
			cout << "[MyObserver] Event received: mrptEventOnDestroy\n";
		else if (e.isOfType<mrptEventWindowResize>())
		{
			const mrptEventWindowResize &ee = static_cast<const mrptEventWindowResize &>(e);
			cout << "[MyObserver] Resize event received from: " << ee.source_object <<", new size: " << ee.new_width << " x " << ee.new_height << "\n";
		}
		else if (e.isOfType<mrptEventWindowChar>())
		{
			const mrptEventWindowChar &ee = static_cast<const mrptEventWindowChar &>(e);
			cout << "[MyObserver] Char event received from: " << ee.source_object<< ". Char code: " <<  ee.char_code << " modif: " << ee.key_modifiers << "\n";
		}
		else if (e.isOfType<mrptEventWindowClosed>())
		{
			const mrptEventWindowClosed &ee = static_cast<const mrptEventWindowClosed &>(e);
			cout << "[MyObserver] Window closed event received from: " << ee.source_object<< "\n";
		}
		else if (e.isOfType<mrptEventMouseDown>())
		{
			const mrptEventMouseDown &ee = static_cast<const mrptEventMouseDown&>(e);
			cout << "[MyObserver] Mouse down event received from: " << ee.source_object<< "pt: " <<ee.coords.x << "," << ee.coords.y << "\n";
		}
		else
			cout << "[MyObserver] Event received: Another mrptEvent \n";
	}
};

// Observe windows for events.
// Declared out of the scope of windows so we can observe their destructors
MyObserver  observer;

void TestGuiWindowsEvents()
{
	// Create some windows:
	CDisplayWindow  win2D("Bitmap window",300,300);

	{
		mrpt::utils::CImage  img(300,300, 3);
		img.filledRectangle(0,0,300,300,TColor(0,0,255));
		img.textOut(50,50,"Hello world!", TColor(255,255,255));
		win2D.showImage(img);
	}

	CDisplayWindow3D win3D("3D window",300,300);

	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CDisplayWindowPlots winPlot("Plots window",300,300);


	// Tile windows nicely:
	win2D.setPos(10,10);
	win3D.setPos(340,10);
	winPlot.setPos(10,340);


	observer.observeBegin(win2D);
	observer.observeBegin(win3D);
	observer.observeBegin(winPlot);


	// Wait until end:
	cout << "Waiting for window events...\n";
	cout << "** Close any window to end the program **\n";

	while (win2D.isOpen() && win3D.isOpen() && winPlot.isOpen())
	{
		mrpt::system::sleep(100);
	}

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestGuiWindowsEvents();
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
