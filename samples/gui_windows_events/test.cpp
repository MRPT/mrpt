/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

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
		else
			cout << "[MyObserver] Event received: Generic mrptEvent \n";
	}
};

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

	// Observe them for events:
	MyObserver  observer;

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
