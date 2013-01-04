/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
