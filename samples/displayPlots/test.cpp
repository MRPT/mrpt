/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace std;


void myOnMenu(int menuID,float x,float y, void*param)
{
	cout << "Menu: " << menuID << endl << " x=" << x << " y="<< y << endl;
}

// ------------------------------------------------------
//				TestDisplayPlots
// ------------------------------------------------------
void TestDisplayPlots()
{
	CDisplayWindowPlots	 win("Example of function plot",400,300);

	win.enableMousePanZoom(true);
	win.addPopupMenuEntry("Mark this point...",1);
	win.setMenuCallback(myOnMenu);

	// Generate data for a 2D plot:
	vector_double  X,Y;
	for (double x=0;x<5;x+=0.01f)
	{
		double y = math::normalPDF(x, 2,0.3);
		X.push_back(x);
		Y.push_back(y);
	}

	win.plot(X,Y,"r-3");
	win.axis_equal(false);
	win.axis_fit();

	win.setPos(10,10);
	// -----------
	CDisplayWindowPlots	 win2("Example of plot update",400,300);

	win2.enableMousePanZoom(true);

	// Add an unnamed & a named ellipse:
	float mean_x = 5;
	float mean_y = 0;
	CMatrixFloat ellipse_cov(2,2);
	ellipse_cov(0,0)=1.0f;
	ellipse_cov(1,1)=1.0f;
	ellipse_cov(0,1)=ellipse_cov(1,0)=0.5f;

	win2.plotEllipse(1.0f,2.0f, ellipse_cov, 3 ,"k-2");
	win2.plotEllipse(mean_x ,mean_y, ellipse_cov, 3 ,"b-2","my_ellipse");

	win2.axis(-10,10,-10,10);
	win2.axis_equal(true);

	win2.setPos(450,10);



	cout << "Press any key to exit..." << endl;
	//win.waitForKey();

	float t = 0;
	ellipse_cov(0,1)=ellipse_cov(1,0)=-0.9f;
	while (!mrpt::system::os::kbhit() &&
		    win.isOpen() &&
			win2.isOpen() &&
			!win.keyHit() &&
			!win2.keyHit() )
	{
		t+=0.05f;
		mean_x = cos(t)*5;
		mean_y = sin(t)*5;
		win2.plotEllipse(mean_x ,mean_y, ellipse_cov, 3 ,"b-2","my_ellipse");

		mrpt::system::sleep(50);
	}
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestDisplayPlots();
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
