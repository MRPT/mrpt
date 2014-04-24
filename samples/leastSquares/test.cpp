/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace std;

// ------------------------------------------------------
//				TestLeastSquares
// ------------------------------------------------------
void TestLeastSquares()
{
	CVectorDouble	x,y;
	normalize(x,y);

	const double X[] = { 1,2,3,4 };
	const double Y[] = { 6,5,7,10 };

	loadVector(x,X);
	loadVector(y,Y);

	CVectorDouble Ts = linspace(-3.0,8.0,100);
	CVectorDouble Is;

	mrpt::math::leastSquareLinearFit(Ts,Is,x,y);

	CDisplayWindowPlots	win("Result of linear least squares");

	win.plot(Ts,Is);
	win.axis_fit();
	win.axis_equal();

	win.plot(x,y,".3r","training_points");

	win.waitForKey();
}

int main(int argc, char **argv)
{
	try
	{
		TestLeastSquares();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
