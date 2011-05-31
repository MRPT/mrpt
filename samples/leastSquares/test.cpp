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
	vector_double	x,y;

	const double X[] = { 1,2,3,4 };
	const double Y[] = { 6,5,7,10 };

	loadVector(x,X);
	loadVector(y,Y);

	vector_double Ts = linspace(-3.0,8.0,100);
	vector_double Is;

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
