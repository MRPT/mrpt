/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** 
 * Execute a least squares approximation of the given x,y data.
 */

#include <mrpt/utils/types_math.h> // CVectorDouble
#include <mrpt/gui.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/utils.h> // normalize()

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

    /** 
     * for convenience, first initialize the data as C arrays and then define std;:vectors from
     * that data
     *
     * loadVector
     * Assignment operator for initializing a std::vector from a C array
     * refer to http://reference.mrpt.org/stable/group__container__ops__grp.html#ga40e8e47dea9f504a28d2a70ea8ddb158
     */
	const double X[] = { 1,2,3,4 };
	const double Y[] = { 6,5,7,10 };
	loadVector(x,X);
	loadVector(y,Y);

    // x points for plotting the least squres line against.
	CVectorDouble Ts;
	linspace(-3.0,8.0,100, Ts);
	CVectorDouble Is;

    /** 
     * Least squares line approximation, based on the provided x,y vectors.
     */
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
