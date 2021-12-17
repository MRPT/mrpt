/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/geometry.h>

#include <iostream>

static void polygonClippingExample()
{
	// Define the polygons:
	const mrpt::math::TPolygon2D subject = {{
		{0.0, 0.0}, {5.0, 0.0}, {7.0, 3.0}, {3.0, 6.0}, {-4.0, 4.0}, {-1.0, 0.0}
		//
	}};

	const mrpt::math::TPolygon2D clipping = {{
		{-6.0, 0.5}, {8.0, 2.0}, {10.0, 4.0}, {-7.0, 3.0}
		//
	}};

	// Compute intersection:
	mrpt::math::TObject2D clippedObj;
	bool doIntersect = mrpt::math::intersect(subject, clipping, clippedObj);
	ASSERT_(doIntersect);

	mrpt::math::TPolygon2D clippedPoly;
	bool isPoly = clippedObj.getPolygon(clippedPoly);
	ASSERT_(isPoly);

	// GUI:
	mrpt::gui::CDisplayWindowPlots winIn(
		"Inputs (red:subject, blue:clipping)", 500, 500);
	winIn.setPos(20, 50);

	const auto [x1, y1] = subject.getPlotData();
	winIn.plot(x1, y1, "-r3");
	winIn.hold_on();

	const auto [x2, y2] = clipping.getPlotData();
	winIn.plot(x2, y2, ":b2");

	mrpt::gui::CDisplayWindowPlots winOut("Result", 500, 500);
	winOut.setPos(600, 50);

	winOut.plot(x2, y2, ":b2");
	winOut.hold_on();

	const auto [xp, yp] = clippedPoly.getPlotData();
	winOut.plot(xp, yp, "-r3");

	winIn.axis_equal();
	winOut.axis_equal();

	std::cout << "Press any key on input window or close it to end program.\n";
	winIn.waitForKey();
}

int main(int argc, char** argv)
{
	try
	{
		polygonClippingExample();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}
