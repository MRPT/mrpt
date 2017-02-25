/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/datetime.h>
#include <mrpt/gui.h>
#include <mrpt/math/geometry.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;

const unsigned int NUMBER_OF_EDGES=20;
const double MIN_RADIUS=5;
const double MAX_RADIUS=40;

const double DIFF_RADIUS=MAX_RADIUS-MIN_RADIUS;

inline double randomDouble(size_t precision=1000)	{
	return MIN_RADIUS+DIFF_RADIUS*(static_cast<double>(rand()%precision)/static_cast<double>(precision-1));
}

void display()	{
	CDisplayWindowPlots win1("Unsplitted polygon",400,300);
	win1.enableMousePanZoom(true);
	TPolygon2D randomPoly(NUMBER_OF_EDGES);
	for (size_t i=0;i<NUMBER_OF_EDGES;i++)	{
		double ang=2*M_PI*i/NUMBER_OF_EDGES;
		double radius=randomDouble();
		randomPoly[i].x=radius*cos(ang);
		randomPoly[i].y=radius*sin(ang);
	}
	randomPoly.removeRedundantVertices();
	std::vector<double> x,y;
	randomPoly.getPlotData(x,y);
	win1.plot(x,y,"b-5","Polygon");
	win1.axis_fit();
	std::vector<TPolygon2D> convexPolys;
 	if (!splitInConvexComponents(randomPoly,convexPolys)) convexPolys.push_back(randomPoly);
	CDisplayWindowPlots win2("Splitted polygon",400,300);
	win2.enableMousePanZoom(true);
	std::string plotName="Polygon0";
	for (std::vector<TPolygon2D>::const_iterator it=convexPolys.begin();it!=convexPolys.end();++it)	{
		plotName[7]++;
		it->getPlotData(x,y);
		win2.plot(x,y,"r-3",plotName);
		if (!it->isConvex()) cout<<"FAIL.\n";
	}
	win2.axis_fit();
	while (!mrpt::system::os::kbhit()&&win1.isOpen()&&win2.isOpen());
}

int main(int argc,char **argv)	{
	srand((unsigned int)mrpt::system::extractDayTimeFromTimestamp(mrpt::system::getCurrentLocalTime()));
	display();
	return 0;
}
