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
