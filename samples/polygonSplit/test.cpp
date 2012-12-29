/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
