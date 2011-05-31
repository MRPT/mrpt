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

using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				TestCapture
// ------------------------------------------------------
void TestCPose3DInterpolation()
{
	mrpt::system::TTimeStamp			iniTs = mrpt::system::now();
	mrpt::system::TTimeStamp			ts = iniTs;
	mrpt::system::TTimeStamp			ots = iniTs;
	mrpt::poses::CPose3D				pose(0,0,0,0,0,0);
	mrpt::poses::CPose3DInterpolator	poseInt;
	std::vector<mrpt::poses::CPose3D>	p;
	mrpt::poses::CPose3D				outPose(0,0,0,0,0,0);
	bool								valid;

	FILE *f = mrpt::system::os::fopen("interpolation.txt","wt");

	// Set the maximum value of the interval time for considering interpolation
	poseInt.setMaxTimeInterpolation(1.0);

	poseInt.insert(ts,pose);			// First point

	ts += 1e7;
	pose.setFromValues(1,1,0,0,1,0);
	poseInt.insert(ts,pose);			// Second point

	ts += 1e7;
	pose.setFromValues(2,2.5,1,0,1.3,0);
	poseInt.insert(ts,pose);			// Third point

	ts += 1e7;
	pose.setFromValues(3,1.7,2,0,1.57,0);
	poseInt.insert(ts,pose);			// Fourth point

	unsigned int i;
	for(i = 0, ots = iniTs; ots <= iniTs + 3e7; ots += 1e6, i++)
	{
		poseInt.interpolate(ots, outPose, valid);
		p.push_back(outPose);
		mrpt::system::os::fprintf(f,"%d %f %f %f %f %f %f\n", i, outPose.x(), outPose.y(), outPose.z(), outPose.yaw(), outPose.pitch(), outPose.roll() );
	}

	mrpt::system::os::fclose(f);

} // end TestCPose3DInterpolation


void TestSplineInterpolation()
{
	FILE *f = mrpt::system::os::fopen("out2","wt");

	vector<double>	x, y;
	double			t;
	x.resize(4);
	y.resize(4);

	//x[0] = 0.0;	x[1] = 1.0;	x[2] = 2.0;	x[3] = 4.0;
	//y[0] = 0.0;	y[1] = 1.0;	y[2] = 1.8;	y[3] = 1.6;

	x[0] = -1.5;	x[1] = -0.5;	x[2] = 0.5;	x[3] = 1.5;
	y[0] = 3.14;	y[1] = -3.10;	y[2] = -3.05;	y[3] = 3.10;

	vector_double ts, ys_interp;

	for(t = x[1]; t <= x[2]; t += 0.01)
	{
		double w = math::spline(t, x, y  ); // wrap no
//		double w = math::spline(t, x, y, true); // wrap yes
		ts.push_back(t);
		ys_interp.push_back(w);
		mrpt::system::os::fprintf(f,"%f %f\n", t, w);
	}

	mrpt::system::os::fclose(f);
	cout << "Done" << endl;

#if MRPT_HAS_WXWIDGETS
	CDisplayWindowPlots		figure("Interpolation results");
	figure.plot(x,y,"r.4",  "true points");
	figure.plot(ts,ys_interp,"b",  "interp");

	figure.axis_equal();
	figure.axis_fit();

	figure.waitForKey();
#endif

}

void TestSplineInterpolationVector()
{
	const size_t N = 15;
	vector_double data_x, data_y(N);

	// Create random data:
	data_x = mrpt::math::linspace(-20.0,20.0,N);
	randomGenerator.drawGaussian1DVector(data_y,  2.0,1.0);

	// Create interpolator
	mrpt::math::CSplineInterpolator1D	interp( data_x, data_y );

	// Generate sequence of where to interpolate:
	vector_double xs = mrpt::math::linspace(-20.0,20.0,500);

	vector_double ys;
	bool valid=interp.queryVector(xs,ys);
	ASSERT_(valid);

#if MRPT_HAS_WXWIDGETS
	CDisplayWindowPlots		figure("Interpolation results");
	figure.plot(data_x,data_y,"r.6",  "true points");
	figure.plot(xs,ys,"b",	"interp");

	figure.axis_equal();
	figure.axis_fit();

	figure.waitForKey();
#endif

}



int main(int argc, char **argv)
{
	try
	{
		TestSplineInterpolationVector();
		//TestSplineInterpolation();
		//TestCPose3DInterpolation();

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
