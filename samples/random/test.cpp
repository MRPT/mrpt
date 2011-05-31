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

#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/math.h>
#include <mrpt/system.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

void TestHist()
{
	CHistogram              hist(0.0,100.0,10u);
	hist.add(86);
	hist.add(7);
	hist.add(45);

	cout << "Histogram test:" << endl;
	cout << "Should be 1: " << hist.getBinCount(0) << endl;          // Result: "1"
	cout << "Should be 0.33: " << hist.getBinRatio(0) << endl;          // Result: "0.33"
}

// ------------------------------------------------------
//				TestRandomGenerators
// ------------------------------------------------------
void TestRandomGenerators()
{
	vector_double x,y;

	randomGenerator.randomize();

	// Uniform numbers integers:
	CDisplayWindowPlots	  win1("Unif(0,5) (integers)");
	win1.setPos(10,10);
	win1.resize(400,400);
	{
		//vector_double v1(100000);
		vector_size_t  v1(100000);
		randomGenerator.drawUniformVector(v1 ,0, 5.999);

		CHistogram  hist(-2,15, 100);
		hist.add(v1);
		hist.getHistogramNormalized(x,y);

		win1.plot(x,y,"b");

		win1.axis_fit();
	}

	// Normalized Gauss:
	CDisplayWindowPlots	  win2("N(mean=0,std=1)");
	win2.setPos(420,10);
	win2.resize(400,400);
	{
		vector_double v1(100000);
		randomGenerator.drawGaussian1DVector(v1 ,0,1);

		CHistogram  hist(-5,5,100);
		hist.add(v1);
		hist.getHistogramNormalized(x,y);

		win2.plot(x,y,"b");

		vector_double y_real(y.size());
		for (vector_double::Index k=0;k<y_real.size();k++)
			y_real[k] = mrpt::math::normalPDF(x[k],0,1);
		win2.plot(x,y_real,"k-","real");

		win2.axis_fit();
	}

	// Example Gauss:
	CDisplayWindowPlots	  win3("N(mean=3,std=2)");
	win3.setPos(10,430);
	win3.resize(400,400);
	{
		vector_double v1(100000);
		randomGenerator.drawGaussian1DVector(v1 ,3,2);

		CHistogram  hist(-5,15,100);
		hist.add(v1);
		hist.getHistogramNormalized(x,y);

		win3.plot(x,y,"b");

		vector_double y_real(y.size());
		for (vector_double::Index k=0;k<y_real.size();k++)
			y_real[k] = mrpt::math::normalPDF(x[k],3,2);
		win3.plot(x,y_real,"k-","real");

		win3.axis_fit();
	}

	// Example multi-variate Gauss:
	CDisplayWindowPlots	  win4("N(mean=[3 4],var=[4 -2;-2 4])");
	win4.setPos(420,430);
	win4.resize(400,400);
	{
		vector<vector_double> v1;
		vector_double Mean(2);
		Mean[0] = 3;
		Mean[1] = 2;

		//CMatrixDouble cov(2,2);
		CMatrixDouble22 cov;
		cov.fromMatlabStringFormat("[7.5 -7;-7 8]");

		randomGenerator.drawGaussianMultivariateMany(v1,10000,cov,&Mean);

#if 0
		vector_double m;
		CMatrixDouble c;
		mrpt::math::meanAndCov(v1,m,c);
		cout << "Mean: " << m << endl;
		cout << "Std: " << endl << c << endl;
#endif

		// pass to (x,y) vectors:
		vector_double x(v1.size()), y(v1.size());
		for (size_t i=0;i<v1.size();i++)
		{
			x[i] = v1[i][0];
			y[i] = v1[i][1];
		}

		win4.plot(x,y,"b.3");

		win4.plotEllipse(Mean[0],Mean[1],cov,3.0,"k-2","99% ellipse",true);

		win4.axis_fit();
	}



	mrpt::system::pause();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		//TestHist();
		TestRandomGenerators();

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

