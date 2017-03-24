/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/**
 * random
 * The example demonstrates the use of the random library.
 */

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

// not run by default. Uncomment corresponding line in the main function.
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
	vector<double> x,y;

	randomGenerator.randomize();

	// Uniform numbers integers:
	CDisplayWindowPlots	  win1("Unif(0,5) (integers)");
	win1.setPos(10,10);
	win1.resize(400,400);
	{
		//CVectorDouble v1(100000);
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
		CVectorDouble v1(100000);
		randomGenerator.drawGaussian1DVector(v1 ,0,1);

		CHistogram  hist(-5,5,100);
		hist.add(v1);
		hist.getHistogramNormalized(x,y);

		win2.plot(x,y,"b");

		CVectorDouble y_real(y.size());
		for (CVectorDouble::Index k=0;k<y_real.size();k++)
			y_real[k] = mrpt::math::normalPDF(x[k],0,1);
		win2.plot(x,y_real,"k-","real");

		win2.axis_fit();
	}

	// Example Gauss:
	CDisplayWindowPlots	  win3("N(mean=3,std=2)");
	win3.setPos(10,430);
	win3.resize(400,400);
	{
		CVectorDouble v1(100000);
		randomGenerator.drawGaussian1DVector(v1 ,3,2);

		CHistogram  hist(-5,15,100);
		hist.add(v1);
		hist.getHistogramNormalized(x,y);

		win3.plot(x,y,"b");

		CVectorDouble y_real(y.size());
		for (CVectorDouble::Index k=0;k<y_real.size();k++)
			y_real[k] = mrpt::math::normalPDF(x[k],3,2);
		win3.plot(x,y_real,"k-","real");

		win3.axis_fit();
	}

	// Example multi-variate Gauss:
	CDisplayWindowPlots	  win4("N(mean=[3 4],var=[4 -2;-2 4])");
	win4.setPos(420,430);
	win4.resize(400,400);
	{
		vector<CVectorDouble> v1;
		CVectorDouble Mean(2);
		Mean[0] = 3;
		Mean[1] = 2;

		//CMatrixDouble cov(2,2);
		CMatrixDouble22 cov;
		cov.fromMatlabStringFormat("[7.5 -7;-7 8]");

		randomGenerator.drawGaussianMultivariateMany(v1,10000,cov,&Mean);

#if 0 
		CVectorDouble m;
		CMatrixDouble c;
		mrpt::math::meanAndCov(v1,m,c);
		cout << "Mean: " << m << endl;
		cout << "Std: " << endl << c << endl;
#endif

		// pass to (x,y) vectors:
		CVectorDouble x(v1.size()), y(v1.size());
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

