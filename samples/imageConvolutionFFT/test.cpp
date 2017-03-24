/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageConvolutionFFT/") );

// ------------------------------------------------------
//				TestImageConvolutionFFT
// ------------------------------------------------------
void TestImageConvolutionFFT()
{
	CTicTac			tictac;
	CImage		img;
	CMatrix			imgCorr;

	// ====================  1  ===================
	img.loadFromFile(myDataDir+string("test_image.jpg"));

	printf("Computing %ux%u image convolution ...",(unsigned)img.getWidth(),(unsigned)img.getHeight());

	CMatrix		res_R,res_I;


	double	meanTime = 0;

	int N = 3;

	// Convolution using FFT 2D:
	for (int nTimes=0;nTimes<N;nTimes++)
	{
		tictac.Tic();

		size_t		x,y;
		size_t		actual_lx = img.getWidth();
		size_t		actual_ly = img.getHeight();
		size_t		lx = math::round2up( actual_lx );
		size_t		ly = math::round2up( actual_ly );

		CMatrix		i1(ly,lx),i2;
		// Get as matrixes, padded with zeros up to power-of-two sizes:
		img.getAsMatrix(i1,false);

		//imgWindow.getAsMatrix(i2,false);
		i2.loadFromTextFile(myDataDir+string("test_convolution_window.txt"));
		i2.setSize(ly,lx);

		if (nTimes==0)
			printf("\nMax real:%f Min real:%f\n", i1.maximum(),i1.minimum() );

		// FFT:
		CMatrix		I1_R,I1_I,I2_R,I2_I;
		CMatrix		ZEROS(ly,lx);
		math::dft2_complex(i1,ZEROS,I1_R,I1_I);
		math::dft2_complex(i2,ZEROS,I2_R,I2_I);

		// Compute the COMPLEX MULTIPLICATION of I2 by I1:
		for (y = 0;y<ly;y++)
			for (x = 0;x<lx;x++)
			{
				float	r1 = I1_R.get_unsafe(y,x);
				float	r2 = I2_R.get_unsafe(y,x);
				float	i1 = I1_I.get_unsafe(y,x);
				float	i2 = I2_I.get_unsafe(y,x);

				I2_R.set_unsafe(y,x, r1*r2-i1*i2);
				I2_I.set_unsafe(y,x, r2*i1+r1*i2);
			}

		// IFFT:
		math::idft2_complex(I2_R,I2_I,res_R,res_I);
		res_R *= 1.0f;		// SCALE!

		meanTime += tictac.Tac();
		printf(" Done,%.06fms\n", tictac.Tac()*1000.0f);
		printf("Max real:%f Min real:%f\n", res_R.maximum(),res_R.minimum() );
	}

	printf("Mean time: %.06fms\n",1000.0f*meanTime/N);

	CDisplayWindow		winR("real");

	res_R.adjustRange(0,1);

	CImage imgR(res_R, true);
	winR.showImage(imgR);
	winR.waitForKey();

//	DEBUG_SAVE_MATRIX(res_R);
//	DEBUG_SAVE_MATRIX(res_I);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestImageConvolutionFFT();

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
