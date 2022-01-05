/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/fourier.h>
#include <mrpt/system/CTicTac.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>
string myDataDir(MRPT_EXAMPLES_BASE_DIRECTORY + string("img_convolution_fft/"));

// ------------------------------------------------------
//				TestImageConvolutionFFT
// ------------------------------------------------------
void TestImageConvolutionFFT()
{
	CTicTac tictac;
	CImage img;
	CMatrixF imgCorr;

	// ====================  1  ===================
	if (!img.loadFromFile(myDataDir + string("test_image.jpg")))
		throw std::runtime_error("Cannot load test image!");

	printf(
		"Computing %ux%u image convolution ...", (unsigned)img.getWidth(),
		(unsigned)img.getHeight());

	CMatrixF res_R, res_I;

	double meanTime = 0;

	int N = 3;

	// Convolution using FFT 2D:
	for (int nTimes = 0; nTimes < N; nTimes++)
	{
		tictac.Tic();

		size_t x, y;
		size_t actual_lx = img.getWidth();
		size_t actual_ly = img.getHeight();
		size_t lx = mrpt::round2up(actual_lx);
		size_t ly = mrpt::round2up(actual_ly);

		CMatrixF i1(ly, lx), i2;
		// Get as matrixes, padded with zeros up to power-of-two sizes:
		img.getAsMatrix(i1, false);

		// imgWindow.getAsMatrix(i2,false);
		i2.loadFromTextFile(myDataDir + string("test_convolution_window.txt"));
		i2.setSize(ly, lx);

		if (nTimes == 0)
			printf("\nMax real:%f Min real:%f\n", i1.maxCoeff(), i1.minCoeff());

		// FFT:
		CMatrixF I1_R, I1_I, I2_R, I2_I;
		CMatrixF ZEROS(ly, lx);
		math::dft2_complex(i1, ZEROS, I1_R, I1_I);
		math::dft2_complex(i2, ZEROS, I2_R, I2_I);

		// Compute the COMPLEX MULTIPLICATION of I2 by I1:
		for (y = 0; y < ly; y++)
			for (x = 0; x < lx; x++)
			{
				float r1 = I1_R(y, x);
				float r2 = I2_R(y, x);
				float i1 = I1_I(y, x);
				float i2 = I2_I(y, x);

				I2_R(y, x) = r1 * r2 - i1 * i2;
				I2_I(y, x) = r2 * i1 + r1 * i2;
			}

		// IFFT:
		math::idft2_complex(I2_R, I2_I, res_R, res_I);
		res_R *= 1.0f;	// SCALE!

		meanTime += tictac.Tac();
		printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);
		printf("Max real:%f Min real:%f\n", res_R.maxCoeff(), res_R.minCoeff());
	}

	printf("Mean time: %.06fms\n", 1000.0f * meanTime / N);

	CDisplayWindow winR("real");

	CImage imgR;
	imgR.setFromMatrix(res_R, false /*it is not normalized */);
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
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
