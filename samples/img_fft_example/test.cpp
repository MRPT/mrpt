/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/math/fourier.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::img;
using namespace std;

#include <mrpt/examples_config.h>
string myDataDir(
	MRPT_EXAMPLES_BASE_DIRECTORY +
	string("img_correlation_example/"));  // Reuse it's images

// ------------------------------------------------------
//				TestFFT_2D_real
// ------------------------------------------------------
void TestFFT_2D_real()
{
	CMatrix A, RES_R, RES_I, B, D;
	CTicTac tictac;

	printf("Loading matrix from file...");
	A.loadFromTextFile("dft2_test.txt");
	printf("ok\n");

	printf(
		"Computing 2D FFT of %ux%u...", (unsigned int)A.rows(),
		(unsigned int)A.cols());
	tictac.Tic();
	math::dft2_real(A, RES_R, RES_I);
	printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);

	RES_R.saveToTextFile("_out_dft2_real.txt");
	RES_I.saveToTextFile("_out_dft2_imag.txt");

	printf(
		"Computing 2D IFFT of %ux%u...", (unsigned int)A.rows(),
		(unsigned int)A.cols());
	tictac.Tic();
	math::idft2_real(RES_R, RES_I, B);
	printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);

	//	B.saveToTextFile("_out_ifft2.txt");
	D = B - A;
	//	D.saveToTextFile("_out_dft2_error_diffs.txt");

	float maxError;
	size_t u, v;
	D.find_index_max_value(u, v, maxError);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n", maxError);
}

// ------------------------------------------------------
//				TestFFT_2D_complex
// ------------------------------------------------------
void TestFFT_2D_complex()
{
	CMatrix DATA_R, DATA_I, RES_R, RES_I, B_R, B_I, D_R, D_I;
	CTicTac tictac;

	printf("Loading matrix from file...");
	DATA_R.loadFromTextFile("complex_dft2_test_real.txt");
	DATA_I.loadFromTextFile("complex_dft2_test_imag.txt");
	printf("ok\n");

	printf(
		"Computing 2D complex FFT of %ux%u...", (unsigned int)DATA_R.rows(),
		(unsigned int)DATA_R.cols());
	tictac.Tic();
	math::dft2_complex(DATA_R, DATA_I, RES_R, RES_I);
	printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);

	RES_R.saveToTextFile("_out_complex_dft2_real.txt");
	RES_I.saveToTextFile("_out_complex_dft2_imag.txt");

	printf(
		"Computing 2D complex IFFT of %ux%u...", (unsigned int)DATA_R.rows(),
		(unsigned int)DATA_R.cols());
	tictac.Tic();
	math::idft2_complex(RES_R, RES_I, B_R, B_I);
	printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);

	//	B.saveToTextFile("_out_ifft2.txt");
	D_R = B_R - DATA_R;
	D_I = B_I - DATA_I;
	//	D.saveToTextFile("_out_dft2_error_diffs.txt");

	float maxError_R, maxError_I;
	size_t u, v;
	D_R.find_index_max_value(u, v, maxError_R);
	D_I.find_index_max_value(u, v, maxError_I);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n", maxError_R);
	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n", maxError_I);
}

// ------------------------------------------------------
//				TestImageFFT
// ------------------------------------------------------
void TestImageFFT()
{
	CTicTac tictac;
	CImage IM1, IM2;
	CMatrix imgCorr;

	IM1.loadFromFile(
		myDataDir + string("fft2_test_image_patch.jpg"), 0);  // "Patch"
	IM2.loadFromFile(
		myDataDir + string("fft2_test_image.jpg"), 0);  // Ref. image

	printf("Computing images correlation...");
	tictac.Tic();
	IM2.cross_correlation_FFT(IM1, imgCorr);
	printf(" Done,%.06fms\n", tictac.Tac() * 1000.0f);

	imgCorr.saveToTextFile("_out_dft2_image_test.txt");
}

int main()
{
	try
	{
		TestImageFFT();

		return 0;
	}
	catch (const std::exception& e)
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
