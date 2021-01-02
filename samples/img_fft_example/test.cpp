/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/fourier.h>
#include <mrpt/system/CTicTac.h>
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
	CMatrixF A, RES_R, RES_I, B, D;
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

	size_t u, v;
	const float maxError = D.maxCoeff(u, v);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n", maxError);
}

// ------------------------------------------------------
//				TestFFT_2D_complex
// ------------------------------------------------------
void TestFFT_2D_complex()
{
	CMatrixF DATA_R, DATA_I, RES_R, RES_I, B_R, B_I, D_R, D_I;
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

	size_t u, v;
	const float maxError_R = D_R.maxCoeff(u, v);
	const float maxError_I = D_I.maxCoeff(u, v);

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
	CMatrixF imgCorr;

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
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
