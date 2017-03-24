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
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageCorrelation/") );


void TestWindow()
{
    CImage IM;
    IM.loadFromFile(myDataDir+string("fft2_test_image_patch.jpg"));

    gui::CDisplayWindow  win("Hola!");

    win.showImage(IM);
    win.waitForKey();

    win.setPos(400,100);
    win.waitForKey();

    win.resize(400,100);
    win.waitForKey();
}


// ------------------------------------------------------
//				TestImageFFT
// ------------------------------------------------------
void TestImageFFT()
{
	CTicTac			tictac;
	CImage		IM1,IM2;
	CMatrix			imgCorr;
	size_t		u,v;
	double	valMax;
	float   valMaxF;
	uint32_t     nTimes;

	// ====================  1  ===================
	IM1.loadFromFile(myDataDir+string("fft2_test_image_patch.jpg"), 0 );		// "Patch"
	IM2.loadFromFile(myDataDir+string("fft2_test_image.jpg"), 0 );		// Ref. image

	// Method I:
	printf("Computing images correlation %ux%u (FFT)...",(unsigned)IM1.getWidth(),(unsigned)IM1.getHeight());

	int		searchWindow_x = 20;
	int		searchWindow_y = 20;
	int		searchWindow_lx = 64;
	int		searchWindow_ly = 64;
	uint32_t	N_TIMES = 100;

	tictac.Tic();
	for (nTimes=0;nTimes<N_TIMES;nTimes++)
	{
		IM2.cross_correlation_FFT(
			IM1,
			imgCorr,
			searchWindow_x,
			searchWindow_y,
			searchWindow_lx,
			searchWindow_ly);
	}
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f/N_TIMES);

	SAVE_MATRIX(imgCorr);

	imgCorr.find_index_max_value(u,v,valMaxF);
	u+=searchWindow_x;
	v+=searchWindow_y;

	printf("Peak found at (%u,%u)=%f\n",(unsigned)u,(unsigned)v,valMaxF);

	imgCorr *= 1.0f/valMaxF;
	CImage	imFl(imgCorr, true);
	imFl.saveToFile("_OUT_CORRELATION_FFT.png");


	// Repeat using OpenCV correlation:
	// -----------------------------------------
	printf("Computing images correlation %ux%u (OpenCV)...",(unsigned)IM1.getWidth(),(unsigned)IM1.getHeight());
	tictac.Tic();
	for (nTimes=0;nTimes<N_TIMES;nTimes++)
	{
        IM2.cross_correlation(
            IM1,
            u,v,valMax,
            searchWindow_x,
            searchWindow_y,
            searchWindow_lx,
            searchWindow_ly );
	}
    printf(" Done,%.06fms\n",tictac.Tac()*1000.0f/N_TIMES);
	printf("Peak found at (%u,%u)=%f  || -image_size/2 = (%u,%u)\n",(unsigned)u,(unsigned)v,valMax,(unsigned)(u-IM1.getWidth()/2),(unsigned)(v-IM1.getHeight()/2));


	// ====================  2  ===================
	printf("Computing correlation (non-FFT method)...");
	tictac.Tic();

	CImage imgCorr2;
	size_t x_max,y_max;
	double corr_max;

	IM2.cross_correlation(
		IM1,
		x_max,y_max, corr_max,
		searchWindow_x, searchWindow_y,
		searchWindow_lx, searchWindow_ly,
		&imgCorr2 );

	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	{
		CDisplayWindow	win("Localization of patch"),win2("patch"),win3("correlation");

		CImage		aux(max(IM1.getWidth(),IM2.getWidth()),max(IM1.getHeight(),IM2.getHeight()));

		aux.drawImage(0,0,IM2);
		aux.rectangle(u,v,u-IM1.getWidth()/2,v-IM1.getHeight()/2,TColor::white,2);

		win.showImage(aux);		win.setPos(30,30);
		win2.showImage(IM1);	win2.setPos(30,400);
		win3.showImage(imFl);	win3.setPos(550,30);

		win3.waitForKey();
		return;
	}

}

// ------------------------------------------------------
//				TestFFT_2D_real
// ------------------------------------------------------
void TestFFT_2D_real()
{
	CMatrix		A, RES_R,RES_I,B,D;
	CTicTac		tictac;

	printf("Loading matrix from file...");
	A.loadFromTextFile(myDataDir+string("fft2_test.txt"));
	printf("ok\n");

	printf("Computing 2D FFT of %ux%u...",(unsigned int)A.getRowCount(),(unsigned int)A.getColCount());
	tictac.Tic();
	math::dft2_real(A,RES_R,RES_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	RES_R.saveToTextFile("_out_fft2_real.txt");
	RES_I.saveToTextFile("_out_fft2_imag.txt");

	printf("Computing 2D IFFT of %ux%u...",(unsigned int)A.getRowCount(),(unsigned int)A.getColCount());
	tictac.Tic();
	math::idft2_real(RES_R,RES_I,B);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

//	B.saveToTextFile("_out_ifft2.txt");
	D = B - A;
//	D.saveToTextFile("_out_fft2_error_diffs.txt");

	float	maxError;
	size_t	u,v;
	D.find_index_max_value(u,v,maxError);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n",maxError);
}

// ------------------------------------------------------
//				TestFFT_2D_complex
// ------------------------------------------------------
void TestFFT_2D_complex()
{
	CMatrix		DATA_R,DATA_I, RES_R,RES_I,B_R,B_I,D_R,D_I;
	CTicTac		tictac;

	printf("Loading matrix from file...");
	DATA_R.loadFromTextFile(myDataDir+string("complex_fft2_test_real.txt"));
	DATA_I.loadFromTextFile(myDataDir+string("complex_fft2_test_imag.txt"));
	printf("ok\n");

	printf("Computing 2D complex FFT of %ux%u...",(unsigned int)DATA_R.getRowCount(),(unsigned int)DATA_R.getColCount());
	tictac.Tic();
	math::dft2_complex(DATA_R,DATA_I,RES_R,RES_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	SAVE_MATRIX(RES_R);
	SAVE_MATRIX(RES_I);

	printf("Computing 2D complex IFFT of %ux%u...",(unsigned int)DATA_R.getRowCount(),(unsigned int)DATA_R.getColCount());
	tictac.Tic();
	math::idft2_complex(RES_R,RES_I,B_R,B_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	D_R = B_R - DATA_R;
	D_I = B_I - DATA_I;

	float	maxError_R,maxError_I;
	size_t	u,v;
	D_R.find_index_max_value(u,v,maxError_R);
	D_I.find_index_max_value(u,v,maxError_I);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n",maxError_R);
	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n",maxError_I);
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		//TestFFT_2D_real();
		//TestFFT_2D_complex();
		TestImageFFT();
		//TestWindow();

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


