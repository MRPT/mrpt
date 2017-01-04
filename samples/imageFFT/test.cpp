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
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageCorrelation/") ); // Reuse it's images

// ------------------------------------------------------
//				TestFFT_2D_real
// ------------------------------------------------------
void TestFFT_2D_real()
{
	CMatrix		A, RES_R,RES_I,B,D;
	CTicTac		tictac;

	printf("Loading matrix from file...");
	A.loadFromTextFile("dft2_test.txt");
	printf("ok\n");

	printf("Computing 2D FFT of %ux%u...",(unsigned int)A.getRowCount(),(unsigned int)A.getColCount());
	tictac.Tic();
	math::dft2_real(A,RES_R,RES_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	RES_R.saveToTextFile("_out_dft2_real.txt");
	RES_I.saveToTextFile("_out_dft2_imag.txt");

	printf("Computing 2D IFFT of %ux%u...",(unsigned int)A.getRowCount(),(unsigned int)A.getColCount());
	tictac.Tic();
	math::idft2_real(RES_R,RES_I,B);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

//	B.saveToTextFile("_out_ifft2.txt");
	D = B - A;
//	D.saveToTextFile("_out_dft2_error_diffs.txt");

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
	DATA_R.loadFromTextFile("complex_dft2_test_real.txt");
	DATA_I.loadFromTextFile("complex_dft2_test_imag.txt");
	printf("ok\n");

	printf("Computing 2D complex FFT of %ux%u...",(unsigned int)DATA_R.getRowCount(),(unsigned int)DATA_R.getColCount());
	tictac.Tic();
	math::dft2_complex(DATA_R,DATA_I,RES_R,RES_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	RES_R.saveToTextFile("_out_complex_dft2_real.txt");
	RES_I.saveToTextFile("_out_complex_dft2_imag.txt");

	printf("Computing 2D complex IFFT of %ux%u...",(unsigned int)DATA_R.getRowCount(),(unsigned int)DATA_R.getColCount());
	tictac.Tic();
	math::idft2_complex(RES_R,RES_I,B_R,B_I);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

//	B.saveToTextFile("_out_ifft2.txt");
	D_R = B_R - DATA_R;
	D_I = B_I - DATA_I;
//	D.saveToTextFile("_out_dft2_error_diffs.txt");

	float	maxError_R,maxError_I;
	size_t	u,v;
	D_R.find_index_max_value(u,v,maxError_R);
	D_I.find_index_max_value(u,v,maxError_I);

	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n",maxError_R);
	printf("Maximum error between 'A' and 'IFFT(FFT(A))'=%e\n",maxError_I);
}

// ------------------------------------------------------
//				TestImageFFT
// ------------------------------------------------------
void TestImageFFT()
{
	CTicTac			tictac;
	CImage		IM1,IM2;
	CMatrix			imgCorr;

	IM1.loadFromFile(myDataDir+string("fft2_test_image_patch.jpg"), 0 );		// "Patch"
	IM2.loadFromFile(myDataDir+string("fft2_test_image.jpg"), 0 );		// Ref. image

	printf("Computing images correlation...");
	tictac.Tic();
	IM2.cross_correlation_FFT(IM1,imgCorr);
	printf(" Done,%.06fms\n",tictac.Tac()*1000.0f);

	imgCorr.saveToTextFile("_out_dft2_image_test.txt");
}

// ------------------------------------------------------
//				TestImageCap
// ------------------------------------------------------
void TestImage3D()
{
/*	// Pixels -> 3D
	CMatrix		A = VisionUtils::defaultIntrinsicParamsMatrix();
	CPoint3D	p;

	FILE	*f=fopen("test.txt","wt");
	for (int x=0;x<320;x+=10)
		for (int y=0;y<240;y+=10)
			for (int d=1;d<20;d+=1)
			{
				p = VisionUtils::pixelTo3D(x,y, A);
				fprintf(f,"%f %f %f\n",p.x,p.y,p.z);
			}

	fclose(f);

	return;
*/
}

// ------------------------------------------------------
//				TestImageCap
// ------------------------------------------------------
void TestImageConversion()
{

	// BMP -> JPEG conversion tester:
	// --------------------------------
	CImage		img,img2;
	CTicTac			tictac;

	{
		tictac.Tic();
		if (!img.loadFromFile("../imageBasics/frame_color.bmp"))
		{
			cerr << "Error loading ../imageBasics/frame_color.bmp" << endl;
			return;
		}
		printf("bmp file loaded in %.03fms\n", 1000.0f*tictac.Tac() );

		CDisplayWindow		win1("BMP file, color");
		win1.showImage( img );
		win1.waitForKey();

		tictac.Tic();
			img.loadFromFile("frame_gray.bmp");
		printf("bmp file loaded in %.03fms\n", 1000.0f*tictac.Tac() );

		CDisplayWindow		win2("BMP file, gray");

		win2.showImage( img );
		win2.waitForKey();
	}

	tictac.Tic();
		img.loadFromFile("frame.jpg");
	printf("jpeg file loaded in %.03fms\n", 1000.0f*tictac.Tac() );


	CDisplayWindow		win1("win1"),win2("win2"),win3("win3");

	CImage			imgSmall( img.scaleHalf() );
	CImage			imgSmall2( imgSmall.scaleHalf() );
	CImage			imgGray( imgSmall2.grayscale() );


	// Test some draw capabilities:
	// ---------------------------------
	imgSmall.rectangle( 85,35, 170,170,TColor(255,0,0),10);

	imgSmall.line( 550,75, 650,25,TColor(0,0,255) );
	imgSmall.line( -10,-20, 20,30,TColor(0,0,255));

	CMatrix 	COV(2,2);
	COV(0,0) = 100;
	COV(1,1) = 50;
	COV(0,1) = COV(1,0) = -30;
	imgSmall.ellipseGaussian( &COV, 600.0f,50.0f, 2, TColor(255,255,0), 4);
	imgGray.ellipseGaussian( &COV, 100.0f,100.0f, 2, TColor(0,0,255), 4);

	imgSmall.drawImage( 400,500,imgGray );

	// Show the windows now:
	// ------------------------------------------------------
	win1.showImage( imgSmall ); win1.setPos(0,0);
	win2.showImage( imgSmall2 ); win2.setPos(810,0);
	win3.showImage( imgGray ); win3.setPos(810,400);

	os::getch();

	tictac.Tic();
	img2.saveToFile("frame_out.jpg");
	printf("jpeg file loaded in %.03fms\n", 1000.0f*tictac.Tac() );

	return;
}

// ------------------------------------------------------
//				TestImageCap
// ------------------------------------------------------
/*void TestImageCap()
{
	CTicTac							tictac;
	bool							ok = true;

	CImageGrabber_OpenCV		cap( 0 );
	CObservationImage				obs;
	CDisplayWindow					win("Capture");


	while (ok && !_kbhit())
	{
		mrpt::system::sleep(10);
		tictac.Tic();
			ok = cap.getObservation( obs );
		printf("Frame grabbed in %.03fms\n", 1000.0f*tictac.Tac() );


		if (ok)
		{
//			CImageFloat		img2; img2 = obs.image;
//			win.showImage( obs.image.grayscale() );

			win.showImage( obs.image );
		}
	};


	if (_kbhit()) _getch();
}
*/
// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestImageFFT();

		//TestFFT_2D_real();
		//TestFFT_2D_complex();
		//TestImageFFT();
		//TestImageCap();
		//TestImageConversion();

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
