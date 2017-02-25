/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/random.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::vision;
using namespace std;

extern void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img );

// ------------------------------------------------------
//				Benchmark: image loading/saving
// ------------------------------------------------------
double image_test_1(int w, int img_quality)
{
    int h=0;
    switch(w)
    {
        case 640: h=480; break;
        case 800: h=600; break;
        case 1024: h=768; break;
        case 1280: h=1024; break;
        default: THROW_EXCEPTION("Invalid 'w'!");
    }

	CImage  img(w,h,3);

	for (int i=0;i<5000;i++)
		img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), TColor( randomGenerator.drawUniform32bit() )  );

	CTicTac	 tictac;

	const string fil = mrpt::system::getTempFileName()+".jpg";

	const size_t N = 30;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		img.saveToFile(fil,img_quality);

    const double T = tictac.Tac()/N;
    mrpt::system::deleteFile(fil);
	return T;
}

// ------------------------------------------------------
//				Benchmark: save/load to disk vs shared mem
// ------------------------------------------------------
template <bool perf_load>
double image_saveload(int iFormat, int to_shm)
{
    const char* format;
    switch (iFormat)
    {
        case 0: format = "bmp"; break;
        case 1: format = "png"; break;
        case 2: format = "jpg"; break;
        default: THROW_EXCEPTION("Wrong format")
    }

    const int w=800, h=600;
	CImage  img(w,h,3);
	for (int i=0;i<5000;i++)
		img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), TColor( randomGenerator.drawUniform32bit() )  );

	CTicTac	 tictac;

	string fil;
	if (to_shm) {
        fil = string("/dev/shm/mrpt_perf_test.") + string(format);
    }
    else {
        fil = mrpt::system::getTempFileName()+ string(".") + string(format);
    }

    double T;
    if (perf_load)
    {
        // LOAD:
        img.saveToFile(fil);

        const size_t N = 30;

        tictac.Tic();
        for (size_t i=0;i<N;i++)
        {
            CImage img_new;
            img.loadFromFile(fil);
        }

        T = tictac.Tac()/N;
    }
    else
    {
        // WRITE:
        const size_t N = 30;

        tictac.Tic();
        for (size_t i=0;i<N;i++)
            img.saveToFile(fil);

        T = tictac.Tac()/N;
    }

    mrpt::system::deleteFile(fil);
	return T;
}



double image_test_2(int w, int h)
{
	CImage  img(w,h,3), img2;

#if MRPT_HAS_OPENCV
//	int oldVal = cvUseOptimized(1);
#endif

	for (int i=0;i<5000;i++)
		img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), TColor( randomGenerator.drawUniform32bit() ) );

	CTicTac	 tictac;

	const size_t N = 50;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		img.filterGaussian(img2,7,7);

	double R = tictac.Tac()/N;

#if MRPT_HAS_OPENCV
//	cvUseOptimized(oldVal);
#endif
	return R;
}

template <int IMG_CHANNELS>
double image_halfsample(int w, int h)
{
	CImage  img(w,h,IMG_CHANNELS), img2;

	CTicTac	 tictac;

	const size_t N = 300;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		img.scaleHalf(img2);

	return tictac.Tac()/N;
}

template <int IMG_CHANNELS>
double image_halfsample_smooth(int w, int h)
{
	CImage  img(w,h,IMG_CHANNELS), img2;

	CTicTac	 tictac;

	const size_t N = 300;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		img.scaleHalfSmooth(img2);

	return tictac.Tac()/N;
}


double image_rgb2gray_8u(int w, int h)
{
	CImage  img(w,h,CH_RGB), img2;

	CTicTac	 tictac;

	const size_t N = 300;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		img.grayscale(img2);

	return tictac.Tac()/N;
}

double image_KLTscore(int WIN, int N)
{
	static const size_t w = 800;
	static const size_t h = 800;
	CImage  img(w,h,CH_GRAY);

	for (int i=0;i<5000;i++)
		img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), TColor( randomGenerator.drawUniform32bit() ) );

	ASSERT_BELOW_(WIN,128)
	int x = 0;
	int y = 0;

	CTicTac	 tictac;
	tictac.Tic();
	for (int i=0;i<N;i++)
	{
		//float r =
		 img.KLT_response(x | 128,y | 128,WIN);
		x++; x &= 0x1FF;
		y++; y &= 0x1FF;
	}

	double R = tictac.Tac()/N;

	return R;
}

template <bool DO_SMOOTH, bool CONVERT_GRAY>
double image_buildPyramid(int N, int NOCTS)
{
	// Get a real image for testing:
	CImage  img;
	getTestImage(0,img);

	mrpt::vision::CImagePyramid pyr;
	// Run once in advance not to count the memory reservation:
	pyr.buildPyramid(img,NOCTS,DO_SMOOTH,CONVERT_GRAY);

	CTicTac	 tictac;
	tictac.Tic();
	for (int i=0;i<N;i++)
	{
		pyr.buildPyramid(img,NOCTS,DO_SMOOTH,CONVERT_GRAY);
	}
	double R = tictac.Tac()/N;
	return R;
}


const char* EXAMPLE_STEREO_CALIB =
    "[CAMERA_PARAMS_LEFT]\n"
    "resolution = [1024 768]\n"
    "cx         = 523.098290\n"
    "cy         = 402.599910\n"
    "fx         = 839.161865\n"
    "fy         = 841.168710\n"
    "dist       = [-3.747584e-001 2.468493e-001 0.000000e+000 0.000000e+000 -1.281381e-001]    // The order is: [K1 K2 T1 T2 K3]\n"
    "[CAMERA_PARAMS_RIGHT]\n"
    "resolution = [1024 768]\n"
    "cx         = 511.173904\n"
    "cy         = 388.440870\n"
    "fx         = 836.761931\n"
    "fy         = 838.133088\n"
    "dist       = [-3.606276e-001 2.116929e-001 0.000000e+000 0.000000e+000 -9.216787e-002]    // The order is: [K1 K2 T1 T2 K3]\n"
    "[CAMERA_PARAMS_LEFT2RIGHT_POSE]\n"
    "translation_only     = [1.193203e-001 -3.361637e-004 3.729054e-004]\n"
    "rotation_matrix_only = [9.9997534298507e-001 -6.2827062556904e-003 -3.1370406439725e-003 ;6.2847721003742e-003 9.9998003994880e-001 6.4910988856303e-004 ;3.1328998617231e-003 -6.6880946896783e-004 9.9999486880301e-001 ]\n"
    "pose_yaw_pitch_roll  = [0.119320 -0.000336 0.000373 0.360095 -0.179502 -0.038320]\n"
    "pose_quaternion      = [0.119320 -0.000336 0.000373 0.999994 -0.000329 -0.001567 0.003142]\n"
    ;


template <int IMG_CHANNELS,int w, int h, int w2, int h2>
double stereoimage_rectify_prepare_map(int, int)
{
    mrpt::utils::TStereoCamera params;
    params.loadFromConfigFile("CAMERA_PARAMS", mrpt::utils::CConfigFileMemory(std::string(EXAMPLE_STEREO_CALIB)) );
    params.scaleToResolution(w,h);

	mrpt::vision::CStereoRectifyMap  rectify_map;
	rectify_map.enableResizeOutput((w2!=w || h2!=h), w2,h2);

	CTicTac	 tictac;

	const size_t N = 100;
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
	    rectify_map.setAlpha(-1); // This invalidates the old map & forces recomputing
        rectify_map.setFromCamParams(params);
	}

	return tictac.Tac()/N;
}

template <int IMG_CHANNELS,int w, int h, int w2, int h2>
double stereoimage_rectify(int, int)
{
	const CImage  imgL(w,h,IMG_CHANNELS), imgR(w,h,IMG_CHANNELS);
	CImage  imgL2, imgR2;

    mrpt::utils::TStereoCamera params;
    params.loadFromConfigFile("CAMERA_PARAMS", mrpt::utils::CConfigFileMemory(std::string(EXAMPLE_STEREO_CALIB)) );
    params.scaleToResolution(w,h);

	mrpt::vision::CStereoRectifyMap  rectify_map;
	rectify_map.enableResizeOutput((w2!=w || h2!=h), w2,h2);
	rectify_map.setFromCamParams(params);

	CTicTac	 tictac;
	const size_t N = 20;
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
	    rectify_map.rectify(imgL,imgR, imgL2,imgR2);
	}

	return tictac.Tac()/N;
}

// ------------------------------------------------------
// register_tests_image
// ------------------------------------------------------
void register_tests_image()
{
	lstTests.push_back( TestData("images: Save as JPEG (640x480, quality=95%)",image_test_1,  640,  95 ) );
	lstTests.push_back( TestData("images: Save as JPEG (800x600, quality=95%)",image_test_1,  800,  95) );
	lstTests.push_back( TestData("images: Save as JPEG (1024x768, quality=95%)",image_test_1, 1024, 95) );

	lstTests.push_back( TestData("images: Save as JPEG (640x480, quality=75%)",image_test_1,  640,  75 ) );
	lstTests.push_back( TestData("images: Save as JPEG (800x600, quality=75%)",image_test_1,  800,  75) );
	lstTests.push_back( TestData("images: Save as JPEG (1024x768, quality=75%)",image_test_1, 1024, 75) );

	lstTests.push_back( TestData("images: Save BMP 800x600 disk",image_saveload<false>, 0, 0  ));
	lstTests.push_back( TestData("images: Save PNG 800x600 disk",image_saveload<false>, 1, 0 ));
	lstTests.push_back( TestData("images: Save JPG 800x600 disk",image_saveload<false>, 2, 0 ));
	lstTests.push_back( TestData("images: Load BMP 800x600 disk",image_saveload<true>, 0, 0 ));
	lstTests.push_back( TestData("images: Load PNG 800x600 disk",image_saveload<true>, 1, 0 ));
	lstTests.push_back( TestData("images: Load JPG 800x600 disk",image_saveload<true>, 2, 0 ));

    if (mrpt::system::directoryExists("/dev/shm"))
    {
        lstTests.push_back( TestData("images: Save BMP 800x600 shared mem",image_saveload<false>, 0, 1 ) );
        lstTests.push_back( TestData("images: Save PNG 800x600 shared mem",image_saveload<false>, 1, 1) );
        lstTests.push_back( TestData("images: Save JPG 800x600 shared mem",image_saveload<false>, 2, 1) );
        lstTests.push_back( TestData("images: Load BMP 800x600 shared mem",image_saveload<true>, 0, 1) );
        lstTests.push_back( TestData("images: Load PNG 800x600 shared mem",image_saveload<true>, 1, 1) );
        lstTests.push_back( TestData("images: Load JPG 800x600 shared mem",image_saveload<true>, 2, 1) );
    }

	lstTests.push_back( TestData("images: Gauss filter (640x480)",image_test_2,  640,480) );
	lstTests.push_back( TestData("images: Gauss filter (800x600)",image_test_2,  800,600) );
	lstTests.push_back( TestData("images: Gauss filter (1024x768)",image_test_2,  1024,768) );

	lstTests.push_back( TestData("images: Half sample GRAY (160x120)",image_halfsample<CH_GRAY>,  160,120) );
	lstTests.push_back( TestData("images: Half sample GRAY (320x240)",image_halfsample<CH_GRAY>,  320,240) );
	lstTests.push_back( TestData("images: Half sample GRAY (640x480)",image_halfsample<CH_GRAY>,  640,480) );
	lstTests.push_back( TestData("images: Half sample GRAY (800x600)",image_halfsample<CH_GRAY>,  800,600) );
	lstTests.push_back( TestData("images: Half sample GRAY (1024x768)",image_halfsample<CH_GRAY>,  1024,768) );
	lstTests.push_back( TestData("images: Half sample GRAY (1280x1024)",image_halfsample<CH_GRAY>,  1280,1024) );

	lstTests.push_back( TestData("images: Half sample RGB (160x120)",image_halfsample<CH_RGB>,  160,120) );
	lstTests.push_back( TestData("images: Half sample RGB (320x240)",image_halfsample<CH_RGB>,  320,240) );
	lstTests.push_back( TestData("images: Half sample RGB (640x480)",image_halfsample<CH_RGB>,  640,480) );
	lstTests.push_back( TestData("images: Half sample RGB (800x600)",image_halfsample<CH_RGB>,  800,600) );
	lstTests.push_back( TestData("images: Half sample RGB (1024x768)",image_halfsample<CH_RGB>,  1024,768) );
	lstTests.push_back( TestData("images: Half sample RGB (1280x1024)",image_halfsample<CH_RGB>,  1280,1024) );

	lstTests.push_back( TestData("images: Half sample smooth GRAY (160x120)",image_halfsample_smooth<CH_GRAY>,  160,120) );
	lstTests.push_back( TestData("images: Half sample smooth GRAY (320x240)",image_halfsample_smooth<CH_GRAY>,  320,240) );
	lstTests.push_back( TestData("images: Half sample smooth GRAY (640x480)",image_halfsample_smooth<CH_GRAY>,  640,480) );
	lstTests.push_back( TestData("images: Half sample smooth GRAY (800x600)",image_halfsample_smooth<CH_GRAY>,  800,600) );
	lstTests.push_back( TestData("images: Half sample smooth GRAY (1024x768)",image_halfsample_smooth<CH_GRAY>,  1024,768) );
	lstTests.push_back( TestData("images: Half sample smooth GRAY (1280x1024)",image_halfsample_smooth<CH_GRAY>,  1280,1024) );

	lstTests.push_back( TestData("images: Half sample smooth RGB (160x120)",image_halfsample_smooth<CH_RGB>,  160,120) );
	lstTests.push_back( TestData("images: Half sample smooth RGB (320x240)",image_halfsample_smooth<CH_RGB>,  320,240) );
	lstTests.push_back( TestData("images: Half sample smooth RGB (640x480)",image_halfsample_smooth<CH_RGB>,  640,480) );
	lstTests.push_back( TestData("images: Half sample smooth RGB (800x600)",image_halfsample_smooth<CH_RGB>,  800,600) );
	lstTests.push_back( TestData("images: Half sample smooth RGB (1024x768)",image_halfsample_smooth<CH_RGB>,  1024,768) );
	lstTests.push_back( TestData("images: Half sample smooth RGB (1280x1024)",image_halfsample_smooth<CH_RGB>,  1280,1024) );


	lstTests.push_back( TestData("images: RGB->GRAY 8u (40x30)",image_rgb2gray_8u,  40,30) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (80x60)",image_rgb2gray_8u,  80,60) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (160x120)",image_rgb2gray_8u,  160,120) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (320x240)",image_rgb2gray_8u,  320,240) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (640x480)",image_rgb2gray_8u,  640,480) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (800x600)",image_rgb2gray_8u,  800,600) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (1024x768)",image_rgb2gray_8u,  1024,768) );
	lstTests.push_back( TestData("images: RGB->GRAY 8u (1280x1024)",image_rgb2gray_8u,  1280,1024) );

	lstTests.push_back( TestData("images: KLT score (WIN=2 5x5)",image_KLTscore, 2,  1e7) );
	lstTests.push_back( TestData("images: KLT score (WIN=3 7x7)",image_KLTscore, 3,  1e7) );
	lstTests.push_back( TestData("images: KLT score (WIN=4 9x9)",image_KLTscore, 4,  1e7) );
	lstTests.push_back( TestData("images: KLT score (WIN=5 10x10)",image_KLTscore, 5,  1e7) );
	lstTests.push_back( TestData("images: KLT score (WIN=6 13x13)",image_KLTscore, 6,  1e7) );
	lstTests.push_back( TestData("images: KLT score (WIN=7 15x15)",image_KLTscore, 7,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=8 17x17)",image_KLTscore, 8,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=9 19x19)",image_KLTscore, 9,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=10 21x21)",image_KLTscore, 10,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=11 23x23)",image_KLTscore, 11,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=12 25x25)",image_KLTscore, 12,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=13 27x27)",image_KLTscore, 13,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=14 29x29)",image_KLTscore, 14,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=15 31x31)",image_KLTscore, 15,  1e6) );
	lstTests.push_back( TestData("images: KLT score (WIN=16 33x33)",image_KLTscore, 16,  1e6) );

	lstTests.push_back( TestData("images: buildPyramid 640x480,4 levs,no smooth,no gray", image_buildPyramid<false,false>, 500, 4) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,4 levs,   smooth,no gray",    image_buildPyramid<true,false>, 500, 4) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,4 levs,no smooth,   gray",    image_buildPyramid<false,true>, 500, 4) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,4 levs,   smooth,   gray",       image_buildPyramid<true,true>, 500, 4) );

	lstTests.push_back( TestData("images: buildPyramid 640x480,8 levs,no smooth,no gray", image_buildPyramid<false,false>, 500, 8) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,8 levs,   smooth,no gray",    image_buildPyramid<true,false>, 500, 8) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,8 levs,no smooth,   gray",    image_buildPyramid<false,true>, 500, 8) );
	lstTests.push_back( TestData("images: buildPyramid 640x480,8 levs,   smooth,   gray",       image_buildPyramid<true,true>, 500, 8) );


	lstTests.push_back( TestData("stereo: prepare rectify map 640x480 RGB",  stereoimage_rectify_prepare_map<CH_RGB,640,480,640,480>) );
	lstTests.push_back( TestData("stereo: prepare rectify map 800x600 RGB", stereoimage_rectify_prepare_map<CH_RGB,800,600,800,600>) );
	lstTests.push_back( TestData("stereo: prepare rectify map 1024x768 RGB", stereoimage_rectify_prepare_map<CH_RGB,1024,768,1024,768>) );
	lstTests.push_back( TestData("stereo: prepare rectify map 1024x768->800x600 RGB", stereoimage_rectify_prepare_map<CH_RGB,1024,768,800,600>) );
	lstTests.push_back( TestData("stereo: prepare rectify map 1024x768->640x480 RGB", stereoimage_rectify_prepare_map<CH_RGB,1024,768,640,480>) );

	lstTests.push_back( TestData("stereo: rectify 640x480 RGB",  stereoimage_rectify<CH_RGB,640,480,640,480>) );
	lstTests.push_back( TestData("stereo: rectify 800x600 RGB", stereoimage_rectify<CH_RGB,800,600,800,600>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768 RGB", stereoimage_rectify<CH_RGB,1024,768,1024,768>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768->800x600 RGB", stereoimage_rectify<CH_RGB,1024,768,800,600>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768->640x480 RGB", stereoimage_rectify<CH_RGB,1024,768,640,480>) );

	lstTests.push_back( TestData("stereo: rectify 640x480 GRAY",  stereoimage_rectify<CH_GRAY,640,480,640,480>) );
	lstTests.push_back( TestData("stereo: rectify 800x600 GRAY", stereoimage_rectify<CH_GRAY,800,600,800,600>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768 GRAY", stereoimage_rectify<CH_GRAY,1024,768,1024,768>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768->800x600 GRAY", stereoimage_rectify<CH_GRAY,1024,768,800,600>) );
	lstTests.push_back( TestData("stereo: rectify 1024x768->640x480 GRAY", stereoimage_rectify<CH_GRAY,1024,768,640,480>) );

}


