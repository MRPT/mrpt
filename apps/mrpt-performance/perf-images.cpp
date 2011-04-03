/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/slam.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace std;

#if MRPT_HAS_OPENCV
	#include <cxcore.h>
#endif

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
}


