/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h> // for ASSERT_FILE_EXISTS_
#include <mrpt/vision.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::vision;
using namespace std;


// ------------------------------------------------------
//				TestStereoCalibrate
// ------------------------------------------------------
int TestStereoCalibrate(int argc, char** argv)
{
	CTimeLogger  timlog;

	// Parse optional arguments:
	if (argc==1 || ((argc-1)&1)!=0)
	{
		cout<< "Usage:\n"
			<< argv[0] << "left_image1 right_image1 [L2 R2] [L3 R3] [...]\n";
		return -1;
	}

	// The stereo calibration structures:
	TCalibrationStereoImageList calib_imgs;
	TStereoCalibResults         calib_result;
	TStereoCalibParams          calib_params;

	// ============ Set parameters ============
	calib_params.check_size_x = 7;
	calib_params.check_size_y = 9;
	calib_params.check_squares_length_X_meters = 22.83e-3;
	calib_params.check_squares_length_Y_meters = 24.31e-3;
	//calib_params.maxIters = 300;
	//calib_params.verbose = true;
	calib_params.optimize_k1 = true;
	calib_params.optimize_k2 = true;


	// Load images:
	const size_t nPairs = (argc>>1);
	for (size_t i=0;i<nPairs;i++)
	{
		const string sImgL = argv[1+2*i+0];
		const string sImgR = argv[1+2*i+1];
		ASSERT_FILE_EXISTS_(sImgL)
		ASSERT_FILE_EXISTS_(sImgR)

		calib_imgs.resize(calib_imgs.size()+1);
		TImageStereoCalibData &stereo_dat = *calib_imgs.rbegin();
		
#if 1
		// Load all images in memory:
		if (!stereo_dat.left.img_original.loadFromFile(sImgL))  THROW_EXCEPTION_CUSTOM_MSG1("Error loading: %s",sImgL.c_str())
		if (!stereo_dat.right.img_original.loadFromFile(sImgR)) THROW_EXCEPTION_CUSTOM_MSG1("Error loading: %s",sImgR.c_str())
#else
		// Don't load images in memory until really needed.
		stereo_dat.left.img_original.setExternalStorage(sImgL);
		stereo_dat.right.img_original.setExternalStorage(sImgR);
#endif
	}

	// Run calibration:
	bool res = mrpt::vision::checkerBoardStereoCalibration( calib_imgs, calib_params, calib_result );

	if (!res) 
	{
		std::cout << "Calibration returned an error status.\n";
		return -1;
	}
	else
	{
		// Calibration was OK:

		// Show detected corners:
		if (1)
		{
			mrpt::gui::CDisplayWindow3D win("Calibration results",1000,480);

			mrpt::opengl::COpenGLViewportPtr view1, view2;
			{
				mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
				view1 = scene->getViewport("main");
				view2 = scene->createViewport("right");

				// Split viewing area into two halves:
				view1->setViewportPosition(0,0,   0.5,1.0);
				view2->setViewportPosition(0.5,0, 0.5,1.0);

				win.unlockAccess3DScene();
			}

			for (size_t i=0;i<nPairs;i++)
			{
				win.get3DSceneAndLock();

				view1->setImageView( calib_imgs[i].left.img_rectified); // img_checkboard );
				view2->setImageView( calib_imgs[i].right.img_rectified); // img_checkboard );
				
				win.setWindowTitle(mrpt::format("Detected corners: %u / %u", static_cast<unsigned int>(i+1),static_cast<unsigned int>(nPairs) ));

				win.unlockAccess3DScene();
				win.repaint();

				win.waitForKey();
			}
		} // end show detected corners


		return 0;
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		return TestStereoCalibrate(argc,argv);
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
