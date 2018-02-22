/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// -----------------------------------------------------------------------------------------------------------------------
// For this example, the rawlog file must contain both laser data and stereo
// images (only the left one will be considered)
// It may be used with single image observations -> just employ
// "CObservationImage::Ptr" instead of "CObservationStereoImages::Ptr"
// and access to the contained "image" instead of "imageLeft".
// -----------------------------------------------------------------------------------------------------------------------

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/serialization/CArchive.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace std;

#include <mrpt/examples_config.h>

// Default path, or user path passed thru command line:
std::string RAWLOG_FILE = MRPT_EXAMPLES_BASE_DIRECTORY
	"../share/mrpt/datasets/localization_demo.rawlog";

// ------------------------------------------------------
//                  TestGeometry3D
// ------------------------------------------------------
void TestLaser2Imgs()
{
	// Set your rawlog file name
	if (!mrpt::system::fileExists(RAWLOG_FILE))
		THROW_EXCEPTION_FMT(
			"Rawlog file does not exist: %s", RAWLOG_FILE.c_str())

	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;
	size_t rawlogEntry = 0;
	// bool					end 			= false;
	CDisplayWindow wind;

	// Set relative path for externally-stored images in rawlogs:
	string rawlog_images_path = extractFileDirectory(RAWLOG_FILE);
	rawlog_images_path += "/Images";
	CImage::setImagesPathBase(rawlog_images_path);  // Set it.

	mrpt::io::CFileGZInputStream rawlogFile(RAWLOG_FILE);

	for (;;)
	{
		if (os::kbhit())
		{
			char c = os::getch();
			if (c == 27) break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		cout << "Reading act/oct pair from rawlog..." << endl;
		auto arch = mrpt::serialization::archiveFrom(rawlogFile);
		if (!CRawlog::readActionObservationPair(
				arch, action, observations, rawlogEntry))
			break;  // file EOF

		// CAMERA............
		// Get CObservationStereoImages
		CObservationStereoImages::Ptr sImgs;
		CObservationImage::Ptr Img;

		sImgs = observations->getObservationByClass<CObservationStereoImages>();
		if (!sImgs)
		{
			Img = observations->getObservationByClass<CObservationImage>();
			if (!Img) continue;
		}

		CPose3D cameraPose;  // Get Camera Pose (B) (CPose3D)
		CMatrixDouble33 K;  // Get Calibration matrix (K)

		sImgs ? sImgs->getSensorPose(cameraPose)
			  : Img->getSensorPose(cameraPose);
		K = sImgs ? sImgs->leftCamera.intrinsicParams
				  : Img->cameraParams.intrinsicParams;

		// LASER.............
		// Get CObservationRange2D
		CObservation2DRangeScan::Ptr laserScan =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (!laserScan) continue;

		// Get Laser Pose (A) (CPose3D)
		CPose3D laserPose;
		laserScan->getSensorPose(laserPose);

		if (abs(laserPose.yaw()) > DEG2RAD(90)) continue;  // Only front lasers

		// Get 3D Point relative to the Laser coordinate Frame (P1) (CPoint3D)
		CPoint3D point;
		CSimplePointsMap mapa;
		mapa.insertionOptions.minDistBetweenLaserPoints = 0;
		observations->insertObservationsInto(
			&mapa);  // <- The map contains the pose of the points (P1)

		// Get the points into the map
		vector<float> X, Y, Z;
		vector<float>::iterator itX, itY, itZ;
		mapa.getAllPoints(X, Y, Z);

		unsigned int imgW =
			sImgs ? sImgs->imageLeft.getWidth() : Img->image.getWidth();
		unsigned int imgH =
			sImgs ? sImgs->imageLeft.getHeight() : Img->image.getHeight();

		// unsigned int			idx = 0;
		vector<float> imgX, imgY;
		vector<float>::iterator itImgX, itImgY;
		imgX.resize(X.size());
		imgY.resize(Y.size());

		CImage image;
		image = sImgs ? sImgs->imageLeft : Img->image;

		// Get pixels in the image:
		// Pimg = (kx,ky,k)^T = K(I|0)*P2
		// Main loop
		for (itX = X.begin(), itY = Y.begin(), itZ = Z.begin(),
			itImgX = imgX.begin(), itImgY = imgY.begin();
			 itX != X.end(); itX++, itY++, itZ++, itImgX++, itImgY++)
		{
			// Coordinates Transformation
			CPoint3D pLaser(*itX, *itY, *itZ);
			CPoint3D pCamera(pLaser - cameraPose);

			if (pCamera.z() > 0)
			{
				*itImgX = -K(0, 0) * ((pCamera.x()) / (pCamera.z())) + K(0, 2);
				*itImgY = -K(1, 1) * ((pCamera.y()) / (pCamera.z())) + K(1, 2);

				if (*itImgX > 0 && *itImgX<imgW&& * itImgY> 0 && *itImgY < imgH)
					image.filledRectangle(
						*itImgX - 1, *itImgY - 1, *itImgX + 1, *itImgY + 1,
						TColor(255, 0, 0));
			}  // end if
		}  // end for

		action.reset();
		observations.reset();

		wind.showImage(image);

		std::this_thread::sleep_for(50ms);
	};  // end for

	mrpt::system::pause();
}

// ------------------------------------------------------
//                        MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		if (argc > 1)
		{
			RAWLOG_FILE = std::string(argv[1]);
		}

		TestLaser2Imgs();
		return 0;
	}
	catch (exception& e)
	{
		cerr << "EXCEPTION: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cerr << "Untyped exception!!";
		return -1;
	}
}
