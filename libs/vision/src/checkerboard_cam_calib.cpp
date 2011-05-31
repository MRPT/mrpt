/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/system/filesystem.h>

#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/chessboard_camera_calib.h>

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

/* -------------------------------------------------------
				checkerBoardCameraCalibration
   ------------------------------------------------------- */
bool mrpt::vision::checkerBoardCameraCalibration(
	TCalibrationImageList &images,
	unsigned int  check_size_x,
	unsigned int  check_size_y,
	double        check_squares_length_X_meters,
	double        check_squares_length_Y_meters,
	CMatrixDouble33			&intrinsicParams,
	std::vector<double>		&distortionParams,
	bool					normalize_image,
	double            *out_MSE,
	bool               skipDrawDetectedImgs,
	bool			   useScaramuzzaAlternativeDetector
	)
{
	// Just a wrapper for the newer version of the function which uses TCamera:
	TCamera  cam;
	bool ret = checkerBoardCameraCalibration(
		images,
		check_size_x,check_size_y,
		check_squares_length_X_meters, check_squares_length_Y_meters,
		cam,
		normalize_image,
		out_MSE,skipDrawDetectedImgs,
		useScaramuzzaAlternativeDetector);

	intrinsicParams = cam.intrinsicParams;
	distortionParams = cam.getDistortionParamsAsVector();
	return ret;
}

/* -------------------------------------------------------
				checkerBoardCameraCalibration
   ------------------------------------------------------- */
bool mrpt::vision::checkerBoardCameraCalibration(
	TCalibrationImageList &images,
	unsigned int  check_size_x,
	unsigned int  check_size_y,
	double        check_squares_length_X_meters,
	double        check_squares_length_Y_meters,
	mrpt::utils::TCamera    &out_camera_params,
	bool					normalize_image,
	double            *out_MSE,
	bool               skipDrawDetectedImgs,
	bool			   useScaramuzzaAlternativeDetector
	)
{
#if MRPT_HAS_OPENCV
	try
	{
		ASSERT_(check_size_x>2);
		ASSERT_(check_size_y>2);
		ASSERT_(check_squares_length_X_meters>0);
		ASSERT_(check_squares_length_Y_meters>0);

		if (images.size()<1)
		{
			std::cout << "ERROR: No input images." << std::endl;
			return false;
		}

		const unsigned CORNERS_COUNT = check_size_x * check_size_y;
		const CvSize check_size = cvSize(check_size_x, check_size_y);

		// First: Assure all images are loaded:
		// -------------------------------------------
		TCalibrationImageList::iterator it;
		for (it=images.begin();it!=images.end();it++)
		{
			TImageCalibData	&dat = it->second;

			dat.projectedPoints_distorted.clear();  // Clear reprojected points.
			dat.projectedPoints_undistorted.clear();

			// Skip if images are marked as "externalStorage":
			if (!dat.img_original.isExternallyStored() && !mrpt::system::extractFileExtension(it->first).empty()  )
			{
				if (!dat.img_original.loadFromFile(it->first))
					THROW_EXCEPTION_CUSTOM_MSG1("Error reading image: %s",it->first.c_str());

				dat.img_checkboard = dat.img_original;
				dat.img_rectified  = dat.img_original;
			}
		}

		// For each image, find checkerboard corners:
		// -----------------------------------------------
		//const unsigned int N = images.size();
		unsigned int i;

        vector<CvPoint2D32f> corners_list; //  = new CvPoint2D32f[ N * CORNERS_COUNT];
        unsigned int  valid_detected_imgs = 0;

        CvSize	imgSize = cvSize(0,0);

		vector<string>   pointsIdx2imageFile;

		int find_chess_flags = CV_CALIB_CB_ADAPTIVE_THRESH;
		if (normalize_image)
			find_chess_flags |= CV_CALIB_CB_NORMALIZE_IMAGE;

		for (i=0,it=images.begin();it!=images.end();it++,i++)
		{
			TImageCalibData	&dat = it->second;

			// Make grayscale version:
			const CImage img_gray( dat.img_original, FAST_REF_OR_CONVERT_TO_GRAY );

			if (!i)
			{
				imgSize = cvSize(img_gray.getWidth(),img_gray.getHeight() );
				out_camera_params.ncols = imgSize.width;
				out_camera_params.nrows = imgSize.height;
			}
			else
			{
				if (imgSize.height != (int)img_gray.getHeight() || imgSize.width != (int)img_gray.getWidth())
				{
					std::cout << "ERROR: All the images must have the same size" << std::endl;
					return false;
				}
			}

			// Try with expanded versions of the image if it fails to detect the checkerboard:
			unsigned corners_count;
			bool corners_found=false;

			corners_count = CORNERS_COUNT;

			corners_list.resize( (1+valid_detected_imgs)*CORNERS_COUNT );

			dat.detected_corners.clear();

			// Do detection (this includes the "refine corners" with cvFindCornerSubPix):
			vector<TPixelCoordf> detectedCoords;
			corners_found = mrpt::vision::findChessboardCorners(
				img_gray,
				detectedCoords,
				check_size_x,check_size_y,
				normalize_image, // normalize_image
				useScaramuzzaAlternativeDetector
				);

			corners_count = detectedCoords.size();

			// Copy the data into the overall array of coords:
			ASSERT_(detectedCoords.size()<=CORNERS_COUNT);
			for (size_t p=0;p<detectedCoords.size();p++)
			{
				corners_list[valid_detected_imgs*CORNERS_COUNT+p].x = detectedCoords[p].x;
				corners_list[valid_detected_imgs*CORNERS_COUNT+p].y = detectedCoords[p].y;
			}

			if (corners_found && corners_count!=CORNERS_COUNT)
				corners_found = false;


			cout << format("Img %s: %s\n", mrpt::system::extractFileName(it->first).c_str() , corners_found ? "DETECTED" : "NOT DETECTED" );

			if( corners_found )
			{
				// save the corners in the data structure:
				int x, y;
				unsigned int k;
				for( y = 0, k = 0; y < check_size.height; y++ )
					for( x = 0; x < check_size.width; x++, k++ )
						dat.detected_corners.push_back( mrpt::utils::TPixelCoordf( corners_list[valid_detected_imgs*CORNERS_COUNT + k].x, corners_list[valid_detected_imgs*CORNERS_COUNT + k].y ) );

				// Draw the checkerboard in the corresponding image:
				// ----------------------------------------------------
				if ( !dat.img_original.isExternallyStored() )
				{
					const int r = 4;
					CvPoint prev_pt = { 0, 0};
					const int line_max = 8;
					CvScalar line_colors[8];

					line_colors[0] = CV_RGB(255,0,0);
					line_colors[1] = CV_RGB(255,128,0);
					line_colors[2] = CV_RGB(255,128,0);
					line_colors[3] = CV_RGB(200,200,0);
					line_colors[4] = CV_RGB(0,255,0);
					line_colors[5] = CV_RGB(0,200,200);
					line_colors[6] = CV_RGB(0,0,255);
					line_colors[7] = CV_RGB(255,0,255);

					// Checkboad as color image:
					dat.img_original.colorImage( dat.img_checkboard );

					void *rgb_img = dat.img_checkboard.getAsIplImage();

					for( y = 0, k = 0; y < check_size.height; y++ )
					{
						CvScalar color = line_colors[y % line_max];
						for( x = 0; x < check_size.width; x++, k++ )
						{
							CvPoint pt;
							pt.x = cvRound(corners_list[valid_detected_imgs*CORNERS_COUNT + k].x);
							pt.y = cvRound(corners_list[valid_detected_imgs*CORNERS_COUNT + k].y);

							if( k != 0 ) cvLine( rgb_img, prev_pt, pt, color );

							cvLine( rgb_img,
									  cvPoint( pt.x - r, pt.y - r ),
									  cvPoint( pt.x + r, pt.y + r ), color );
							cvLine( rgb_img,
									  cvPoint( pt.x - r, pt.y + r),
									  cvPoint( pt.x + r, pt.y - r), color );
							cvCircle( rgb_img, pt, r+1, color );
							prev_pt = pt;
						}
					}
				}
			}

			if( corners_found )
			{
				pointsIdx2imageFile.push_back( it->first );
				valid_detected_imgs++;
			}

		} // end find corners

		std::cout << valid_detected_imgs << " valid images." << std::endl;
		if (!valid_detected_imgs)
		{
			std::cout << "ERROR: No valid images. Perhaps the checkerboard size is incorrect?" << std::endl;
			return false;
		}

		// ---------------------------------------------
		// Calculate the camera parameters
		// ---------------------------------------------
		// Was: FillEtalonObjPoints
		vector<CvPoint3D32f> obj_points( valid_detected_imgs * CORNERS_COUNT );

		{
			unsigned int y,k;
			for( y = 0, k = 0; y < check_size_y; y++ )
			{
				for( unsigned int x = 0; x < check_size_x; x++, k++ )
				{
					obj_points[k].x =-check_squares_length_X_meters * x;  // The "-" is for convenience, so the camera poses appear with Z>0
					obj_points[k].y = check_squares_length_Y_meters * y;
					obj_points[k].z = 0;
				}
			}
		}

		// Repeat the pattern N times:
		for( i= 1; i< valid_detected_imgs; i++ )
			memcpy( &obj_points[CORNERS_COUNT*i], &obj_points[0], CORNERS_COUNT*sizeof(obj_points[0]));

		// Number of detected points in each image (constant):
		vector<int> numsPoints(valid_detected_imgs, (int)CORNERS_COUNT );

		float   proj_matrix[9];
		float   distortion[4];
		//float   rotMatr[9];
		//float   transVect[3];

		vector<CvPoint3D32f> transVects( valid_detected_imgs );
        vector<float>        rotMatrs( valid_detected_imgs * 9 );

		// Calirate camera
		cvCalibrateCamera(
			valid_detected_imgs,
			&numsPoints[0],
			imgSize,
			&corners_list[0],
			&obj_points[0],
			distortion,
			proj_matrix,
			(float*)&transVects[0],
			&rotMatrs[0],
			0 );

		// Load matrix:
		//out_camera_params.intrinsicParams = CMatrixDouble33( CMatrixFloat33( proj_matrix ) );
		out_camera_params.intrinsicParams = CMatrixFloat33( proj_matrix ).cast<double>();

		out_camera_params.dist.assign(0);
		for (int i=0;i<4;i++)
			out_camera_params.dist[i] = distortion[i];

		// Load camera poses:
		for (i=0;i<valid_detected_imgs;i++)
		{
			float *R = &rotMatrs[9*i];

			CMatrixDouble HM(4,4);
			HM.zeros();
			HM(3,3)=1;

			HM(0,0)=R[0];
			HM(1,0)=R[3];
			HM(2,0)=R[6];

			HM(0,1)=R[1];
			HM(1,1)=R[4];
			HM(2,1)=R[7];

			HM(0,2)=R[2];
			HM(1,2)=R[5];
			HM(2,2)=R[8];

			HM(0,3)=transVects[i].x;
			HM(1,3)=transVects[i].y;
			HM(2,3)=transVects[i].z;

			CPose3D p = CPose3D(0,0,0) - CPose3D(HM);

			images[ pointsIdx2imageFile[i] ].reconstructed_camera_pose = p;

			std::cout << "Img: " <<  mrpt::system::extractFileName(pointsIdx2imageFile[i])  << ": " << p << std::endl;
		}

		{
			CConfigFileMemory cfg;
			out_camera_params.saveToConfigFile("CAMERA_PARAMS",cfg);
			std::cout << cfg.getContent() << std::endl;
		}

		// ----------------------------------------
		// Undistort images:
		// ----------------------------------------
		for (it=images.begin();it!=images.end();it++)
		{
			TImageCalibData	&dat = it->second;
			if (!dat.img_original.isExternallyStored())
				dat.img_original.rectifyImage( dat.img_rectified, out_camera_params);
		} // end undistort

		// -----------------------------------------------
		// Reproject points to measure the fit sqr error
		// -----------------------------------------------
		double sqrErr = 0;

		for (i=0;i<valid_detected_imgs;i++)
		{
			TImageCalibData  & dat = images[ pointsIdx2imageFile[i] ];
			if (dat.detected_corners.size()!=CORNERS_COUNT) continue;

			// Reproject all the points into pixel coordinates:
			// -----------------------------------------------------

			vector<CPoint3D>  lstPatternPoints(CORNERS_COUNT);	// Points as seen from the camera:
			for (unsigned int p=0;p<CORNERS_COUNT;p++)
				lstPatternPoints[p] = CPoint3D(obj_points[p].x,obj_points[p].y,obj_points[p].z);

			vector<TPixelCoordf>	&projectedPoints = dat.projectedPoints_undistorted;
			vector<TPixelCoordf>	&projectedPoints_distorted = dat.projectedPoints_distorted;

			vision::pinhole::projectPoints_no_distortion(
				lstPatternPoints, // Input points
				dat.reconstructed_camera_pose,
				out_camera_params.intrinsicParams, // calib matrix
				projectedPoints  // Output points in pixels
				);

			vision::pinhole::projectPoints_with_distortion(
				lstPatternPoints, // Input points
				dat.reconstructed_camera_pose,
				out_camera_params.intrinsicParams, // calib matrix
				out_camera_params.getDistortionParamsAsVector(),
				projectedPoints_distorted// Output points in pixels
				);

			ASSERT_(projectedPoints.size()==CORNERS_COUNT);
			ASSERT_(projectedPoints_distorted.size()==CORNERS_COUNT);


			for (unsigned int p=0;p<CORNERS_COUNT;p++)
			{
				const double px = projectedPoints[p].x;
				const double py = projectedPoints[p].y;

				const double px_d = projectedPoints_distorted[p].x;
				const double py_d = projectedPoints_distorted[p].y;

				// Only draw if the img is NOT external:
				if (!dat.img_original.isExternallyStored())
				{
					if( px >= 0 && px < imgSize.width && py >= 0 && py < imgSize.height )
						cvCircle( dat.img_rectified.getAsIplImage(), cvPoint(px,py), 4, CV_RGB(0,0,255) );
				}

				// Accumulate error:
				sqrErr+=square(px_d-dat.detected_corners[p].x)+square(py_d-dat.detected_corners[p].y); // Error relative to the original (distorted) image.
			}
		}

		if (valid_detected_imgs)
		{
			sqrErr /= CORNERS_COUNT*valid_detected_imgs;
			std::cout << "Average err. of reprojection: " << sqrt(sqrErr) << " pixels" << std::endl;
		}
		if(out_MSE) *out_MSE = sqrt(sqrErr);

		return true;
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return false;
	}
#else
	THROW_EXCEPTION("Function not available: MRPT was compiled without OpenCV")
#endif
}




