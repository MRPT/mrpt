/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFileMemory.h>

#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/chessboard_camera_calib.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
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

#if MRPT_HAS_OPENCV
// JL says:  This was copied here since it seems OpenCV 2.3 had a broken <opencv2/core/eigen.hpp> header.
//  It should be removed in the future when 2.3 becomes too old to support.
namespace cv {
template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void my_cv2eigen( const Mat& src,
			   Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst )
{
	CV_DbgAssert(src.rows == _rows && src.cols == _cols);
	if( !(dst.Flags & Eigen::RowMajorBit) )
	{
		Mat _dst(src.cols, src.rows, DataType<_Tp>::type,
				 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
		if( src.type() == _dst.type() )
			transpose(src, _dst);
		else if( src.cols == src.rows )
		{
			src.convertTo(_dst, _dst.type());
			transpose(_dst, _dst);
		}
		else
			Mat(src.t()).convertTo(_dst, _dst.type());
		CV_DbgAssert(_dst.data == (uchar*)dst.data());
	}
	else
	{
		Mat _dst(src.rows, src.cols, DataType<_Tp>::type,
				 dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
		src.convertTo(_dst, _dst.type());
		CV_DbgAssert(_dst.data == (uchar*)dst.data());
	}
}
}
#endif

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
	MRPT_UNUSED_PARAM(skipDrawDetectedImgs);
#if MRPT_HAS_OPENCV
	try
	{
		ASSERT_(check_size_x>2)
		ASSERT_(check_size_y>2)
		ASSERT_(check_squares_length_X_meters>0)
		ASSERT_(check_squares_length_Y_meters>0)

		if (images.size()<1)
		{
			std::cout << "ERROR: No input images." << std::endl;
			return false;
		}

		const unsigned CORNERS_COUNT = check_size_x * check_size_y;
		const CvSize check_size = cvSize(check_size_x, check_size_y);

		// Fill the pattern of expected pattern points only once out of the loop:
		vector<cv::Point3f> pattern_obj_points(CORNERS_COUNT);
		{
			unsigned int y,k;
			for( y = 0, k = 0; y < check_size_y; y++ )
			{
				for( unsigned int x = 0; x < check_size_x; x++, k++ )
				{
					pattern_obj_points[k].x =-check_squares_length_X_meters * x;  // The "-" is for convenience, so the camera poses appear with Z>0
					pattern_obj_points[k].y = check_squares_length_Y_meters * y;
					pattern_obj_points[k].z = 0;
				}
			}
		}

		// First: Assure all images are loaded:
		// -------------------------------------------
		TCalibrationImageList::iterator it;
		for (it=images.begin();it!=images.end();++it)
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
		vector<vector<cv::Point3f> > objectPoints;  // final container for detected stuff
		vector<vector<cv::Point2f> > imagePoints;   // final container for detected stuff

		unsigned int  valid_detected_imgs = 0;
		vector<string>   pointsIdx2imageFile;
		cv::Size imgSize(0,0);

		unsigned int i;
		for (i=0,it=images.begin();it!=images.end();it++,i++)
		{
			TImageCalibData	&dat = it->second;

			// Make grayscale version:
			const CImage img_gray( dat.img_original, FAST_REF_OR_CONVERT_TO_GRAY );

			if (!i)
			{
				imgSize = cv::Size(img_gray.getWidth(),img_gray.getHeight() );
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

			vector<cv::Point2f> this_img_pts(CORNERS_COUNT);  // Temporary buffer for points, to be added if the points pass the checks.

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
				this_img_pts[p].x = detectedCoords[p].x;
				this_img_pts[p].y = detectedCoords[p].y;
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
						dat.detected_corners.push_back( mrpt::utils::TPixelCoordf( this_img_pts[k].x, this_img_pts[k].y ) );

				// Draw the checkerboard in the corresponding image:
				// ----------------------------------------------------
				if ( !dat.img_original.isExternallyStored() )
				{
					const int r = 4;
					CvPoint prev_pt= cvPoint(0, 0);
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

					void *rgb_img = dat.img_checkboard.getAs<IplImage>();

					for( y = 0, k = 0; y < check_size.height; y++ )
					{
						CvScalar color = line_colors[y % line_max];
						for( x = 0; x < check_size.width; x++, k++ )
						{
							CvPoint pt;
							pt.x = cvRound(this_img_pts[k].x);
							pt.y = cvRound(this_img_pts[k].y);

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

				// Accept this image as good:
				pointsIdx2imageFile.push_back( it->first );
				imagePoints.push_back( this_img_pts );
				objectPoints.push_back( pattern_obj_points );

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
		// Calibrate camera
		cv::Mat cameraMatrix, distCoeffs(1,5,CV_64F,cv::Scalar::all(0));
		vector<cv::Mat> rvecs, tvecs;

		const double cv_calib_err = 
		cv::calibrateCamera(
			objectPoints,imagePoints,imgSize,
			cameraMatrix, distCoeffs, rvecs, tvecs,
			0 /*flags*/ );

		// Load matrix:
		out_camera_params.intrinsicParams = CMatrixDouble33( cameraMatrix.ptr<double>() );

		out_camera_params.dist.assign(0);
		for (int i=0;i<5;i++)
			out_camera_params.dist[i] = distCoeffs.ptr<double>()[i];

		// Load camera poses:
		for (i=0;i<valid_detected_imgs;i++)
		{
			CMatrixDouble44 HM;
			HM.zeros();
			HM(3,3)=1;

			{
				// Convert rotation vectors -> rot matrices:
				cv::Mat cv_rot;
				cv::Rodrigues(rvecs[i],cv_rot);

				Eigen::Matrix3d rot;
				cv::my_cv2eigen(cv_rot, rot );
				HM.block<3,3>(0,0) = rot;
			}

			{
				Eigen::Matrix<double,3,1> trans;
				cv::my_cv2eigen(tvecs[i], trans );
				HM.block<3,1>(0,3) = trans;
			}

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
		for (it=images.begin();it!=images.end();++it)
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
			vector<TPoint3D>  lstPatternPoints(CORNERS_COUNT);	// Points as seen from the camera:
			for (unsigned int p=0;p<CORNERS_COUNT;p++)
				lstPatternPoints[p] = TPoint3D(pattern_obj_points[p].x,pattern_obj_points[p].y,pattern_obj_points[p].z);

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
						cvCircle( dat.img_rectified.getAs<IplImage>(), cvPoint(px,py), 4, CV_RGB(0,0,255) );
				}

				// Accumulate error:
				sqrErr+=square(px_d-dat.detected_corners[p].x)+square(py_d-dat.detected_corners[p].y); // Error relative to the original (distorted) image.
			}
		}

		if (valid_detected_imgs)
		{
			sqrErr /= CORNERS_COUNT*valid_detected_imgs;
			std::cout << "Average err. of reprojection: " << sqrt(sqrErr) << " pixels (OpenCV error=" << cv_calib_err << ")\n";
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




