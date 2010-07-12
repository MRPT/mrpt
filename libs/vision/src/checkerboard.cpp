/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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


#include <mrpt/vision/utils.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/CFeature.h>

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/geometry.h>


#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

int cvFindChessboardCorners3( const void* arr, CvSize pattern_size,
                             CvPoint2D32f* out_corners, int* out_corner_count,
                             int min_number_of_corners );

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
	bool               skipDrawDetectedImgs
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
		out_MSE,skipDrawDetectedImgs);

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
	bool               skipDrawDetectedImgs
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

			CImage	img_gray_org;
			dat.img_original.grayscale(img_gray_org);

			if (!i)
			{
				imgSize = cvSize(img_gray_org.getWidth(),img_gray_org.getHeight() );
				out_camera_params.ncols = imgSize.width;
				out_camera_params.nrows = imgSize.height;
			}
			else
			{
				if (imgSize.height != (int)img_gray_org.getHeight() || imgSize.width != (int)img_gray_org.getWidth())
				{
					std::cout << "ERROR: All the images must have the same size" << std::endl;
					return false;
				}
			}

			// Try with expanded versions of the image if it fails to detect the checkerboard:
			int corners_count;
			bool corners_found=false;

			//for (int scale=1;scale<=1;scale++)
			{
				corners_count = CORNERS_COUNT;

				corners_list.resize( (1+valid_detected_imgs)*CORNERS_COUNT );

				dat.detected_corners.clear();

				CImage	&img_gray = img_gray_org;

				/*if( scale > 1 )
				{
					img_gray.scaleImage( imgSize.width*scale, imgSize.height*scale, IMG_INTERP_LINEAR );
				}*/


				corners_found = 0 !=
#if 1
				cvFindChessboardCorners(
					img_gray.getAsIplImage(),
					check_size,
					&corners_list[valid_detected_imgs*CORNERS_COUNT],
					&corners_count,
					find_chess_flags);
#else
				cvFindChessboardCorners3(
					img_gray.getAsIplImage(),
					check_size,
					&corners_list[valid_detected_imgs*CORNERS_COUNT],
					&corners_count,
					find_chess_flags);
#endif


				if (corners_found && corners_count!=CORNERS_COUNT)
					corners_found = false;

				if (0)
				{
					CImage dbg_img;
					img_gray.colorImage(dbg_img);
					int x, y;
					unsigned int k;
					std::vector<TPixelCoordf>  cors;
					for( y = 0, k = 0; y < check_size.height; y++ )
						for( x = 0; x < check_size.width; x++, k++ )
							cors.push_back( TPixelCoordf( corners_list[valid_detected_imgs*CORNERS_COUNT + k].x, corners_list[valid_detected_imgs*CORNERS_COUNT + k].y ));

					dbg_img.drawChessboardCorners(cors,check_size.width,check_size.height);
					dbg_img.saveToFile("aa.png");
				}

				if( corners_found )
				{
					// Refine corners:
					cvFindCornerSubPix(
						img_gray.getAsIplImage(),
						&corners_list[valid_detected_imgs*CORNERS_COUNT],
						corners_count,
						cvSize(5,5), 	// window
						cvSize(-1,-1),
						cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

				}

				/*if( corners_found )
				{
					if (scale>1)
					{
						// Correct corner coordinates:
						for (unsigned int c=0;c<CORNERS_COUNT;c++)
						{
							corners_list[valid_detected_imgs*CORNERS_COUNT+c].x /= scale;
							corners_list[valid_detected_imgs*CORNERS_COUNT+c].y /= scale;
						}
					}
					break; // Done with this scale!
				} // Else, try with another scale...
				*/

			} // end try different scales

			cout << format("Img %s: %s\n", mrpt::system::extractFileName(it->first).c_str() , corners_found ? "DETECTED" : "NOT DETECTED" );

			if( corners_found )
			{
				// save the corners in the data structure:
				int x, y;
				unsigned int k;
				for( y = 0, k = 0; y < check_size.height; y++ )
					for( x = 0; x < check_size.width; x++, k++ )
						dat.detected_corners.push_back( mrpt::poses::CPoint2D( corners_list[valid_detected_imgs*CORNERS_COUNT + k].x, corners_list[valid_detected_imgs*CORNERS_COUNT + k].y ) );

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
		out_camera_params.intrinsicParams = CMatrixFloat33( CMatrixDouble33( proj_matrix ) );
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
				sqrErr+=dat.detected_corners[p].distance2DToSquare(px_d,py_d); // Error relative to the original (distorted) image.
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



/************************************************************************************\
    This is improved variant of chessboard corner detection algorithm that
    uses a graph of connected quads. It is based on the code contributed
    by Vladimir Vezhnevets and Philip Gruebele.
    Here is the copyright notice from the original Vladimir's code:
    ===============================================================

    The algorithms developed and implemented by Vezhnevets Vldimir
    aka Dead Moroz (vvp@graphics.cs.msu.ru)
    See http://graphics.cs.msu.su/en/research/calibration/opencv.html
    for detailed information.

    Reliability additions and modifications made by Philip Gruebele.
    <a href="mailto:pgruebele@cox.net">pgruebele@cox.net</a>

	His code was adapted for use with low resolution and omnidirectional cameras
	by Martin Rufli during his Master Thesis under supervision of Davide Scaramuzza, at the ETH Zurich. Further enhancements include:
		- Increased chance of correct corner matching.
		- Corner matching over all dilation runs.

If you use this code, please cite the following articles:

1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China, October 2006.
2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems  (ICVS'06), New York, January 5-7, 2006.
3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.

\************************************************************************************/

//===========================================================================
// CODE STARTS HERE
//===========================================================================
// Include files
//#include "cv.h"
//#include <highgui.h>
#include <time.h>
#include <fstream>
using namespace std;
using std::ifstream;

#define EXIT  goto exit;

// Defines
#define MAX_CONTOUR_APPROX  7


//Ming #define VIS 1
#define VIS 0
// Turn on visualization
#define TIMER 0					// Elapse the function duration times


// Definition Contour Struct
typedef struct CvContourEx
{
    CV_CONTOUR_FIELDS()
    int counter;
}
CvContourEx;


// Definition Corner Struct
typedef struct CvCBCorner
{
    CvPoint2D32f pt;					// X and y coordinates
	int row;							// Row and column of the corner
	int column;							// in the found pattern
	bool needsNeighbor;					// Does the corner require a neighbor?
    int count;							// number of corner neighbors
    struct CvCBCorner* neighbors[4];	// pointer to all corner neighbors
}
CvCBCorner;


// Definition Quadrangle Struct
// This structure stores information about the chessboard quadrange
typedef struct CvCBQuad
{
    int count;							// Number of quad neihbors
    int group_idx;						// Quad group ID
    float edge_len;						// Smallest side length^2
    CvCBCorner *corners[4];				// Coordinates of quad corners
    struct CvCBQuad *neighbors[4];		// Pointers of quad neighbors
	bool labeled;						// Has this corner been labeled?
}
CvCBQuad;



//===========================================================================
// FUNCTION PROTOTYPES
//===========================================================================
static int icvGenerateQuads( CvCBQuad **quads, CvCBCorner **corners,
                             CvMemStorage *storage, CvMat *image, int flags, int dilation,
							 bool firstRun );

static void mrFindQuadNeighbors2( CvCBQuad *quads, int quad_count, int dilation);

static int mrAugmentBestRun( CvCBQuad *new_quads, int new_quad_count, int new_dilation,
							 CvCBQuad **old_quads, int old_quad_count, int old_dilation );

static int icvFindConnectedQuads( CvCBQuad *quads, int quad_count, CvCBQuad **quad_group,
								  int group_idx,
                                  CvMemStorage* storage, int dilation );

static void mrLabelQuadGroup( CvCBQuad **quad_group, int count, CvSize pattern_size,
							  bool firstRun );

static void mrCopyQuadGroup( CvCBQuad **temp_quad_group, CvCBQuad **out_quad_group,
							 int count );

static int icvCleanFoundConnectedQuads( int quad_count, CvCBQuad **quads,
									    CvSize pattern_size );

static int mrWriteCorners( CvCBQuad **output_quads, int count, CvSize pattern_size,
						   int min_number_of_corners );



//===========================================================================
// MAIN FUNCTION
//===========================================================================
int cvFindChessboardCorners3( const void* arr, CvSize pattern_size,
                             CvPoint2D32f* out_corners, int* out_corner_count,
                             int min_number_of_corners )
{
//START TIMER
#if TIMER
	ofstream FindChessboardCorners2;
    time_t  start_time = clock();
#endif

	// PART 0: INITIALIZATION
	//-----------------------------------------------------------------------
	// Initialize variables
	int flags					=  1;	// not part of the function call anymore!
	int max_count				=  0;
	int max_dilation_run_ID		= -1;
    const int min_dilations		=  1;
    const int max_dilations		=  6;
    int found					=  0;
    CvMat* norm_img				=  0;
    CvMat* thresh_img			=  0;
	CvMat* thresh_img_save		=  0;
    CvMemStorage* storage		=  0;

	CvCBQuad *quads				=  0;
	CvCBQuad **quad_group		=  0;
    CvCBCorner *corners			=  0;
	CvCBCorner **corner_group	=  0;
	CvCBQuad **output_quad_group = 0;

    // debug trial. Martin Rufli, 28. Ocober, 2008
	int block_size = 0;


	// Create error message file
	ofstream error("cToMatlab/error.txt");


	// Set openCV function name and label the function start
    CV_FUNCNAME( "cvFindChessBoardCornerGuesses2" );
    //__BEGIN__;


	// Further initializations
    int quad_count, group_idx, dilations;
    CvMat stub, *img = (CvMat*)arr;


	// Read image from input
    CV_CALL( img = cvGetMat( img, &stub ));


	// Error handling, write error message to error.txt
    if( CV_MAT_DEPTH( img->type ) != CV_8U || CV_MAT_CN( img->type ) == 2 )
	{
        error << "Only 8-bit grayscale or color images are supported" << endl;
		error.close();
		return -1;
	}
    if( pattern_size.width < 2 || pattern_size.height < 2 )
	{
        error << "Pattern should have at least 2x2 size" << endl;
		error.close();
		return -1;
	}
	if( pattern_size.width > 127 || pattern_size.height > 127 )
	{
        error << "Pattern should not have a size larger than 127 x 127" << endl;
		error.close();
		return -1;
	}
	/*
	if( pattern_size.width != pattern_size.height )
	{
        error << "In this implementation only square sized checker boards are supported" << endl;
		error.close();
		return -1;
	}
	*/
	if( !out_corners )
	{
        error << "Null pointer to corners encountered" << endl;
		error.close();
		return -1;
	}


	// Create memory storage
    CV_CALL( storage = cvCreateMemStorage(0) );
    CV_CALL( thresh_img = cvCreateMat( img->rows, img->cols, CV_8UC1 ));
	CV_CALL( thresh_img_save = cvCreateMat( img->rows, img->cols, CV_8UC1 ));


	// Image histogramm normalization and
	// BGR to Grayscale image conversion (if applicable)
	// MARTIN: Set to "false"
    if( CV_MAT_CN(img->type) != 1 || (flags & CV_CALIB_CB_NORMALIZE_IMAGE) )
    {
        CV_CALL( norm_img = cvCreateMat( img->rows, img->cols, CV_8UC1 ));

        if( CV_MAT_CN(img->type) != 1 )
        {
            CV_CALL( cvCvtColor( img, norm_img, CV_BGR2GRAY ));
            img = norm_img;
        }

        if(false)
        {
            cvEqualizeHist( img, norm_img );
            img = norm_img;
        }
    }

// EVALUATE TIMER
#if TIMER
	float time0_1 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindChessboardCorners2.open("timer/FindChessboardCorners2.txt", ofstream::app);
	FindChessboardCorners2 << "Time 0.1 for cvFindChessboardCorners2 was " << time0_1 << " seconds." << endl;
#endif

	// For image binarization (thresholding)
    // we use an adaptive threshold with a gaussian mask
	// ATTENTION: Gaussian thresholding takes MUCH more time than Mean thresholding!
    block_size = cvRound(MIN(img->cols,img->rows)*0.2)|1;
    cvAdaptiveThreshold( img, thresh_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, block_size, 0 );
	cvCopy( thresh_img, thresh_img_save);


	// PART 1: FIND LARGEST PATTERN
	//-----------------------------------------------------------------------
	// Checker patterns are tried to be found by dilating the background and
	// then applying a canny edge finder on the closed contours (checkers).
	// Try one dilation run, but if the pattern is not found, repeat until
	// max_dilations is reached.
    for( dilations = min_dilations; dilations <= max_dilations; dilations++ )
    {
		// Calling "cvCopy" again is much faster than rerunning "cvAdaptiveThreshold"
		cvCopy( thresh_img_save, thresh_img);

// EVALUATE TIMER
#if TIMER
		float time0_2 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
		FindChessboardCorners2 << "Time 0.2 for cvFindChessboardCorners2 was " << time0_2 << " seconds." << endl;
#endif

//VISUALIZATION--------------------------------------------------------------
#if VIS
 		cvNamedWindow( "Original Image", 1 );
		cvShowImage( "Original Image", img);
		//cvSaveImage("pictureVis/OrigImg.png", img);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


		// MARTIN's Code
		// Use both a rectangular and a cross kernel. In this way, a more
		// homogeneous dilation is performed, which is crucial for small,
		// distorted checkers. Use the CROSS kernel first, since its action
		// on the image is more subtle
		IplConvKernel *kernel1 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CROSS,NULL);
		IplConvKernel *kernel2 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);

        if (dilations >= 1)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 2)
			cvDilate( thresh_img, thresh_img, kernel2, 1);
		if (dilations >= 3)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 4)
			cvDilate( thresh_img, thresh_img, kernel2, 1);
		if (dilations >= 5)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 6)
			cvDilate( thresh_img, thresh_img, kernel2, 1);

// EVALUATE TIMER
#if TIMER
		float time0_3 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
		FindChessboardCorners2 << "Time 0.3 for cvFindChessboardCorners2 was " << time0_3 << " seconds." << endl;
#endif

//VISUALIZATION--------------------------------------------------------------
#if VIS
		cvNamedWindow( "After adaptive Threshold (and Dilation)", 1 );
		cvShowImage( "After adaptive Threshold (and Dilation)", thresh_img);
		//cvSaveImage("pictureVis/afterDilation.png", thresh_img);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


        // In order to find rectangles that go to the edge, we draw a white
		// line around the image edge. Otherwise FindContours will miss those
		// clipped rectangle contours. The border color will be the image mean,
		// because otherwise we risk screwing up filters like cvSmooth()
        cvRectangle( thresh_img, cvPoint(0,0), cvPoint(thresh_img->cols-1,
                     thresh_img->rows-1), CV_RGB(255,255,255), 3, 8);


		// Generate quadrangles in the following function
		// "quad_count" is the number of cound quadrangles
        CV_CALL( quad_count = icvGenerateQuads( &quads, &corners, storage, thresh_img, flags, dilations, true ));
        if( quad_count <= 0 )
            continue;

// EVALUATE TIMER
#if TIMER
		float time0_4 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
		FindChessboardCorners2 << "Time 0.4 for cvFindChessboardCorners2 was " << time0_4 << " seconds." << endl;
#endif

//VISUALIZATION--------------------------------------------------------------
#if VIS
  		cvNamedWindow( "all found quads per dilation run", 1 );
		IplImage* imageCopy2 = cvCreateImage( cvGetSize(thresh_img), 8, 1 );
		IplImage* imageCopy22 = cvCreateImage( cvGetSize(thresh_img), 8, 3 );
		cvCopy( thresh_img, imageCopy2);
		cvCvtColor( imageCopy2, imageCopy22, CV_GRAY2BGR );

		for( int kkk = 0; kkk < quad_count; kkk++ )
		{
			CvCBQuad* print_quad = &quads[kkk];
			CvPoint pt[4];
			pt[0].x = (int)print_quad->corners[0]->pt.x;
			pt[0].y = (int)print_quad->corners[0]->pt.y;
			pt[1].x = (int)print_quad->corners[1]->pt.x;
			pt[1].y = (int)print_quad->corners[1]->pt.y;
			pt[2].x = (int)print_quad->corners[2]->pt.x;
			pt[2].y = (int)print_quad->corners[2]->pt.y;
			pt[3].x = (int)print_quad->corners[3]->pt.x;
			pt[3].y = (int)print_quad->corners[3]->pt.y;
			cvLine( imageCopy22, pt[0], pt[1], CV_RGB(255,255,0), 1, 8 );
			cvLine( imageCopy22, pt[1], pt[2], CV_RGB(255,255,0), 1, 8 );
			cvLine( imageCopy22, pt[2], pt[3], CV_RGB(255,255,0), 1, 8 );
			cvLine( imageCopy22, pt[3], pt[0], CV_RGB(255,255,0), 1, 8 );
		}
		cvShowImage( "all found quads per dilation run", imageCopy22);
		//cvSaveImage("pictureVis/allFoundQuads.png", imageCopy22);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


        // The following function finds and assigns neighbor quads to every
		// quadrangle in the immediate vicinity fulfilling certain
		// prerequisites
        CV_CALL( mrFindQuadNeighbors2( quads, quad_count, dilations));

//VISUALIZATION--------------------------------------------------------------
#if VIS
		cvNamedWindow( "quads with neighbors", 1 );
		IplImage* imageCopy3 = cvCreateImage( cvGetSize(thresh_img), 8, 3 );
		cvCopy( imageCopy22, imageCopy3);
		CvPoint pt;
		int scale = 0;
		int line_type = CV_AA;
		CvScalar color = {{0,0,255}};
		for( int kkk = 0; kkk < quad_count; kkk++ )
		{
			CvCBQuad* print_quad2 = &quads[kkk];
			for( int kkkk = 0; kkkk < 4; kkkk++ )
			{
				if( print_quad2->neighbors[kkkk] )
				{
					pt.x = (int)(print_quad2->corners[kkkk]->pt.x);
					pt.y = (int)(print_quad2->corners[kkkk]->pt.y);
					cvCircle( imageCopy3, pt, 3, color, 1, line_type, scale);
				}
			}
		}
		cvShowImage( "quads with neighbors", imageCopy3);
		//cvSaveImage("pictureVis/allFoundNeighbors.png", imageCopy3);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


		// Allocate memory
        CV_CALL( quad_group = (CvCBQuad**)cvAlloc( sizeof(quad_group[0]) * quad_count));
        CV_CALL( corner_group = (CvCBCorner**)cvAlloc( sizeof(corner_group[0]) * quad_count*4 ));


		// The connected quads will be organized in groups. The following loop
		// increases a "group_idx" identifier.
		// The function "icvFindConnectedQuads assigns all connected quads
		// a unique group ID.
		// If more quadrangles were assigned to a given group (i.e. connected)
		// than are expected by the input variable "pattern_size", the
		// function "icvCleanFoundConnectedQuads" erases the surplus
		// quadrangles by minimizing the convex hull of the remaining pattern.
        for( group_idx = 0; ; group_idx++ )
        {
            int count;
            CV_CALL( count = icvFindConnectedQuads( quads, quad_count, quad_group, group_idx, storage, dilations ));

            if( count == 0 )
                break;

			CV_CALL( count = icvCleanFoundConnectedQuads( count, quad_group, pattern_size ));


			// MARTIN's Code
			// To save computational time, only proceed, if the number of
			// found quads during this dilation run is larger than the
			// largest previous found number
			if( count >= max_count)
			{
				// set max_count to its new value
				max_count = count;
				max_dilation_run_ID = dilations;

				// The following function labels all corners of every quad
				// with a row and column entry.
				// "count" specifies the number of found quads in "quad_group"
				// with group identifier "group_idx"
				// The last parameter is set to "true", because this is the
				// first function call and some initializations need to be
				// made.
				mrLabelQuadGroup( quad_group, max_count, pattern_size, true );


//VISUALIZATION--------------------------------------------------------------
#if VIS
				// display all corners in INCREASING ROW AND COLUMN ORDER
				cvNamedWindow( "Corners in increasing order", 1 );
				IplImage* imageCopy11 = cvCreateImage( cvGetSize(thresh_img), 8, 3 );
				cvCopy( imageCopy22, imageCopy11);
				// Assume min and max rows here, since we are outside of the
				// relevant function
				int min_row = -15;
				int max_row = 15;
				int min_column = -15;
				int max_column = 15;
				for(int i = min_row; i <= max_row; i++)
				{
						for(int j = min_column; j <= max_column; j++)
						{
							for(int k = 0; k < count; k++)
							{
								for(int l = 0; l < 4; l++)
								{
									if( ((quad_group[k])->corners[l]->row == i) && ((quad_group[k])->corners[l]->column == j) )
									{
										// draw the row and column numbers
										char str[255];
										sprintf(str,"%i/%i",i,j);
										CvFont font;
										cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.2, 0.2, 0, 1);
										CvPoint ptt;
										ptt.x = (int) quad_group[k]->corners[l]->pt.x;
										ptt.y = (int) quad_group[k]->corners[l]->pt.y;
										// Mark central corners with a different color than
										// border corners
										if ((quad_group[k])->corners[l]->needsNeighbor == false)
										{
											cvPutText(imageCopy11, str, ptt, &font, CV_RGB(0,255,0));
										}
										else
										{
											cvPutText(imageCopy11, str, ptt, &font, CV_RGB(255,0,0));
										}
										cvShowImage( "Corners in increasing order", imageCopy11);
										//cvSaveImage("pictureVis/CornersIncreasingOrder.tif", imageCopy11);
										//cvWaitKey(0);
									}
								}
							}
						}
				}
				cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


				// Allocate memory
				CV_CALL( output_quad_group = (CvCBQuad**)cvAlloc( sizeof(output_quad_group[0]) * ((pattern_size.height+2) * (pattern_size.width+2)) ));


				// The following function copies every member of "quad_group"
				// to "output_quad_group", because "quad_group" will be
				// overwritten during the next loop pass.
				// "output_quad_group" is a true copy of "quad_group" and
				// later used for output
				mrCopyQuadGroup( quad_group, output_quad_group, max_count );
			}
        }


		// Free the allocated variables
        cvFree( &quads );
        cvFree( &corners );
    }


// EVALUATE TIMER
#if TIMER
	float time1 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindChessboardCorners2.open("timer/FindChessboardCorners2.txt", ofstream::app);
	FindChessboardCorners2 << "Time 1 for cvFindChessboardCorners2 was " << time1 << " seconds." << endl;
#endif

	// If enough corners have been found already, then there is no need for PART 2 ->EXIT
	found = mrWriteCorners( output_quad_group, max_count, pattern_size, min_number_of_corners);
		if (found == -1 || found == 1)
			EXIT;

	// PART 2: AUGMENT LARGEST PATTERN
	//-----------------------------------------------------------------------
	// Instead of saving all found quads of all dilation runs from PART 1, we
	// just recompute them again, but skipping the dilation run which
	// produced the maximum number of found quadrangles.
	// In essence the first section of PART 2 is identical to the first
	// section of PART 1.
    for( dilations = max_dilations; dilations >= min_dilations; dilations-- )
    {
		//if(max_dilation_run_ID == dilations)
		//	continue;

		// Calling "cvCopy" again is much faster than rerunning "cvAdaptiveThreshold"
		cvCopy( thresh_img_save, thresh_img);

		IplConvKernel *kernel1 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CROSS,NULL);
		IplConvKernel *kernel2 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);

        if (dilations >= 1)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 2)
			cvDilate( thresh_img, thresh_img, kernel2, 1);
		if (dilations >= 3)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 4)
			cvDilate( thresh_img, thresh_img, kernel2, 1);
		if (dilations >= 5)
			cvDilate( thresh_img, thresh_img, kernel1, 1);
		if (dilations >= 6)
			cvDilate( thresh_img, thresh_img, kernel2, 1);

        cvRectangle( thresh_img, cvPoint(0,0), cvPoint(thresh_img->cols-1,
                     thresh_img->rows-1), CV_RGB(255,255,255), 3, 8);

//VISUALIZATION--------------------------------------------------------------
#if VIS
		cvNamedWindow( "PART2: Starting Point", 1 );
		IplImage* imageCopy23 = cvCreateImage( cvGetSize(thresh_img), 8, 3 );
		cvCvtColor( thresh_img, imageCopy23, CV_GRAY2BGR );

		CvPoint *pt = new CvPoint[4];
		for( int kkk = 0; kkk < max_count; kkk++ )
		{
			CvCBQuad* print_quad2 = output_quad_group[kkk];
			for( int kkkk = 0; kkkk < 4; kkkk++ )
			{
				pt[kkkk].x = (int) print_quad2->corners[kkkk]->pt.x;
				pt[kkkk].y = (int) print_quad2->corners[kkkk]->pt.y;
			}
			// draw a filled polygon
			cvFillConvexPoly ( imageCopy23, pt, 4, CV_RGB(255*0.1,255*0.25,255*0.6));
		}
		// indicate the dilation run
		char str[255];
		sprintf(str,"Dilation Run No.: %i",dilations);
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);
		//cvPutText(imageCopy23, str, cvPoint(20,20), &font, CV_RGB(0,255,0));

		cvShowImage( "PART2: Starting Point", imageCopy23);
		cvSaveImage("pictureVis/part2Start.png", imageCopy23);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


        CV_CALL( quad_count = icvGenerateQuads( &quads, &corners, storage, thresh_img, flags, dilations, false ));
        if( quad_count <= 0 )
            continue;


//VISUALIZATION--------------------------------------------------------------
#if VIS
		//draw on top of previous image
		for( int kkk = 0; kkk < quad_count; kkk++ )
		{
			CvCBQuad* print_quad = &quads[kkk];

			CvPoint pt[4];
			pt[0].x = (int)print_quad->corners[0]->pt.x;
			pt[0].y = (int)print_quad->corners[0]->pt.y;
			pt[1].x = (int)print_quad->corners[1]->pt.x;
			pt[1].y = (int)print_quad->corners[1]->pt.y;
			pt[2].x = (int)print_quad->corners[2]->pt.x;
			pt[2].y = (int)print_quad->corners[2]->pt.y;
			pt[3].x = (int)print_quad->corners[3]->pt.x;
			pt[3].y = (int)print_quad->corners[3]->pt.y;
			cvLine( imageCopy23, pt[0], pt[1], CV_RGB(255,0,0), 1, 8 );
			cvLine( imageCopy23, pt[1], pt[2], CV_RGB(255,0,0), 1, 8 );
			cvLine( imageCopy23, pt[2], pt[3], CV_RGB(255,0,0), 1, 8 );
			cvLine( imageCopy23, pt[3], pt[0], CV_RGB(255,0,0), 1, 8 );
			//compute center of print_quad
			int x1 = (pt[0].x + pt[1].x)/2;
			int y1 = (pt[0].y + pt[1].y)/2;
			int x2 = (pt[2].x + pt[3].x)/2;
			int y2 = (pt[2].y + pt[3].y)/2;

			//int x3 = (x1 + x2)/2;
			//int y3 = (y1 + y2)/2;
			// indicate the quad number in the image
			//char str[255];
			//sprintf(str,"%i",kkk);
			//CvFont font;
			//cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1);
			//cvPutText(imageCopy23, str, cvPoint(x3,y3), &font, CV_RGB(0,255,255));
		}

		for( int kkk = 0; kkk < max_count; kkk++ )
		{
			CvCBQuad* print_quad = output_quad_group[kkk];

			CvPoint pt[4];
			pt[0].x = (int)print_quad->corners[0]->pt.x;
			pt[0].y = (int)print_quad->corners[0]->pt.y;
			pt[1].x = (int)print_quad->corners[1]->pt.x;
			pt[1].y = (int)print_quad->corners[1]->pt.y;
			pt[2].x = (int)print_quad->corners[2]->pt.x;
			pt[2].y = (int)print_quad->corners[2]->pt.y;
			pt[3].x = (int)print_quad->corners[3]->pt.x;
			pt[3].y = (int)print_quad->corners[3]->pt.y;
			//compute center of print_quad
			int x1 = (pt[0].x + pt[1].x)/2;
			int y1 = (pt[0].y + pt[1].y)/2;
			int x2 = (pt[2].x + pt[3].x)/2;
			int y2 = (pt[2].y + pt[3].y)/2;

			int x3 = (x1 + x2)/2;
			int y3 = (y1 + y2)/2;

			// indicate the quad number in the image
			char str[255];
			sprintf(str,"%i",kkk);
			//CvFont font;
			//cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1);
			//cvPutText(imageCopy23, str, cvPoint(x3,y3), &font, CV_RGB(0,0,0));
		}

		cvShowImage( "PART2: Starting Point", imageCopy23);
		cvSaveImage("pictureVis/part2StartAndNewQuads.png", imageCopy23);
		cvWaitKey(0);
#endif
//END------------------------------------------------------------------------


		// MARTIN's Code
		// The following loop is executed until no more newly found quads
		// can be matched to one of the border corners of the largest found
		// pattern from PART 1.
		// The function "mrAugmentBestRun" tests whether a quad can be linked
		// to the existng pattern.
		// The function "mrLabelQuadGroup" then labels the newly added corners
		// with the respective row and column entries.
		int feedBack = -1;
		while ( feedBack == -1)
		{
			feedBack = mrAugmentBestRun( quads, quad_count, dilations,
            							 output_quad_group, max_count, max_dilation_run_ID );


//VISUALIZATION--------------------------------------------------------------
#if VIS
			if( feedBack == -1)
			{
				CvCBQuad* remember_quad;
				for( int kkk = max_count; kkk < max_count+1; kkk++ )
				{
					CvCBQuad* print_quad = output_quad_group[kkk];
					remember_quad = print_quad;
					CvPoint pt[4];
					pt[0].x = (int)print_quad->corners[0]->pt.x;
					pt[0].y = (int)print_quad->corners[0]->pt.y;
					pt[1].x = (int)print_quad->corners[1]->pt.x;
					pt[1].y = (int)print_quad->corners[1]->pt.y;
					pt[2].x = (int)print_quad->corners[2]->pt.x;
					pt[2].y = (int)print_quad->corners[2]->pt.y;
					pt[3].x = (int)print_quad->corners[3]->pt.x;
					pt[3].y = (int)print_quad->corners[3]->pt.y;
					cvLine( imageCopy23, pt[0], pt[1], CV_RGB(255,0,0), 2, 8 );
					cvLine( imageCopy23, pt[1], pt[2], CV_RGB(255,0,0), 2, 8 );
					cvLine( imageCopy23, pt[2], pt[3], CV_RGB(255,0,0), 2, 8 );
					cvLine( imageCopy23, pt[3], pt[0], CV_RGB(255,0,0), 2, 8 );
				}

				cvWaitKey(0);
				// also draw the corner to which it is connected
				// Remember it is not yet completely linked!!!
				for( int kkk = 0; kkk < max_count; kkk++ )
				{
					CvCBQuad* print_quad = output_quad_group[kkk];

					for( int kkkk = 0; kkkk < 4; kkkk++)
					{
						if(print_quad->neighbors[kkkk] == remember_quad)
						{
							CvPoint pt[4];
							pt[0].x = (int)print_quad->corners[0]->pt.x;
							pt[0].y = (int)print_quad->corners[0]->pt.y;
							pt[1].x = (int)print_quad->corners[1]->pt.x;
							pt[1].y = (int)print_quad->corners[1]->pt.y;
							pt[2].x = (int)print_quad->corners[2]->pt.x;
							pt[2].y = (int)print_quad->corners[2]->pt.y;
							pt[3].x = (int)print_quad->corners[3]->pt.x;
							pt[3].y = (int)print_quad->corners[3]->pt.y;
							cvLine( imageCopy23, pt[0], pt[1], CV_RGB(255,0,0), 2, 8 );
							cvLine( imageCopy23, pt[1], pt[2], CV_RGB(255,0,0), 2, 8 );
							cvLine( imageCopy23, pt[2], pt[3], CV_RGB(255,0,0), 2, 8 );
							cvLine( imageCopy23, pt[3], pt[0], CV_RGB(255,0,0), 2, 8 );
						}
					}
				}
				cvShowImage( "PART2: Starting Point", imageCopy23);
				cvSaveImage("pictureVis/part2StartAndSelectedQuad.png", imageCopy23);
				cvWaitKey(0);
			}
#endif
//END------------------------------------------------------------------------


			// if we have found a new matching quad
			if (feedBack == -1)
			{
				// increase max_count by one
				max_count = max_count + 1;
   				mrLabelQuadGroup( output_quad_group, max_count, pattern_size, false );


				// write the found corners to output array
				// Go to //__END__, if enough corners have been found
				found = mrWriteCorners( output_quad_group, max_count, pattern_size, min_number_of_corners);
				if (found == -1 || found == 1)
					EXIT;
			}
		}
	}


	// "End of file" jump point
	// After the command "EXIT" the code jumps here
    //__END__;
    exit:


	/*
	// MARTIN:
	found = mrWriteCorners( output_quad_group, max_count, pattern_size, min_number_of_corners);
	*/

	// If a linking problem was encountered, throw an error message
    if( found == -1 )
	{
        error << "While linking the corners a problem was encountered. No corner sequence is returned. " << endl;
		error.close();
		return -1;
	}


	// Release allocated memory
    cvReleaseMemStorage( &storage );
    cvReleaseMat( &norm_img );
    cvReleaseMat( &thresh_img );
    cvFree( &quads );
    cvFree( &corners );
    cvFree( &quad_group );
    cvFree( &corner_group );
	cvFree( &output_quad_group );
	error.close();

// EVALUATE TIMER
#if TIMER
	float time3 = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindChessboardCorners2 << "Time 3 for cvFindChessboardCorners2 was " << time3 << " seconds." << endl;
	FindChessboardCorners2.close();
#endif

	// Return found
	// Found can have the values
	// -1  ->	Error or corner linking problem, see error.txt for more information
	//  0  ->	Not enough corners were found
	//  1  ->	Enough corners were found
    return found;
}


//===========================================================================
// ERASE OVERHEAD
//===========================================================================
// If we found too many connected quads, remove those which probably do not
// belong.
static int
icvCleanFoundConnectedQuads( int quad_count, CvCBQuad **quad_group, CvSize pattern_size )
{
    CvMemStorage *temp_storage = 0;
    CvPoint2D32f *centers = 0;

    CV_FUNCNAME( "icvCleanFoundConnectedQuads" );

    //__BEGIN__;

    CvPoint2D32f center = {0,0};
    int i, j, k;


    // Number of quads this pattern should contain
    int count = ((pattern_size.width + 1)*(pattern_size.height + 1) + 1)/2;


    // If we have more quadrangles than we should, try to eliminate duplicates
	// or ones which don't belong to the pattern rectangle. Else go to the end
	// of the function
    if( quad_count <= count )
        EXIT;


    // Create an array of quadrangle centers
    CV_CALL( centers = (CvPoint2D32f *)cvAlloc( sizeof(centers[0])*quad_count ));
    CV_CALL( temp_storage = cvCreateMemStorage(0));

    for( i = 0; i < quad_count; i++ )
    {
        CvPoint2D32f ci = {0,0};
        CvCBQuad* q = quad_group[i];

        for( j = 0; j < 4; j++ )
        {
            CvPoint2D32f pt = q->corners[j]->pt;
            ci.x += pt.x;
            ci.y += pt.y;
        }

        ci.x *= 0.25f;
        ci.y *= 0.25f;


		// Centers(i), is the geometric center of quad(i)
		// Center, is the center of all found quads
        centers[i] = ci;
        center.x += ci.x;
        center.y += ci.y;
    }
    center.x /= quad_count;
    center.y /= quad_count;

    // If we have more quadrangles than we should, we try to eliminate bad
	// ones based on minimizing the bounding box. We iteratively remove the
	// point which reduces the size of the bounding box of the blobs the most
    // (since we want the rectangle to be as small as possible) remove the
	// quadrange that causes the biggest reduction in pattern size until we
	// have the correct number
    for( ; quad_count > count; quad_count-- )
    {
        double min_box_area = DBL_MAX;
        int skip, min_box_area_index = -1;
        CvCBQuad *q0, *q;


        // For each point, calculate box area without that point
        for( skip = 0; skip < quad_count; skip++ )
        {
            // get bounding rectangle
            CvPoint2D32f temp = centers[skip];
            centers[skip] = center;
            CvMat pointMat = cvMat(1, quad_count, CV_32FC2, centers);
            CvSeq *hull = cvConvexHull2( &pointMat, temp_storage, CV_CLOCKWISE, 1 );
            centers[skip] = temp;
            double hull_area = fabs(cvContourArea(hull, CV_WHOLE_SEQ));


            // remember smallest box area
            if( hull_area < min_box_area )
            {
                min_box_area = hull_area;
                min_box_area_index = skip;
            }
            cvClearMemStorage( temp_storage );
        }

        q0 = quad_group[min_box_area_index];


        // remove any references to this quad as a neighbor
        for( i = 0; i < quad_count; i++ )
        {
            q = quad_group[i];
            for( j = 0; j < 4; j++ )
            {
                if( q->neighbors[j] == q0 )
                {
                    q->neighbors[j] = 0;
                    q->count--;
                    for( k = 0; k < 4; k++ )
                        if( q0->neighbors[k] == q )
                        {
                            q0->neighbors[k] = 0;
                            q0->count--;
                            break;
                        }
                    break;
                }
            }
        }

		// remove the quad by copying th last quad in the list into its place
        quad_count--;
        quad_group[min_box_area_index] = quad_group[quad_count];
        centers[min_box_area_index] = centers[quad_count];
    }

    //__END__;
	exit:

    cvReleaseMemStorage( &temp_storage );
    cvFree( &centers );

    return quad_count;
}



//===========================================================================
// FIND COONECTED QUADS
//===========================================================================
static int
icvFindConnectedQuads( CvCBQuad *quad, int quad_count, CvCBQuad **out_group,
                       int group_idx, CvMemStorage* storage, int dilation )
{
//START TIMER
#if TIMER
	ofstream FindConnectedQuads;
    time_t  start_time = clock();
#endif

	// initializations
    CvMemStorage* temp_storage = cvCreateChildMemStorage( storage );
    CvSeq* stack = cvCreateSeq( 0, sizeof(*stack), sizeof(void*), temp_storage );
	int i, count = 0;


    // Scan the array for a first unlabeled quad
    for( i = 0; i < quad_count; i++ )
    {
        if( quad[i].count > 0 && quad[i].group_idx < 0)
            break;
    }


    // Recursively find a group of connected quads starting from the seed
	// quad[i]
    if( i < quad_count )
    {
        CvCBQuad* q = &quad[i];
        cvSeqPush( stack, &q );
        out_group[count++] = q;
        q->group_idx = group_idx;

        while( stack->total )
        {
            cvSeqPop( stack, &q );
            for( i = 0; i < 4; i++ )
            {
                CvCBQuad *neighbor = q->neighbors[i];


				// If he neighbor exists and the neighbor has more than 0
				// neighbors and the neighbor has not been classified yet.
                if( neighbor && neighbor->count > 0 && neighbor->group_idx < 0 )
                {
                    cvSeqPush( stack, &neighbor );
                    out_group[count++] = neighbor;
                    neighbor->group_idx = group_idx;
                }
            }
        }
    }

    cvReleaseMemStorage( &temp_storage );

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindConnectedQuads.open("timer/FindConnectedQuads.txt", ofstream::app);
	FindConnectedQuads << "Time for cvFindConnectedQuads was " << time << " seconds." << endl;
	FindConnectedQuads.close();
#endif

    return count;
}



//===========================================================================
// LABEL CORNER WITH ROW AND COLUMN //DONE
//===========================================================================
static void mrLabelQuadGroup( CvCBQuad **quad_group, int count, CvSize pattern_size, bool firstRun )
{
//START TIMER
#if TIMER
	ofstream LabelQuadGroup;
    time_t  start_time = clock();
#endif

	// If this is the first function call, a seed quad needs to be selected
	if (firstRun == true)
	{
		// Search for the (first) quad with the maximum number of neighbors
		// (usually 4). This will be our starting point.
		int max_id = -1;
		int max_number = -1;
		for(int i = 0; i < count; i++ )
		{
			CvCBQuad* q = quad_group[i];
			if( q->count > max_number)
			{
				max_number = q->count;
				max_id = i;

				if (max_number == 4)
					break;
			}
		}


		// Mark the starting quad's (per definition) upper left corner with
		//(0,0) and then proceed clockwise
		// The following labeling sequence enshures a "right coordinate system"
		(quad_group[max_id])->labeled = true;

		(quad_group[max_id])->corners[0]->row = 0;
        (quad_group[max_id])->corners[0]->column = 0;
		(quad_group[max_id])->corners[1]->row = 0;
		(quad_group[max_id])->corners[1]->column = 1;
		(quad_group[max_id])->corners[2]->row = 1;
		(quad_group[max_id])->corners[2]->column = 1;
		(quad_group[max_id])->corners[3]->row = 1;
		(quad_group[max_id])->corners[3]->column = 0;
	}


	// Mark all other corners with their respective row and column
	bool flag_changed = true;
	while( flag_changed == true )
	{
		// First reset the flag to "false"
		flag_changed = false;


		// Go through all quads top down is faster, since unlabeled quads will
		// be inserted at the end of the list
		for( int i = (count-1); i >= 0; i-- )
		{
			// Check whether quad "i" has been labeled already
 			if ( (quad_group[i])->labeled == false )
			{
				// Check its neighbors, whether some of them have been labeled
				// already
				for( int j = 0; j < 4; j++ )
				{
					// Check whether the neighbor exists (i.e. is not the NULL
					// pointer)
					if( (quad_group[i])->neighbors[j] )
					{
						CvCBQuad *quadNeighborJ = (quad_group[i])->neighbors[j];


						// Only proceed, if neighbor "j" was labeled
						if( quadNeighborJ->labeled == true)
						{
							// For every quad it could happen to pass here
							// multiple times. We therefore "break" later.
							// Check whitch of the neighbors corners is
							// connected to the current quad
							int connectedNeighborCornerId = -1;
							for( int k = 0; k < 4; k++)
							{
								if( quadNeighborJ->neighbors[k] == quad_group[i] )
								{
									connectedNeighborCornerId = k;


									// there is only one, therefore
									break;
								}
							}


							// For the following calculations we need the row
							// and column of the connected neighbor corner and
							// all other corners of the connected quad "j",
							// clockwise (CW)
							CvCBCorner *conCorner	 = quadNeighborJ->corners[connectedNeighborCornerId];
							CvCBCorner *conCornerCW1 = quadNeighborJ->corners[(connectedNeighborCornerId+1)%4];
							CvCBCorner *conCornerCW2 = quadNeighborJ->corners[(connectedNeighborCornerId+2)%4];
							CvCBCorner *conCornerCW3 = quadNeighborJ->corners[(connectedNeighborCornerId+3)%4];

							(quad_group[i])->corners[j]->row			=	conCorner->row;
							(quad_group[i])->corners[j]->column			=	conCorner->column;
							(quad_group[i])->corners[(j+1)%4]->row		=	conCorner->row - conCornerCW2->row + conCornerCW3->row;
							(quad_group[i])->corners[(j+1)%4]->column	=	conCorner->column - conCornerCW2->column + conCornerCW3->column;
							(quad_group[i])->corners[(j+2)%4]->row		=	conCorner->row + conCorner->row - conCornerCW2->row;
							(quad_group[i])->corners[(j+2)%4]->column	=	conCorner->column + conCorner->column - conCornerCW2->column;
							(quad_group[i])->corners[(j+3)%4]->row		=	conCorner->row - conCornerCW2->row + conCornerCW1->row;
							(quad_group[i])->corners[(j+3)%4]->column	=	conCorner->column - conCornerCW2->column + conCornerCW1->column;


							// Mark this quad as labeled
							(quad_group[i])->labeled = true;


							// Changes have taken place, set the flag
							flag_changed = true;


							// once is enough!
							break;
						}
					}
				}
			}
		}
	}


	// All corners are marked with row and column
	// Record the minimal and maximal row and column indices
	// It is unlikely that more than 8bit checkers are used per dimension, if there are
	// an error would have been thrown at the beginning of "cvFindChessboardCorners2"
	int min_row		=  127;
	int max_row		= -127;
	int min_column	=  127;
	int max_column	= -127;

	for(int i = 0; i < count; i++ )
    {
		CvCBQuad* q = quad_group[i];

		for(int j = 0; j < 4; j++ )
		{
			if( (q->corners[j])->row > max_row)
				max_row = (q->corners[j])->row;

			if( (q->corners[j])->row < min_row)
				min_row = (q->corners[j])->row;

			if( (q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;

			if( (q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}

	// Label all internal corners with "needsNeighbor" = false
	// Label all external corners with "needsNeighbor" = true,
	// except if in a given dimension the pattern size is reached
	for(int i = min_row; i <= max_row; i++)
	{
			for(int j = min_column; j <= max_column; j++)
			{
				// A flag that indicates, wheter a row/column combination is
				// executed multiple times
				bool flagg = false;


				// Remember corner and quad
				int cornerID;
				int quadID;

				for(int k = 0; k < count; k++)
				{
					for(int l = 0; l < 4; l++)
					{
						if( ((quad_group[k])->corners[l]->row == i) && ((quad_group[k])->corners[l]->column == j) )
						{

							if (flagg == true)
							{
								// Passed at least twice through here
								(quad_group[k])->corners[l]->needsNeighbor = false;
								(quad_group[quadID])->corners[cornerID]->needsNeighbor = false;
							}
							else
							{
								// Mark with needs a neighbor, but note the
								// address
								(quad_group[k])->corners[l]->needsNeighbor = true;
								cornerID = l;
								quadID = k;
							}


							// set the flag to true
							flagg = true;
						}
					}
				}
			}
	}


	// Complete Linking:
	// sometimes not all corners were properly linked in "mrFindQuadNeighbors2",
	// but after labeling each corner with its respective row and column, it is
	// possible to match them anyway.
	for(int i = min_row; i <= max_row; i++)
	{
			for(int j = min_column; j <= max_column; j++)
			{
				// the following "number" indicates the number of corners which
				// correspond to the given (i,j) value
				// 1	is a border corner or a conrer which still needs a neighbor
				// 2	is a fully connected internal corner
				// >2	something went wrong during labeling, report a warning
				int number = 1;


				// remember corner and quad
				int cornerID;
				int quadID;

				for(int k = 0; k < count; k++)
				{
					for(int l = 0; l < 4; l++)
					{
						if( ((quad_group[k])->corners[l]->row == i) && ((quad_group[k])->corners[l]->column == j) )
						{

							if (number == 1)
							{
								// First corner, note its ID
								cornerID = l;
								quadID = k;
							}

							else if (number == 2)
							{
								// Second corner, check wheter this and the
								// first one have equal coordinates, else
								// interpolate
								float delta_x = (quad_group[k])->corners[l]->pt.x - (quad_group[quadID])->corners[cornerID]->pt.x;
								float delta_y = (quad_group[k])->corners[l]->pt.y - (quad_group[quadID])->corners[cornerID]->pt.y;

								if (delta_x != 0 || delta_y != 0)
								{
									// Interpolate
									(quad_group[k])->corners[l]->pt.x = (quad_group[k])->corners[l]->pt.x - delta_x/2;
									(quad_group[quadID])->corners[cornerID]->pt.x = (quad_group[quadID])->corners[cornerID]->pt.x + delta_x/2;
									(quad_group[k])->corners[l]->pt.y = (quad_group[k])->corners[l]->pt.y - delta_y/2;
									(quad_group[quadID])->corners[cornerID]->pt.y = (quad_group[quadID])->corners[cornerID]->pt.y + delta_y/2;
								}
							}
							else if (number > 2)
							{
								// Something went wrong during row/column labeling
								// Report a Warning
								// ->Implemented in the function "mrWriteCorners"
							}

							// increase the number by one
							number = number + 1;
						}
					}
				}
			}
	}


	// Bordercorners don't need any neighbors, if the pattern size in the
	// respective direction is reached
	// The only time we can make shure that the target pattern size is reached in a given
	// dimension, is when the larger side has reached the target size in the maximal
	// direction, or if the larger side is larger than the smaller target size and the
	// smaller side equals the smaller target size
	int largerDimPattern = max(pattern_size.height,pattern_size.width);
	int smallerDimPattern = min(pattern_size.height,pattern_size.width);
	bool flagSmallerDim1 = false;
	bool flagSmallerDim2 = false;

	if((largerDimPattern + 1) == max_column - min_column)
	{
		flagSmallerDim1 = true;
		// We found out that in the column direction the target pattern size is reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for( int k = 0; k < count; k++ )
		{
			for( int l = 0; l < 4; l++ )
			{
				if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}
	}

	if((largerDimPattern + 1) == max_row - min_row)
	{
		flagSmallerDim2 = true;
		// We found out that in the column direction the target pattern size is reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for( int k = 0; k < count; k++ )
		{
			for( int l = 0; l < 4; l++ )
			{
				if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}
	}


	// Check the two flags:
	//	-	If one is true and the other false, then the pattern target
	//		size was reached in in one direction -> We can check, whether the target
	//		pattern size is also reached in the other direction
	//  -	If both are set to true, then we deal with a square board -> do nothing
	//  -	If both are set to false -> There is a possibility that the larger side is
	//		larger than the smaller target size -> Check and if true, then check whether
	//		the other side has the same size as the smaller target size
	if( (flagSmallerDim1 == false && flagSmallerDim2 == true) )
	{
		// Larger target pattern size is in row direction, check wheter smaller target
		// pattern size is reached in column direction
		if((smallerDimPattern + 1) == max_column - min_column)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == true && flagSmallerDim2 == false) )
	{
		// Larger target pattern size is in column direction, check wheter smaller target
		// pattern size is reached in row direction
		if((smallerDimPattern + 1) == max_row - min_row)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_column - min_column )
	{
		// Larger target pattern size is in column direction, check wheter smaller target
		// pattern size is reached in row direction
		if((smallerDimPattern + 1) == max_row - min_row)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_row - min_row )
	{
		// Larger target pattern size is in row direction, check wheter smaller target
		// pattern size is reached in column direction
		if((smallerDimPattern + 1) == max_column - min_column)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}



// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	LabelQuadGroup.open("timer/LabelQuadGroup.txt", ofstream::app);
	LabelQuadGroup << "Time for mrLabelQuadGroup was " << time << " seconds." << endl;
	LabelQuadGroup.close();
#endif

}



//===========================================================================
// PRESERVE LARGEST QUAD GROUP
//===========================================================================
// Copies all necessary information of every quad of the largest found group
// into a new Quad struct array.
// This information is then again needed in PART 2 of the MAIN LOOP
static void mrCopyQuadGroup( CvCBQuad **temp_quad_group, CvCBQuad **for_out_quad_group, int count )
{
	for (int i = 0; i < count; i++)
	{
		for_out_quad_group[i]				= new CvCBQuad;
		for_out_quad_group[i]->count		= temp_quad_group[i]->count;
		for_out_quad_group[i]->edge_len		= temp_quad_group[i]->edge_len;
		for_out_quad_group[i]->group_idx	= temp_quad_group[i]->group_idx;
		for_out_quad_group[i]->labeled		= temp_quad_group[i]->labeled;

		for (int j = 0; j < 4; j++)
		{
			for_out_quad_group[i]->corners[j]					= new CvCBCorner;
			for_out_quad_group[i]->corners[j]->pt.x				= temp_quad_group[i]->corners[j]->pt.x;
			for_out_quad_group[i]->corners[j]->pt.y				= temp_quad_group[i]->corners[j]->pt.y;
			for_out_quad_group[i]->corners[j]->row				= temp_quad_group[i]->corners[j]->row;
			for_out_quad_group[i]->corners[j]->column			= temp_quad_group[i]->corners[j]->column;
			for_out_quad_group[i]->corners[j]->needsNeighbor	= temp_quad_group[i]->corners[j]->needsNeighbor;
		}
	}
}



//===========================================================================
// GIVE A GROUP IDX
//===========================================================================
// This function replaces mrFindQuadNeighbors, which in turn replaced
// icvFindQuadNeighbors
static void mrFindQuadNeighbors2( CvCBQuad *quads, int quad_count, int dilation)
{
//START TIMER
#if TIMER
	ofstream FindQuadNeighbors2;
    time_t  start_time = clock();
#endif

	// Thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	const float thresh_dilation = (float)(2*dilation+3)*(2*dilation+3)*2;	// the "*2" is for the x and y component
    int idx, i, k, j;														// the "3" is for initial corner mismatch
    float dx, dy, dist;
	int cur_quad_group = -1;


    // Find quad neighbors
    for( idx = 0; idx < quad_count; idx++ )
    {
        CvCBQuad* cur_quad = &quads[idx];


		// Go through all quadrangles and label them in groups
        // For each corner of this quadrangle
        for( i = 0; i < 4; i++ )
        {
            CvPoint2D32f pt;
            float min_dist = FLT_MAX;
            int closest_corner_idx = -1;
            CvCBQuad *closest_quad = 0;
            CvCBCorner *closest_corner = 0;

            if( cur_quad->neighbors[i] )
                continue;

            pt = cur_quad->corners[i]->pt;


            // Find the closest corner in all other quadrangles
            for( k = 0; k < quad_count; k++ )
            {
                if( k == idx )
                    continue;

                for( j = 0; j < 4; j++ )
                {
					// If it already has a neighbor
                    if( quads[k].neighbors[j] )
                        continue;

                    dx = pt.x - quads[k].corners[j]->pt.x;
                    dy = pt.y - quads[k].corners[j]->pt.y;
                    dist = dx * dx + dy * dy;


					// The following "if" checks, whether "dist" is the
					// shortest so far and smaller than the smallest
					// edge length of the current and target quads
                    if( dist < min_dist &&
						dist <= (cur_quad->edge_len + thresh_dilation) &&
                        dist <= (quads[k].edge_len + thresh_dilation)    )
                    {
						// First Check everything from the viewpoint of the current quad
						// compute midpoints of "parallel" quad sides 1
						float x1 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+1)%4]->pt.x)/2;
						float y1 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+1)%4]->pt.y)/2;
						float x2 = (cur_quad->corners[(i+2)%4]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y2 = (cur_quad->corners[(i+2)%4]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;
						// compute midpoints of "parallel" quad sides 2
						float x3 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y3 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;
						float x4 = (cur_quad->corners[(i+1)%4]->pt.x + cur_quad->corners[(i+2)%4]->pt.x)/2;
						float y4 = (cur_quad->corners[(i+1)%4]->pt.y + cur_quad->corners[(i+2)%4]->pt.y)/2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a1 = x1 - x2;
						float b1 = y1 - y2;
						// the current corner
						float c11 = cur_quad->corners[i]->pt.x - x2;
						float d11 = cur_quad->corners[i]->pt.y - y2;
						// the candidate corner
						float c12 = quads[k].corners[j]->pt.x - x2;
						float d12 = quads[k].corners[j]->pt.y - y2;
						float sign11 = a1*d11 - c11*b1;
						float sign12 = a1*d12 - c12*b1;

						float a2 = x3 - x4;
						float b2 = y3 - y4;
						// the current corner
						float c21 = cur_quad->corners[i]->pt.x - x4;
						float d21 = cur_quad->corners[i]->pt.y - y4;
						// the candidate corner
						float c22 = quads[k].corners[j]->pt.x - x4;
						float d22 = quads[k].corners[j]->pt.y - y4;
						float sign21 = a2*d21 - c21*b2;
						float sign22 = a2*d22 - c22*b2;


						// Then make shure that two border quads of the same row or
						// column don't link. Check from the current corner's view,
						// whether the corner diagonal from the candidate corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c13 = quads[k].corners[(j+2)%4]->pt.x - x2;
						float d13 = quads[k].corners[(j+2)%4]->pt.y - y2;
						float c23 = quads[k].corners[(j+2)%4]->pt.x - x4;
						float d23 = quads[k].corners[(j+2)%4]->pt.y - y4;
						float sign13 = a1*d13 - c13*b1;
						float sign23 = a2*d23 - c23*b2;


						// Then check everything from the viewpoint of the candidate quad
						// compute midpoints of "parallel" quad sides 1
						float u1 = (quads[k].corners[j]->pt.x + quads[k].corners[(j+1)%4]->pt.x)/2;
						float v1 = (quads[k].corners[j]->pt.y + quads[k].corners[(j+1)%4]->pt.y)/2;
						float u2 = (quads[k].corners[(j+2)%4]->pt.x + quads[k].corners[(j+3)%4]->pt.x)/2;
						float v2 = (quads[k].corners[(j+2)%4]->pt.y + quads[k].corners[(j+3)%4]->pt.y)/2;
						// compute midpoints of "parallel" quad sides 2
						float u3 = (quads[k].corners[j]->pt.x + quads[k].corners[(j+3)%4]->pt.x)/2;
						float v3 = (quads[k].corners[j]->pt.y + quads[k].corners[(j+3)%4]->pt.y)/2;
						float u4 = (quads[k].corners[(j+1)%4]->pt.x + quads[k].corners[(j+2)%4]->pt.x)/2;
						float v4 = (quads[k].corners[(j+1)%4]->pt.y + quads[k].corners[(j+2)%4]->pt.y)/2;

						// MARTIN: Heuristic
						// for the corner "j" of quad "k" to be considered, it
						// needs to be on the same side of the two lines as
						// corner "i". This is again given, if the cross
						//product has the same sign for both computations below:
						float a3 = u1 - u2;
						float b3 = v1 - v2;
						// the current corner
						float c31 = cur_quad->corners[i]->pt.x - u2;
						float d31 = cur_quad->corners[i]->pt.y - v2;
						// the candidate corner
						float c32 = quads[k].corners[j]->pt.x - u2;
						float d32 = quads[k].corners[j]->pt.y - v2;
						float sign31 = a3*d31-c31*b3;
						float sign32 = a3*d32-c32*b3;

						float a4 = u3 - u4;
						float b4 = v3 - v4;
						// the current corner
						float c41 = cur_quad->corners[i]->pt.x - u4;
						float d41 = cur_quad->corners[i]->pt.y - v4;
						// the candidate corner
						float c42 = quads[k].corners[j]->pt.x - u4;
						float d42 = quads[k].corners[j]->pt.y - v4;
						float sign41 = a4*d41-c41*b4;
						float sign42 = a4*d42-c42*b4;


						// Then make shure that two border quads of the same row or
						// column don't link. Check from the candidate corner's view,
						// whether the corner diagonal from the current corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c33 = cur_quad->corners[(i+2)%4]->pt.x - u2;
						float d33 = cur_quad->corners[(i+2)%4]->pt.y - v2;
						float c43 = cur_quad->corners[(i+2)%4]->pt.x - u4;
						float d43 = cur_quad->corners[(i+2)%4]->pt.y - v4;
						float sign33 = a3*d33-c33*b3;
						float sign43 = a4*d43-c43*b4;


						// Check whether conditions are fulfilled
						if ( ((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0))  &&
							 ((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0))  &&
							 ((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0))  &&
							 ((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0))  &&
							 ((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0))  &&
							 ((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0))  &&
							 ((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0))  &&
							 ((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0))    )

						{
							closest_corner_idx = j;
							closest_quad = &quads[k];
							min_dist = dist;
						}
                    }
                }
            }

            // Have we found a matching corner point?
            if( closest_corner_idx >= 0 && min_dist < FLT_MAX )
            {
                closest_corner = closest_quad->corners[closest_corner_idx];


                // Make shure that the closest quad does not have the current
				// quad as neighbor already
                for( j = 0; j < 4; j++ )
                {
                    if( closest_quad->neighbors[j] == cur_quad )
                        break;
                }
                if( j < 4 )
                    continue;


				// We've found one more corner - remember it
                closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
                closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;

                cur_quad->count++;
                cur_quad->neighbors[i] = closest_quad;
                cur_quad->corners[i] = closest_corner;

                closest_quad->count++;
                closest_quad->neighbors[closest_corner_idx] = cur_quad;
				closest_quad->corners[closest_corner_idx] = closest_corner;
            }
        }
    }

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindQuadNeighbors2.open("timer/FindQuadNeighbors2.txt", ofstream::app);
	FindQuadNeighbors2 << "Time for mrFindQuadNeighbors2 was " << time << " seconds." << endl;
	FindQuadNeighbors2.close();
#endif
}



//===========================================================================
// AUGMENT PATTERN WITH ADDITIONAL QUADS
//===========================================================================
// The first part of the function is basically a copy of
// "mrFindQuadNeighbors2"
// The comparisons between two points and two lines could be computed in their
// own function
static int mrAugmentBestRun( CvCBQuad *new_quads, int new_quad_count, int new_dilation,
							  CvCBQuad **old_quads, int old_quad_count, int old_dilation )
{
//START TIMER
#if TIMER
	ofstream AugmentBestRun;
    time_t  start_time = clock();
#endif

	// thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	const float thresh_dilation = (float)(2*new_dilation+3)*(2*old_dilation+3)*2;	// the "*2" is for the x and y component
    int idx, i, k, j;																// the "3" is for initial corner mismatch
    float dx, dy, dist;


    // Search all old quads which have a neighbor that needs to be linked
    for( idx = 0; idx < old_quad_count; idx++ )
    {
        CvCBQuad* cur_quad = old_quads[idx];


        // For each corner of this quadrangle
        for( i = 0; i < 4; i++ )
        {
            CvPoint2D32f pt;
            float min_dist = FLT_MAX;
            int closest_corner_idx = -1;
            CvCBQuad *closest_quad = 0;
            CvCBCorner *closest_corner = 0;


			// If cur_quad corner[i] is already linked, continue
            if( cur_quad->corners[i]->needsNeighbor == false )
                continue;

            pt = cur_quad->corners[i]->pt;


            // Look for a match in all new_quads' corners
            for( k = 0; k < new_quad_count; k++ )
            {
				// Only look at unlabeled new quads
				if( new_quads[k].labeled == true)
					continue;

                for( j = 0; j < 4; j++ )
                {

					// Only proceed if they are less than dist away from each
					// other
                    dx = pt.x - new_quads[k].corners[j]->pt.x;
                    dy = pt.y - new_quads[k].corners[j]->pt.y;
                    dist = dx * dx + dy * dy;

                    if( (dist < min_dist) &&
						dist <= (cur_quad->edge_len + thresh_dilation) &&
                        dist <= (new_quads[k].edge_len + thresh_dilation) )
                    {
						// First Check everything from the viewpoint of the
						// current quad compute midpoints of "parallel" quad
						// sides 1
						float x1 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+1)%4]->pt.x)/2;
						float y1 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+1)%4]->pt.y)/2;
						float x2 = (cur_quad->corners[(i+2)%4]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y2 = (cur_quad->corners[(i+2)%4]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;
						// compute midpoints of "parallel" quad sides 2
						float x3 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y3 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;
						float x4 = (cur_quad->corners[(i+1)%4]->pt.x + cur_quad->corners[(i+2)%4]->pt.x)/2;
						float y4 = (cur_quad->corners[(i+1)%4]->pt.y + cur_quad->corners[(i+2)%4]->pt.y)/2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a1 = x1 - x2;
						float b1 = y1 - y2;
						// the current corner
						float c11 = cur_quad->corners[i]->pt.x - x2;
						float d11 = cur_quad->corners[i]->pt.y - y2;
						// the candidate corner
						float c12 = new_quads[k].corners[j]->pt.x - x2;
						float d12 = new_quads[k].corners[j]->pt.y - y2;
						float sign11 = a1*d11 - c11*b1;
						float sign12 = a1*d12 - c12*b1;

						float a2 = x3 - x4;
						float b2 = y3 - y4;
						// the current corner
						float c21 = cur_quad->corners[i]->pt.x - x4;
						float d21 = cur_quad->corners[i]->pt.y - y4;
						// the candidate corner
						float c22 = new_quads[k].corners[j]->pt.x - x4;
						float d22 = new_quads[k].corners[j]->pt.y - y4;
						float sign21 = a2*d21 - c21*b2;
						float sign22 = a2*d22 - c22*b2;

						// Also make shure that two border quads of the same row or
						// column don't link. Check from the current corner's view,
						// whether the corner diagonal from the candidate corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c13 = new_quads[k].corners[(j+2)%4]->pt.x - x2;
						float d13 = new_quads[k].corners[(j+2)%4]->pt.y - y2;
						float c23 = new_quads[k].corners[(j+2)%4]->pt.x - x4;
						float d23 = new_quads[k].corners[(j+2)%4]->pt.y - y4;
						float sign13 = a1*d13 - c13*b1;
						float sign23 = a2*d23 - c23*b2;


						// Second: Then check everything from the viewpoint of
						// the candidate quad. Compute midpoints of "parallel"
						// quad sides 1
						float u1 = (new_quads[k].corners[j]->pt.x + new_quads[k].corners[(j+1)%4]->pt.x)/2;
						float v1 = (new_quads[k].corners[j]->pt.y + new_quads[k].corners[(j+1)%4]->pt.y)/2;
						float u2 = (new_quads[k].corners[(j+2)%4]->pt.x + new_quads[k].corners[(j+3)%4]->pt.x)/2;
						float v2 = (new_quads[k].corners[(j+2)%4]->pt.y + new_quads[k].corners[(j+3)%4]->pt.y)/2;
						// compute midpoints of "parallel" quad sides 2
						float u3 = (new_quads[k].corners[j]->pt.x + new_quads[k].corners[(j+3)%4]->pt.x)/2;
						float v3 = (new_quads[k].corners[j]->pt.y + new_quads[k].corners[(j+3)%4]->pt.y)/2;
						float u4 = (new_quads[k].corners[(j+1)%4]->pt.x + new_quads[k].corners[(j+2)%4]->pt.x)/2;
						float v4 = (new_quads[k].corners[(j+1)%4]->pt.y + new_quads[k].corners[(j+2)%4]->pt.y)/2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a3 = u1 - u2;
						float b3 = v1 - v2;
						// the current corner
						float c31 = cur_quad->corners[i]->pt.x - u2;
						float d31 = cur_quad->corners[i]->pt.y - v2;
						// the candidate corner
						float c32 = new_quads[k].corners[j]->pt.x - u2;
						float d32 = new_quads[k].corners[j]->pt.y - v2;
						float sign31 = a3*d31-c31*b3;
						float sign32 = a3*d32-c32*b3;

						float a4 = u3 - u4;
						float b4 = v3 - v4;
						// the current corner
						float c41 = cur_quad->corners[i]->pt.x - u4;
						float d41 = cur_quad->corners[i]->pt.y - v4;
						// the candidate corner
						float c42 = new_quads[k].corners[j]->pt.x - u4;
						float d42 = new_quads[k].corners[j]->pt.y - v4;
						float sign41 = a4*d41-c41*b4;
						float sign42 = a4*d42-c42*b4;

						// Also make shure that two border quads of the same row or
						// column don't link. Check from the candidate corner's view,
						// whether the corner diagonal from the current corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c33 = cur_quad->corners[(i+2)%4]->pt.x - u2;
						float d33 = cur_quad->corners[(i+2)%4]->pt.y - v2;
						float c43 = cur_quad->corners[(i+2)%4]->pt.x - u4;
						float d43 = cur_quad->corners[(i+2)%4]->pt.y - v4;
						float sign33 = a3*d33-c33*b3;
						float sign43 = a4*d43-c43*b4;


						// This time we also need to make shure, that no quad
						// is linked to a quad of another dilation run which
						// may lie INSIDE it!!!
						// Third: Therefore check everything from the viewpoint
						// of the current quad compute midpoints of "parallel"
						// quad sides 1
						float x5 = cur_quad->corners[i]->pt.x;
						float y5 = cur_quad->corners[i]->pt.y;
						float x6 = cur_quad->corners[(i+1)%4]->pt.x;
						float y6 = cur_quad->corners[(i+1)%4]->pt.y;
						// compute midpoints of "parallel" quad sides 2
						float x7 = x5;
						float y7 = y5;
						float x8 = cur_quad->corners[(i+3)%4]->pt.x;
						float y8 = cur_quad->corners[(i+3)%4]->pt.y;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the other side of the two lines than
						// corner "i". This is given, if the cross product has
						// a different sign for both computations below:
						float a5 = x6 - x5;
						float b5 = y6 - y5;
						// the current corner
						float c51 = cur_quad->corners[(i+2)%4]->pt.x - x5;
						float d51 = cur_quad->corners[(i+2)%4]->pt.y - y5;
						// the candidate corner
						float c52 = new_quads[k].corners[j]->pt.x - x5;
						float d52 = new_quads[k].corners[j]->pt.y - y5;
						float sign51 = a5*d51 - c51*b5;
						float sign52 = a5*d52 - c52*b5;

						float a6 = x8 - x7;
						float b6 = y8 - y7;
						// the current corner
						float c61 = cur_quad->corners[(i+2)%4]->pt.x - x7;
						float d61 = cur_quad->corners[(i+2)%4]->pt.y - y7;
						// the candidate corner
						float c62 = new_quads[k].corners[j]->pt.x - x7;
						float d62 = new_quads[k].corners[j]->pt.y - y7;
						float sign61 = a6*d61 - c61*b6;
						float sign62 = a6*d62 - c62*b6;


						// Fourth: Then check everything from the viewpoint of
						// the candidate quad compute midpoints of "parallel"
						// quad sides 1
						float u5 = new_quads[k].corners[j]->pt.x;
						float v5 = new_quads[k].corners[j]->pt.y;
						float u6 = new_quads[k].corners[(j+1)%4]->pt.x;
						float v6 = new_quads[k].corners[(j+1)%4]->pt.y;
						// compute midpoints of "parallel" quad sides 2
						float u7 = u5;
						float v7 = v5;
						float u8 = new_quads[k].corners[(j+3)%4]->pt.x;
						float v8 = new_quads[k].corners[(j+3)%4]->pt.y;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the other side of the two lines than
						// corner "i". This is given, if the cross product has
						// a different sign for both computations below:
						float a7 = u6 - u5;
						float b7 = v6 - v5;
						// the current corner
						float c71 = cur_quad->corners[i]->pt.x - u5;
						float d71 = cur_quad->corners[i]->pt.y - v5;
						// the candidate corner
						float c72 = new_quads[k].corners[(j+2)%4]->pt.x - u5;
						float d72 = new_quads[k].corners[(j+2)%4]->pt.y - v5;
						float sign71 = a7*d71-c71*b7;
						float sign72 = a7*d72-c72*b7;

						float a8 = u8 - u7;
						float b8 = v8 - v7;
						// the current corner
						float c81 = cur_quad->corners[i]->pt.x - u7;
						float d81 = cur_quad->corners[i]->pt.y - v7;
						// the candidate corner
						float c82 = new_quads[k].corners[(j+2)%4]->pt.x - u7;
						float d82 = new_quads[k].corners[(j+2)%4]->pt.y - v7;
						float sign81 = a8*d81-c81*b8;
						float sign82 = a8*d82-c82*b8;





						// Check whether conditions are fulfilled
						if ( ((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0))  &&
							 ((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0))  &&
							 ((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0))  &&
							 ((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0))	 &&
							 ((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0))  &&
							 ((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0))  &&
							 ((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0))  &&
							 ((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0))  &&
							 ((sign51 < 0 && sign52 > 0) || (sign51 > 0 && sign52 < 0))  &&
							 ((sign61 < 0 && sign62 > 0) || (sign61 > 0 && sign62 < 0))  &&
							 ((sign71 < 0 && sign72 > 0) || (sign71 > 0 && sign72 < 0))  &&
							 ((sign81 < 0 && sign82 > 0) || (sign81 > 0 && sign82 < 0)) )
						{
							closest_corner_idx = j;
							closest_quad = &new_quads[k];
							min_dist = dist;
						}
                    }
                }
            }

            // Have we found a matching corner point?
            if( closest_corner_idx >= 0 && min_dist < FLT_MAX )
            {
                closest_corner = closest_quad->corners[closest_corner_idx];
                closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
                closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;


                // We've found one more corner - remember it
				// ATTENTION: write the corner x and y coordinates separately,
				// else the crucial row/column entries will be overwritten !!!
                cur_quad->corners[i]->pt.x = closest_corner->pt.x;
				cur_quad->corners[i]->pt.y = closest_corner->pt.y;
				cur_quad->neighbors[i] = closest_quad;
				closest_quad->corners[closest_corner_idx]->pt.x = closest_corner->pt.x;
				closest_quad->corners[closest_corner_idx]->pt.y = closest_corner->pt.y;


				// Label closest quad as labeled. In this way we exclude it
				// being considered again during the next loop iteration
				closest_quad->labeled = true;


				// We have a new member of the final pattern, copy it over
				old_quads[old_quad_count]				= new CvCBQuad;
				old_quads[old_quad_count]->count		= 1;
				old_quads[old_quad_count]->edge_len		= closest_quad->edge_len;
				old_quads[old_quad_count]->group_idx	= cur_quad->group_idx;	//the same as the current quad
				old_quads[old_quad_count]->labeled		= false;				//do it right afterwards


				// We only know one neighbor for shure, initialize rest with
				// the NULL pointer
				old_quads[old_quad_count]->neighbors[closest_corner_idx]		= cur_quad;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+1)%4]	= NULL;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+2)%4]	= NULL;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+3)%4]	= NULL;

				for (int j = 0; j < 4; j++)
				{
					old_quads[old_quad_count]->corners[j]					= new CvCBCorner;
					old_quads[old_quad_count]->corners[j]->pt.x				= closest_quad->corners[j]->pt.x;
					old_quads[old_quad_count]->corners[j]->pt.y				= closest_quad->corners[j]->pt.y;
				}

				cur_quad->neighbors[i] = old_quads[old_quad_count];


				// Start the function again
				return -1;
            }
        }
    }

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	AugmentBestRun.open("timer/AugmentBestRun.txt", ofstream::app);
	AugmentBestRun << "Time for mrAugmentBestRun was " << time << " seconds." << endl;
	AugmentBestRun.close();
#endif

	// Finished, don't start the function again
	return 1;
}



//===========================================================================
// GENERATE QUADRANGLES
//===========================================================================
static int
icvGenerateQuads( CvCBQuad **out_quads, CvCBCorner **out_corners,
                  CvMemStorage *storage, CvMat *image, int flags, int dilation, bool firstRun )
{
//START TIMER
#if TIMER
	ofstream GenerateQuads;
    time_t  start_time = clock();
#endif

	// Initializations
    int quad_count = 0;
    CvMemStorage *temp_storage = 0;

    if( out_quads )
        *out_quads = 0;

    if( out_corners )
        *out_corners = 0;

    CV_FUNCNAME( "icvGenerateQuads" );

    //__BEGIN__;

    CvSeq *src_contour = 0;
    CvSeq *root;
    CvContourEx* board = 0;
    CvContourScanner scanner;
    int i, idx, min_size;

    CV_ASSERT( out_corners && out_quads );


    // Empiric sower bound for the size of allowable quadrangles.
	// MARTIN, modified: Added "*0.1" in order to find smaller quads.
	min_size = cvRound( image->cols * image->rows * .03 * 0.01 * 0.92 * 0.1);


    // Create temporary storage for contours and the sequence of pointers to
	// found quadrangles
    CV_CALL( temp_storage = cvCreateChildMemStorage( storage ));
    CV_CALL( root = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSeq*), temp_storage ));


    // Initialize contour retrieving routine
    CV_CALL( scanner = cvStartFindContours( image, temp_storage, sizeof(CvContourEx),
                                            CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ));


    // Get all the contours one by one
    while( (src_contour = cvFindNextContour( scanner )) != 0 )
    {
        CvSeq *dst_contour = 0;
        CvRect rect = ((CvContour*)src_contour)->rect;



        // Reject contours with a too small perimeter and contours which are
		// completely surrounded by another contour
		// MARTIN: If this function is called during PART 1, then the parameter "first run"
		// is set to "true". This guarantees, that only "nice" squares are detected.
		// During PART 2, we allow the polygonial approcimation function below to
		// approximate more freely, which can result in recognized "squares" that are in
		// reality multiple blurred and sticked together squares.
        if( CV_IS_SEQ_HOLE(src_contour) && rect.width*rect.height >= min_size )
        {
            int min_approx_level = 2, max_approx_level;
			if (firstRun == true)
				max_approx_level = 3;
			else
				max_approx_level = MAX_CONTOUR_APPROX;
            int approx_level;
            for( approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++ )
            {
                dst_contour = cvApproxPoly( src_contour, sizeof(CvContour), temp_storage,
                                            CV_POLY_APPROX_DP, (float)approx_level );


				// We call this again on its own output, because sometimes
                // cvApproxPoly() does not simplify as much as it should.
                dst_contour = cvApproxPoly( dst_contour, sizeof(CvContour), temp_storage,
                                            CV_POLY_APPROX_DP, (float)approx_level );

				if( dst_contour->total == 4 )
                    break;
            }


            // Reject non-quadrangles
            if(dst_contour->total == 4 && cvCheckContourConvexity(dst_contour) )
            {
                CvPoint pt[4];
                double d1, d2, p = cvContourPerimeter(dst_contour);
                double area = fabs(cvContourArea(dst_contour, CV_WHOLE_SEQ));
                double dx, dy;

                for( i = 0; i < 4; i++ )
                    pt[i] = *(CvPoint*)cvGetSeqElem(dst_contour, i);

                dx = pt[0].x - pt[2].x;
                dy = pt[0].y - pt[2].y;
                d1 = sqrt(dx*dx + dy*dy);

                dx = pt[1].x - pt[3].x;
                dy = pt[1].y - pt[3].y;
                d2 = sqrt(dx*dx + dy*dy);

                // PHILIPG: Only accept those quadrangles which are more
				// square than rectangular and which are big enough
                double d3, d4;
                dx = pt[0].x - pt[1].x;
                dy = pt[0].y - pt[1].y;
                d3 = sqrt(dx*dx + dy*dy);
                dx = pt[1].x - pt[2].x;
                dy = pt[1].y - pt[2].y;
                d4 = sqrt(dx*dx + dy*dy);
                if(true)//!(flags & CV_CALIB_CB_FILTER_QUADS) ||
                    //d3*4 > d4 && d4*4 > d3 && d3*d4 < area*1.5 && area > min_size &&
                    //d1 >= 0.15 * p && d2 >= 0.15 * p )
                {
					CvContourEx* parent = (CvContourEx*)(src_contour->v_prev);
					parent->counter++;
					if( !board || board->counter < parent->counter )
						board = parent;
					dst_contour->v_prev = (CvSeq*)parent;
					cvSeqPush( root, &dst_contour );
                }
            }
        }
    }


    // Finish contour retrieving
    cvEndFindContours( &scanner );


    // Allocate quad & corner buffers
    CV_CALL( *out_quads = (CvCBQuad*)cvAlloc(root->total * sizeof((*out_quads)[0])));
    CV_CALL( *out_corners = (CvCBCorner*)cvAlloc(root->total * 4 * sizeof((*out_corners)[0])));


    // Create array of quads structures
    for( idx = 0; idx < root->total; idx++ )
    {
        CvCBQuad* q = &(*out_quads)[quad_count];
        src_contour = *(CvSeq**)cvGetSeqElem( root, idx );
        if( (flags & CV_CALIB_CB_FILTER_QUADS) && src_contour->v_prev != (CvSeq*)board )
            continue;


        // Reset group ID
        memset( q, 0, sizeof(*q) );
        q->group_idx = -1;
        assert( src_contour->total == 4 );
        for( i = 0; i < 4; i++ )
        {
            CvPoint2D32f pt = cvPointTo32f(*(CvPoint*)cvGetSeqElem(src_contour, i));
            CvCBCorner* corner = &(*out_corners)[quad_count*4 + i];

            memset( corner, 0, sizeof(*corner) );
            corner->pt = pt;
            q->corners[i] = corner;
        }
        q->edge_len = FLT_MAX;
        for( i = 0; i < 4; i++ )
        {
            float dx = q->corners[i]->pt.x - q->corners[(i+1)&3]->pt.x;
            float dy = q->corners[i]->pt.y - q->corners[(i+1)&3]->pt.y;
            float d = dx*dx + dy*dy;
            if( q->edge_len > d )
                q->edge_len = d;
        }
        quad_count++;
    }

    //__END__;
exit:

    if( cvGetErrStatus() < 0 )
    {
        if( out_quads )
            cvFree( out_quads );
        if( out_corners )
            cvFree( out_corners );
        quad_count = 0;
    }

    cvReleaseMemStorage( &temp_storage );

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	GenerateQuads.open("timer/GenerateQuads.txt", ofstream::app);
	GenerateQuads << "Time for icvGenerateQuads was " << time << " seconds." << endl;
	GenerateQuads.close();
#endif

    return quad_count;
}



//===========================================================================
// WRITE CORNERS TO FILE
//===========================================================================
static int mrWriteCorners( CvCBQuad **output_quads, int count, CvSize pattern_size, int min_number_of_corners )
{
	// Initialize
	int corner_count = 0;
	bool flagRow = false;
	bool flagColumn = false;
	int maxPattern_sizeRow = -1;
	int maxPattern_sizeColumn = -1;


	// Return variable
	int internal_found = 0;


	// Compute the minimum and maximum row / column ID
	// (it is unlikely that more than 8bit checkers are used per dimension)
	int min_row		=  127;
	int max_row		= -127;
	int min_column	=  127;
	int max_column	= -127;

	for(int i = 0; i < count; i++ )
    {
		CvCBQuad* q = output_quads[i];

		for(int j = 0; j < 4; j++ )
		{
			if( (q->corners[j])->row > max_row)
				max_row = (q->corners[j])->row;
			if( (q->corners[j])->row < min_row)
				min_row = (q->corners[j])->row;
			if( (q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;
			if( (q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}


	// If in a given direction the target pattern size is reached, we know exactly how
	// the checkerboard is oriented.
	// Else we need to prepare enought "dummy" corners for the worst case.
	for(int i = 0; i < count; i++ )
    {
		CvCBQuad* q = output_quads[i];

		for(int j = 0; j < 4; j++ )
		{
			if( (q->corners[j])->column == max_column && (q->corners[j])->row != min_row && (q->corners[j])->row != max_row )
			{
				if( (q->corners[j]->needsNeighbor) == false)
				{
					// We know, that the target pattern size is reached
					// in column direction
					flagColumn = true;
				}
			}
			if( (q->corners[j])->row == max_row && (q->corners[j])->column != min_column && (q->corners[j])->column != max_column )
			{
				if( (q->corners[j]->needsNeighbor) == false)
				{
					// We know, that the target pattern size is reached
					// in row direction
					flagRow = true;
				}
			}
		}
	}

	if( flagColumn == true)
	{
		if( max_column - min_column == pattern_size.width + 1)
		{
			maxPattern_sizeColumn = pattern_size.width;
			maxPattern_sizeRow = pattern_size.height;
		}
		else
		{
			maxPattern_sizeColumn = pattern_size.height;
			maxPattern_sizeRow = pattern_size.width;
		}
	}
	else if( flagRow == true)
	{
		if( max_row - min_row == pattern_size.width + 1)
		{
			maxPattern_sizeRow = pattern_size.width;
			maxPattern_sizeColumn = pattern_size.height;
		}
		else
		{
			maxPattern_sizeRow = pattern_size.height;
			maxPattern_sizeColumn = pattern_size.width;
		}
	}
	else
	{
		// If target pattern size is not reached in at least one of the two
		// directions,  then we do not know where the remaining corners are
		// located. Account for this.
		maxPattern_sizeColumn = max(pattern_size.width, pattern_size.height);
		maxPattern_sizeRow = max(pattern_size.width, pattern_size.height);
	}


	// Open the output files
	ofstream cornersX("cToMatlab/cornersX.txt");
	ofstream cornersY("cToMatlab/cornersY.txt");
	ofstream cornerInfo("cToMatlab/cornerInfo.txt");


	// Write the corners in increasing order to the output file
	for(int i = min_row + 1; i < maxPattern_sizeRow + min_row + 1; i++)
	{
		for(int j = min_column + 1; j < maxPattern_sizeColumn + min_column + 1; j++)
		{
			// Reset the iterator
			int iter = 1;

			for(int k = 0; k < count; k++)
			{
				for(int l = 0; l < 4; l++)
				{
					if(((output_quads[k])->corners[l]->row == i) && ((output_quads[k])->corners[l]->column == j) )
					{
						// Only write corners to the output file, which are connected
						// i.e. only if iter == 2
						if( iter == 2)
						{
							// The respective row and column have been found, print it to
							// the output file, only do this once
							cornersX << (output_quads[k])->corners[l]->pt.x;
							cornersX << " ";
							cornersY << (output_quads[k])->corners[l]->pt.y;
							cornersY << " ";

							corner_count++;
						}


						// If the iterator is larger than two, this means that more than
						// two corners have the same row / column entries. Then some
						// linking errors must have occured and we should not use the found
						// pattern
						if (iter > 2)
							return -1;

						iter++;
					}
				}
			}

			// If the respective row / column is non - existent or is a border corner
			if (iter == 1 || iter == 2)
			{
				cornersX << -1;
				cornersX << " ";
				cornersY << -1;
				cornersY << " ";
			}
		}
		cornersX << endl;
		cornersY << endl;
	}


	// Write to the corner matrix size info file
	cornerInfo << maxPattern_sizeRow<< " " << maxPattern_sizeColumn << endl;


	// Close the output files
	cornersX.close();
	cornersY.close();
	cornerInfo.close();


	// check whether enough corners have been found
	if (corner_count >= min_number_of_corners)
		internal_found = 1;
	else
		internal_found = 0;


	// pattern found, or not found?
	return internal_found;
}

//===========================================================================
// END OF FILE
//===========================================================================

