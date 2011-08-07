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

#include <mrpt/hwdrivers.h> // Precompiled header

using namespace mrpt::hwdrivers;

#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV
	// Include the OPENCV libraries:
	#if defined(_MSC_VER)
		#pragma comment (lib,"PGRFlyCapture.lib")
		#pragma comment (lib,"triclops.lib")
	#endif
	#define CV_NO_CVV_IMAGE   // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#	else
#		include <cv.h>
#		include <highgui.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif

	#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#endif

#if MRPT_HAS_BUMBLEBEE
	#include <PGRFlyCapture.h>
	#include <triclops.h>
	#include <pgrflycapturestereo.h>
	#include <pnmutils.h>
#endif

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
	 return; \
   } \
} \

#define _HANDLE_TRICLOPS_ERROR_RET( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
	 return false; \
   } \
} \


//
// Macro to check, report on, and handle Flycapture API error codes.
//
#define _HANDLE_FLYCAPTURE_ERROR( function, error ) \
{ \
   if( error != FLYCAPTURE_OK ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 flycaptureErrorToString( error ) ); \
	 return; \
   } \
} \

#define _HANDLE_FLYCAPTURE_ERROR_RET( function, error ) \
{ \
   if( error != FLYCAPTURE_OK ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 flycaptureErrorToString( error ) ); \
	 return false; \
   } \
} \

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CStereoGrabber_Bumblebee::CStereoGrabber_Bumblebee(
	int								cameraIndex,
	const TCaptureOptions_bumblebee &options ) :
		m_triclops( NULL ),
		m_flycapture( NULL ),
		m_imgBuff( ),
		m_bInitialized(false),
		m_resolutionX( options.frame_width ),
		m_resolutionY( options.frame_height ),
		m_baseline( 0 ),
		m_focalLength( 0 ),
		m_centerCol( 0 ),
		m_centerRow( 0 ),
		m_options( options )
{
    MRPT_START
#if MRPT_HAS_BUMBLEBEE

	// Error handling
	TriclopsError     te;
	FlyCaptureError   fe;

	// open the Digiclops
	fe = flycaptureCreateContext( &m_flycapture );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureCreateContext()", fe );

	// assume we are getting the first one on the bus, device 0
	fe = flycaptureInitialize( m_flycapture, cameraIndex );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureInitialize()", fe );

	// get the camera module configuration
	char* szCalFile;
	fe = flycaptureGetCalibrationFileFromCamera( m_flycapture, &szCalFile );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCalibrationFileFromCamera()", fe );

	// Get context triclops context from file
	te = triclopsGetDefaultContextFromFile(&m_triclops,szCalFile );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

	// Get camera information and ensure that it is correct
	FlyCaptureInfoEx	   pInfo;
	fe = flycaptureGetCameraInfo( m_flycapture, &pInfo );
	_HANDLE_FLYCAPTURE_ERROR( "flycatpureGetCameraInfo()", fe );

	ASSERT_( pInfo.CameraType == FLYCAPTURE_COLOR );				// Color Camera
	ASSERT_( pInfo.CameraModel == FLYCAPTURE_BUMBLEBEE2 );			// Bumblebee 2 Camera

	unsigned long ulValue;
	fe = flycaptureGetCameraRegister( m_flycapture, 0x1F28, &ulValue );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCameraRegister()", fe );
	ASSERT_( (ulValue & 0x2) == 0 );								// High-resolution

	// Set some camera parameters.
	const unsigned int		MODE		= 3;					// Supported modes: 0 & 3 (stereo)	(1024x768)
	const unsigned int		sCol		= 0;					// Starting column
	const unsigned int		sRow		= 0;					// Starting row
	const float				fBW			= 100.0f;				// Bandwith percent for transmitting
	const int				iMaxCols	= 1024;					// Ending column
	const int				iMaxRows	= 768;					// Ending row
	FlyCapturePixelFormat	pixelFormat	= FLYCAPTURE_RAW16;		// 16 bit raw data output from sensor (color)
	//FlyCapturePixelFormat pixelFormat	= FLYCAPTURE_MONO8;
	
	// Start transferring images from the camera to the computer
	fe = flycaptureStartCustomImage( m_flycapture, MODE, sCol, sRow, iMaxCols, iMaxRows, fBW, pixelFormat );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureStartCustomImage()", fe );

	//  ------------------------------------------------------
	//   TRICLOPS CONFIGURATION
	//  ------------------------------------------------------

	// Set rectified resolution
	te = triclopsSetResolution( m_triclops, m_resolutionY, m_resolutionX );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );

	// Retrieve camera parameters
	te = triclopsGetBaseline( m_triclops, &m_baseline );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetBaseline()", te );

	te = triclopsGetFocalLength( m_triclops, &m_focalLength );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetFocalLength()", te );

	te = triclopsGetImageCenter( m_triclops, &m_centerRow, &m_centerCol);
	_HANDLE_TRICLOPS_ERROR( "triclopsGetImageCenter()", te );

	//FlyCaptureVideoMode video;
	//FlyCaptureFrameRate cFR;

	//fe = flycaptureGetCurrentVideoMode( m_flycapture, &video, &cFR );
	//_HANDLE_FLYCAPTURE_ERROR( "flycaptureStartCustomImage()", fe );

	// set disparity range
	//te = triclopsSetDisparity( m_triclops, 1, round(0.3f*m_resolutionX)) ;
 //  	_HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );

	//// set the edge and stereo masks
	//te = triclopsSetStereoMask( m_triclops, 11 );
 //  	_HANDLE_TRICLOPS_ERROR( "triclopsSetStereoMask()", te );
	//te = triclopsSetEdgeCorrelation( m_triclops, 1 );
	//_HANDLE_TRICLOPS_ERROR( "triclopsSetEdgeCorrelation()", te );
	//te = triclopsSetEdgeMask( m_triclops, 11 );
	//_HANDLE_TRICLOPS_ERROR( "triclopsSetEdgeMask()", te );

	//te = triclopsSetTextureValidation( m_triclops, 1 );
 //  	_HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te );
	//te = triclopsSetTextureValidationThreshold( m_triclops, 4.00f );
 //   _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidationThreshold()", te );
	//te = triclopsSetUniquenessValidation( m_triclops, 1 );
 //  	_HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te );
	//te = triclopsSetUniquenessValidationThreshold( m_triclops, 0.84f);
	//_HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidationThreshold()", te );

	//// turn on sub-pixel interpolation
	//te = triclopsSetSubpixelInterpolation( m_triclops, 1 );
	//_HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );


	// set the digiclops to deliver all images
   // if (FLYCAPTURE_OK != digiclopsSetImageTypes( m_flycapture, ALL_IMAGES ) ) return;

   // set the Digiclops resolution
   // use 'HALF' resolution when you need faster throughput, especially for
   // color images
   // digiclopsSetImageResolution( digiclops, DIGICLOPS_HALF );
   //if (FLYCAPTURE_OK != digiclopsSetImageResolution(m_digiclops, DIGICLOPS_FULL ) ) return;

   //// start grabbing
   //if (FLYCAPTURE_OK != digiclopsStart( m_digiclops ) ) return;

   // Set up image buffers for the context
   //triclopsSetImageBuffer( m_triclops, m_pRectImageLeft, TriImg_RECTIFIED, TriCam_LEFT );
   //triclopsSetImageBuffer( m_triclops, m_pRectImageRight, TriImg_RECTIFIED, TriCam_RIGHT );
   //triclopsSetImage16Buffer( m_triclops, m_pDispImage, TriImg16_DISPARITY, TriCam_REFERENCE );

   // remember that we successfully initialized everything
   m_bInitialized = true;

#else
	MRPT_UNUSED_PARAM(cameraIndex);
	MRPT_UNUSED_PARAM(options);
	THROW_EXCEPTION("This class is not available. Recompile MRPT with option 'MRPT_HAS_BUMBLEBEE' on");
#endif
    MRPT_END
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CStereoGrabber_Bumblebee::~CStereoGrabber_Bumblebee()
{
#if MRPT_HAS_BUMBLEBEE
	MRPT_START

	FlyCaptureError		fe;
	TriclopsError		te;

	if ( m_bInitialized )
	{
		// Stop grabbing images
		fe = flycaptureStop( m_flycapture );
		_HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );

		// Destroy data contexts
		fe = flycaptureDestroyContext( m_flycapture );
		_HANDLE_FLYCAPTURE_ERROR( "flycaptureDestroyContext()", fe );
		te = triclopsDestroyContext( m_triclops );
		_HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
		
	}

	MRPT_END
#endif
}

/*-------------------------------------------------------------
					getStereoObservation
 -------------------------------------------------------------*/
bool  CStereoGrabber_Bumblebee::getStereoObservation( mrpt::slam::CObservationStereoImages &out_observation )
{
#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV

	FlyCaptureError				fe;
	TriclopsError				te;

	FlyCaptureImage				flycaptureImage;

	IplImage					*imageL, *imageR;	// Output pair of images
	TriclopsImage				imL, imR;

	// Set the timestamp:
	out_observation.timestamp = mrpt::system::now();

	// Grab the image
	fe = flycaptureGrabImage2( m_flycapture, &flycaptureImage );
	_HANDLE_FLYCAPTURE_ERROR_RET( "flycaptureGrabImage2()", fe );

	// Get image information:
	int	nImages					= flycaptureImage.iNumImages;
	int	nRows					= flycaptureImage.iRows;
	int	nCols					= flycaptureImage.iCols;
	int	rowInc					= flycaptureImage.iRowInc;

	// Reserve memory according to image color mode
	m_imgBuff.resize( m_options.color ? nCols*nRows*nImages*4 : nCols*nRows*nImages );

	// Create a temporary FlyCaptureImage for preparing the stereo image
	FlyCaptureImage tempImage;
	tempImage.pData = &m_imgBuff[0];

	// Convert the pixel interleaved raw data to row interleaved format
	fe = flycapturePrepareStereoImage( m_flycapture, flycaptureImage,  m_options.color ? NULL : &tempImage, m_options.color ? &tempImage : NULL);
	_HANDLE_FLYCAPTURE_ERROR_RET( "flycapturePrepareStereoImage()", fe );

	// Create output images
	imageL = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, m_options.color ? 3 : 1 );
	imageR = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, m_options.color ? 3 : 1 );

	if( m_options.color )
	{
		if( m_options.getRectified )
		{
			// Create temp images to store the 4 channel output images
			IplImage *tmpImageL = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, 4 );
			IplImage *tmpImageR = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, 4 );

			TriclopsInput				colorInput;
			TriclopsPackedColorImage	colorImageL, colorImageR;

			// Use the row interleaved images to build up a packed TriclopsInput.
			// A packed triclops input will contain a single image with 32 bpp.
			te = triclopsBuildPackedTriclopsInput(
				nCols,
				nRows,
				rowInc * 4,
				flycaptureImage.timeStamp.ulSeconds,
				flycaptureImage.timeStamp.ulMicroSeconds,
				&m_imgBuff[0],
				&colorInput );
			_HANDLE_TRICLOPS_ERROR_RET( "triclopsBuildPackedTriclopsInput()", te );

			// Rectify
			te = triclopsRectifyPackedColorImage( m_triclops, TriCam_LEFT, &colorInput, &colorImageL );
			_HANDLE_TRICLOPS_ERROR_RET( "triclopsRectifyPackedColorImage()", te );

			te = triclopsRectifyPackedColorImage( m_triclops, TriCam_RIGHT, &colorInput, &colorImageR );
			_HANDLE_TRICLOPS_ERROR_RET( "triclopsRectifyPackedColorImage()", te );

			// Copy image data
			memcpy( tmpImageL->imageData, colorImageL.data, colorImageL.nrows*colorImageL.rowinc );
			tmpImageL->widthStep = colorImageL.rowinc;
			memcpy( tmpImageR->imageData, colorImageR.data, colorImageL.nrows*colorImageL.rowinc );
			tmpImageR->widthStep = colorImageR.rowinc;

			// Convert images to BGR (3 channels) and set origins
			cvCvtColor( tmpImageL, imageL, CV_BGRA2BGR );	// Left
			imageL->origin = tmpImageL->origin;

			cvCvtColor( tmpImageR, imageR, CV_BGRA2BGR );	// Right
			imageR->origin = tmpImageR->origin;

			// Release temp images
			cvReleaseImage( &tmpImageL );
			cvReleaseImage( &tmpImageR );

			// Load the images:
			out_observation.imageLeft.setFromIplImage( imageL );
			out_observation.imageRight.setFromIplImage( imageR );

		} // end COLOR & RECTIFIED
		else
		{
			if( flycaptureImage.bStippled )
			{
				FlyCaptureImage colorImage;
				colorImage.pData			= new unsigned char[flycaptureImage.iRows*flycaptureImage.iCols*flycaptureImage.iNumImages*3*2];	// ROWS x COLS x NUM_IMGS x 3 COLORS x 2 BYTES
				colorImage.pixelFormat		= FLYCAPTURE_BGR;													// Format BGR

				fe = flycaptureConvertImage( m_flycapture, &flycaptureImage, &colorImage );
				
				// ColorImage contains the two raw images in format BGR
				if( fe != FLYCAPTURE_OK )
				{
					delete [] colorImage.pData;
					cout << "Error when converting the grabbed image to BGR(U)." << endl;
				}

				IplImage* tmpL = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, 4 /*BGRU*/);
				IplImage* tmpR = cvCreateImage( cvSize( m_options.frame_width, m_options.frame_height ), IPL_DEPTH_8U, 4 /*BGRU*/);
				
				convertFlyCaptureImagesToIplImages( &colorImage, tmpL, tmpR );

				// Free allocated memory
				delete [] colorImage.pData;

				// Convert images to BGR (3 channels) and set origins
				cvCvtColor( tmpL, imageL, CV_BGRA2BGR );	// Left
				imageL->origin = tmpL->origin;

				cvCvtColor( tmpR, imageR, CV_BGRA2BGR );	// Right
				imageR->origin = tmpR->origin;

				out_observation.imageLeft.setFromIplImage( imageL );
				out_observation.imageRight.setFromIplImage( imageR );

				cvReleaseImage( &tmpL );
				cvReleaseImage( &tmpR );
			}
			else
				cout << "Error: image is not stippled -> consider this case" << endl;

		} // end COLOR & NOT RECTIFIED

	} // END COLOR
	else
	{
		TriclopsInput	Input;

		// Pointers to positions in the mono buffer that correspond to the beginning
		// of the red, green and blue sections
		unsigned char* redMono = NULL;
		unsigned char* greenMono = NULL;
		unsigned char* blueMono = NULL;
		
		redMono		= &m_imgBuff[0];
		greenMono	= redMono + nCols;
		blueMono	= redMono + nCols;

		// Use the row interleaved images to build up an RGB TriclopsInput.
		// An RGB triclops input will contain the 3 raw images (1 from each camera).
		te = triclopsBuildRGBTriclopsInput(
				nCols,
				nRows,
				rowInc,
				flycaptureImage.timeStamp.ulSeconds,
				flycaptureImage.timeStamp.ulMicroSeconds,
				redMono,
				greenMono,
				blueMono,
				&Input );
		_HANDLE_TRICLOPS_ERROR_RET( "triclopsBuildRGBTriclopsInput()", te );

		te = triclopsRectify( m_triclops, &Input );
		_HANDLE_TRICLOPS_ERROR_RET( "triclopsRectify()", te );

		te = triclopsGetImage( m_triclops, m_options.getRectified ? TriImg_RECTIFIED : TriImg_RAW, TriCam_LEFT, &imL );
		_HANDLE_TRICLOPS_ERROR_RET( "triclopsGetImage()", te );
		te = triclopsGetImage( m_triclops, m_options.getRectified ? TriImg_RECTIFIED : TriImg_RAW, TriCam_RIGHT, &imR );
		_HANDLE_TRICLOPS_ERROR_RET( "triclopsGetImage()", te );

		if( m_options.getRectified )
		{
			// The images
			memcpy( imageL->imageData, imL.data, imL.nrows*imL.rowinc );
			imageL->widthStep = imL.rowinc;
			memcpy( imageR->imageData, imR.data, imR.nrows*imR.rowinc );
			imageR->widthStep = imR.rowinc;
		}
		else
		{
			// Both imL and imR contains the two raw images -> we must separate them
			// Resize to the desired resolution (if necessary)
			if( out_observation.imageLeft.getWidth() != imL.ncols || out_observation.imageLeft.getHeight() != imL.nrows )		// Resize to desired output image
				out_observation.imageLeft.resize( imL.ncols, imL.nrows, 1 /*nChannels*/, true /*Top-Left origin*/);
			if( out_observation.imageRight.getWidth() != imR.ncols || out_observation.imageRight.getHeight() != imR.nrows )		// Resize to desired output image
				out_observation.imageRight.resize( imR.ncols, imR.nrows, 1 /*nChannels*/, true /*Top-Left origin*/);

			// Split the TriclopsImage into two separate IplImages
			convertTriclopsImagesToIplImages( &imL, imageL, imageR );
		}
		// Load the images:
		out_observation.imageLeft.setFromIplImage( imageL );
		out_observation.imageRight.setFromIplImage( imageR );
	} // END GRAYSCALE

	// Fill output observation:
	out_observation.rightCameraPose.x( m_baseline );
	out_observation.rightCameraPose.y( 0 );
	out_observation.rightCameraPose.z( 0 );

	out_observation.rightCameraPose.quat().r( 1 );
	out_observation.rightCameraPose.quat().x( 0 );
	out_observation.rightCameraPose.quat().y( 0 );
	out_observation.rightCameraPose.quat().z( 0 );
		
	out_observation.cameraPose.x( 0 );
	out_observation.cameraPose.y( 0 );
	out_observation.cameraPose.z( 0 );

	out_observation.cameraPose.quat().r( 1 );
	out_observation.cameraPose.quat().x( 0 );
	out_observation.cameraPose.quat().y( 0 );
	out_observation.cameraPose.quat().z( 0 );

	out_observation.leftCamera.setIntrinsicParamsFromValues( m_focalLength, m_focalLength, m_centerCol, m_centerRow );

	return true;
#else
	MRPT_UNUSED_PARAM(out_observation);
	return false;
#endif
}

/*-------------------------------------------------------------
					scaleImage
 -------------------------------------------------------------*/
void CStereoGrabber_Bumblebee::scaleImage( void* _image, unsigned char	ucMinOut,  unsigned char	ucMaxOut )
{
#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV
	TriclopsImage* image = static_cast<TriclopsImage*>(_image);
	int r, c;

	double dMinOut = (double) ucMinOut;
	double dMaxOut = (double) ucMaxOut;

	// find the max and minimum disparities
	double dMaxDisp = 0;
	double dMinDisp = 255;
	for ( r = 0; r < image->nrows; r++ )
	{
		unsigned char* 	pucSrc = image->data + r*image->rowinc;
		for ( c = 0; c < image->ncols; c++ )
		{
			// note: 240 is the limit of the normal disparity range
			if ( pucSrc[c] < 240 )
			{
				double dDisp = (double) pucSrc[c];
				if ( dMaxDisp < dDisp )
					dMaxDisp = dDisp;
				if ( dMinDisp > dDisp )
					dMinDisp = dDisp;
			}
		} // end for
	} // end for

    // scale the output to take the disparity values of the input image that fall within
    // dMinDisp to dMaxDisp to fall within ucMinOut and ucMaxOut for the 8 bit output image
    for ( r = 0; r < image->nrows; r++ )
    {
		unsigned char* 	pucSrc = image->data + r*image->rowinc;
		for ( c = 0; c < image->ncols; c++ )
		{
			if ( pucSrc[c] < 240 )
			{
				double dDisp = (double) pucSrc[c];
				double dOut = (dDisp-dMinDisp)*(dMaxOut-dMinOut)/(dMaxDisp-dMinDisp);
				dOut += dMinOut;
				pucSrc[c]	= (unsigned char) dOut;
			 }
			else
			{
				pucSrc[c]	= 0;
			}
		} // end for
	} // end for
#endif
} // end scaleImage


void CStereoGrabber_Bumblebee::convertFlyCaptureImagesToIplImages( void* flycapImage, void* dstL, void* dstR ) // COLOR
{
#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV
	FlyCaptureImage*	fcImg		= static_cast<FlyCaptureImage*>(flycapImage);
	IplImage*			leftImg		= static_cast<IplImage*>(dstL);
	IplImage*			rightImg	= static_cast<IplImage*>(dstR);

	IplImage			*tmpL, *tmpR;
	bool				mustResize = false;

	if( leftImg->width != fcImg->iCols/2 || leftImg->height != fcImg->iRows )
		mustResize = true;

	if( mustResize )
	{
		tmpL = cvCreateImage( cvSize(fcImg->iCols/2,fcImg->iRows), IPL_DEPTH_8U, 4 );
		tmpR = cvCreateImage( cvSize(fcImg->iCols/2,fcImg->iRows), IPL_DEPTH_8U, 4 );
	}

	ASSERT_( fcImg->iRowInc == 4*fcImg->iCols );		// 2 images of type BRGU
	ASSERT_( leftImg->width <= fcImg->iCols/2 && rightImg->width <= fcImg->iCols/2 && leftImg->height <= fcImg->iRows && rightImg->height <= fcImg->iRows );
	ASSERT_( leftImg->width == rightImg->width && leftImg->height == rightImg->height );
	ASSERT_( leftImg->depth == IPL_DEPTH_8U && rightImg->depth == IPL_DEPTH_8U );

	unsigned char *p1Src, *p2Src;
	char *p1Dst, *p2Dst;
	p1Src = &(fcImg->pData[0]);												// Source
	p1Dst = mustResize ? &(tmpL->imageData[0]) : &(leftImg->imageData[0]);	// Destination
	p2Src = p1Src + 2*fcImg->iCols;											// Source
	p2Dst = mustResize ? &(tmpR->imageData[0]) : &(rightImg->imageData[0]);	// Destination
	for( int r = 0; r < fcImg->iRows; r++ )
	{
		memcpy( p1Dst, p1Src, 2*fcImg->iCols );
		memcpy( p2Dst, p2Src, 2*fcImg->iCols );
		p1Src += fcImg->iRowInc;
		p2Src += fcImg->iRowInc;
		p1Dst += 2*fcImg->iCols;
		p2Dst += 2*fcImg->iCols;
	}

	if( mustResize )
	{
		// Resize the output images
		cvResize( tmpL, dstL );
		cvResize( tmpR, dstR );

		cvReleaseImage( &tmpL );
		cvReleaseImage( &tmpR );
	}

#endif
} // end-convertFlyCaptureImagesToIplImages

/*-------------------------------------------------------------
					convertTriclopsImagesToIplImages
 -------------------------------------------------------------*/
void CStereoGrabber_Bumblebee::convertTriclopsImagesToIplImages( void* triclopsImage, void* dstL, void* dstR ) // GRAY
{
#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV
	TriclopsImage*	trImg		= static_cast<TriclopsImage*>(triclopsImage);
	IplImage*		leftImg		= static_cast<IplImage*>(dstL);
	IplImage*		rightImg	= static_cast<IplImage*>(dstR);

	IplImage		*tmpL, *tmpR;
	bool			mustResize = false;

	if( leftImg->width != trImg->ncols/2 || leftImg->height != trImg->nrows )
		mustResize = true;
	
	if( mustResize )
	{
		tmpL = cvCreateImage( cvSize(trImg->ncols,trImg->nrows), IPL_DEPTH_8U, 1 );
		tmpR = cvCreateImage( cvSize(trImg->ncols,trImg->nrows), IPL_DEPTH_8U, 1 );
	}

	ASSERT_( trImg->rowinc == 2*trImg->ncols );
	ASSERT_( leftImg->width <= trImg->ncols && rightImg->width <= trImg->ncols && leftImg->height <= trImg->nrows && rightImg->height <= trImg->nrows );
	ASSERT_( leftImg->width == rightImg->width && leftImg->height == rightImg->height );
	ASSERT_( leftImg->depth == IPL_DEPTH_8U && rightImg->depth == IPL_DEPTH_8U );

	unsigned char *p1Src, *p2Src;
	char *p1Dst, *p2Dst;
	p1Src = &(trImg->data[0]);				// Source
	p1Dst = mustResize ? &(tmpL->imageData[0]) : &(leftImg->imageData[0]);		// Destination
	p2Src = p1Src + trImg->ncols;			// Source
	p2Dst = mustResize ? &(tmpR->imageData[0]) : &(rightImg->imageData[0]);		// Destination
	for( int r = 0; r < trImg->nrows; r++ )
	{
		memcpy( p1Dst, p1Src, trImg->ncols );
		memcpy( p2Dst, p2Src, trImg->ncols );
		p1Src += trImg->rowinc;
		p2Src += trImg->rowinc;
		p1Dst += trImg->ncols;
		p2Dst += trImg->ncols;
	}
	
	if( mustResize )
	{
		// Resize the output images
		cvResize( tmpL, dstL );
		cvResize( tmpR, dstR );

		cvReleaseImage( &tmpL );
		cvReleaseImage( &tmpR );
	}
#endif
}

/*-------------------------------------------------------------
					convertTriclopsImageTo8BitsIplImage
 -------------------------------------------------------------*/
void CStereoGrabber_Bumblebee::convertTriclopsImageTo8BitsIplImage( void *src, void* dst )
{
#if MRPT_HAS_BUMBLEBEE && MRPT_HAS_OPENCV
	TriclopsImage* image = static_cast<TriclopsImage*>(src);
	IplImage* imageDst = static_cast<IplImage*>(dst);

	ASSERT_( image->rowinc == 2*image->ncols );
	ASSERT_( imageDst->depth == IPL_DEPTH_8U );

	double pixel, mxPixel = 0;
	int idx, r, c;

	for( r = 0; r < image->nrows; r++ )
	{
		for( c = 0; c < image->rowinc; c += 2 )
		{
			idx = c + r*image->rowinc;
			pixel = image->data[idx+1] + (image->data[idx] << 8);
			if( pixel > mxPixel )
				mxPixel = pixel;
		} // end for
	}

	double fc = 255/mxPixel;
	char* wDst = &imageDst->imageData[0];
	for( r = 0; r < image->nrows; r++ )
	{
		for( c = 0; c < image->rowinc; c += 2 )
		{
			idx = c + r*image->rowinc;
			pixel = image->data[idx+1] + (image->data[idx] << 8);

			(*wDst) = (char)(fc*pixel);
			wDst++;
		} // end for
	}
#endif
} // end convertTriclopsImageTo8BitsIplImage

// ******************************** FAMD ****************************************************
/*-------------------------------------------------------------
					getObservation with ROI
 -------------------------------------------------------------*/

//bool  CStereoGrabber_Bumblebee::getObservation(
//		TROI ROI,
//		mrpt::slam::CObservationVisualLandmarks &out_observation)
//{
//#if MRPT_HAS_BUMBLEBEE
//	MRPT_START
//
//	unsigned int		x,y;
//	unsigned short		disp;
//
//	float				x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f, z_min = 0.0f, z_max = 0.0f;
//
//	TriclopsColorImage  colorImage;
//	TriclopsInput       colorData;
//
//	// Set the timestep:
//	out_observation.timestamp = mrpt::system::now();
//
//	// Start with an empty landmarks map:
////	out_observation.landmarks.landmarks.reserve(30000);
//	out_observation.landmarks.landmarks.clear();
//
//
//	if (FLYCAPTURE_OK != digiclopsGrabImage( m_digiclops ) )
//		return false;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, STEREO_IMAGE, &m_triclopsInput ) )
//		return false;
//
//	// For getting the colors of the 3D points later:
//	digiclopsExtractTriclopsInput( m_digiclops, RIGHT_IMAGE, &colorData );
//
//	// Disparity will be stored in our custom buffer:
//	triclopsPreprocess( m_triclops, &m_triclopsInput );
//	triclopsStereo ( m_triclops );
//	triclopsRectifyColorImage( m_triclops, TriCam_REFERENCE,&colorData, &colorImage );
//
//	// Create a points-cloud representation:
//	// ---------------------------------------------
//
//	// Go across all the pixels:
//	// --------------------------------------------
//	for (y=0;y<m_resolutionY;y++)
//	{
//		for (x=0;x<m_resolutionX;x++)
//		{
//			int k = x+y*m_resolutionX;
//			disp = m_pDispImage[k];
//			if ( disp < 0xFF00 )
//			{
//				mrpt::slam::CLandmark		lm;
//				lm.type			= mrpt::slam::CLandmark::vlColor;
//
//				// convert the 16 bit disparity value to floating point x,y,z
//				// And set the pose PDF:
//				triclopsRCD16ToXYZ( m_triclops, y, x, disp, &lm.pose_mean_x, &lm.pose_mean_y, &lm.pose_mean_z ) ;
//
//				// If the pixel is inside the ROI -> compute the rest of parameters and add to the Observation
//				// Process ROI
//				(ROI.xMin == 0 && ROI.xMax == 0) ? x_min = -50.0f, x_max = 50.0f : x_min = ROI.xMin, x_max = ROI.xMax;
//				(ROI.yMin == 0 && ROI.zMax == 0) ? y_min = -50.0f, y_max = 50.0f : y_min = ROI.yMin, y_max = ROI.yMax;
//				(ROI.zMin == 0 && ROI.zMax == 0) ? z_min = -1.0f, z_max = 100.0f : z_min = ROI.zMin, z_max = ROI.zMax;
//
//				if ( ( lm.pose_mean_x < x_min ) || ( lm.pose_mean_x > x_max ) ||
//					 ( lm.pose_mean_y < y_min ) || ( lm.pose_mean_y > y_max ) ||
//					 ( lm.pose_mean_z < z_min ) || ( lm.pose_mean_z > z_max ) )
//					continue;
//
//				// Fill all required fields:
//				// --------------------------------------------
//				lm.pose_cov_11 = 1e-4f;
//				lm.pose_cov_22 = 1e-4f;
//				lm.pose_cov_33 = 1e-4f;
//				lm.pose_cov_12 = lm.pose_cov_13 = lm.pose_cov_23 = 0;
//
//				// The normal to the point (point-view direction):
//				// Normalized vector:
//				float		K = -1.0f / sqrt( square(lm.pose_mean_x)+square(lm.pose_mean_y)+square(lm.pose_mean_z) );
//				lm.normal_x = K * lm.pose_mean_x;
//				lm.normal_y = K * lm.pose_mean_y;
//				lm.normal_z = K * lm.pose_mean_z;
//
//				// Set the color:
//				lm.descriptor1.resize(3);
//				lm.descriptor1[0] = colorImage.red[k];
//				lm.descriptor1[1] = colorImage.green[k];
//				lm.descriptor1[2] = colorImage.blue[k];
//
//				// And add new landmark:
//				// --------------------------------------------
//				out_observation.landmarks.landmarks.push_back( &lm );
//			}
//
//		} // end for x
//
//	}	// end for y
//
//	return true;
//	MRPT_END
//#else
//	MRPT_UNUSED_PARAM(out_observation);
//	MRPT_UNUSED_PARAM(ROI);
//	return false;
//
//
//#endif
//}
// ****************************** END FAMD ***************************************************
///*-------------------------------------------------------------
//					getObservation
// -------------------------------------------------------------*/
//bool  CStereoGrabber_Bumblebee::getObservation(
//		mrpt::slam::CObservationVisualLandmarks &out_observation)
//{
//#if MRPT_HAS_BUMBLEBEE
//	unsigned int		x,y;
//	unsigned short		disp;
//	TriclopsColorImage  colorImage;
//	TriclopsInput       colorData;
//
//	// Set the timestep:
//	out_observation.timestamp = mrpt::system::getCurrentTime();
//
//	// Start with an empty landmarks map:
////	out_observation.landmarks.landmarks.reserve(30000);
//	out_observation.landmarks.landmarks.clear();
//
//
//	if (FLYCAPTURE_OK != digiclopsGrabImage( m_digiclops ) )
//		return false;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, STEREO_IMAGE, &m_triclopsInput ) )
//		return false;
//
//	// For getting the colors of the 3D points later:
//	digiclopsExtractTriclopsInput( m_digiclops, RIGHT_IMAGE, &colorData );
//
//	// Disparity will be stored in our custom buffer:
//	triclopsPreprocess( m_triclops, &m_triclopsInput );
//	triclopsStereo ( m_triclops );
//	triclopsRectifyColorImage( m_triclops, TriCam_REFERENCE,&colorData, &colorImage );
//
//	// Create a points-cloud representation:
//	// ---------------------------------------------
//
//	// Go across all the pixels:
//	// --------------------------------------------
//	for (y=0;y<m_resolutionY;y++)
//	{
//		for (x=0;x<m_resolutionX;x++)
//		{
//			int k = x+y*m_resolutionX;
//			disp = m_pDispImage[k];
//			if ( disp < 0xFF00 )
//			{
//				mrpt::slam::CLandmark		lm;
//				lm.type			= mrpt::slam::CLandmark::vlColor;
//
//				// convert the 16 bit disparity value to floating point x,y,z
//				// And set the pose PDF:
//				triclopsRCD16ToXYZ( m_triclops, y, x, disp, &lm.pose_mean_x, &lm.pose_mean_y, &lm.pose_mean_z ) ;
//
//				// Fill all required fields:
//				// --------------------------------------------
//				lm.pose_cov_11 = 1e-4f;
//				lm.pose_cov_22 = 1e-4f;
//				lm.pose_cov_33 = 1e-4f;
//				lm.pose_cov_12 = lm.pose_cov_13 = lm.pose_cov_23 = 0;
//
//				// The normal to the point (point-view direction):
//				// Normalized vector:
//				float		K = -1.0f / sqrt( square(lm.pose_mean_x)+square(lm.pose_mean_y)+square(lm.pose_mean_z) );
//				lm.normal_x = K * lm.pose_mean_x;
//				lm.normal_y = K * lm.pose_mean_y;
//				lm.normal_z = K * lm.pose_mean_z;
//
//				// Set the color:
//				lm.descriptor1.resize(3);
//				lm.descriptor1[0] = colorImage.red[k];
//				lm.descriptor1[1] = colorImage.green[k];
//				lm.descriptor1[2] = colorImage.blue[k];
//
//				// And add new landmark:
//				// --------------------------------------------
//				out_observation.landmarks.landmarks.push_back( &lm );
//			}
//
//		} // End for x
//
//	}	// end for y
//
//	return true;
//#else
//	MRPT_UNUSED_PARAM(out_observation);
//	return false;
//#endif
//}




///*-------------------------------------------------------------
//					getBothObservation
// -------------------------------------------------------------*/
//bool  CStereoGrabber_Bumblebee::getBothObservation(
//			mrpt::slam::CObservationVisualLandmarks	&out_observation,
//			mrpt::slam::CObservationStereoImages		&out_observationStereo )
//{
//#if MRPT_HAS_BUMBLEBEE
//	unsigned int		x,y;
//	unsigned short		disp;
//	TriclopsColorImage  colorImage;
//	TriclopsInput       colorData;
//
//	// Set the timestep:
//	out_observation.timestamp = out_observationStereo.timestamp = mrpt::system::getCurrentTime();
//
//	// Start with an empty landmarks map:
////	out_observation.landmarks.landmarks.reserve(30000);
//	out_observation.landmarks.landmarks.clear();
//
//
//	if (FLYCAPTURE_OK != digiclopsGrabImage( m_digiclops ) )
//		return false;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, STEREO_IMAGE, &m_triclopsInput ) )
//		return false;
//
//	// For getting the colors of the 3D points later:
//	digiclopsExtractTriclopsInput( m_digiclops, RIGHT_IMAGE, &colorData );
//
//	// Disparity will be stored in our custom buffer:
//	triclopsPreprocess( m_triclops, &m_triclopsInput );
//	triclopsStereo ( m_triclops );
//	triclopsRectifyColorImage( m_triclops, TriCam_REFERENCE,&colorData, &colorImage );
//
//	// Create a points-cloud representation:
//	// ---------------------------------------------
//
//	// Go across all the pixels:
//	// --------------------------------------------
//	for (y=0;y<m_resolutionY;y++)
//	{
//		for (x=0;x<m_resolutionX;x++)
//		{
//			int k = x+y*m_resolutionX;
//			disp = m_pDispImage[k];
//			if ( disp < 0xFF00 )
//			{
//				mrpt::slam::CLandmark		lm;
//				lm.type			= mrpt::slam::CLandmark::vlColor;
//
//				// convert the 16 bit disparity value to floating point x,y,z
//				// And set the pose PDF:
//				triclopsRCD16ToXYZ( m_triclops, y, x, disp, &lm.pose_mean_x, &lm.pose_mean_y, &lm.pose_mean_z ) ;
//
//				// Fill all required fields:
//				// --------------------------------------------
//				lm.pose_cov_11 = 1e-4f;
//				lm.pose_cov_22 = 1e-4f;
//				lm.pose_cov_33 = 1e-4f;
//				lm.pose_cov_12 = lm.pose_cov_13 = lm.pose_cov_23 = 0;
//
//				// The normal to the point (point-view direction):
//				// Normalized vector:
//				float		K = -1.0f / sqrt( square(lm.pose_mean_x)+square(lm.pose_mean_y)+square(lm.pose_mean_z) );
//				lm.normal_x = K * lm.pose_mean_x;
//				lm.normal_y = K * lm.pose_mean_y;
//				lm.normal_z = K * lm.pose_mean_z;
//
//				// Set the color:
//				lm.descriptor1.resize(3);
//				lm.descriptor1[0] = colorImage.red[k];
//				lm.descriptor1[1] = colorImage.green[k];
//				lm.descriptor1[2] = colorImage.blue[k];
//
//				// And add new landmark:
//				// --------------------------------------------
//				out_observation.landmarks.landmarks.push_back( &lm );
//			}
//
//		} // End for x
//
//	}	// end for y
//
//	// ------------------------
//
//	TriclopsColorImage  colorImage_L,colorImage_R;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, RIGHT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_RIGHT,&colorData, &colorImage_R );
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, LEFT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_LEFT,&colorData, &colorImage_L );
//
//	// Set the intrinsic parameters matrix:
//	out_observationStereo.intrinsicParams = vision::vision::defaultIntrinsicParamsMatrix(
//				0,				// 0 = Bumblebee camera
//				m_resolutionX,
//				m_resolutionY );
//
//	// BASE LINE:
//	out_observationStereo.rightCameraPose.setFromValues(0.119415f,0,0, 0,0,0);
//
//	// Load the images:
//	out_observationStereo.imageRight.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_R.rowinc,colorImage_R.red,colorImage_R.green,colorImage_R.blue);
//	out_observationStereo.imageLeft.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_L.rowinc,colorImage_L.red,colorImage_L.green,colorImage_L.blue);
//
//	return true;
//#else
//	MRPT_UNUSED_PARAM(out_observation);
//	MRPT_UNUSED_PARAM(out_observationStereo);
//	return false;
//#endif
//}

///*-------------------------------------------------------------
//					getBothObservation
// -------------------------------------------------------------*/
//bool  CStereoGrabber_Bumblebee::getBothObservation(
//			vector_float						&vX,
//			vector_float						&vY,
//			vector_float						&vZ,
//			mrpt::slam::CObservationStereoImages		&out_observationStereo )
//{
//#if MRPT_HAS_BUMBLEBEE
//	unsigned int		x,y;
//	unsigned short		disp;
//	TriclopsColorImage  colorImage;
//	TriclopsInput       colorData;
//
//	vX.clear(); vX.reserve( 30000 );
//	vY.clear(); vY.reserve( 30000 );
//	vZ.clear(); vZ.reserve( 30000 );
//
//	// Set the timestep:
//	out_observationStereo.timestamp = mrpt::system::getCurrentTime();
//
//	if (FLYCAPTURE_OK != digiclopsGrabImage( m_digiclops ) )
//		return false;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, STEREO_IMAGE, &m_triclopsInput ) )
//		return false;
//
//	// For getting the colors of the 3D points later:
//	digiclopsExtractTriclopsInput( m_digiclops, RIGHT_IMAGE, &colorData );
//
//	// Disparity will be stored in our custom buffer:
//	triclopsPreprocess( m_triclops, &m_triclopsInput );
//	triclopsStereo ( m_triclops );
//
//	triclopsRectifyColorImage( m_triclops, TriCam_REFERENCE, &colorData, &colorImage );
//
//	// Create a points-cloud representation:
//	// ---------------------------------------------
//
//	// Go across all the pixels:
//	// --------------------------------------------
//	for (y=0;y<m_resolutionY;y++)
//	{
//		for (x=0;x<m_resolutionX;x++)
//		{
//			int k = x+y*m_resolutionX;
//			disp = m_pDispImage[k];
//			if ( disp < 0xFF00 )
//			{
//				float x3D, y3D, z3D;
//
//				// convert the 16 bit disparity value to floating point x,y,z
//				// And set the pose PDF:
//				triclopsRCD16ToXYZ( m_triclops, y, x, disp, &x3D, &y3D, &z3D );
//
//				vX.push_back( x3D );
//				vY.push_back( y3D );
//				vZ.push_back( z3D );
//			}
//
//		} // End for x
//
//	}	// end for y
//
//	// ------------------------
//
//	TriclopsColorImage  colorImage_L,colorImage_R;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, RIGHT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_RIGHT,&colorData, &colorImage_R );
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, LEFT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_LEFT,&colorData, &colorImage_L );
//
//	// Set the intrinsic parameters matrix:
//	out_observationStereo.intrinsicParams = vision::vision::defaultIntrinsicParamsMatrix(
//				0,				// 0 = Bumblebee camera
//				m_resolutionX,
//				m_resolutionY );
//
//	// BASE LINE:
//	out_observationStereo.rightCameraPose.setFromValues(0.119415f,0,0, 0,0,0);
//
//	// Load the images:
//	out_observationStereo.imageRight.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_R.rowinc,colorImage_R.red,colorImage_R.green,colorImage_R.blue);
//	out_observationStereo.imageLeft.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_L.rowinc,colorImage_L.red,colorImage_L.green,colorImage_L.blue);
//
//	return true;
//#else
//	MRPT_UNUSED_PARAM(vX);
//	MRPT_UNUSED_PARAM(vY);
//	MRPT_UNUSED_PARAM(vZ);
//	MRPT_UNUSED_PARAM(out_observationStereo);
//	return false;
//#endif
//}

// ****************************** FAMD ***************************************************
///*-------------------------------------------------------------
//					getBothObservation with ROI
// -------------------------------------------------------------*/
//bool  CStereoGrabber_Bumblebee::getBothObservation(
//			TROI								ROI,
//			mrpt::slam::CObservationVisualLandmarks	&out_observation,
//			mrpt::slam::CObservationStereoImages		&out_observationStereo )
//{
//#if MRPT_HAS_BUMBLEBEE
//	unsigned int		x,y;
//	unsigned short		disp;
//
//	float				x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f, z_min = 0.0f, z_max = 0.0f;
//
//	TriclopsColorImage  colorImage;
//	TriclopsInput       colorData;
//
//	// Set the timestep:
//	out_observation.timestamp = out_observationStereo.timestamp = mrpt::system::getCurrentTime();
//
//	// Start with an empty landmarks map:
////	out_observation.landmarks.landmarks.reserve(30000);
//	out_observation.landmarks.landmarks.clear();
//
//
//	if (FLYCAPTURE_OK != digiclopsGrabImage( m_digiclops ) )
//		return false;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, STEREO_IMAGE, &m_triclopsInput ) )
//		return false;
//
//	// For getting the colors of the 3D points later:
//	digiclopsExtractTriclopsInput( m_digiclops, RIGHT_IMAGE, &colorData );
//
//	// Disparity will be stored in our custom buffer:
//	triclopsPreprocess( m_triclops, &m_triclopsInput );
//	triclopsStereo ( m_triclops );
//	triclopsRectifyColorImage( m_triclops, TriCam_REFERENCE,&colorData, &colorImage );
//
//	// Create a points-cloud representation:
//	// ---------------------------------------------
//
//	// Go across all the pixels:
//	// --------------------------------------------
//	for (y=0;y<m_resolutionY;y++)
//	{
//		for (x=0;x<m_resolutionX;x++)
//		{
//			int k = x+y*m_resolutionX;
//			disp = m_pDispImage[k];
//			if ( disp < 0xFF00 )
//			{
//				mrpt::slam::CLandmark		lm;
//				lm.type			= mrpt::slam::CLandmark::vlColor;
//
//				// convert the 16 bit disparity value to floating point x,y,z
//				// And set the pose PDF:
//				triclopsRCD16ToXYZ( m_triclops, y, x, disp, &lm.pose_mean_x, &lm.pose_mean_y, &lm.pose_mean_z ) ;
//
//				// If the pixel is inside the ROI -> compute the rest of parameters and add to the Observation
//				// Process ROI
//				(ROI.xMin == 0 && ROI.xMax == 0) ? x_min = -50.0f, x_max = 50.0f : x_min = ROI.xMin, x_max = ROI.xMax;
//				(ROI.yMin == 0 && ROI.zMax == 0) ? y_min = -50.0f, y_max = 50.0f : x_min = ROI.yMin, x_max = ROI.yMax;
//				(ROI.zMin == 0 && ROI.zMax == 0) ? z_min = -1.0f, z_max = 100.0f : x_min = ROI.zMin, x_max = ROI.zMax;
//
//				if ( ( lm.pose_mean_x < x_min ) || ( lm.pose_mean_x > x_max ) ||
//					 ( lm.pose_mean_y < y_min ) || ( lm.pose_mean_y > y_max ) ||
//					 ( lm.pose_mean_z < z_min ) || ( lm.pose_mean_z > z_max ) )
//					continue;
//
//				// Fill all required fields:
//				// --------------------------------------------
//				lm.pose_cov_11 = 1e-4f;
//				lm.pose_cov_22 = 1e-4f;
//				lm.pose_cov_33 = 1e-4f;
//				lm.pose_cov_12 = lm.pose_cov_13 = lm.pose_cov_23 = 0;
//
//				// The normal to the point (point-view direction):
//				// Normalized vector:
//				float		K = -1.0f / sqrt( square(lm.pose_mean_x)+square(lm.pose_mean_y)+square(lm.pose_mean_z) );
//				lm.normal_x = K * lm.pose_mean_x;
//				lm.normal_y = K * lm.pose_mean_y;
//				lm.normal_z = K * lm.pose_mean_z;
//
//				// Set the color:
//				lm.descriptor1.resize(3);
//				lm.descriptor1[0] = colorImage.red[k];
//				lm.descriptor1[1] = colorImage.green[k];
//				lm.descriptor1[2] = colorImage.blue[k];
//
//				// And add new landmark:
//				// --------------------------------------------
//				out_observation.landmarks.landmarks.push_back( &lm );
//			}
//
//		} // End for x
//
//	}	// end for y
//
//	// ------------------------
//
//	TriclopsColorImage  colorImage_L,colorImage_R;
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, RIGHT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_RIGHT,&colorData, &colorImage_R );
//
//	if (FLYCAPTURE_OK != digiclopsExtractTriclopsInput(m_digiclops, LEFT_IMAGE, &colorData ) )
//		return false;
//
//	// Rectify images:
//	triclopsRectifyColorImage( m_triclops, TriCam_LEFT,&colorData, &colorImage_L );
//
//	// Set the intrinsic parameters matrix:
//	out_observationStereo.intrinsicParams = vision::vision::defaultIntrinsicParamsMatrix(
//				0,				// 0 = Bumblebee camera
//				m_resolutionX,
//				m_resolutionY );
//
//	// BASE LINE:
//	out_observationStereo.rightCameraPose.setFromValues(0.119415f,0,0, 0,0,0);
//
//	// Load the images:
//	out_observationStereo.imageRight.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_R.rowinc,colorImage_R.red,colorImage_R.green,colorImage_R.blue);
//	out_observationStereo.imageLeft.loadFromMemoryBuffer(m_resolutionX,m_resolutionY, colorImage_L.rowinc,colorImage_L.red,colorImage_L.green,colorImage_L.blue);
//
//	return true;
//#else
//	MRPT_UNUSED_PARAM(out_observation);
//	MRPT_UNUSED_PARAM(out_observationStereo);
//	MRPT_UNUSED_PARAM(ROI);
//	return false;
//#endif
//}
// ****************************** END FAMD ***************************************************
