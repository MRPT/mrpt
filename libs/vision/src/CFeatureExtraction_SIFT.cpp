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

#include <mrpt/system/threads.h>
#include <mrpt/vision/CFeatureExtraction.h>

#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	#include "sift-hess/sift.h"
	#include "sift-hess/imgfeatures.h"
	#include "sift-hess/utils.h"
#endif


#include "do_opencv_includes.h"


// TODO: Remove, it's just for GetTempPathA
#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM>=0x211
		using namespace cv;
#	endif
#endif

/************************* Local Function Prototypes for Hess' SIFT *************************/
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
IplImage* create_init_img( IplImage*, int, double );
IplImage* convert_to_gray32( IplImage* );
IplImage*** build_gauss_pyr( IplImage*, int, int, double );
IplImage* downsample( IplImage* );
IplImage*** build_dog_pyr( IplImage***, int, int );
CvSeq* scale_space_extrema( IplImage***, int, int, double, int, CvMemStorage*);
int is_extremum( IplImage***, int, int, int, int );
struct feature* interp_extremum( IplImage***, int, int, int, int, int, double);
void interp_step( IplImage***, int, int, int, int, double*, double*, double* );
CvMat* deriv_3D( IplImage***, int, int, int, int );
CvMat* hessian_3D( IplImage***, int, int, int, int );
double interp_contr( IplImage***, int, int, int, int, double, double, double );
struct feature* new_feature( void );
int is_too_edge_like( IplImage*, int, int, int );
void calc_feature_scales( CvSeq*, double, int );
void adjust_for_img_dbl( CvSeq* );
void calc_feature_oris( CvSeq*, IplImage*** );
double* ori_hist( IplImage*, int, int, int, int, double );
int calc_grad_mag_ori( IplImage*, int, int, double*, double* );
void smooth_ori_hist( double*, int );
double dominant_ori( double*, int );
void add_good_ori_features( CvSeq*, double*, int, double, struct feature* );
struct feature* clone_feature( struct feature* );
void compute_descriptors( CvSeq*, IplImage***, int, int );
double*** descr_hist( IplImage*, int, int, double, double, int, int );
void interp_hist_entry( double***, double, double, double, double, int, int);
void hist_to_descr( double***, int, int, struct feature* );
void normalize_descr( struct feature* );
int feature_cmp( void*, void*, void* );
void release_descr_hist( double****, int );
void release_pyr( IplImage****, int, int );

#endif // MRPT_HAS_OPENCV


/************************************************************************************************
*								extractFeaturesSIFT  									        *
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesSIFT(
		const CImage			&img,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures,
		const TImageROI			&ROI) const
{
	bool usingROI = false;
	if( ROI.xMin != 0 || ROI.xMin != 0 || ROI.xMin != 0 || ROI.xMin != 0 )
		usingROI = true;	// A ROI has been defined

	// ROI can not be managed properly (yet) with these method, so we extract a subimage

	// use a smart pointer so we just copy the pointer if the image is grayscale, or we'll create a new one if it was RGB:
	CImagePtr auxImgPtr;
	if( usingROI )
	{
		ASSERT_( ROI.xMin >= 0 && ROI.xMin < ROI.xMax && ROI.xMax < img.getWidth() && ROI.yMin >= 0 && ROI.yMax < img.getHeight() && ROI.yMin < ROI.yMax );
		auxImgPtr = CImage::Create();
		img.extract_patch( *auxImgPtr, ROI.xMin, ROI.yMin, ROI.xMax-ROI.xMin+1, ROI.yMax-ROI.yMin+1 ); // Subimage in "auxImg"
	}
	else																	// There is no ROI -> use the whole image (a copy has to be made)
		auxImgPtr.setFromPointerDoNotFreeAtDtor(&img);  // Dont reserve mem. for a new image, reuse the current one and tell the smart pointer not to free the memory at the end.

	switch( options.SIFTOptions.implementation )
	{
// --------------------------------------------------------------------------------------
//		Binary in C# -> OPTIONAL: Feature position already computed
// --------------------------------------------------------------------------------------
	case CSBinary:
		{
#ifdef MRPT_OS_WINDOWS
			char			filImg[2000],filOut[2000],filFeat[2000];
			char			paramImg[2000];

			GetTempPathA(1000,filOut);	os::strcat(filOut,1000,"temp_out.txt");			// OUTPUT FILE
			GetTempPathA(1000,filImg);	os::strcat(filImg,1000,"temp_img.bmp");			// INPUT IMAGE (BMP) FOR BINARY IN (C#)

			bool onlyDesc = feats.size() > 0 ? true : false;

			if( onlyDesc )
			{
				GetTempPathA(1000,filFeat);	os::strcat(filFeat,1000,"temp_feats.txt");		// KEYPOINTS INPUT FILE
				CMatrix		listPoints(feats.size(),2);
				for (size_t i= 0;i<feats.size();i++)
				{
					listPoints(i,0) = feats[i]->x;
					listPoints(i,1) = feats[i]->y;
				}
				listPoints.saveToTextFile( filFeat, MATRIX_FORMAT_FIXED /*Float format*/ );
			} // end if
			// -------------------------------------------
			//		CALL TO "extractSIFT.exe"
			// -------------------------------------------
			auxImgPtr->saveToFile( filImg );

			// ------------------------------------
			// Version  with "CreateProcess":
			// ------------------------------------
			os::strcpy(paramImg,1000,"extractSIFT.exe -i"); os::strcat(paramImg,1000,filImg);
			os::strcat(paramImg,1000," -f"); os::strcat(paramImg,1000,filOut);
			os::strcat(paramImg,1000," -l"); os::strcat(paramImg,1000,filFeat);

			// ------------------------------------
			// Launch process
			// ------------------------------------
			bool ret = mrpt::system::launchProcess( paramImg );

			if( !ret )
				THROW_EXCEPTION( "[extractFeaturesSIFT] Could not launch external process... (extractSIFT.exe)" )

			// Process Results
			CFeatureList::iterator	itFeat = feats.begin();
			size_t	nFeats;

			CMatrix		aux;
			aux.loadFromTextFile( filOut );
			std::cout << "[computeSiftFeatures] " << aux.getRowCount() << " features." << std::endl;

			if( onlyDesc )
				nFeats = feats.size();
			else
			{
				nFeats = aux.getRowCount();
				feats.resize( nFeats );
			}

			for( size_t i = 0;
                 itFeat != feats.end();
				 i++, itFeat++)
			{
				(*itFeat)->type			= featSIFT;
				(*itFeat)->x			= usingROI ? aux(i,0) + ROI.xMin : aux(i,0);
				(*itFeat)->y			= usingROI ? aux(i,1) + ROI.yMin : aux(i,1);
				(*itFeat)->orientation	= aux(i,2);
				(*itFeat)->scale		= aux(i,3);
				(*itFeat)->ID			= init_ID + i;

				// The descriptor:
				aux.extractRow(i, (*itFeat)->descriptors.SIFT, 4);
			}
			remove(filImg);
			remove(filOut);
#else
    THROW_EXCEPTION("Unfortunately, this SIFT Implementation only runs in Windows OS, try Hess implementation");
#endif
			break;
		} // end case Binary in C#
	case VedaldiBinary:
		{
		// --------------------------------------------------------------------------------------
		//		Binary by Vedaldi: NOT IMPLEMENTED YET. Input in PGM format
		// --------------------------------------------------------------------------------------
#ifdef MRPT_OS_WINDOWS
		THROW_EXCEPTION("Usage of Vedaldi Binary not implemented yet, please, try another one");
#else
	    THROW_EXCEPTION("Unfortunately, this SIFT Implementation only runs in Windows OS, try Hess implementation");
#endif
		break;
		} // end case Binary by Vedaldi
// --------------------------------------------------------------------------------------
//		Binary by David Lowe
// --------------------------------------------------------------------------------------
	case LoweBinary:			// Binary by Lowe
		{
#ifdef MRPT_OS_WINDOWS
			char			filImg[2000],filOut[2000];
			char			paramImg[2000];

			feats.clear();
			CImage auxIMG = auxImgPtr->grayscale();

			GetTempPathA(1000,filOut);	os::strcat(filOut,1000,"temp_out.txt");			// OUTPUT FILE
			GetTempPathA(1000,filImg);	os::strcat(filImg,1000,"temp_img.pgm");			// INPUT IMAGE (PGM) FOR ORIGINAL BINARY BY LOWE

			bool valid = auxIMG.saveToFile( filImg );
			if(!valid)
				THROW_EXCEPTION( "An error occurred when saving input image into a .pgm file");

			// CONVERT TO UNCOMPRESSED RAW PGM (TODO: Solve in a better way this issue)
			os::strcpy( paramImg,1000, format( "cmd /C gmic.exe %s -o %s -quiet", filImg, filImg ).c_str() );

			bool ret = mrpt::system::launchProcess( paramImg );

			if(!ret)
				THROW_EXCEPTION("[extractFeaturesSIFT] Could not launch external process... (gmic.exe)");

			// ------------------------------------
			// Version  with "CreateProcess":
			// ------------------------------------
			os::strcpy(paramImg,1000,"cmd /C siftWin32.exe <"); os::strcat(paramImg,1000,filImg);
			os::strcat(paramImg,1000," >"); os::strcat(paramImg,1000,filOut);

			ret = mrpt::system::launchProcess( paramImg );

			if(!ret)
				THROW_EXCEPTION("[extractFeaturesSIFT] Could not launch external process... (siftWin32.exe)");

			// ------------------------------------
			// Process Results
			// ------------------------------------
			unsigned int dLen, nFeats;
			FILE *f = os::fopen( filOut, "rt");
			if(!f)
				THROW_EXCEPTION( "Error in extract SIFT with Lowe binary, output file not found!" );
			fscanf( f,"%u %u", &nFeats, &dLen);	// Number of feats and length of the descriptor

			for( size_t i = 0; i < nFeats; i++ )
			{
				CFeaturePtr feat	= CFeature::Create();

				feat->type			= featSIFT;			// Type
				feat->ID			= init_ID + i;		// Identifier

				// Position, orientation and scale
				// IMPORTANTE NOTE: Lowe format stores first the 'y' coordinate and then the 'x' one
				float fx,fy,fo,fs;
				fscanf( f, "%f %f %f %f", &fy, &fx, &fo, &fs );

				feat->x				= usingROI ? fx + ROI.xMin : fx;
				feat->y				= usingROI ? fy + ROI.yMin : fy;
				feat->orientation	= fo;
				feat->scale			= fs;

				// The descriptor
				feat->descriptors.SIFT.resize( dLen );
				unsigned int c;
				for(unsigned int k = 0; k < dLen; k++)
				{
					fscanf( f, "%u", &c );
					feat->descriptors.SIFT[k] = (unsigned char)c;
				}

				feats.push_back( feat );
			} // end for
			os::fclose( f );
			remove(filImg);
			remove(filOut);
#else
    THROW_EXCEPTION("Unfortunately, this SIFT Implementation only runs in Windows OS, try Hess implementation");
#endif
		break;
		} // end case Binary by Lowe
// --------------------------------------------------------------------------------------
//		Hess implementation
// --------------------------------------------------------------------------------------
		case Hess:			// Implementation by Robert Hess
		{
#if !MRPT_HAS_SIFT_HESS
			THROW_EXCEPTION("Method not available since MRPT has been compiled without Hess' SIFT library")
#elif MRPT_HAS_OPENCV	// OK, we have Hess' sift:
			IplImage* init_img;
			IplImage*** gauss_pyr, *** dog_pyr;
			CvMemStorage* storage;
			CvSeq* features;
			int octvs;

			/* check arguments */
			ASSERT_(auxImgPtr->getWidth() != 0 && auxImgPtr->getHeight() != 0);

			/* build scale space pyramid; smallest dimension of top level is ~4 pixels */
			IplImage* ipl_im = static_cast<IplImage *>( auxImgPtr->getAsIplImage() );

			init_img = create_init_img( ipl_im, SIFT_IMG_DBL, SIFT_SIGMA );
			octvs = log( (float)(MIN( init_img->width, init_img->height )) ) / log((float)2) - 2;
			gauss_pyr = build_gauss_pyr( init_img, octvs, SIFT_INTVLS, SIFT_SIGMA );
			dog_pyr = build_dog_pyr( gauss_pyr, octvs, SIFT_INTVLS );

			storage = cvCreateMemStorage( 0 );
			features = scale_space_extrema( dog_pyr, octvs, SIFT_INTVLS, SIFT_CONTR_THR,
				SIFT_CURV_THR, storage );
			calc_feature_scales( features, SIFT_SIGMA, SIFT_INTVLS );
			if( SIFT_IMG_DBL )
				adjust_for_img_dbl( features );
			calc_feature_oris( features, gauss_pyr );
			compute_descriptors( features, gauss_pyr, SIFT_DESCR_WIDTH, SIFT_DESCR_HIST_BINS );

			/* sort features by decreasing scale and move from CvSeq to array */
			cvSeqSort( features, (CvCmpFunc)feature_cmp, NULL );

			/* get only the desired features */
			if( nDesiredFeatures > 0 )
			{
				if( nDesiredFeatures < (unsigned int)features->total )
					cvSeqPopMulti( features, NULL, features->total - nDesiredFeatures );
				else
					cout << "[Warning] Detected less features than the requested " << features->total << " vs " << nDesiredFeatures << endl;
			} // end if

			/* convert CvSeq into a FeatureList */
			convertCvSeqInCFeatureList( features, feats, init_ID, ROI );

			// clear Hess-features
			cvClearSeq( features );
			cvReleaseMemStorage( &storage );
			cvReleaseImage( &init_img );
			release_pyr( &gauss_pyr, octvs, SIFT_INTVLS + 3 );
			release_pyr( &dog_pyr, octvs, SIFT_INTVLS + 2 );
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
			break;
		} // end case Hess
//***********************************************************************************************
// USING OPENCV
//***********************************************************************************************
		case OpenCV:
		{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM >= 0x211
			SiftFeatureDetector SIFTDetector(
				SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
				SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD() );

			SiftDescriptorExtractor SIFTDescriptor;

			vector<KeyPoint> cv_feats;									// The OpenCV output feature list

			IplImage* iplimg, *cGrey;
			iplimg = (IplImage*)img.getAsIplImage();

			if( iplimg->nChannels == 1 )
				cGrey = iplimg;											// Input image is already 'grayscale'
			else
			{
				cGrey = cvCreateImage( cvGetSize( iplimg ), 8, 1);
				cvCvtColor( iplimg, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
			}
			CTicTac tt;

			Mat theImg = cvarrToMat( cGrey );
			tt.Tic();
			SIFTDetector.detect( theImg, cv_feats );
			cout << "Detect: " << tt.Tac()*1000.0f << endl;

			Mat desc;
			tt.Tic();
			SIFTDescriptor.compute( theImg, cv_feats, desc );
			cout << "Describe: " << tt.Tac()*1000.0f << endl;

			//fromOpenCVToMRPT( theImg, cv_feats, desc, nDesiredFeatures, outList );
			const size_t	N			= cv_feats.size();
			unsigned int	nMax		= nDesiredFeatures != 0 && N > nDesiredFeatures ? nDesiredFeatures : N;
			const int 		offset		= (int)this->options.patchSize/2 + 1;
			const size_t	size_2		= options.patchSize/2;
			const size_t 	imgH		= img.getHeight();
			const size_t 	imgW		= img.getWidth();
			unsigned int	i			= 0;
			unsigned int	cont		= 0;
			TFeatureID		nextID		= init_ID;
			feats.clear();
			while( cont != nMax && i != N )
			{
				const int xBorderInf = (int)floor( cv_feats[i].pt.x - size_2 );
				const int xBorderSup = (int)floor( cv_feats[i].pt.x + size_2 );
				const int yBorderInf = (int)floor( cv_feats[i].pt.y - size_2 );
				const int yBorderSup = (int)floor( cv_feats[i].pt.y + size_2 );

				if( options.patchSize==0 || ( (xBorderSup < (int)imgW) && (xBorderInf > 0) && (yBorderSup < (int)imgH) && (yBorderInf > 0) ) )
				{
					CFeaturePtr ft		= CFeature::Create();
					ft->type			= featSIFT;
					ft->ID				= nextID++;
					ft->x				= cv_feats[i].pt.x;
					ft->y				= cv_feats[i].pt.y;
					ft->response		= cv_feats[i].response;
					ft->orientation		= cv_feats[i].angle;
					ft->scale			= cv_feats[i].size;
					ft->patchSize		= options.patchSize;														// The size of the feature patch
					ft->descriptors.SIFT.resize( 128 );
					memcpy( &(ft->descriptors.SIFT[0]), &desc.data[128*i], 128*sizeof(ft->descriptors.SIFT[0]) );	// The descriptor

					if( options.patchSize > 0 )
					{
						img.extract_patch(
							ft->patch,
							round( ft->x ) - offset,
							round( ft->y ) - offset,
							options.patchSize,
							options.patchSize );						// Image patch surronding the feature
					}
					feats.push_back( ft );
					++cont;
				}
				++i;
			}
			feats.resize( cont );
			cvReleaseImage( &cGrey );
#else
	THROW_EXCEPTION("This method requires OpenCV >= 2.1.1")
#endif
			break;
		} // end case OpenCV
		return;
		default:{break;} // end default
	} // end switch
} // end extractFeaturesSIFT


/************************************************************************************************
*								computeSiftDescriptors 									        *
************************************************************************************************/
// Compute SIFT descriptors on a set of already localized points
void  CFeatureExtraction::internal_computeSiftDescriptors( const CImage	&in_img,
												  CFeatureList		&in_features) const
{

	ASSERT_( in_features.size() > 0 );
	switch( options.SIFTOptions.implementation )
	{
		case CSBinary:
		{
#ifdef MRPT_OS_WINDOWS
			char	filImg[2000],filOut[2000],filFeat[2000];
			char	paramImg[2000];

			CFeatureList::iterator		feat;

			// Save to temporary file
			GetTempPathA(1000,filImg);	os::strcat( filImg, 1000, "temp_img.bmp" );
			GetTempPathA(1000,filOut);	os::strcat( filOut,1000, "temp_feats.txt" );
			GetTempPathA(1000,filFeat);	os::strcat( filFeat, 1000, "temp_KLT_feats.txt" );

			// Fill the input file
			FILE *fout = os::fopen( filFeat,"wt");
			for(feat = in_features.begin(); feat != in_features.end(); feat++)
				os::fprintf( fout, "%.6f %.6f\n", (*feat)->x, (*feat)->y );

			os::fclose(fout);

			// -------------------------------------------
			//		CALL TO "extractSIFT.exe"
			// -------------------------------------------
			in_img.saveToFile( filImg );

			// Version  with "CreateProcess":
			// ------------------------------------
			os::strcpy(paramImg,1000,"extractSIFT.exe -i"); os::strcat(paramImg,1000,filImg);
			os::strcat(paramImg,1000," -f"); os::strcat(paramImg,1000,filOut);
			os::strcat(paramImg,1000," -l"); os::strcat(paramImg,1000,filFeat);

			bool ret = mrpt::system::launchProcess( paramImg );

			if(!ret)
				THROW_EXCEPTION("[extractFeaturesSIFT] Could not launch external process... (extractSIFT.exe)");

			MRPT_START
			// Load the results:
			CMatrix		aux;
			aux.loadFromTextFile( filOut );
			size_t nRows = aux.getRowCount();

			printf("[computeSiftFeatures1] %u features\n", nRows);

			unsigned int i;
			float lx,ly;
			lx = 0.0;
			ly = 0.0;
			feat = in_features.begin();

			for(i = 0; i < nRows; i++)
			{
				if( aux(i,0) != lx  || aux(i,1) != ly ) // Only one descriptor for feature
				{
					(*feat)->type			= featSIFT;
					(*feat)->orientation	= aux(i,2);
					(*feat)->scale			= aux(i,3);

					// The descriptor:
					aux.extractRow(i, (*feat)->descriptors.SIFT, 4);

					lx = aux(i,0);
					ly = aux(i,1);

					feat++;
				} // end if
			} // end for
			MRPT_END

			break;
#else
    THROW_EXCEPTION("TO DO @ linux OS!");
#endif
		} // end case CSBinary
		case Hess: // Implementation by Hess
		{
#if !MRPT_HAS_SIFT_HESS
			THROW_EXCEPTION("Method not available since MRPT has been compiled without Hess' SIFT library")
#elif MRPT_HAS_OPENCV	// OK, we have Hess' sift:
			IplImage* init_img;
			IplImage*** gauss_pyr, *** dog_pyr;
			CvMemStorage* storage;
			CvSeq* features;
			int octvs; //, n = 0;

			/* check arguments */
			ASSERT_(in_img.getWidth() != 0 && in_img.getHeight() != 0);

			/* build scale space pyramid; smallest dimension of top level is ~4 pixels */
			IplImage* ipl_im = static_cast<IplImage *>( in_img.getAsIplImage() );

			init_img = create_init_img( ipl_im, SIFT_IMG_DBL, SIFT_SIGMA );
			octvs = log( (float)(MIN( init_img->width, init_img->height )) ) / log((float)2) - 2;
			gauss_pyr = build_gauss_pyr( init_img, octvs, SIFT_INTVLS, SIFT_SIGMA );
			dog_pyr = build_dog_pyr( gauss_pyr, octvs, SIFT_INTVLS );

			storage = cvCreateMemStorage( 0 );
			features = static_cast<CvSeq*>(my_scale_space_extrema( in_features, dog_pyr, octvs, SIFT_INTVLS, SIFT_CONTR_THR,
				SIFT_CURV_THR, storage ));
			calc_feature_scales( features, SIFT_SIGMA, SIFT_INTVLS );
			if( SIFT_IMG_DBL )
				my_adjust_for_img_dbl( features );
			calc_feature_oris( features, gauss_pyr );
			compute_descriptors( features, gauss_pyr, SIFT_DESCR_WIDTH, SIFT_DESCR_HIST_BINS );

			// merge Hess-features and MRPT-features
			insertCvSeqInCFeatureList( features, in_features );

			// clear Hess-features
			cvClearSeq( features );

			// Free memory
			cvReleaseMemStorage( &storage );
			cvReleaseImage( &init_img );
			release_pyr( &gauss_pyr, octvs, SIFT_INTVLS + 3 );
			release_pyr( &dog_pyr, octvs, SIFT_INTVLS + 2 );

			/* sort features by decreasing scale and move from CvSeq to array */
			//cvSeqSort( features, (CvCmpFunc)feature_cmp, NULL );
			//n = features->total;

			//struct feature** feat;
			//*feat = (feature*)calloc( n, sizeof(struct feature) );
			//*feat = (feature*)cvCvtSeqToArray( features, *feat, CV_WHOLE_SEQ );
			//for( i = 0; i < n; i++ )
			//{
			//	free( (*feat)[i].feature_data );
			//	(*feat)[i].feature_data = NULL;
			//}

			//// The number of features must be the same
			//cout << "NUMBER OF INPUT FEATURES: " << in_features.size() << endl;
			//cout << "NUMBER OF OUTPUT FEATURES: " << n << endl;

			//// Transform to CFeatureList:
			//CFeatureList::iterator	itFeat;
			//int						m;
			//for( itFeat = in_features.begin(), m = 0; itFeat != in_features.end(); itFeat++, m++)
			//{
			//	(*itFeat)->type = featSIFT;
			//	(*itFeat)->x = (*feat)[m].x;
			//	(*itFeat)->y = (*feat)[m].y;
			//	(*itFeat)->orientation = (*feat)[m].ori;
			//	(*itFeat)->scale = (*feat)[m].scl;
			//	for( unsigned int k = 0; k < FEATURE_MAX_D; k++)
			//		(*itFeat)->descriptors.SIFT[k] = (*feat)[m].descr[k];
			//	(*itFeat)->hasDescriptor = true;
			//} // end for features
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
		break;
		} // end case Hess
		default:
		{
			cout << "SIFT Extraction method not supported for features with already known image coordinates" << endl;
			break;
		}
	} // end switch

} // end computeSiftDescriptors


// ------------------------------------------------------------------------------------
//								convertCvSeqInCFeatureList
// ------------------------------------------------------------------------------------
void CFeatureExtraction::convertCvSeqInCFeatureList( void* features_, CFeatureList &list, unsigned int init_ID, const TImageROI &ROI ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	CvSeq* features = reinterpret_cast<CvSeq*>( features_ );

	// Is there a defined ROI?
	bool usingROI = false;
	if( ROI.xMin != 0 || ROI.xMax != 0 || ROI.yMin != 0 || ROI.yMax != 0 )
		usingROI = true;

	int n = features->total;
	ASSERT_(n > 0);

	list.clear();
	struct feature* thisFeat;
	for( int k = 0; k < n; k++ )
	{
		thisFeat			= (feature*)cvGetSeqElem( features, k );
		CFeaturePtr feat	= CFeature::Create();
		feat->ID			= (TFeatureID)(k + init_ID);
		feat->x				= usingROI ? thisFeat->x + ROI.xMin : thisFeat->x;
		feat->y				= usingROI ? thisFeat->y + ROI.yMin : thisFeat->y;
		feat->type			= featSIFT;
		feat->orientation	= thisFeat->ori;
		feat->scale			= thisFeat->scl;
		feat->descriptors.SIFT.resize( thisFeat->d );
		for( int i = 0; i < thisFeat->d; i++ )
			feat->descriptors.SIFT[i] = (unsigned char)thisFeat->descr[i];

		list.push_back(feat);
	} // end for
#else
	THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
}
// ------------------------------------------------------------------------------------
//								insertCvSeqInCFeatureList
// ------------------------------------------------------------------------------------
void CFeatureExtraction::insertCvSeqInCFeatureList( void* features_, CFeatureList &list, unsigned int init_ID ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	CvSeq* features = reinterpret_cast<CvSeq*>( features_ );

	int n = features->total;
	ASSERT_(n > 0);

	CFeatureList::iterator itFeat;
	struct feature* thisFeat;
	int k;
	for( itFeat = list.begin(), k = 0; itFeat != list.end() && k < n; k++ )
	{
		thisFeat = (feature*)cvGetSeqElem( features, k );
		if( (*itFeat)->x == thisFeat->x && (*itFeat)->y == thisFeat->y )
		{
			(*itFeat)->ID = (TFeatureID)(k + init_ID);
			(*itFeat)->orientation = thisFeat->ori;
			(*itFeat)->scale = thisFeat->scl;
			(*itFeat)->descriptors.SIFT.resize( thisFeat->d );
			for( int i = 0; i < thisFeat->d; i++ )
				(*itFeat)->descriptors.SIFT[i] = (unsigned char)thisFeat->descr[i];

			itFeat++;
		} // end if
	} // end for
#else
	THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
}

/*
Halves feature coordinates and scale in case the input image was doubled
prior to scale space construction.

@param features array of features
*/
void CFeatureExtraction::my_adjust_for_img_dbl( void* features_ ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	CvSeq* features = reinterpret_cast<CvSeq*>( features_ );
	struct feature* feat;
	int i, n;

	n = features->total;
	for( i = 0; i < n; i++ )
	{
		feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		feat->scl /= 2.0;
	}
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
}

// ------------------------------------------------------------------------------------
//								my_scale_space_extrema
// ------------------------------------------------------------------------------------
void* CFeatureExtraction::my_scale_space_extrema(
	CFeatureList &featList, void* dog_pyr_,
	int octvs, int intvls, double contr_thr, int curv_thr,
	void* storage_ ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	CvMemStorage*	storage = reinterpret_cast<CvMemStorage*>( storage_ );
	IplImage***		dog_pyr = reinterpret_cast<IplImage***>( dog_pyr_ );

	CvSeq*	features;
//	double	prelim_contr_thr = 0.5 * contr_thr / intvls;
	struct	feature* feat;
	struct	detection_data* ddata;
	int		o, i;

	CFeatureList::iterator itFeats;

	features = cvCreateSeq( 0, sizeof(CvSeq), sizeof(struct feature), storage );
	for(itFeats = featList.begin(); itFeats != featList.end(); itFeats++)
	{
		//unsigned int	nMax, nMin;
		double	gExt	= -1e5;
		int		go		= 0, gi = 1;
		float	gr		= 0.0, gc = 0.0;

		if( (*itFeats)->y < 0 || (*itFeats)->x < 0 )
			continue;										// Do not process this feature

		// Find the best octave and interval where the feature is the 'best' extrema
		for( o = 0; o < octvs; o++ )
		{
			float r = (*itFeats)->y/pow(2.0,o);							// Dog pyramid coordinates
			float c = (*itFeats)->x/pow(2.0,o);

			for( i = 1; i <= intvls; i++ )
			{
				double s	= SIFT_SIGMA * pow(2.0, o+i/intvls);
				double val	= s*getLaplacianValue( dog_pyr, o, i, r, c );
				if( val > gExt )
				{
					gExt	= val;
					go		= o;
					gi		= i;
					gr		= r;
					gc		= c;
				}
				//getTimesExtrema( dog_pyr, o, i, r, c, nMax,	nMin );		// Get number of times the point is bigger or lower than the surroundings
				//if( nMax > gExt || nMin > gExt )
				//{
				//	gExt	= max(nMax,nMin);
				//	go		= o;
				//	gi		= i;
				//	gr		= r;
				//	gc		= c;
				//} // end if
			} // end for intervals
		} // end for octaves

		feat = new_feature();						// Create the new feature
		ddata = feat_detection_data( feat );		// Feature data
		feat->img_pt.x = feat->x = (*itFeats)->x;	// Feature Coordinates in the original image
		feat->img_pt.y = feat->y = (*itFeats)->y;
		ddata->r = (int)gr;							// Feature Coordinates in the proper octave
		ddata->c = (int)gc;
		ddata->octv = go;							// Octave
		ddata->intvl = gi;							// Interval
		ddata->subintvl = 0.0;						// Subinterval (not used)

		cvSeqPush( features, feat );				// Insert into the feature sequence
	} // end for features in the list

	return features;
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
}

/************************************************************************************************
*								getLaplaceValue     									        *
************************************************************************************************/
double CFeatureExtraction::getLaplacianValue( void* dog_pyr_, int octv, int intvl, float row, float col ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	IplImage*** dog_pyr = reinterpret_cast<IplImage***>( dog_pyr_ );
	double value = 0.0;
	int j, k;

	for( j = -1; j <= 1; j++ )
		for( k = -1; k <= 1; k++ )
		{
			// Apply convolution mask for computing the Laplacian:
			// -1 -1 -1
			// -1 +8 -1
			// -1 -1 -1
			if( k == 0 && j == 0 )
				value += 8*pixval32f( dog_pyr[octv][intvl], (int)(row + j), (int)(col + k) );
			else
				value -= pixval32f( dog_pyr[octv][intvl], (int)(row + j), (int)(col + k) );
		}
	return value;
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
} // end getLaplacianValue

/************************************************************************************************
*								getTimesExtrema     									        *
************************************************************************************************/
//void CFeatureExtraction::getTimesExtrema( IplImage*** dog_pyr, int octv, int intvl, float row, float col, unsigned int &nMin, unsigned int &nMax )
void CFeatureExtraction::getTimesExtrema( void* dog_pyr_, int octv, int intvl, float row, float col, unsigned int &nMin, unsigned int &nMax ) const
{
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	IplImage*** dog_pyr = reinterpret_cast<IplImage***>( dog_pyr_ );

	float val = pixval32f( dog_pyr[octv][intvl], (int)row, (int)col );
	int i, j, k;

	nMax = 0;
	nMin = 0;
	/* check for extrema */
	//cout << "VAL: " << pixval32f( dog_pyr[octv][intvl], row, col );
	for( i = -1; i <= 1; i++ )
		for( j = -1; j <= 1; j++ )
			for( k = -1; k <= 1; k++ )
			{
				//cout << pixval32f( dog_pyr[octv][intvl+i], row + j, col + k ) << endl;
				if( val > pixval32f( dog_pyr[octv][intvl+i], (int)(row + j), (int)(col + k) ) )
					nMax++;
				if( val < pixval32f( dog_pyr[octv][intvl+i], (int)(row + j), (int)(col + k) ) )
					nMin++;
			}
#else
			THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
#endif //MRPT_HAS_OPENCV
} // end getTimesExtrema



#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS

/*
Converts an image to 8-bit grayscale and Gaussian-smooths it.  The image is
optionally doubled in size prior to smoothing.

@param img input image
@param img_dbl if true, image is doubled in size prior to smoothing
@param sigma total std of Gaussian smoothing
*/
IplImage* create_init_img( IplImage* img, int img_dbl, double sigma )
{
	IplImage* gray, * dbl;
	float sig_diff;

	gray = convert_to_gray32( img );
	if( img_dbl )
	{
		sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4 );
		dbl = cvCreateImage( cvSize( img->width*2, img->height*2 ),
			IPL_DEPTH_32F, 1 );
		cvResize( gray, dbl, CV_INTER_CUBIC );
		cvSmooth( dbl, dbl, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
		cvReleaseImage( &gray );
		return dbl;
	}
	else
	{
		sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA );
		cvSmooth( gray, gray, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
		return gray;
	}
}



/*
Converts an image to 32-bit grayscale

@param img a 3-channel 8-bit color (BGR) or 8-bit gray image

@return Returns a 32-bit grayscale image
*/
IplImage* convert_to_gray32( IplImage* img )
{
	IplImage* gray8, * gray32;

	gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
	gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

	if( img->nChannels == 1 )
		gray8 = (IplImage*)cvClone( img );
	else
		cvCvtColor( img, gray8, CV_BGR2GRAY );
	cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );

	cvReleaseImage( &gray8 );
	return gray32;
}



/*
Builds Gaussian scale space pyramid from an image

@param base base image of the pyramid
@param octvs number of octaves of scale space
@param intvls number of intervals per octave
@param sigma amount of Gaussian smoothing per octave

@return Returns a Gaussian scale space pyramid as an octvs x (intvls + 3) array
*/
IplImage*** build_gauss_pyr( IplImage* base, int octvs,
							int intvls, double sigma )
{
	IplImage*** gauss_pyr;
	double* sig = (double*)calloc( intvls + 3, sizeof(double));
	double sig_total, sig_prev, k;
	int i, o;

	gauss_pyr = (IplImage***)calloc( octvs, sizeof( IplImage** ) );
	for( i = 0; i < octvs; i++ )
		gauss_pyr[i] = (IplImage**)calloc( intvls + 3, sizeof( IplImage* ) );

	/*
		precompute Gaussian sigmas using the following formula:

		\sigma_{total}^2 = \sigma_{i}^2 + \sigma_{i-1}^2
	*/
	sig[0] = sigma;
	k = pow( 2.0, 1.0 / intvls );
	for( i = 1; i < intvls + 3; i++ )
	{
		sig_prev = pow( k, i - 1 ) * sigma;
		sig_total = sig_prev * k;
		sig[i] = sqrt( sig_total * sig_total - sig_prev * sig_prev );
	}

	for( o = 0; o < octvs; o++ )
		for( i = 0; i < intvls + 3; i++ )
		{
			if( o == 0  &&  i == 0 )
				gauss_pyr[o][i] = cvCloneImage(base);

			/* base of new octvave is halved image from end of previous octave */
			else if( i == 0 )
				gauss_pyr[o][i] = downsample( gauss_pyr[o-1][intvls] );

			/* blur the current octave's last image to create the next one */
			else
			{
				gauss_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i-1]),
					IPL_DEPTH_32F, 1 );
				cvSmooth( gauss_pyr[o][i-1], gauss_pyr[o][i],
					CV_GAUSSIAN, 0, 0, sig[i], sig[i] );
			}
		}

	free( sig );
	return gauss_pyr;
}



/*
Downsamples an image to a quarter of its size (half in each dimension)
using nearest-neighbor interpolation

@param img an image

@return Returns an image whose dimensions are half those of img
*/
IplImage* downsample( IplImage* img )
{
	IplImage* smaller = cvCreateImage( cvSize(img->width / 2, img->height / 2),
		img->depth, img->nChannels );
	cvResize( img, smaller, CV_INTER_NN );

	return smaller;
}



/*
Builds a difference of Gaussians scale space pyramid by subtracting adjacent
intervals of a Gaussian pyramid

@param gauss_pyr Gaussian scale-space pyramid
@param octvs number of octaves of scale space
@param intvls number of intervals per octave

@return Returns a difference of Gaussians scale space pyramid as an
	octvs x (intvls + 2) array
*/
IplImage*** build_dog_pyr( IplImage*** gauss_pyr, int octvs, int intvls )
{
	IplImage*** dog_pyr;
	int i, o;

	dog_pyr = (IplImage***)calloc( octvs, sizeof( IplImage** ) );
	for( i = 0; i < octvs; i++ )
		dog_pyr[i] = (IplImage**)calloc( intvls + 2, sizeof(IplImage*) );

	for( o = 0; o < octvs; o++ )
		for( i = 0; i < intvls + 2; i++ )
		{
			dog_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i]),
				IPL_DEPTH_32F, 1 );
			cvSub( gauss_pyr[o][i+1], gauss_pyr[o][i], dog_pyr[o][i], NULL );
		}

	return dog_pyr;
}



/*
Detects features at extrema in DoG scale space.  Bad features are discarded
based on contrast and ratio of principal curvatures.

@param dog_pyr DoG scale space pyramid
@param octvs octaves of scale space represented by dog_pyr
@param intvls intervals per octave
@param contr_thr low threshold on feature contrast
@param curv_thr high threshold on feature ratio of principal curvatures
@param storage memory storage in which to store detected features

@return Returns an array of detected features whose scales, orientations,
	and descriptors are yet to be determined.
*/
CvSeq* scale_space_extrema( IplImage*** dog_pyr, int octvs, int intvls,
						   double contr_thr, int curv_thr,
						   CvMemStorage* storage )
{
	CvSeq* features;
	double prelim_contr_thr = 0.5 * contr_thr / intvls;
	struct feature* feat;
	struct detection_data* ddata;
	int o, i, r, c;

	features = cvCreateSeq( 0, sizeof(CvSeq), sizeof(struct feature), storage );
	for( o = 0; o < octvs; o++ )
		for( i = 1; i <= intvls; i++ )
			for(r = SIFT_IMG_BORDER; r < dog_pyr[o][0]->height-SIFT_IMG_BORDER; r++)
				for(c = SIFT_IMG_BORDER; c < dog_pyr[o][0]->width-SIFT_IMG_BORDER; c++)
					/* perform preliminary check on contrast */
					if( ABS( pixval32f( dog_pyr[o][i], r, c ) ) > prelim_contr_thr )
						if( is_extremum( dog_pyr, o, i, r, c ) )
						{
							feat = interp_extremum(dog_pyr, o, i, r, c, intvls, contr_thr);
							if( feat )
							{
								ddata = feat_detection_data( feat );
								if( ! is_too_edge_like( dog_pyr[ddata->octv][ddata->intvl],
									ddata->r, ddata->c, curv_thr ) )
								{
									cvSeqPush( features, feat );
								}
								else
									free( ddata );
								free( feat );
							}
						}

	return features;
}



/*
Determines whether a pixel is a scale-space extremum by comparing it to it's
3x3x3 pixel neighborhood.

@param dog_pyr DoG scale space pyramid
@param octv pixel's scale space octave
@param intvl pixel's within-octave interval
@param r pixel's image row
@param c pixel's image col

@return Returns 1 if the specified pixel is an extremum (max or min) among
	it's 3x3x3 pixel neighborhood.
*/
int is_extremum( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	float val = pixval32f( dog_pyr[octv][intvl], r, c );
	int i, j, k;

	/* check for maximum */
	if( val > 0 )
	{
		for( i = -1; i <= 1; i++ )
			for( j = -1; j <= 1; j++ )
				for( k = -1; k <= 1; k++ )
					if( val < pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
						return 0;
	}

	/* check for minimum */
	else
	{
		for( i = -1; i <= 1; i++ )
			for( j = -1; j <= 1; j++ )
				for( k = -1; k <= 1; k++ )
					if( val > pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
						return 0;
	}

	return 1;
}



/*
Interpolates a scale-space extremum's location and scale to subpixel
accuracy to form an image feature.  Rejects features with low contrast.
Based on Section 4 of Lowe's paper.

@param dog_pyr DoG scale space pyramid
@param octv feature's octave of scale space
@param intvl feature's within-octave interval
@param r feature's image row
@param c feature's image column
@param intvls total intervals per octave
@param contr_thr threshold on feature contrast

@return Returns the feature resulting from interpolation of the given
	parameters or NULL if the given location could not be interpolated or
	if contrast at the interpolated loation was too low.  If a feature is
	returned, its scale, orientation, and descriptor are yet to be determined.
*/
struct feature* interp_extremum( IplImage*** dog_pyr, int octv, int intvl,
								int r, int c, int intvls, double contr_thr )
{
	struct feature* feat;
	struct detection_data* ddata;
	double xi, xr, xc, contr;
	int i = 0;

	while( i < SIFT_MAX_INTERP_STEPS )
	{
		interp_step( dog_pyr, octv, intvl, r, c, &xi, &xr, &xc );
		if( ABS( xi ) < 0.5  &&  ABS( xr ) < 0.5  &&  ABS( xc ) < 0.5 )
			break;

		c += cvRound( xc );
		r += cvRound( xr );
		intvl += cvRound( xi );

		if( intvl < 1  ||
			intvl > intvls  ||
			c < SIFT_IMG_BORDER  ||
			r < SIFT_IMG_BORDER  ||
			c >= dog_pyr[octv][0]->width - SIFT_IMG_BORDER  ||
			r >= dog_pyr[octv][0]->height - SIFT_IMG_BORDER )
		{
			return NULL;
		}

		i++;
	}

	/* ensure convergence of interpolation */
	if( i >= SIFT_MAX_INTERP_STEPS )
		return NULL;

	contr = interp_contr( dog_pyr, octv, intvl, r, c, xi, xr, xc );
	if( ABS( contr ) < contr_thr / intvls )
		return NULL;

	feat = new_feature();
	ddata = feat_detection_data( feat );
	feat->img_pt.x = feat->x = ( c + xc ) * pow( 2.0, octv );
	feat->img_pt.y = feat->y = ( r + xr ) * pow( 2.0, octv );
	ddata->r = r;
	ddata->c = c;
	ddata->octv = octv;
	ddata->intvl = intvl;
	ddata->subintvl = xi;

	return feat;
}



/*
Performs one step of extremum interpolation.  Based on Eqn. (3) in Lowe's
paper.

@param dog_pyr difference of Gaussians scale space pyramid
@param octv octave of scale space
@param intvl interval being interpolated
@param r row being interpolated
@param c column being interpolated
@param xi output as interpolated subpixel increment to interval
@param xr output as interpolated subpixel increment to row
@param xc output as interpolated subpixel increment to col
*/

void interp_step( IplImage*** dog_pyr, int octv, int intvl, int r, int c,
				 double* xi, double* xr, double* xc )
{
	CvMat* dD, * H, * H_inv, X;
	double x[3] = { 0 };

	dD = deriv_3D( dog_pyr, octv, intvl, r, c );
	H = hessian_3D( dog_pyr, octv, intvl, r, c );
	H_inv = cvCreateMat( 3, 3, CV_64FC1 );
	cvInvert( H, H_inv, CV_SVD );
	cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
	cvGEMM( H_inv, dD, -1, NULL, 0, &X, 0 );

	cvReleaseMat( &dD );
	cvReleaseMat( &H );
	cvReleaseMat( &H_inv );

	*xi = x[2];
	*xr = x[1];
	*xc = x[0];
}



/*
Computes the partial derivatives in x, y, and scale of a pixel in the DoG
scale space pyramid.

@param dog_pyr DoG scale space pyramid
@param octv pixel's octave in dog_pyr
@param intvl pixel's interval in octv
@param r pixel's image row
@param c pixel's image col

@return Returns the vector of partial derivatives for pixel I
	{ dI/dx, dI/dy, dI/ds }^T as a CvMat*
*/
CvMat* deriv_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	CvMat* dI;
	double dx, dy, ds;

	dx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) -
		pixval32f( dog_pyr[octv][intvl], r, c-1 ) ) / 2.0;
	dy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) -
		pixval32f( dog_pyr[octv][intvl], r-1, c ) ) / 2.0;
	ds = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) -
		pixval32f( dog_pyr[octv][intvl-1], r, c ) ) / 2.0;

	dI = cvCreateMat( 3, 1, CV_64FC1 );
	cvmSet( dI, 0, 0, dx );
	cvmSet( dI, 1, 0, dy );
	cvmSet( dI, 2, 0, ds );

	return dI;
}



/*
Computes the 3D Hessian matrix for a pixel in the DoG scale space pyramid.

@param dog_pyr DoG scale space pyramid
@param octv pixel's octave in dog_pyr
@param intvl pixel's interval in octv
@param r pixel's image row
@param c pixel's image col

@return Returns the Hessian matrix (below) for pixel I as a CvMat*

	/ Ixx  Ixy  Ixs \ <BR>
	| Ixy  Iyy  Iys | <BR>
	\ Ixs  Iys  Iss /
*/
CvMat* hessian_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	CvMat* H;
	double v, dxx, dyy, dss, dxy, dxs, dys;

	v = pixval32f( dog_pyr[octv][intvl], r, c );
	dxx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) +
			pixval32f( dog_pyr[octv][intvl], r, c-1 ) - 2 * v );
	dyy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) +
			pixval32f( dog_pyr[octv][intvl], r-1, c ) - 2 * v );
	dss = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) +
			pixval32f( dog_pyr[octv][intvl-1], r, c ) - 2 * v );
	dxy = ( pixval32f( dog_pyr[octv][intvl], r+1, c+1 ) -
			pixval32f( dog_pyr[octv][intvl], r+1, c-1 ) -
			pixval32f( dog_pyr[octv][intvl], r-1, c+1 ) +
			pixval32f( dog_pyr[octv][intvl], r-1, c-1 ) ) / 4.0;
	dxs = ( pixval32f( dog_pyr[octv][intvl+1], r, c+1 ) -
			pixval32f( dog_pyr[octv][intvl+1], r, c-1 ) -
			pixval32f( dog_pyr[octv][intvl-1], r, c+1 ) +
			pixval32f( dog_pyr[octv][intvl-1], r, c-1 ) ) / 4.0;
	dys = ( pixval32f( dog_pyr[octv][intvl+1], r+1, c ) -
			pixval32f( dog_pyr[octv][intvl+1], r-1, c ) -
			pixval32f( dog_pyr[octv][intvl-1], r+1, c ) +
			pixval32f( dog_pyr[octv][intvl-1], r-1, c ) ) / 4.0;

	H = cvCreateMat( 3, 3, CV_64FC1 );
	cvmSet( H, 0, 0, dxx );
	cvmSet( H, 0, 1, dxy );
	cvmSet( H, 0, 2, dxs );
	cvmSet( H, 1, 0, dxy );
	cvmSet( H, 1, 1, dyy );
	cvmSet( H, 1, 2, dys );
	cvmSet( H, 2, 0, dxs );
	cvmSet( H, 2, 1, dys );
	cvmSet( H, 2, 2, dss );

	return H;
}

/*
Calculates interpolated pixel contrast.  Based on Eqn. (3) in Lowe's paper.

@param dog_pyr difference of Gaussians scale space pyramid
@param octv octave of scale space
@param intvl within-octave interval
@param r pixel row
@param c pixel column
@param xi interpolated subpixel increment to interval
@param xr interpolated subpixel increment to row
@param xc interpolated subpixel increment to col

@param Returns interpolated contrast.
*/
double interp_contr( IplImage*** dog_pyr, int octv, int intvl, int r,
					int c, double xi, double xr, double xc )
{
	CvMat* dD, X, T;
	double t[1], x[3] = { xc, xr, xi };

	cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
	cvInitMatHeader( &T, 1, 1, CV_64FC1, t, CV_AUTOSTEP );
	dD = deriv_3D( dog_pyr, octv, intvl, r, c );
	cvGEMM( dD, &X, 1, NULL, 0, &T,  CV_GEMM_A_T );
	cvReleaseMat( &dD );

	return pixval32f( dog_pyr[octv][intvl], r, c ) + t[0] * 0.5;
}



/*
Allocates and initializes a new feature

@return Returns a pointer to the new feature
*/
struct feature* new_feature( void )
{
	struct feature* feat;
	struct detection_data* ddata;

	feat = (feature*)malloc( sizeof( struct feature ) );
	memset( feat, 0, sizeof( struct feature ) );
	ddata = (detection_data*)malloc( sizeof( struct detection_data ) );
	memset( ddata, 0, sizeof( struct detection_data ) );
	feat->feature_data = ddata;
	feat->type = FEATURE_LOWE;

	return feat;
}



/*
Determines whether a feature is too edge like to be stable by computing the
ratio of principal curvatures at that feature.  Based on Section 4.1 of
Lowe's paper.

@param dog_img image from the DoG pyramid in which feature was detected
@param r feature row
@param c feature col
@param curv_thr high threshold on ratio of principal curvatures

@return Returns 0 if the feature at (r,c) in dog_img is sufficiently
	corner-like or 1 otherwise.
*/
int is_too_edge_like( IplImage* dog_img, int r, int c, int curv_thr )
{
	double d, dxx, dyy, dxy, tr, det;

	/* principal curvatures are computed using the trace and det of Hessian */
	d = pixval32f(dog_img, r, c);
	dxx = pixval32f( dog_img, r, c+1 ) + pixval32f( dog_img, r, c-1 ) - 2 * d;
	dyy = pixval32f( dog_img, r+1, c ) + pixval32f( dog_img, r-1, c ) - 2 * d;
	dxy = ( pixval32f(dog_img, r+1, c+1) - pixval32f(dog_img, r+1, c-1) -
			pixval32f(dog_img, r-1, c+1) + pixval32f(dog_img, r-1, c-1) ) / 4.0;
	tr = dxx + dyy;
	det = dxx * dyy - dxy * dxy;

	/* negative determinant -> curvatures have different signs; reject feature */
	if( det <= 0 )
		return 1;

	if( tr * tr / det < ( curv_thr + 1.0 )*( curv_thr + 1.0 ) / curv_thr )
		return 0;
	return 1;
}



/*
Calculates characteristic scale for each feature in an array.

@param features array of features
@param sigma amount of Gaussian smoothing per octave of scale space
@param intvls intervals per octave of scale space
*/
void calc_feature_scales( CvSeq* features, double sigma, int intvls )
{
	struct feature* feat;
	struct detection_data* ddata;
	double intvl;
	int i, n;

	n = features->total;
	for( i = 0; i < n; i++ )
	{
		feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		ddata = feat_detection_data( feat );
		intvl = ddata->intvl + ddata->subintvl;
		feat->scl = sigma * pow( 2.0, ddata->octv + intvl / intvls );
		ddata->scl_octv = sigma * pow( 2.0, intvl / intvls );
	}
}



/*
Halves feature coordinates and scale in case the input image was doubled
prior to scale space construction.

@param features array of features
*/
void adjust_for_img_dbl( CvSeq* features )
{
	struct feature* feat;
	int i, n;

	n = features->total;
	for( i = 0; i < n; i++ )
	{
		feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		feat->x /= 2.0;
		feat->y /= 2.0;
		feat->scl /= 2.0;
		feat->img_pt.x /= 2.0;
		feat->img_pt.y /= 2.0;
	}
}



/*
Computes a canonical orientation for each image feature in an array.  Based
on Section 5 of Lowe's paper.  This function adds features to the array when
there is more than one dominant orientation at a given feature location.

@param features an array of image features
@param gauss_pyr Gaussian scale space pyramid
*/
void calc_feature_oris( CvSeq* features, IplImage*** gauss_pyr )
{
	struct feature* feat;
	struct detection_data* ddata;
	double* hist;
	double omax;
	int i, j, n = features->total;

	for( i = 0; i < n; i++ )
	{
		feat = (feature*)malloc( sizeof( struct feature ) );
		cvSeqPopFront( features, feat );
		ddata = feat_detection_data( feat );
		hist = ori_hist( gauss_pyr[ddata->octv][ddata->intvl],
						ddata->r, ddata->c, SIFT_ORI_HIST_BINS,
						cvRound( SIFT_ORI_RADIUS * ddata->scl_octv ),
						SIFT_ORI_SIG_FCTR * ddata->scl_octv );
		for( j = 0; j < SIFT_ORI_SMOOTH_PASSES; j++ )
			smooth_ori_hist( hist, SIFT_ORI_HIST_BINS );
		omax = dominant_ori( hist, SIFT_ORI_HIST_BINS );
		add_good_ori_features( features, hist, SIFT_ORI_HIST_BINS,
								omax * SIFT_ORI_PEAK_RATIO, feat );
		free( ddata );
		free( feat );
		free( hist );
	}
}



/*
Computes a gradient orientation histogram at a specified pixel.

@param img image
@param r pixel row
@param c pixel col
@param n number of histogram bins
@param rad radius of region over which histogram is computed
@param sigma std for Gaussian weighting of histogram entries

@return Returns an n-element array containing an orientation histogram
	representing orientations between 0 and 2 PI.
*/
double* ori_hist( IplImage* img, int r, int c, int n, int rad, double sigma)
{
	double* hist;
	double mag, ori, w, exp_denom, PI2 = CV_PI * 2.0;
	int bin, i, j;

	hist = (double*)calloc( n, sizeof( double ) );
	exp_denom = 2.0 * sigma * sigma;
	for( i = -rad; i <= rad; i++ )
		for( j = -rad; j <= rad; j++ )
			if( calc_grad_mag_ori( img, r + i, c + j, &mag, &ori ) )
			{
				w = exp( -( i*i + j*j ) / exp_denom );
				bin = cvRound( n * ( ori + CV_PI ) / PI2 );
				bin = ( bin < n )? bin : 0;
				hist[bin] += w * mag;
			}

	return hist;
}



/*
Calculates the gradient magnitude and orientation at a given pixel.

@param img image
@param r pixel row
@param c pixel col
@param mag output as gradient magnitude at pixel (r,c)
@param ori output as gradient orientation at pixel (r,c)

@return Returns 1 if the specified pixel is a valid one and sets mag and
	ori accordingly; otherwise returns 0
*/
int calc_grad_mag_ori( IplImage* img, int r, int c, double* mag, double* ori )
{
	double dx, dy;

	if( r > 0  &&  r < img->height - 1  &&  c > 0  &&  c < img->width - 1 )
	{
		dx = pixval32f( img, r, c+1 ) - pixval32f( img, r, c-1 );
		dy = pixval32f( img, r-1, c ) - pixval32f( img, r+1, c );
		*mag = sqrt( dx*dx + dy*dy );
		*ori = atan2( dy, dx );
		return 1;
	}

	else
		return 0;
}



/*
Gaussian smooths an orientation histogram.

@param hist an orientation histogram
@param n number of bins
*/
void smooth_ori_hist( double* hist, int n )
{
	double prev, tmp, h0 = hist[0];
	int i;

	prev = hist[n-1];
	for( i = 0; i < n; i++ )
	{
		tmp = hist[i];
		hist[i] = 0.25 * prev + 0.5 * hist[i] +
			0.25 * ( ( i+1 == n )? h0 : hist[i+1] );
		prev = tmp;
	}
}



/*
Finds the magnitude of the dominant orientation in a histogram

@param hist an orientation histogram
@param n number of bins

@return Returns the value of the largest bin in hist
*/
double dominant_ori( double* hist, int n )
{
	double omax;
	int i;

	omax = hist[0];
	for( i = 1; i < n; i++ )
		if( hist[i] > omax )
		{
			omax = hist[i];
		}
	return omax;
}



/*
Interpolates a histogram peak from left, center, and right values
*/
#define interp_hist_peak( l, c, r ) ( 0.5 * ((l)-(r)) / ((l) - 2.0*(c) + (r)) )



/*
Adds features to an array for every orientation in a histogram greater than
a specified threshold.

@param features new features are added to the end of this array
@param hist orientation histogram
@param n number of bins in hist
@param mag_thr new features are added for entries in hist greater than this
@param feat new features are clones of this with different orientations
*/
void add_good_ori_features( CvSeq* features, double* hist, int n,
						   double mag_thr, struct feature* feat )
{
	struct feature* new_feat;
	double bin, PI2 = CV_PI * 2.0;
	int l, r, i;

	for( i = 0; i < n; i++ )
	{
		l = ( i == 0 )? n - 1 : i-1;
		r = ( i + 1 ) % n;

		if( hist[i] > hist[l]  &&  hist[i] > hist[r]  &&  hist[i] >= mag_thr )
		{
			bin = i + interp_hist_peak( hist[l], hist[i], hist[r] );
			bin = ( bin < 0 )? n + bin : ( bin >= n )? bin - n : bin;
			new_feat = clone_feature( feat );
			new_feat->ori = ( ( PI2 * bin ) / n ) - CV_PI;
			cvSeqPush( features, new_feat );
			free( new_feat );
		}
	}
}



/*
Makes a deep copy of a feature

@param feat feature to be cloned

@return Returns a deep copy of feat
*/
struct feature* clone_feature( struct feature* feat )
{
	struct feature* new_feat;
	struct detection_data* ddata;

	new_feat = new_feature();
	ddata = feat_detection_data( new_feat );
	memcpy( new_feat, feat, sizeof( struct feature ) );
	memcpy( ddata, feat_detection_data(feat), sizeof( struct detection_data ) );
	new_feat->feature_data = ddata;

	return new_feat;
}



/*
Computes feature descriptors for features in an array.  Based on Section 6
of Lowe's paper.

@param features array of features
@param gauss_pyr Gaussian scale space pyramid
@param d width of 2D array of orientation histograms
@param n number of bins per orientation histogram
*/
void compute_descriptors( CvSeq* features, IplImage*** gauss_pyr, int d, int n)
{
	struct feature* feat;
	struct detection_data* ddata;
	double*** hist;
	int i, k = features->total;

	for( i = 0; i < k; i++ )
	{
		feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		ddata = feat_detection_data( feat );
		hist = descr_hist( gauss_pyr[ddata->octv][ddata->intvl], ddata->r,
			ddata->c, feat->ori, ddata->scl_octv, d, n );
		hist_to_descr( hist, d, n, feat );
		release_descr_hist( &hist, d );
	}
}



/*
Computes the 2D array of orientation histograms that form the feature
descriptor.  Based on Section 6.1 of Lowe's paper.

@param img image used in descriptor computation
@param r row coord of center of orientation histogram array
@param c column coord of center of orientation histogram array
@param ori canonical orientation of feature whose descr is being computed
@param scl scale relative to img of feature whose descr is being computed
@param d width of 2d array of orientation histograms
@param n bins per orientation histogram

@return Returns a d x d array of n-bin orientation histograms.
*/
double*** descr_hist( IplImage* img, int r, int c, double ori,
					 double scl, int d, int n )
{
	double*** hist;
	double cos_t, sin_t, hist_width, exp_denom, r_rot, c_rot, grad_mag,
		grad_ori, w, rbin, cbin, obin, bins_per_rad, PI2 = 2.0 * CV_PI;
	int radius, i, j;

	hist = (double***)calloc( d, sizeof( double** ) );
	for( i = 0; i < d; i++ )
	{
		hist[i] = (double**)calloc( d, sizeof( double* ) );
		for( j = 0; j < d; j++ )
			hist[i][j] = (double*)calloc( n, sizeof( double ) );
	}

	cos_t = cos( ori );
	sin_t = sin( ori );
	bins_per_rad = n / PI2;
	exp_denom = d * d * 0.5;
	hist_width = SIFT_DESCR_SCL_FCTR * scl;
	radius = hist_width * sqrt(2.0) * ( d + 1.0 ) * 0.5 + 0.5;
	for( i = -radius; i <= radius; i++ )
		for( j = -radius; j <= radius; j++ )
		{
			/*
			Calculate sample's histogram array coords rotated relative to ori.
			Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
			r_rot = 1.5) have full weight placed in row 1 after interpolation.
			*/
			c_rot = ( j * cos_t - i * sin_t ) / hist_width;
			r_rot = ( j * sin_t + i * cos_t ) / hist_width;
			rbin = r_rot + d / 2 - 0.5;
			cbin = c_rot + d / 2 - 0.5;

			if( rbin > -1.0  &&  rbin < d  &&  cbin > -1.0  &&  cbin < d )
				if( calc_grad_mag_ori( img, r + i, c + j, &grad_mag, &grad_ori ))
				{
					grad_ori -= ori;
					while( grad_ori < 0.0 )
						grad_ori += PI2;
					while( grad_ori >= PI2 )
						grad_ori -= PI2;

					obin = grad_ori * bins_per_rad;
					w = exp( -(c_rot * c_rot + r_rot * r_rot) / exp_denom );
					interp_hist_entry( hist, rbin, cbin, obin, grad_mag * w, d, n );
				}
		}

	return hist;
}



/*
Interpolates an entry into the array of orientation histograms that form
the feature descriptor.

@param hist 2D array of orientation histograms
@param rbin sub-bin row coordinate of entry
@param cbin sub-bin column coordinate of entry
@param obin sub-bin orientation coordinate of entry
@param mag size of entry
@param d width of 2D array of orientation histograms
@param n number of bins per orientation histogram
*/
void interp_hist_entry( double*** hist, double rbin, double cbin,
					   double obin, double mag, int d, int n )
{
	double d_r, d_c, d_o, v_r, v_c, v_o;
	double** row, * h;
	int r0, c0, o0, rb, cb, ob, r, c, o;

	r0 = cvFloor( rbin );
	c0 = cvFloor( cbin );
	o0 = cvFloor( obin );
	d_r = rbin - r0;
	d_c = cbin - c0;
	d_o = obin - o0;

	/*
	The entry is distributed into up to 8 bins.  Each entry into a bin
	is multiplied by a weight of 1 - d for each dimension, where d is the
	distance from the center value of the bin measured in bin units.
	*/
	for( r = 0; r <= 1; r++ )
	{
		rb = r0 + r;
		if( rb >= 0  &&  rb < d )
		{
			v_r = mag * ( ( r == 0 )? 1.0 - d_r : d_r );
			row = hist[rb];
			for( c = 0; c <= 1; c++ )
			{
				cb = c0 + c;
				if( cb >= 0  &&  cb < d )
				{
					v_c = v_r * ( ( c == 0 )? 1.0 - d_c : d_c );
					h = row[cb];
					for( o = 0; o <= 1; o++ )
					{
						ob = ( o0 + o ) % n;
						v_o = v_c * ( ( o == 0 )? 1.0 - d_o : d_o );
						h[ob] += v_o;
					}
				}
			}
		}
	}
}



/*
Converts the 2D array of orientation histograms into a feature's descriptor
vector.

@param hist 2D array of orientation histograms
@param d width of hist
@param n bins per histogram
@param feat feature into which to store descriptor
*/
void hist_to_descr( double*** hist, int d, int n, struct feature* feat )
{
	int int_val, i, r, c, o, k = 0;

	for( r = 0; r < d; r++ )
		for( c = 0; c < d; c++ )
			for( o = 0; o < n; o++ )
				feat->descr[k++] = hist[r][c][o];

	feat->d = k;
	normalize_descr( feat );
	for( i = 0; i < k; i++ )
		if( feat->descr[i] > SIFT_DESCR_MAG_THR )
			feat->descr[i] = SIFT_DESCR_MAG_THR;
	normalize_descr( feat );

	/* convert floating-point descriptor to integer valued descriptor */
	for( i = 0; i < k; i++ )
	{
		int_val = SIFT_INT_DESCR_FCTR * feat->descr[i];
		feat->descr[i] = MIN( 255, int_val );
	}
}



/*
Normalizes a feature's descriptor vector to unitl length

@param feat feature
*/
void normalize_descr( struct feature* feat )
{
	double cur, len_inv, len_sq = 0.0;
	int i, d = feat->d;

	for( i = 0; i < d; i++ )
	{
		cur = feat->descr[i];
		len_sq += cur*cur;
	}
	len_inv = 1.0 / sqrt( len_sq );
	for( i = 0; i < d; i++ )
		feat->descr[i] *= len_inv;
}



/*
Compares features for a decreasing-scale ordering.  Intended for use with
CvSeqSort

@param feat1 first feature
@param feat2 second feature
@param param unused

@return Returns 1 if feat1's scale is greater than feat2's, -1 if vice versa,
and 0 if their scales are equal
*/
int feature_cmp( void* feat1, void* feat2, void* param )
{
	struct feature* f1 = (struct feature*) feat1;
	struct feature* f2 = (struct feature*) feat2;

	if( f1->scl < f2->scl )
		return 1;
	if( f1->scl > f2->scl )
		return -1;
	return 0;
}



/*
De-allocates memory held by a descriptor histogram

@param hist pointer to a 2D array of orientation histograms
@param d width of hist
*/
void release_descr_hist( double**** hist, int d )
{
	int i, j;

	for( i = 0; i < d; i++)
	{
		for( j = 0; j < d; j++ )
			free( (*hist)[i][j] );
		free( (*hist)[i] );
	}
	free( *hist );
	*hist = NULL;
}


/*
De-allocates memory held by a scale space pyramid

@param pyr scale space pyramid
@param octvs number of octaves of scale space
@param n number of images per octave
*/
void release_pyr( IplImage**** pyr, int octvs, int n )
{
	int i, j;
	for( i = 0; i < octvs; i++ )
	{
		for( j = 0; j < n; j++ )
			cvReleaseImage( &(*pyr)[i][j] );
		free( (*pyr)[i] );
	}
	free( *pyr );
	*pyr = NULL;
}

#endif // MRPT_HAS_OPENCV  for individual, auxiliary functions for SIFT

