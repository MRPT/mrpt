/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/threads.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>

#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
	#include "sift-hess/sift.h"
	#include "sift-hess/imgfeatures.h"
	#include "sift-hess/utils.h"
#endif


// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>
#ifdef HAVE_OPENCV_NONFREE  //MRPT_HAS_OPENCV_NONFREE
# include <opencv2/nonfree/nonfree.hpp>
#endif
#ifdef HAVE_OPENCV_FEATURES2D
# include <opencv2/features2d/features2d.hpp>
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
# include <opencv2/xfeatures2d.hpp>
#endif


// TODO: Remove, it's just for GetTempPathA
#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM>=0x211
		using namespace cv;
#	endif
#endif

/************************* Local Function Prototypes for Hess' SIFT *************************/
#if MRPT_HAS_OPENCV && MRPT_HAS_SIFT_HESS
extern "C"  // This is mandatory since the implementations are in ".c" files
{ 
IplImage* create_init_img(const IplImage*, int, double );
IplImage*** build_gauss_pyr( IplImage*, int, int, double );
IplImage*** build_dog_pyr( IplImage***, int, int );
CvSeq* scale_space_extrema( IplImage***, int, int, double, int, CvMemStorage*);
int is_extremum( IplImage***, int, int, int, int );
struct feature* new_feature( void );
void calc_feature_scales( CvSeq*, double, int );
void adjust_for_img_dbl( CvSeq* );
void calc_feature_oris( CvSeq*, IplImage*** );
void compute_descriptors( CvSeq*, IplImage***, int, int );
int feature_cmp( void*, void*, void* );
void release_pyr( IplImage****, int, int );
}
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
	if( ROI.xMin != 0 || ROI.xMax != 0 || ROI.yMin != 0 || ROI.yMax != 0 )
		usingROI = true;	// A ROI has been defined
	
	// ROI can not be managed properly (yet) with these method, so we extract a subimage

	// use a smart pointer so we just copy the pointer if the image is grayscale, or we'll create a new one if it was RGB:
	CImage img_grayscale(img, FAST_REF_OR_CONVERT_TO_GRAY); // Was: auxImgPtr;
	if( usingROI )
	{
		ASSERT_( ROI.xMin >= 0 && ROI.xMin < ROI.xMax && ROI.xMax < img.getWidth() && ROI.yMin >= 0 && ROI.yMax < img.getHeight() && ROI.yMin < ROI.yMax );
		CImage auximg; 
		img_grayscale.extract_patch( auximg, ROI.xMin, ROI.yMin, ROI.xMax-ROI.xMin+1, ROI.yMax-ROI.yMin+1 ); // Subimage in "auxImg"
		img_grayscale.swap(auximg);
	}

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
			img_grayscale.saveToFile( filImg );

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

			GetTempPathA(1000,filOut);	os::strcat(filOut,1000,"temp_out.txt");			// OUTPUT FILE
			GetTempPathA(1000,filImg);	os::strcat(filImg,1000,"temp_img.pgm");			// INPUT IMAGE (PGM) FOR ORIGINAL BINARY BY LOWE

			bool valid = img_grayscale.saveToFile( filImg );
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
			ASSERT_(img_grayscale.getWidth() != 0 && img_grayscale.getHeight() != 0);
			/* build scale space pyramid; smallest dimension of top level is ~4 pixels */
			const IplImage* ipl_im = img_grayscale.getAs<IplImage>();
			init_img = create_init_img( ipl_im, SIFT_IMG_DBL, SIFT_SIGMA );
			octvs = log( (float)(MIN( init_img->width, init_img->height )) ) / log((float)2) - 2;
			gauss_pyr = build_gauss_pyr( init_img, octvs, SIFT_INTVLS, SIFT_SIGMA );
			dog_pyr = build_dog_pyr( gauss_pyr, octvs, SIFT_INTVLS );
			storage = cvCreateMemStorage( 0 );
			features = scale_space_extrema( dog_pyr, octvs, SIFT_INTVLS, 
				options.SIFTOptions.threshold, // SIFT_CONTR_THR,
				options.SIFTOptions.edgeThreshold, // SIFT_CURV_THR
				storage );
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
#if defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)  //MRPT_HAS_OPENCV_NONFREE

	#if MRPT_OPENCV_VERSION_NUM >= 0x211 && MRPT_OPENCV_VERSION_NUM < 0x300 

			SiftFeatureDetector SIFTDetector(
				options.SIFTOptions.threshold, //SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
				options.SIFTOptions.edgeThreshold //SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD() );
				); 

			SiftDescriptorExtractor SIFTDescriptor;

			vector<KeyPoint> cv_feats;									// The OpenCV output feature list


			const IplImage* cGrey = img_grayscale.getAs<IplImage>();

			Mat theImg = cvarrToMat( cGrey );
			SIFTDetector.detect( theImg, cv_feats );

			Mat desc;
			SIFTDescriptor.compute( theImg, cv_feats, desc );

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
	#else
	// MRPT_OPENCV_VERSION_NUM >= 0x300
			using namespace cv;
			vector<KeyPoint> cv_feats;

			cv::Ptr<cv::xfeatures2d::SIFT>sift = cv::xfeatures2d::SIFT::create(nDesiredFeatures,3, options.SIFTOptions.threshold, options.SIFTOptions.edgeThreshold,1.6 ); //gb
			const IplImage* cGrey = img_grayscale.getAs<IplImage>();
			Mat theImg = cvarrToMat(cGrey);
			//SIFTDetector.detect(theImg, cv_feats);
			sift->detect(theImg, cv_feats); //gb
			Mat desc;
			//SIFTDescriptor.compute(theImg, cv_feats, desc);
			sift->compute(theImg, cv_feats, desc);
			
			//fromOpenCVToMRPT( theImg, cv_feats, desc, nDesiredFeatures, outList );
			const size_t	N = cv_feats.size();
			unsigned int	nMax = nDesiredFeatures != 0 && N > nDesiredFeatures ? nDesiredFeatures : N;
			const int 		offset = (int)this->options.patchSize / 2 + 1;
			const size_t	size_2 = options.patchSize / 2;
			const size_t 	imgH = img.getHeight();
			const size_t 	imgW = img.getWidth();
			unsigned int	i = 0;
			unsigned int	cont = 0;
			TFeatureID		nextID = init_ID;
			feats.clear();


			while (cont != nMax && i != N)
			{
				const int xBorderInf = (int)floor(cv_feats[i].pt.x - size_2);
				const int xBorderSup = (int)floor(cv_feats[i].pt.x + size_2);
				const int yBorderInf = (int)floor(cv_feats[i].pt.y - size_2);
				const int yBorderSup = (int)floor(cv_feats[i].pt.y + size_2);

				if (options.patchSize == 0 || ((xBorderSup < (int)imgW) && (xBorderInf > 0) && (yBorderSup < (int)imgH) && (yBorderInf > 0)))
				{
					CFeaturePtr ft = CFeature::Create();
					ft->type = featSIFT;
					ft->ID = nextID++;
					ft->x = cv_feats[i].pt.x;
					ft->y = cv_feats[i].pt.y;
					ft->response = cv_feats[i].response;
					ft->orientation = cv_feats[i].angle;
					ft->scale = cv_feats[i].size;
					ft->patchSize = options.patchSize;														// The size of the feature patch
					ft->descriptors.SIFT.resize(128);
					memcpy(&(ft->descriptors.SIFT[0]), &desc.data[128 * i], 128 * sizeof(ft->descriptors.SIFT[0]));	// The descriptor

					if (options.patchSize > 0)
					{
						img.extract_patch(
							ft->patch,
							round(ft->x) - offset,
							round(ft->y) - offset,
							options.patchSize,
							options.patchSize);						// Image patch surronding the feature
					}
					feats.push_back(ft);
					++cont;
				}
				++i;
			}
			feats.resize(cont);
	#endif
#else
	THROW_EXCEPTION("This method requires OpenCV >= 2.1.1 with nonfree module")
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

			std::cout << "[computeSiftFeatures1] " << nRows << " features.\n";

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
			const CImage img_grayscale(in_img, FAST_REF_OR_CONVERT_TO_GRAY);
			const IplImage* ipl_im = img_grayscale.getAs<IplImage>();

			init_img = create_init_img( ipl_im, SIFT_IMG_DBL, SIFT_SIGMA );
			octvs = log( (float)(MIN( init_img->width, init_img->height )) ) / log((float)2) - 2;
			gauss_pyr = build_gauss_pyr( init_img, octvs, SIFT_INTVLS, SIFT_SIGMA );
			dog_pyr = build_dog_pyr( gauss_pyr, octvs, SIFT_INTVLS );

			storage = cvCreateMemStorage( 0 );
			features = static_cast<CvSeq*>(my_scale_space_extrema( in_features, dog_pyr, octvs, SIFT_INTVLS, 
				options.SIFTOptions.threshold, // SIFT_CONTR_THR,
				options.SIFTOptions.edgeThreshold, // SIFT_CURV_THR
				storage ));
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
	MRPT_UNUSED_PARAM(contr_thr); MRPT_UNUSED_PARAM(curv_thr);
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

