//
// Created by raghavender on 28/06/17.
//


#include "opencv2/core.hpp"

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>

#include <vector>
#include <iostream>


#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h> // important import
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/otherlibs/do_opencv_includes.h>
using namespace cv::line_descriptor;


using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace mrpt::math;
using namespace mrpt;

using namespace std;
using namespace cv;


void  CFeatureExtraction::extractFeaturesLSD(const mrpt::utils::CImage &inImg, CFeatureList &feats,
                                             unsigned int init_ID, unsigned int nDesiredFeatures,
                                             const TImageROI &ROI) const
{

    MRPT_UNUSED_PARAM(ROI);
    MRPT_START
#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x210
        THROW_EXCEPTION("This function requires OpenCV > 2.1.0")
#	else

        using namespace cv;

        vector<KeyPoint> cv_feats; // The opencv keypoint output vector
        vector<KeyLine> cv_line;

        // Make sure we operate on a gray-scale version of the image:
        const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );

        // JL: Instead of
        //	int aux = options.FASTOptions.threshold; ....
        //  It's better to use an adaptive threshold, controlled from our caller outside.

#if MRPT_OPENCV_VERSION_NUM >= 0x211

//    cv::Mat *mask ;
//    if( _mask )
//       mask = static_cast<cv::Mat*>(_mask);

        const Mat theImg = cvarrToMat( inImg_gray.getAs<IplImage>() );


        /* create a random binary mask */
        cv::Mat mask = Mat::ones( theImg.size(), CV_8UC1 );

        /* create a pointer to a BinaryDescriptor object with deafult parameters */
        Ptr<LSDDetector> bd = LSDDetector::createLSDDetector();

        /* create a structure to store extracted lines */


        /* extract lines */
        cv::Mat output = theImg.clone();
        bd->detect( theImg, cv_line, options.LSDOptions.scale, options.LSDOptions.nOctaves, mask );


        // *All* the features have been extracted.
        const size_t N = cv_line.size();

#endif


        // Now:
        //  1) Sort them by "response": It's ~100 times faster to sort a list of
        //      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
        //std::vector<size_t> sorted_indices(N);
        //for (size_t i=0;i<N;i++)  sorted_indices[i]=i;
        //std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter<vector<KeyLine> >(cv_line) );

        // sort the LSD features by line length
        for(int i=0; i<N ;i++)
        {
            for(int j=i+1; j<N ; j++)
            {
                if(cv_line.at(j).lineLength > cv_line.at(i).lineLength)
                {
                    KeyLine temp_line = cv_line.at(i);
                    cv_line.at(i) = cv_line.at(j);
                    cv_line.at(j) = temp_line;
                }
            }
        }

        //  2) Filter by "min-distance" (in options.FASTOptions.min_distance)  // NOT REQUIRED FOR LSD Features
        //  3) Convert to MRPT CFeatureList format.
        // Steps 2 & 3 are done together in the while() below.
        // The "min-distance" filter is done by means of a 2D binary matrix where each cell is marked when one
        // feature falls within it. This is not exactly the same than a pure "min-distance" but is pretty close
        // and for large numbers of features is much faster than brute force search of kd-trees.
        // (An intermediate approach would be the creation of a mask image updated for each accepted feature, etc.)




        unsigned int	nMax		= (nDesiredFeatures!=0 && N > nDesiredFeatures) ? nDesiredFeatures : N;
        const int 		offset		= (int)this->options.patchSize/2 + 1;
        const size_t	size_2		= options.patchSize/2;
        const size_t 	imgH		= inImg.getHeight();
        const size_t 	imgW		= inImg.getWidth();
        unsigned int	i			= 0;
        unsigned int	cont		= 0;
        TFeatureID		nextID		= init_ID;


        if( !options.addNewFeatures )
            feats.clear();

        /* draw lines extracted from octave 0 */
        if( output.channels() == 1 )
            cvtColor( output, output, COLOR_GRAY2BGR );


        while( cont != nMax && i!=N )
        {
            KeyLine kl = cv_line[i];
            KeyPoint kp;


            if(kl.octave == 0)
            {

                Point pt1 = Point2f(kl.startPointX, kl.startPointY);
                Point pt2 = Point2f(kl.endPointX, kl.endPointY);

                kp.pt.x = (pt1.x + pt2.x) / 2;
                kp.pt.y = (pt1.y + pt2.y) / 2;
                i++;
                // Patch out of the image??
                const int xBorderInf = (int) floor(kp.pt.x - size_2);
                const int xBorderSup = (int) floor(kp.pt.x + size_2);
                const int yBorderInf = (int) floor(kp.pt.y - size_2);
                const int yBorderSup = (int) floor(kp.pt.y + size_2);

                if (!(xBorderSup < (int) imgW && xBorderInf > 0 && yBorderSup < (int) imgH && yBorderInf > 0))
                    continue; // nope, skip.

                // All tests passed: add new feature:
                CFeature::Ptr ft = std::make_shared<CFeature>();
                ft->type = featLSD;
                ft->ID = nextID++;
                ft->x = kp.pt.x;
                ft->y = kp.pt.y;
                ft->x2[0] = pt1.x;
                ft->x2[1] = pt2.x;
                ft->y2[0] = pt1.y;
                ft->y2[1] = pt2.y;
                ft->response		= kl.response;
                //ft->orientation		= kp.angle;
                ft->scale			= kl.octave;
                //ft->patchSize		= options.patchSize;		// The size of the feature patch

                if (options.patchSize > 0) {
                    inImg.extract_patch(
                            ft->patch,
                            round(ft->x) - offset,
                            round(ft->y) - offset,
                            options.patchSize,
                            options.patchSize);                        // Image patch surronding the feature
                }
                feats.push_back(ft);
            }
            ++cont;
            //cout << ft->x << "  " << ft->y << endl;
        }
        //feats.resize( cont );  // JL: really needed???

#	endif
#endif
    MRPT_END


}


/************************************************************************************************
*						internal_computeBLDDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computeBLDLineDescriptors(
        const mrpt::utils::CImage &in_img,
        CFeatureList &in_features) const
{
//#if HAVE_OPENCV_WITH_SURF
    using namespace cv;

    if (in_features.empty()) return;

	const CImage img_grayscale(in_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat img = cvarrToMat( img_grayscale.getAs<IplImage>() );

	vector<KeyPoint> cv_feats; // OpenCV keypoint output vector
	Mat              cv_descs; // OpenCV descriptor output


    cv::Mat mask = Mat::ones( img.size(), CV_8UC1 );

    BinaryDescriptor::Params params;
    params.ksize_               = options.BLDOptions.ksize_;
    params.reductionRatio       = options.BLDOptions.reductionRatio;
    params.numOfOctave_         = options.BLDOptions.numOfOctave;
    params.widthOfBand_         = options.BLDOptions.widthOfBand;

    Ptr<BinaryDescriptor> bd2 = BinaryDescriptor::createBinaryDescriptor(params);
    /* compute lines */
    std::vector<KeyLine> keylines;

    bd2->detect( img, keylines, mask );

    /* compute descriptors */

    bd2->compute( img, keylines, cv_descs);


    keylines.resize(in_features.size());



//gb redesign end
	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	CFeatureList::iterator	itList;
	int i;
	for (i=0, itList=in_features.begin();itList!=in_features.end();itList++,i++)
	{
		CFeature::Ptr ft = *itList;

        // Get the BLD descriptor
		ft->descriptors.BLD.resize( cv_descs.cols );
		for( int m = 0; m < cv_descs.cols; ++m )
			ft->descriptors.BLD[m] = cv_descs.at<int>(i,m);		// Get the SURF descriptor
	} // end for

//#else
  //  THROW_EXCEPTION("Method not available: MRPT compiled without OpenCV, or against a version of OpenCV without SURF")
//#endif //MRPT_HAS_OPENCV
}  // end internal_computeSurfDescriptors


