//
// Created by raghavender on 28/06/17.
//

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <vector>
#include <iostream>


#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h> // important import
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/otherlibs/do_opencv_includes.h>


using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace mrpt::math;
using namespace mrpt;

using namespace std;
using namespace cv;

void  CFeatureExtraction::extractFeaturesAKAZE(
        const mrpt::utils::CImage	& inImg,
        CFeatureList			    & feats,
        unsigned int			    init_ID,
        unsigned int			    nDesiredFeatures,
        const TImageROI			    & ROI)  const
{

    MRPT_UNUSED_PARAM(ROI);
    MRPT_START
#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x210
        THROW_EXCEPTION("This function requires OpenCV > 2.1.0")
#	else

        using namespace cv;

        vector<KeyPoint> cv_feats; // The opencv keypoint output vector

        // Make sure we operate on a gray-scale version of the image:
        const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );



#if MRPT_OPENCV_VERSION_NUM >= 0x211

//    cv::Mat *mask ;
//    if( _mask )
//       mask = static_cast<cv::Mat*>(_mask);

        const Mat theImg = cvarrToMat( inImg_gray.getAs<IplImage>() );


        Ptr<AKAZE> akaze = AKAZE::create(options.AKAZEOptions.descriptor_type , options.AKAZEOptions.descriptor_size ,
        options.AKAZEOptions.descriptor_channels , options.AKAZEOptions.threshold , options.AKAZEOptions.nOctaves ,
        options.AKAZEOptions.nOctaveLayers , options.AKAZEOptions.diffusivity);

        akaze->detect(theImg,cv_feats);

        // *All* the features have been extracted.
        const size_t N = cv_feats.size();

#endif

        // Now:
        //  1) Sort them by "response": It's ~100 times faster to sort a list of
        //      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
        //std::vector<size_t> sorted_indices(N);
        //for (size_t i=0;i<N;i++)  sorted_indices[i]=i;
        //std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter<vector<KeyPoint> >(cv_feats) );
        // sort the AKAZE features by line length
        for(int i=0; i<N ;i++)
        {
            for(int j=i+1; j<N ; j++)
            {
                if(cv_feats.at(j).response > cv_feats.at(i).response)
                {
                    KeyPoint temp_point = cv_feats.at(i);
                    cv_feats.at(i) = cv_feats.at(j);
                    cv_feats.at(j) = temp_point;
                }
            }
        }

        //  2) Filter by "min-distance" (in options.FASTOptions.min_distance) // not REQUIRED FOR LSD FEATYRES
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

        while( cont != nMax && i!=N )
        {
            // Take the next feature fromt the ordered list of good features:
            const KeyPoint &kp = cv_feats[i];
            i++;

            // Patch out of the image??
            const int xBorderInf = (int)floor( kp.pt.x - size_2 );
            const int xBorderSup = (int)floor( kp.pt.x + size_2 );
            const int yBorderInf = (int)floor( kp.pt.y - size_2 );
            const int yBorderSup = (int)floor( kp.pt.y + size_2 );

            if (!( xBorderSup < (int)imgW && xBorderInf > 0 && yBorderSup < (int)imgH && yBorderInf > 0 ))
                continue; // nope, skip.


            // All tests passed: add new feature:
            CFeature::Ptr ft		= std::make_shared<CFeature>();
            ft->type			= featAKAZE;
            ft->ID				= nextID++;
            ft->x				= kp.pt.x;
            ft->y				= kp.pt.y;
            ft->response		= kp.response;
            ft->orientation		= kp.angle;
            ft->scale			= kp.octave;
            ft->patchSize		= options.patchSize;		// The size of the feature patch

            if( options.patchSize > 0 )
            {
                inImg.extract_patch(
                        ft->patch,
                        round( ft->x ) - offset,
                        round( ft->y ) - offset,
                        options.patchSize,
                        options.patchSize );						// Image patch surronding the feature
            }
            feats.push_back( ft );
            ++cont;
            cout << ft->x << "  " << ft->y << endl;
        }
        //feats.resize( cont );  // JL: really needed???

#	endif
#endif
    MRPT_END


}
