//
// Created by raghavender on 28/06/17.
//




#include <vector>
#include <iostream>


#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h> // important import
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/otherlibs/do_opencv_includes.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#ifdef HAVE_OPENCV_NONFREE  //MRPT_HAS_OPENCV_NONFREE
# include <opencv2/nonfree/nonfree.hpp>
#endif
#ifdef HAVE_OPENCV_FEATURES2D
# include <opencv2/features2d/features2d.hpp>
# include <opencv2/line_descriptor.hpp>
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
# include <opencv2/xfeatures2d.hpp>
#endif



using namespace cv::line_descriptor;


using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace mrpt::math;
using namespace mrpt;

using namespace std;
#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM>=0x211
using namespace cv;
#	endif
#endif

/************************************************************************************************
*						internal_computeLATCHDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computeLATCHDescriptors(
        const mrpt::utils::CImage &in_img,
        CFeatureList &in_features) const
{
    MRPT_START
#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x210
    THROW_EXCEPTION("This function requires OpenCV > 2.1.0")
#	else


    //cout << "I am in LATCH descriptor" << endl;
    if (in_features.empty()) return;

    const size_t n_feats = in_features.size();
    // Make sure we operate on a gray-scale version of the image:
    const CImage inImg_gray( in_img, FAST_REF_OR_CONVERT_TO_GRAY );

    // convert from CFeatureList to vector<KeyPoint>
    vector<KeyPoint> cv_feats( n_feats );
    for( size_t k = 0; k < n_feats; ++k )
    {
        KeyPoint & kp = cv_feats[k];
        kp.pt.x		= in_features[k]->x;
        kp.pt.y		= in_features[k]->y;
        kp.angle	= in_features[k]->orientation;
        kp.size		= in_features[k]->scale;
    } // end-for

    Mat cvImg(cv::cvarrToMat(inImg_gray.getAs<IplImage>()));
    Mat cv_descs; // OpenCV descriptor output


    Ptr<xfeatures2d::LATCH> latch = xfeatures2d::LATCH::create(options.LATCHOptions.bytes, options.LATCHOptions.rotationInvariance, options.LATCHOptions.half_ssd_size);

    latch->compute(cvImg, cv_feats, cv_descs);

    // -----------------------------------------------------------------
    // MRPT Wrapping
    // -----------------------------------------------------------------
    CFeatureList::iterator	itList;
    int i;
    for (i=0, itList=in_features.begin();itList!=in_features.end();itList++,i++)
    {
        CFeature::Ptr ft = *itList;

        // Get the LATCH descriptor
        ft->descriptors.LATCH.resize( cv_descs.cols );
        for( int m = 0; m < cv_descs.cols; ++m )
            ft->descriptors.LATCH[m] = cv_descs.at<int>(i,m);		// Get the LATCH descriptor
    } // end for-

#endif
#endif
    MRPT_END
}  // end internal_computeLatchDescriptors


