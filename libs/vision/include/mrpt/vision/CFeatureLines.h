/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFeatureLines_H
#define CFeatureLines_H

#include <mrpt/vision/utils.h>
//#include <mrpt/utils/CImage.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/eigen.hpp>
//#include <pcl/point_types.h>

namespace mrpt
{
    namespace vision
    {
        /**  This class wraps different line detectors and descriptors from OpenCV.
          *
          *  \ingroup mrpt_vision_grp
          */
        class CFeatureLines
        {
          public:
            void extractLines (const cv::Mat & image,
                                std::vector<cv::Vec4i> & segments,
                                size_t threshold , const bool display = false);

            void extractLines_CannyHough(const cv::Mat & canny_image,
                                         const std::vector<cv::Vec2f> lines,
                                         std::vector<cv::Vec4i> & segments,
                                         size_t threshold );
        }; // end of class

    } // end of namespace
} // end of namespace

#endif
