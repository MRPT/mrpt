/*

The MIT License

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/


#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"


#include <mrpt/vision/CFeatureExtraction.h>

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <opencv2/line_descriptor.hpp>

#include <opencv2/plot.hpp>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>

using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;
using namespace cv;
using namespace std;

#define MAX_FRAME 2000
#define MIN_NUM_FEAT 2000




class VisualOdometry
{

public:
    int detector_selected;
    CFeatureExtraction fext;
    int numFeats;

public:
    /**
     * constructor which initializes the basic variables
     * @param detector_selected the input detector selected currently by the user in the mainwindow,
     * this constructor is invoked when the user clicks on generate viusal odometry button
     * @param fext this variable holds the selected detector information
     * @param numFeats this holds the number of features to be selected
     */
    VisualOdometry(int detector_selected, CFeatureExtraction fext, int numFeats);

    /**
     * this tracks the features in subsequent frames using a KLT tracker as described in https://avisingh599.github.io/vision/monocular-vo/
     * @param img_1 the input image 1 to the tracker
     * @param img_2 the input image 2 for the tracker
     * @param points1 key-points in image 1
     * @param points2 corresponding tracked keypoints in image 2
     * @param status
     */
    void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

    /**
     * this function detects key-points based on the detector and the detector parameters specified by the user
     * @param img1 input image in which to detect the key-points
     * @param points1  the keypoints in the input image
     * @param feat_type type of the feature
     */
    void featureDetection(CImage img1, vector<Point2f>& points1, int feat_type);

    /**
     * this function gets the absolute scale using the ground truth in the case of KITTI, normally this can be obtained from another sensor like a speedometer too.
     * @param frame_id the frame id of which the scale is required
     * @param sequence_id not used
     * @param z_cal not used
     * @param poses_ground_truth the file-path of the ground truth file
     * @return
     */
    double getAbsoluteScale(int frame_id, int sequence_id, double z_cal, string poses_ground_truth);

    /**
     * this is the main function which does the tracking by calling the other required function in it
     * @param dataset this holds the file-path to the dataset of which the visual odometry needs to be calculated
     * @param groundtruth this has the file-path of the ground truth file, for KITTI it maybe any of 00.txt, 01.txt, ...
     * @param feat_type type of the feature
     * @return
     */
    Mat generateVO(string dataset, string groundtruth, int feat_type );

    /**
     * this function stores the ground truth in an appropriate arrat
     * @param ground_truth_poses this has the file=path of the ground truth file
     */
    void storeGroundTruth(string ground_truth_poses);

    /**
     * this function computes the Odometry error, might need to changes this
     * @return
     */
    double computeOdomError();

    /**
     * display function is there just for testing/debugging purposes, maybe deleted later
     */
    void display();
};