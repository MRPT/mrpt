/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: benchmarkingImageFeatures_gui
	FILE: visual_odometry.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

/** part of this code is built on top of Avi Singh's code for monocular visual
 * odometry, hence the following license is attached
 * 156+62 = 218 lines of code have been used as the starting point from from
 * https://github.com/avisingh599/mono-vo
 */

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

/// OpenCV includes
#include <mrpt/otherlibs/do_opencv_includes.h>

/// basic C++ includes
#include <iostream>
#include <ctype.h>
#include <algorithm>  // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <array>

/// MRPT includes
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/img/CImage.h>

/// Qt includes
#include <QMainWindow>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QString>

using mrpt::img::CImage;

using namespace mrpt::vision;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;
using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

class Counter : public QObject
{
	Q_OBJECT

   public:
	int m_value;

   public:
	Counter() { m_value = 0; };
	//~Counter();

	int value() const { return m_value; };
   public slots:
	void setValue(int value)
	{
		m_value = value;
		emit valueChanged(value);
		// cout << value <<  " you called me in SLOT 2" << endl;
	};

   signals:
	void valueChanged(int newValue);
};

class VisualOdometry  //: public QDialog
{
	// Q_OBJECT

   public:
	QWidget* dialog_gui;
	QLabel* VO_progress;

	QGridLayout* layout_grid;
	int detector_selected;
	CFeatureExtraction fext;
	int numFeats;

	int current_frame;
	QString curr_frame;
	Counter cnt;

   public:
	/**
	 * constructor which initializes the basic variables
	 * @param detector_selected the input detector selected currently by the
	 * user in the mainwindow,
	 * this constructor is invoked when the user clicks on generate viusal
	 * odometry button
	 * @param fext this variable holds the selected detector information
	 * @param numFeats this holds the number of features to be selected
	 */
	VisualOdometry();

	/**
	 * this tracks the features in subsequent frames using a KLT tracker as
	 * described in https://avisingh599.github.io/vision/monocular-vo/
	 * @param img_1 the input image 1 to the tracker
	 * @param img_2 the input image 2 for the tracker
	 * @param points1 key-points in image 1
	 * @param points2 corresponding tracked keypoints in image 2
	 * @param status
	 */
	void featureTracking(
		Mat img_1, Mat img_2, vector<Point2f>& points1,
		vector<Point2f>& points2, vector<uchar>& status);

	/**
	 * this function detects key-points based on the detector and the detector
	 * parameters specified by the user
	 * @param img1 input image in which to detect the key-points
	 * @param points1  the keypoints in the input image
	 * @param feat_type type of the feature
	 */
	void featureDetection(CImage img1, vector<Point2f>& points1, int feat_type);

	/**
	 * this function gets the absolute scale using the ground truth in the case
	 * of KITTI, normally this can be obtained from another sensor like a
	 * speedometer too.
	 * @param frame_id the frame id of which the scale is required
	 * @param sequence_id not used
	 * @param z_cal not used
	 * @param poses_ground_truth the file-path of the ground truth file
	 * @return
	 */
	double getAbsoluteScale(
		int frame_id, int sequence_id, double z_cal, string poses_ground_truth);

	/**
	 * this is the main function which does the tracking by calling the other
	 * required function in it
	 * @param dataset this holds the file-path to the dataset of which the
	 * visual odometry needs to be calculated
	 * @param groundtruth this has the file-path of the ground truth file, for
	 * KITTI it maybe any of 00.txt, 01.txt, ...
	 * @param feat_type type of the feature
	 * @return
	 */
	// Mat generateVO(CFeatureExtraction fext, int numFeats, string dataset,
	// string groundtruth, string calibration_file, int feat_type );
	Mat generateVO(
		CFeatureExtraction fext, int numFeats,
		std::array<std::string, 3> file_paths, int feat_type);

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
	 * display function is there just for testing/debugging purposes, maybe
	 * deleted later
	 */
	void display();

	/**
	 * getCalibrationParams reads the camera calibration parameters from the
	 * calibration file and loads into the required variables
	 * @param calibratin_file input path to the calibration file
	 * @return returns the calibration parameters
	 */
	vector<double> getCalibrationParams(string calibratin_file);

	/**
	 * need to change the values based on eah sequence starting point the
	 * starting point of each
	 * trajectory in KITTI goes here
	 * @param ch
	 * @return
	 */
	vector<int> computeStartingPoint(char ch);
};
