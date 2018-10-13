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
	FILE: mainwindow.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define NUM_DETECTORS 11
#define NUM_DESCRIPTORS 8
#define IMAGE_WIDTH 600  // 400
#define IMAGE_HEIGHT 600  // 400
#define WIDGET_HEIGHT 30
#define PARAMS_HEIGHT 30
#define PARAMS_WIDTH 80
#define WIDGET_WIDTH 160
#define BUTTON_WIDTH 150
#define BUTTON_HEIGHT 30
#define CROSS_THICKNESS 3
#define CROSS_SIZE 20

/// for the visualize descriptor part
#define DESCRIPTOR_WIDTH 100
#define DESCRIPTOR_HEIGHT 100
#define MAX_DESC 500

/// Qt includes
#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QApplication>
#include <QtWidgets>
#include <QButtonGroup>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QRadioButton>
#include <QLineEdit>
#include <QMessageBox>
#include <QGroupBox>
#include <QListWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QMap>
#include <QtGui>
#include <QVector>
#include <QMouseEvent>
#include <QEvent>
#include <QThread>
#include <QtCore>
#include <QProgressBar>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <QtConcurrent/QtConcurrentRun>

/// standard c++ includes
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <functional>  // bind
#include <mutex>  // scoped_lock

/// opencv includes
#include <mrpt/otherlibs/do_opencv_includes.h>

/// MRPT includes
#include <mrpt/vision/CFeatureExtraction.h>

/// vision tasks includes
#include "my_qlabel.h"
#include "tracker.h"
#include "visual_odometry.h"
#include "place_recognition.h"

using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt;

class MainWindow : public QMainWindow
{
	Q_OBJECT

   public:
	QWidget* window_gui;  //!< main window_gui widget

	double sampling_rate;  //!< sampling rate/factor for image decimation

	QPushButton* button_generate;  //!< this is the button for visualize
	//! descriptor option
	QPushButton* button_close;  //!< this is button for closing the gui,
	//! currently not being used
	QPushButton* next_desc;  //!< unsued variable

	QPushButton* prev_button;  //!< previous button to go back to previous image
	//! when loading image datasets
	QPushButton* next_button;  //!< previous button to go forward one image when
	//! loading image datasets
	unsigned long current_imageIndex;  //!< used as a counter variable in
	//! readFilesFromFolder function

	QPushButton*
		browse_button;  //!< browse button to load image/folder for first image
	QPushButton* browse_button2;  //!< browse button to load image/folder for
	//! second image
	QPushButton* generateVisualOdometry;  //!< button which generates visual
	//! odometry for monocular datasets
	QGridLayout* layout_grid;  //!< main layut for the GUI app

	int currentInputIndex;  //!< stores the users selected choice among single
	//! image, stereo image, rawlog, single dataset or
	//! stereo dataset
	int detector_selected;  //!< stores the users selection for the chosen
	//! detector
	int descriptor_selected;  //!< stores the users selection for the chosen
	//! descriptor

	QGroupBox*
		groupBox1;  //!< stores detector, descriptor comboboxes and buttons
	QGroupBox* groupBox2;
	QGroupBox* groupBox_images;  //!< stores the images

	QComboBox*
		inputs;  //!< combobox contains type of input data specified by the user
	QLineEdit* inputFilePath;  //!< text field which takes the input for the
	//! first image / dataset file path
	QLineEdit* inputFilePath2;  //!< text field which takes the input for the
	//! second image / dataset file path
	string file_path1;  //!< string reads from inputFilePath to store file path
	//! entered by used
	string file_path2;  //!< string reads from inputFilePath2 to store file path
	//! entered by used

	QLineEdit*
		numFeaturesLineEdit;  //!< Text field to ask user for number of features

	QComboBox* detectors_select;  //! asks type of detector from the user
	QComboBox* descriptors_select;  //!< asks type of descriptor from the user

	my_qlabel* image1;  //!< QLabel which stores the first image which can be
	//! clicked to select the keypoint from it
	QLabel* image2;  //!< QLabel to hold the second image
	QImage qimage1;  //!< stores the first image
	QImage qimage2;  //!< store the second image
	int resolution_x, resolution_y;  //!< stores the resolution of the image

	QLabel* param1;  //!< QLabel for detector parameter 1
	QLabel* param2;  //!< QLabel for detector parameter 2
	QLabel* param3;  //!< QLabel for detector parameter 3
	QLabel* param4;  //!< QLabel for detector parameter 4
	QLabel* param5;  //!< QLabel for detector parameter 5

	QLineEdit* param1_edit;  //!< TextField for detector parameter 1
	QLineEdit* param2_edit;  //!< TextField for detector parameter 2
	QLineEdit* param3_edit;  //!< TextField for detector parameter 3
	QLineEdit* param4_edit;  //!< TextField for detector parameter 4
	QLineEdit* param5_edit;  //!< TextField for detector parameter 5

	QLabel* param1_desc;  //!< QLabel for descriptor parameter 1
	QLabel* param2_desc;  //!< QLabel for descriptor parameter 2
	QLabel* param3_desc;  //!< QLabel for descriptor parameter 3
	QLabel* param4_desc;  //!< QLabel for descriptor parameter 4
	QLabel* param5_desc;  //!< QLabel for descriptor parameter 5

	QLineEdit* param1_edit_desc;  //!< TextField for descriptor parameter 1
	QLineEdit* param2_edit_desc;  //!< TextField for descriptor parameter 2
	QLineEdit* param3_edit_desc;  //!< TextField for descriptor parameter 3
	QLineEdit* param4_edit_desc;  //!< TextField for descriptor parameter 4
	QLineEdit* param5_edit_desc;  //!< TextField for descriptor parameter 5

	QCheckBox* param1_boolean;
	QCheckBox* param2_boolean;

	QCheckBox* param1_boolean_desc;
	QCheckBox* param2_boolean_desc;

	/// provide user options like repeatability, activate/deactivate non-maximal
	/// suppression, image decimation, step-by-step playback of images.
	QCheckBox* homography_enable;  //!< Checkbox to activate homography based
	//! repeatability
	QLineEdit*
		inputHomogrpahyPath;  //!< Text field for the homographies folder path
	string homography_path;  //!< string which stores the folder path of the
	//! homographies from the text field
	QPushButton* browseHomography;  //!< button to browse for the homography
	//! containing folder
	bool homography_activated;  //!< indicates if homography based repeatability
	//! is activated or not

	/// widgets / options for tracking vision task
	QCheckBox* tracking_enable;  //!< Checkbox to activate tracking of keypoints
	bool tracking_activated;  //!< indicates if tracking is activated or not
	//!(currently not used)
	// int current_imageIndex_tracking;
	QPushButton*
		trackIt;  //!< button which tracks the key-points in subseuent frames
	vector<string> files_fullpath_tracking;  //!< vector which stores all
	//! files_paths for the selected
	//! monocular single dataset
	int tracking_image_counter;  //!< counter for moving forward in the dataset

	Tracker tracker_obj;  //<! tracker oject which calls the tracking method to
	// perform tracking

	/// tracker parameter variables
	QCheckBox* tracker_param1;  //! Checkbox for tracking parameter 1
	QCheckBox* tracker_param2;  //! Checkbox for tracking parameter 2
	QLabel* tracker_param3;  //! Label for tracking parameter 3
	QLabel* tracker_param4;  //! Label for tracking parameter 4
	QLabel* tracker_param5;  //! Label for tracking parameter 5
	QLabel* tracker_param6;  //! Label for tracking parameter 6

	QLineEdit* tracker_param1_edit;  //!< not used
	QLineEdit* tracker_param2_edit;  //!< not used
	QLineEdit* tracker_param3_edit;  //!< Text field for tracking parameter 3
	QLineEdit* tracker_param4_edit;  //!< Text field for tracking parameter 4
	QLineEdit* tracker_param5_edit;  //!< Text field for tracking parameter 5
	QLineEdit* tracker_param6_edit;  //!< Text field for tracking parameter 6

	/// image decimation options
	QLineEdit* decimateFactor;  //!< Text field to enter Decimation factor from
	//! the user

	/// visual odom parameters

	VisualOdometry visual_odom;  //!< visual odometry object to perform the
	//! vision task of estimating camera trajector
	//! for Monocular Datasets like KITTI
	QCheckBox* visual_odom_enable;  //!< Checkbox to activate/deactivate VO
	QLineEdit*
		inputFilePath3;  //!< Text field to store ground truth for VO task
	QPushButton* browse_button3;  //!< browse button to look for the ground
	//! truth Odometry file
	string file_path3;  //!< stores the ground truth for poses

	string calibration_file;  //!< string which stores the calibration fil path
	QLineEdit* inputCalibration;  //!< textbox which has the input calibration
	//! file path to be entered from the user
	QPushButton* browseCalibration;  //!< browse button to browse the
	//! calibration file path

	QLabel* visualOdom;  //!< Label which stores and displays the image for the
	//! VO output

	QFutureWatcher<cv::Mat>
		FutureWatcher;  //!< FutureWatcher for showing progress bar
	QProgressBar* progressBar;  //!< progress bar to show VO progress
	QDialog* vo_message_dialog;  //!< vo_message dialog to handle error case
	QGridLayout* vo_layout;  //!< gridlayout to hold widgets like buttons,
	//! textfields specific forVO task
	QLabel* vo_message_running;  //!< show the message to user to indicate that
	//! VO is currently running
	QLabel* VO_progress;  //!< Label to show VO progress

	/// Place Recognition Parameters
	// VisualOdometry visual_odom;         //!< visual odometry object to
	// perform the vision task of estimating camera trajector for Monocular
	// Datasets like KITTI
	QCheckBox* place_recog_enable;  //!< checkbox to enable Place Recognition
	//! vision task
	QLineEdit*
		training_set;  //!< text box to enter training dataset folder path
	QPushButton* browse_training;  //!< button to browse for the folder which
	//! has the training dataset
	string training_set_path;  //!< string which has the folder path for
	//! training dataset
	QLineEdit* testing_set;  //!< textbox for he user to enter the folder path
	//! for the testing dataset
	QPushButton* browse_testing;  //!< button to browse for the folder which has
	//! the testing dataset
	string testing_set_path;  //!< string which stores the testing dataset
	//! folder path

	vector<string> training_files_paths;  //!< vector to store the training
	//! images file paths
	vector<string>
		testing_files_paths;  //!< vector to store the testing images file paths

	QPushButton*
		perform_place_recog;  //!< button to start the place recognition task
	QPushButton* iterate_place_recog;

	QLabel* placeRecognition_results;  //!< label which displays the Place
	//! Recognition result
	PlaceRecognition* place_recog_obj;  //!< the main object which is reponsible
	//! for performing the PLace Recognition

	QLabel* place_recog_image;  //!< holds the place recognition image,
	//! currently NOT used
	QImage place_recog_qimage;  //!< image which has the current testing image
	//! which dispayed in GUI
	QLabel* place_recog_label;
	QGroupBox* placeRecogGroupBox;  //!< groupBox to hold the output widgets for
	//! the Place Recognition task

	bool placeRecog_clicked_flag;  //!< flag to check if the user pressed place
	//! recognition button
	int current_place_recog_index;  //!< counter to iterate over the testing
	//! dataset

	bool placeRecog_checked_flag;

	/// Rawlog support parameters
	// int rawlog_image_counter;

	/// main Feature Extraction params
	CFeatureExtraction fext;  //!< CFeatureExtraction object to hold/store all
	//! parameters for detectors/descriptors
	CFeatureList featsImage1,
		featsImage2;  //!< stores the features in image 1 and 2
	CImage img1, img2;  //!< stores image 1 and 2
	TDescriptorType
		desc_to_compute;  //!< stores the type of the descriptor to be computed

	int numFeats;  //!< stores the number of features to be computed

	// Detector OPTIONS

	/** FAST Options */
	struct FASTOptions
	{
		float threshold;  //!< default value = 20
		float min_distance;  //!< default value = 5
		bool non_max_suppresion;  //!< default value = true
		bool use_KLT_response;  //!< default value = true
	} fast_opts;

	/** KLT Options */
	struct KLTOptions
	{
		float threshold;  //!< default value = 0.1
		int radius;  //!< default value = 7
		float min_distance;  //!< default value = 7
		bool tile_image;  //!< default value = true
	} klt_opts;

	/** Harris Options */
	struct HarrisOptions
	{
		float threshold;  //!< default value = 0.005
		float k;  //!< default value = 0.04
		float sigma;  //!< default value = 1.5
		float radius;  //!< default value = 3
		float min_distance;  //!< default value = 100, not asked from user
		bool tile_image;  //!< default value = true
	} harris_opts;

	/** SIFT Options */
	struct SIFTOptions
	{
		float edge_threshold;  //!< default value = 0.04
		float threshold;  //!< default value = 10
	} SIFT_opts;

	/** SURF Options */
	struct SURFOptions
	{
		int hessianThreshold;  //!< default value = 600
		int nLayersPerOctave;  //!< default value = 4
		int nOctaves;  //!< default value = 2
		bool rotation_invariant;  //!< default value = true
	} SURF_opts;

	/** ORB Options */
	struct ORBOptions
	{
		int min_distance;  //!< default value = 0
		int n_levels;  //!< default value = 8
		float scale_factor;  //!< default value = 1.2
		bool extract_patch;  //!< default value = false
	} ORB_opts;

	// not providing option to choose descriptor_type and diffusivity, using
	// default values currently
	/** AKAZE Options */
	struct AKAZEOptions
	{
		// int descriptor_type
		int descriptor_size;  //!< default value = 0
		int descriptor_channels;  //!< default value = 3
		float threshold;  //!< default value = 0.001
		int nOctaves;  //!< default value = 4
		int nOctaveLayers;  //!< default value = 4
		// int  	diffusivity;
	} AKAZE_opts;

	/** LSD Options */
	struct LSDOptions
	{
		int scale;  //!< default value = 2
		int nOctaves;  //!< default value = 1
	} LSD_opts;

	// DESCRIPTOR OPTIONS
	/** SpinImagesOptions Options
	 */
	struct SpinImageOptions
	{
		int radius;  //!< default value = 20
		int hist_size_intensity;  //!< default value = 10
		int hist_size_distance;  //!< default value = 10
		float std_dist;  //!< default value = 0.4
		float std_intensity;  //!< default value = 20
	} spin_opts;

	/** PolarImagesOptions Options
	 */
	struct PolarImageOptions
	{
		int radius;  //!< default value = 20
		int bins_angle;  //!< default value = 8
		int bins_distance;  //!< default value = 6
	} polar_opts;

	/** LogPolarImagesOptions Options
	 */
	struct LogPolarOptions
	{
		int radius;  //!< default value = 30
		int num_angles;  //!< default value = 16
		float rho_scale;  //!< default value = 5

	} log_polar_opts;

	/** BLDOptions Options
	 */
	struct BLDOptions
	{
		int ksize_;  //!< default value = 1
		int reductionRatio;  //!< default value = 2
		int numOfOctave;  //!< default value = 7
		int widthOfBand;  //!< default value = 1
	} BLD_opts;

	/** LATCHOptions Options
	 */
	struct LATCHOptions
	{
		int bytes;  //!< default value = 32
		bool rotationInvariance;  //!< default value = true
		int half_ssd_size;  //!< default value = 3
	} LATCH_opts;

	/// FOR THE VISUALIZE DESCRIPTOR PART
   public:
	QLabel* images_static;  //!< For Image1, stores the image associated with
	//! the first class of descriptors (Spin, Polar, Log
	//! polar images, etc.)
	QLabel* images_static_sift_surf;  //!< For Image1, stores the image
	//! associated with the second class of
	//! descriptors (SIFT, SURF, ORB, etc.)

	QLabel* images_static2;  //!< For Image2, stores the image associated with
	//! the first class of descriptors (Spin, Polar, Log
	//! polar images, etc.)
	QLabel* images_static_sift_surf2;  //!< For Image2, stores the image
	//! associated with the second class of
	//! descriptors (SIFT, SURF, ORB, etc.)

	QLabel* plotInfo;  //!< stores the plot of distances of descriptors

	QGridLayout* desc_VisualizeGrid;  //!< grid layout to hold the widgets
	//! associated with the descriptor
	//! visualization part

	long numDesc1, numDesc2;  //!< stores the current size of the descriptors
	//! like for SIFT=128
	int cnt;  //!< counter to iterate over all the descriptors

	QLabel* images_label_coordinates;  //!< NOT USED
	QLabel* images_label_coordinates_sift_surf;  //!< NOT USED

	QLabel* featureMatched;  //!< Label to show feature matches information

	QLabel* images_plots_sift_surf;  //!< Label which holds the image to show
	//! the descriptor distance splot

	double mouse_x, mouse_y;  //!< stores the coordinates of the mouse click

	bool flag_descriptor_match;  //!< this is used to fix the overlaying of
	//! labels on top of each other.
	bool flag_read_files_bug;  //!< used to get rid of a bug while reading files

	/// variables for bug fixes

	bool evaluate_detector_clicked;  //!< variable to see if user pressed
	//! detector button
	bool evaluate_descriptor_clicked;  //!< variable to see if user pressed
	//! descriptor button
	bool visualize_descriptor_clicked;  //!< variable to see if user pressed
	//! visualize descriptor button

	bool activate_homogrphy_repeatability;  //!< variable to store if user has
	//! currently activated homography
	//! based repeatability or not

	//! <Parameters for Evaluations Characteritics

	double elapsedTime_detector;  //!< stores the time taken to compute detector
	//! key-points
	double elapsedTime_descriptor;  //!< stores the time taken to compute the
	//! descriptors around the key-points
	QLabel* detector_info;  //!< Label to show detector evaluation info
	QLabel* descriptor_info;  //!< Label to show descriptor evaluation info
	QLabel* descriptor_info2;  //!< Label to show descriptor evaluaiton info2
	QLabel* descriptor_info3;  //!< Label to show descriptor evaluation info3
	QLabel* evaluation_info;  //!< Label to show detector evaluation info
	double closest_dist;  //!< NOT USED

   signals:
	// None yet
	// void valueChanged(int newValue);
   public:
	/**
	 * MainWindow(QWidget) constructor which creates all the widgets and
	 * displays them on screen registers all the signals and slots for the
	 * buttons, comboboxes, etc.
	 * it sets the layout/sizes for all the widgets to be displayed in the QMain
	 * window
	 * @param window_gui asks a QWidget default=0
	 */
	explicit MainWindow(QWidget* parent = nullptr);

	/**
	 * readInputFormat() reads and stores the states of the user selections for
	 * different inputs like (currentInputIndex,
	 * detector_selected, descriptor selected, file_path1 and file_path2) and
	 * can be used by other functions when required
	 */
	void ReadInputFormat();

	/**
	 * makeAllDetectorParamsVisible(bool) makes all the detector parameters
	 * (QLabel, QLineEdits) in the main window either visible or hides them
	 * depending on the flag variable's value
	 * @param flag variable to store if all labels/text fields are to be made
	 * visible or hidden
	 */
	void makeAllDetectorParamsVisible(bool flag);

	/**
	 * makeAllDescriptorParamsVisible(bool) makes all the descriptors parameters
	 * (QLabel, QLineEdits) in the main window either visible or hides them
	 * depending on the flag variable's value
	 * @param flag variable to store if all labels/text fields are to be made
	 * visible or hidden
	 */
	void makeAllDescriptorParamsVisible(bool flag);

	/**
	 * fillDetectorInfo() function is used to fill in the required parameters
	 * for the chosen detector, the parameters entered
	 * by the user are read and stored in the appropriate variables, later these
	 * values are stored in the fext.options.DETECTOR.parameter field
	 * example fext.options.KLTOptions.min_distance,
	 * fext.options.harrisOptions.sigma, etc.
	 */
	void fillDetectorInfo();

	/**
	 * fillDescriptorInfo() function is used to fill in the required parameters
	 * for the chosen descriptor, the parameters entered
	 * by the user are read and stored in the appropriate variables, later these
	 * values are stored in the fext.options.DESCRIPTOR.parameter field
	 * example fext.options.SURFOptions.hessianThreshold,
	 * fext.options.SIFTOptions.edgeThreshold, etc.
	 */
	void fillDescriptorInfo();

	/**
	 * readFilesFromFolder(int) reads the files from a folder, the function is
	 * called when the user presses the next or previous button
	 * it iterates over the images present in the image dataset folder
	 * specified by the user
	 * the function works under the assumption both the stereo image datasets
	 * have the same number of images
	 * the function also supports rawlog images datasets, has implementations
	 * for both single and stereo datasets
	 * @param next_prev indicates 1 for next image and 0 for previous button
	 * being pressed
	 */
	void readFilesFromFolder(int next_prev);

	/**
	 * displayImagesWithoutDetector() function displays the images without
	 * detectors as QLabels, this function is called when NEXT or PREVIOUS
	 * buttons are pressed
	 */
	void displayImagesWithoutDetector();

	/**
	 * initializeParameters() initializes the content for the
	 * detector/descriptor QLabels/QLineEdits, it also sets the size of these
	 * labels and text fields
	 */
	void initializeParameters();

	/**
	 * findClosest function finds the closest point to the input (x,y)
	 * coordinates from the array of points (X[],Y[])
	 * the function is called when the user clicks on a key-point in the QLabel
	 * which has the QImage
	 * @param x input mouse click's x-coordinate
	 * @param y input mouse click's y-coordinate
	 * @param X input array of X coordinates which has all key-points'
	 * x-coordinates
	 * @param Y input array of Y coordinates which has all key-points'
	 * y-coordinates
	 * @param n input the size of the X[],Y[] array of key-points
	 * @return returns the index of the closest key-point to that clicked by the
	 * mouse
	 */
	int findClosest(double x, double y, double X[], double Y[], int n);

	/**
	 * drawLineLSD() draws the lines (LSD line detectors) in the image on the
	 * key-points for the image.
	 * @param img the input image on which the lines need to be drawn
	 * @param image_left_right  this indicates the image on which to draw the
	 * lines (0: left; 1:right)
	 */
	void drawLineLSD(Mat img, int image_left_right);

	/**
	 * findRepearability this function computes the repeatability for the
	 * selected key-point in the image
	 * it goes through all the images inside the selected folder and computes
	 * how many times the selected key-point
	 * repeat in all the images based on a spatial window of say +/-5 pixels
	 * @param mouse_x the x-coordinate of the selected key-point
	 * @param mouse_y the y-coordinate of the selected key-point
	 * @return
	 */
	string findRepeatability(float mouse_x, float mouse_y);

	/**
	 * findRepearabilityHomograhy this function computes the repeatability for
	 * the selected key-point in the image
	 * it goes through all the images inside the selected folder and computes
	 * how many times the selected key-point
	 * repeat in all the images based on a spatial window of say +/-5 pixels
	 * based on the homography transformation of the i'th
	 * frame w.r.t. to the first frame
	 * @param mouse_x the x-coordinate of the selected key-point
	 * @param mouse_y the y-coordinate of the selected key-point
	 * @return
	 */
	string findRepeatabilityHomography(float mouse_x, float mouse_y);

	/**
	 * checkIfSamePoint this checks if the point (x2,y2) appears within +/-
	 * threshold pixels around the point (x,y)
	 * this function is mainly used by repeatability computation method
	 * @param x input point x-coordinate
	 * @param y input point y-coordinate
	 * @param x2 input point x-coordinate
	 * @param y2 input point y-coordinate
	 * @param threshold threshold
	 * @return
	 */
	bool checkIfSamePoint(float x, float y, float x2, float y2, int threshold);

	/**
	 * falsePositivesNegatives function computes the false positives and
	 * negatives and returns the string representation of the
	 * result to be displayed as the text for evaluation metric for the
	 * descriptor selected. Mainly used for stereo images only
	 */
	string falsePositivesNegatives();

	/**
	 * showEvaluation this function exists to show the evaluation info, function
	 * created just for simplicity
	 * @param mode variable not currently being used
	 */
	void showEvaluation(int mode);

	/**
	 * trackKeyPoint function finds the corresponding key-point from the first
	 * image in the second image
	 * it basically finds the new position of the key-point in the second image
	 * to track it
	 * THIS FUNCTION IS CURRENTLY NOT BEING USED
	 * @param img_org the first image which has the initial key-point
	 * @param img_test  the seconds image on which the key-point needs to be
	 * found out
	 * @param feat_test the key-points from the second-image, the corresponding
	 * key-point is one of these key-points
	 * @param org_x the x-coordinate of the key-point to be tracked from the
	 * first image
	 * @param org_y the y-coordinate of the key-point to be tracked from the
	 * first image
	 * @return
	 */
	Point trackKeyPoint(CImage img_org, CImage img_test, int x, int y);

	/**
	 * initializeTrackerParams this function initializes all the widgets
	 * (labels, textboxes, etc.) associated with the tracker
	 */
	void initializeTrackerParams();

	/**
	 * makeVisualOdomParamsVisible this function makes all the widgets
	 * (parameters) associated with the tracker visible/hidden
	 * @param flag stores if the variables need to hidden/visible
	 */
	void makeTrackerParamVisible(bool flag);

	/**
	 * makeVisualOdomParamsVisible this function makes all the widgets
	 * (parameters) associated with the visual odometry visible/hidden
	 * @param flag stores if the variables need to hidden/visible
	 */
	void makeVisualOdomParamsVisible(bool flag);

	/**
	 * makeGraphsVisible this function makes all the widgets (parameters)
	 * associated with the visualiztion of descriptor visible/hidden
	 * @param flag stores if the variables need to hidden/visible
	 */
	void makeGraphsVisible(bool flag);

	/**
	 * makeVisionOptionsVisible this function makes all the widgets (check
	 * boxes for VO, tracking and homography) Vision tasks visible/hidden
	 * @param flag stores if the variables need to hidden/visible
	 */
	void makeVisionOptionsVisible(bool flag);

	/**
	 * makeHomographyParamsVisible this function makes all the widgets
	 * (parameters) associated with the homography based repeatability
	 * visible/hidden
	 * @param flag stores if the variables need to hidden/visible
	 */
	void makeHomographyParamsVisible(bool flag);

	/**
	 * makePlaceRecognitionParamVisible function makes the widgets specific to
	 * Place Recognition visible or hidden based on the flag
	 * @param flag the state of the flag indicates if the widgets are currently
	 * hidden or visible
	 */
	void makePlaceRecognitionParamVisible(bool flag);

	/**
	 * store_Training_TestingSets function stores the training and testing
	 * images paths in the respective variables of vector<string> type
	 */
	void store_Training_TestingSets();

	/**
	 * displayVector function is used to display the contents of the given
	 * vector of strings, currently NOT used, only meant for debugging code
	 * @param paths
	 */
	void displayVector(vector<string> paths);

	/**
	 * readRawlofFiles stores the base folder of the rawlog files containing the
	 * images into appropriate variables in the GUI
	 * @param rawlog
	 */
	void readRawlogFiles(string rawlog);

	/**
	 * getImageDir this simply returns the base image Dir by breaking it based
	 * on the last '/' character to separate the image
	 * @param path
	 * @return the required base folder storing the images
	 */
	string getImageDir(string path);

   private slots:

	/**
	 * Mouse_current_pos() slot function simply used to test the functionality
	 * of MouseEvents, not currently used
	 */
	void Mouse_current_pos();

	/**
	 * Mouse_Pressed() slot function called when the user clicks on a particular
	 * point in the QLabel for image 1 in the GUI, it finds the closest keypoint
	 * in image1
	 * and shows the corresponding best image in image2, it reads the
	 * descriptors stored by the on_button_generate_clicked() from global
	 * variables
	 * it then displays the descriptors from the single/stereo images onto the
	 * GUI
	 * also shows the plot of all descriptor distances in image2 to the
	 * descriptor (around key-point) clicked by the user
	 */
	void Mouse_Pressed();

	/**
	 * Mouse_left() slot function called when mouse leaves the QLabel on which
	 * this function is registeres as slot, not currently used
	 */
	void Mouse_left();

   public slots:

	/**
	 * on_browseTraining_clicked is called when the user wants to browse for the
	 * folder which has the training images dataset
	 */
	void on_browseTraining_clicked();

	/**
	 * on_browseTesting_clicked is called when the user wants to browse for the
	 * folder which has the testing images dataset
	 */
	void on_browseTesting_clicked();

	/**
	 * onPlaceRecogChecked function is called when the user checks/ticks or
	 * unchecks the checkbox, this makes the place recognition specific
	 * widgets hidden or visible based on the checkbox state
	 * @param state
	 */
	void onPlaceRecogChecked(int state);

	/**
	 * on_place_recog_clicked function is called when the user clicks on the
	 * Train Place Recognition button to train the model for the place recog
	 * vision task
	 */
	void on_place_recog_clicked();

	/**
	 * on_place_recog_clicked_iterate function is called when the user clicks on
	 * the Recognize Place Recognition button to recognize new places in the
	 * test set
	 */
	void on_place_recog_clicked_iterate();
	/**
	 * onVisualOdomChecked this function makes the parameters visible / hidden
	 * based on the state of the checkbox "Perform Visual Odometry"
	 * @param state holds if checkbox is ticked or not
	 */
	void onVisualOdomChecked(int state);

	/**
	 * on_trackIt_clicked function is called when the user clicks on the trackIt
	 * button to iterate over subsequent frames
	 * this method tracks the key-points from the first image in the subsequent
	 * image
	 * CURRENTLY UNDER CONSTRUCTION
	 */
	void on_trackIt_clicked();

	/**
	 * on_browse_button_clicked3 this function is called when the user needs to
	 * browse for the homogrphy ground truth files for the Grafitti
	 * dataset for the task of homography based repeatability, the user needs to
	 * select the foler for the appropriate homographyoes. The homographies
	 * need to be in a text files and separated from the images, only text files
	 * in the folder should be there, no images should be present
	 */
	void on_browse_homography_clicked3();

	/**
	 * on_button_generate_clicked() is called when user clicks on visualize
	 * descriptor.
	 * the function computes and stores the visualizations in memory to be later
	 * used when the user clicks on the QLabel to show the descriptors.
	 * the function populates all the descriptors in the left and right images
	 * in appropriate variables and also stores the which of the descriptors in
	 * image1
	 * match best to those in image2.
	 */
	void on_button_generate_clicked();

	/**
	 * button_close_clicked() simply closes the wondow
	 */
	void button_close_clicked();

	/**
	 * on_detector_button_clicked() function shows the performance of the
	 * selected detector on the input image/dataset selected by the user
	 * detected key-points are shown on the images which are rendered as QLabels
	 * For the case when user selects SingleImage/StereoImage Dataset, this
	 * calls the readFilesFromFolder(int) function which stores the file names
	 * for all images present in the folder
	 * this function also calls fillDetectorInfo to populate detector parameters
	 * the performance metric displayed is in terms of repeatability, dispersion
	 * of image, computational cost, number of found points, etc.
	 * */
	void on_detector_button_clicked();

	/**
	 * on_descriptor_button_clicked() slot function is called when the user
	 * clicks the evaluate Descriptor button, it computes the selected
	 * descriptor from the QComboBox
	 * for the image and stores the descriptor in the featsImage1 and
	 * featsImage2 variables.
	 * the function calls the fillDescriptorInfo() function which fills the
	 * parameters for the selected descriptor
	 */
	void on_descriptor_button_clicked();

	/**
	 * on_browse_button_clicked() slot function browses for the image files that
	 * the user can select or the directory that the user can select for the
	 * first image/folder
	 * it stores the file paths in the appropriate global variables which are
	 * then used by other functions to read the images from
	 */
	void on_browse_button_clicked();

	/**
	 * on_browse_button_clicked2() slot function browses for the image files
	 * that the user can select or the directory that the user can select for
	 * the second image/folder
	 * it stores the file paths in the appropriate global variables which are
	 * then used by other functions to read the images from
	 */
	void on_browse_button_clicked2();

	/**
	 * on_browse_button_clicked3 this function is called when the user needs to
	 * browse for the ground truth files for the KITTI
	 * visual odometry, the user needs to select the files for the appropriate
	 * sequence like 00.txt, 01.txt, etc..
	 */
	void on_browse_button_clicked3();

	/**
	 * on_descriptor_choose() slot function is called when the user selects the
	 * descriptor from the list of available descriptors from the QComboBox
	 * the function populates the QLabels and the QLineEdits for the selected
	 * descriptor option, it also hides the QLabels and QLineEdits that are not
	 * required
	 * @param choice stores the descriptor chosen by the user
	 */
	void on_descriptor_choose(int choice);

	/**
	 * on_detector_choose() slot function is called when the user selects the
	 * detector from the list of available detectors from the QComboBox
	 * the function populates the QLabels and the QLineEdits for the selected
	 * detector option, it also hides the QLabels and QLineEdits that are not
	 * required
	 * @param choice stores the detector chosen by the user
	 */
	void on_detector_choose(int choice);

	/**
	 * on_file_input_choose(int) slot function is called when the user selects
	 * the type of the input to be used from the QCombo
	 * @param choice stores the type of input selected, 0: single image, 1:
	 * stereo iamge, 2: image raw log file, 3: single image dataset, 4: stereo
	 * image dataset
	 */
	void on_file_input_choose(int choice);

	/**
	 * this function is for enabling homogrphy option for repeatability
	 * @param state
	 */
	void onHomographyChecked(int state);

	/**
	 * on_next_button_clicked() function is called when the user clicks on the
	 * next button t oiterate over the images in the dataset
	 * this function calls the readFilesFromFolder(int) and
	 * displayImagesWithoutDetector() functions
	 */
	void on_next_button_clicked();

	/**
	 * on_prev_button_clicked() function is called when the user clicks on the
	 * previous button to iterate over the images in the dataset
	 * this function calls the readFilesFromFolder(int) and
	 * displayImagesWithoutDetector() functions
	 */
	void on_prev_button_clicked();

	/**
	 * on_sample_clicked() slot function decimates the image by the decimation
	 * factor entered by the user
	 * e.g, entering 0.1 enlarges the image to 110% and -0.2 reduces it to 80%
	 * of its actual resolution
	 */
	void on_sample_clicked();

	/**
	 * on_generateVisualOdometry_clicked this function is called when the user
	 * clicks on the generate visual odometry button, this
	 * instantiates the visual odometry class and calls the required function
	 * which stores the output of the visual odomtery result
	 * and finally show the output of the KITTI odometry dataset result
	 */
	void on_generateVisualOdometry_clicked();

	/**
	 * on_browse_calibration_clicked function gets the calibration parameters
	 * the focal length and the principal point from the
	 * file specified
	 */
	void on_browse_calibration_clicked();

	/**
	 * onTrackingEnabled function is called when the user checks / enables the
	 * tracking mode, after checking this, the user has the option
	 * to track the key-points in the subsequent frames by clickin on the
	 * trackIt button to iterate over the images and view the tracking results
	 * one by one in the subsequent frames
	 * @param state this holds the state of the checkbox being ticked or
	 * unticked
	 */
	void onTrackingEnabled(int state);

	/**
	 * updateVOProgress Slot function called to change a label, currently
	 * FUNCITON NOT USED
	 */
	void updateVOProgress();

	/**
	 * slot_function used for progress bar visualization
	 */
	void slot_finished();
};

#endif  // MAINWINDOW_H
