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
	FILE: place_recognition.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

#ifndef MRPT_PLACERECOGNITION_H
#define MRPT_PLACERECOGNITION_H

/// openCV includes
#include <mrpt/otherlibs/do_opencv_includes.h>

/// standard C++ includes
#include <iostream>
#include <ctype.h>
#include <algorithm>  // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

/// MRPT includes
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/img/CImage.h>

#define NUM_CLASSES 5

using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt;
using namespace cv;
using namespace std;
using mrpt::img::CImage;

class PlaceRecognition
{
   public:
	vector<string> training_paths;  //!< this holds the training image paths for
	//! the training dataset
	vector<string> testing_paths;  //!< this holds the testing image paths for
	//! the testing dataset
	TDescriptorType desc_to_compute;  //!< this holds the type of the descriptor
	//! which the user selects from the GUI
	int numFeats;  //!< the number of features which the user wants to extract
	//! from each image
	int descriptor_selected;  //!< the type of descriptor the user selects from
	//! the GUI

	vector<float>* training_words_org;  //!< the training words/descriptors
	//! extracted from the training dataset
	//! for descriptor SURF
	vector<uint8_t>* training_words_org2;  //!< the training words/descriptors
	//! extracted from the training
	//! dataset for descriptors SIFT,
	//! ORB,BLD,LATCH

	int* training_word_labels_org;  //!< this holds the training labels
	//! associated with each word in
	//! training_words_org / training_words_org2
	//! variables
	int total_vocab_size_org;  //!< the total number of words/descriptors
	//! extracted from the training dataset

	CFeatureList* feats_testing_org;  //!< this is pointer to all the features
	//! extracted from the testing images
	//! dataset
	int current_index_test_image;  //!< this is variable to iterate over the
	//! testing images when the user wants to
	//! perform place recognition over the testing
	//! dataset

	bool
		trained_flag;  //!< this flag is to perform feature extraction only once

	bool training_file_written_flag;  //!< this flag is to write the features to
	//! the files only once

	CFeatureList* feats_training2;
	CFeatureList* feats_testing2;

	/// stores the number of i'th class instances at the position index 'i'
	int training_count[NUM_CLASSES];  //!< this has the class specific counts
	//! for the occurrence of each class type
	//! in the training dataset
	int testing_count[NUM_CLASSES];  //!< this has the class specific counts for
	//! the occurrence of each class type in the
	//! testing dataset

	int len_training;  //!< the length of the training images dataset / number
	//! of images
	int len_testing;  //!< the length of the testing images dataset / number of
	//! images

	int correct;  //!< counter to count the current number of correct
	//! classifications of the place
	int incorrect;  //!< counter to count the current number of incorrect
	//! classifications of the place

	CFeatureList* feats_training;  //!< the features extracted from all the
	//! training images dataset
	CFeatureList* feats_testing;  //!< the features extracted from all the
	//! testing images dataset

	CImage* training;  //!< the images present in the training images dataset
	CImage* testing;  //!< the images present in the testing images dataset

	vector<float>* training_words2;  //!< holds training words/descriptors for
	//! all points in the training dataset for
	//! SURF
	vector<uint8_t>* training_words1;  //!< holds training words/descriptors for
	//! all points in the training dataset for
	//! SIFT,ORB,BLD,LATCH

   public:
	/**
	 * PlaceRecognition constructor which initializes the object that does place
	 * recognition.
	 * @param training_paths this has all the training file paths which is
	 * specified by the user in the text box from the GUI
	 * @param testing_paths this has all the testing file paths which is
	 * specified by the user in the text box from the GUI
	 * @param desc_to_compute this is the descriptor type which the user selects
	 * @param descriptor_selected the descriptor selected by the user
	 * @param numFeats number of key-points to be detected in each training
	 * image
	 */
	PlaceRecognition(
		vector<string> training_paths, vector<string> testing_paths,

		TDescriptorType desc_to_compute, int descriptor_selected, int numFeats);

	/**
	 * startPlaceRecognition function starts the place recognition code, it does
	 * the training by extracting features only the first time
	 * when the user clicks on perform place recognition button. training
	 * usually takes around 30 seconds for approx. 900 images
	 * @param fext this has the information about the type of detector and
	 * descriptor and the parameters associated with each detector/descriptor
	 * @return returns the place recognition results
	 */
	string startPlaceRecognition(CFeatureExtraction fext);

	/**
	 * this computes the labels for the given dataset, currently it is supported
	 * for doing place recognition only on the
	 * KTH IDOL Dataset available at http://www.cas.kth.se/IDOL/
	 * @param file_paths this has the training or the testing file paths from
	 * which the respective labels are computed
	 * @param counts this has the count of occurrence of each class type in the
	 * training/testing dataset
	 * @param labels this has the labels associated with each training/testing
	 * image
	 */
	void computeLabels(
		vector<string> file_paths, int counts[NUM_CLASSES], int* labels);

	/**
	 * predictLabel this function predicts the class based on the features
	 * extracted from a testing image, it only handles descriptors of type: SURF
	 * @param feats_testingAll this is a pointer to the entire testing dataset
	 * features
	 * @param training_words this has all the training descriptors present in
	 * the training dataset
	 * @param training_word_labels this has the labels associated with the
	 * training dataset descriptors
	 * @param total_vocab_size this is the total number of descriptors present
	 * in the training dataset, for KTH this number is around 70-80k
	 * @param current_image this is the image index of the testing image which
	 * needs to be tested currently
	 * @return returns the predicted class/place of the given test feature
	 */
	int predictLabel(
		CFeatureList* feats_testing, vector<float>* training_words,
		int* training_word_labels, int total_vocab_size, int current_image);

	/**
	 * predictLabel2 this function predicts the class based on the features
	 * extracted from a testing image, it only handles descriptors of type:
	 * SIFT, ORB, BLD and LATCH
	 * @param feats_testingAll this is a pointer to the entire testing dataset
	 * features
	 * @param training_words this has all the training descriptors present in
	 * the training dataset
	 * @param training_word_labels this has the labels associated with the
	 * training dataset descriptors
	 * @param total_vocab_size this is the total number of descriptors present
	 * in the training dataset, for KTH this number is around 70-80k
	 * @param current_image this is the image index of the testing image which
	 * needs to be tested currently
	 * @return returns the predicted class/place of the given test feature
	 */
	int predictLabel2(
		CFeatureList* feats_testing, vector<uint8_t>* training_words,
		int* training_word_labels, int total_vocab_size, int current_image);

	/**
	 * findMax function finds the element that occurs maximum number of times in
	 * the given labels array
	 * @param labels the array which holds the prediction of the key-point
	 * feature belonging to a particualr calss
	 * @param feats_size this is the number of key-points detected in each
	 * image, also the number of features in each image
	 * @return returns the class from labels which has the maximum number of
	 * occurrences
	 */
	int findMax(int* labels, int feats_size);

	/**
	 * findClassName function simply finds the Place name associated with the
	 * type of class, a simple mapping function
	 * @param type input to find the class of !
	 * @return returns the name of the string associated with the place type
	 */
	string findPlaceName(int type);
};

#endif  // MRPT_PLACERECOGNITION_H
