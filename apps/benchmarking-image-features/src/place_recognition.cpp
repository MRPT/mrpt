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
	FILE: place_recognition.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

#include "place_recognition.h"

using mrpt::system::CTicTac;

/************************************************************************************************
 *								Place Recognition Constructor *
 ************************************************************************************************/
PlaceRecognition::PlaceRecognition(
	vector<string> training_paths, vector<string> testing_paths,

	TDescriptorType desc_to_compute, int descriptor_selected, int numFeats)
{
	this->testing_paths = testing_paths;
	this->training_paths = training_paths;
	// this->fext = fext ;
	this->desc_to_compute = desc_to_compute;
	this->numFeats = numFeats;
	this->descriptor_selected = descriptor_selected;

	current_index_test_image = 0;
	trained_flag = false;
	training_file_written_flag = false;

	len_training = training_paths.size();
	len_testing = testing_paths.size();

	correct = 0;
	incorrect = 0;

	feats_training = new CFeatureList[len_training];
	feats_testing = new CFeatureList[len_testing];

	training = new CImage[len_training];
	testing = new CImage[len_testing];
}

/************************************************************************************************
 *								Compute Labels function *
 ************************************************************************************************/
void PlaceRecognition::computeLabels(
	vector<string> file_paths, int counts[NUM_CLASSES], int* labels)
{
	// initialize all labels to zero.
	for (int i = 0; i < NUM_CLASSES; i++) counts[i] = 0;

	/// assign labels based on the label/place name that appears in the file
	/// name
	for (unsigned int i = 0; i < file_paths.size(); i++)
	{
		if (file_paths.at(i).find("rPA") != std::string::npos)
		{
			labels[i] = 1;
			counts[0]++;
		}
		else if (file_paths.at(i).find("rCR") != std::string::npos)
		{
			labels[i] = 2;
			counts[1]++;
		}
		else if (file_paths.at(i).find("rBO") != std::string::npos)
		{
			labels[i] = 3;
			counts[2]++;
		}
		else if (file_paths.at(i).find("rKT") != std::string::npos)
		{
			labels[i] = 4;
			counts[3]++;
		}
		else if (file_paths.at(i).find("rEO") != std::string::npos)
		{
			labels[i] = 5;
			counts[4]++;
		}
	}
}

/************************************************************************************************
 *								Predict Label2 function *
 ************************************************************************************************/
int PlaceRecognition::predictLabel2(
	CFeatureList* feats_testingAll, vector<uint8_t>* training_words,
	int* training_word_labels, int total_vocab_size, int current_image)
{
	CFeatureList feats_testing = feats_testingAll[current_image];
	int feats_size = feats_testing.size();

	/// PUT A CONDITION IF feats_size =0 OUTPUT A RANDOM CLASS INSTEAD OF GOING
	/// THROUGH BLANK STUFF, actually kinda doing that only currently
	int labels[feats_size];
	for (int i = 0; i < feats_size; i++) labels[i] = 0;

	/// following for loop iterates over all key-points in the given
	/// feats_testingAll CFeatureList and assigns a place label
	/// associated with the descriptor around each key-point
	for (int i = 0; i < feats_size;
		 i++)  // feat_size is the number of key-points
	{
		double min = std::numeric_limits<double>::max();  // 99999;
		vector<uint8_t> temp_feat;

		if (descriptor_selected == 5)
			temp_feat = feats_testing.getByID(i).get()->descriptors.ORB;
		else if (descriptor_selected == 6)
			temp_feat = feats_testing.getByID(i).get()->descriptors.BLD;
		else if (descriptor_selected == 7)
			temp_feat = feats_testing.getByID(i).get()->descriptors.LATCH;

		long descriptor_size = temp_feat.size();
		/// following for loop iterates over the entire vocabulary in the
		/// training images and computes the best matching descriptor
		/// from the training descriptors for each i'th key-point
		for (int j = 0; j < total_vocab_size; j++)
		{
			double temp_sum = 0;
			if (training_words[j].size() != 0)
			{
				for (int k = 0; k < descriptor_size; k++)
				{
					/// Use abs if better results
					// temp_sum = temp_sum + abs((temp_feat.at(k) -
					// training_words[j].at(k)))
					temp_sum =
						temp_sum +
						pow((temp_feat.at(k) - (int)training_words[j].at(k)),
							2);
				}
				/// computes the best descriptor match with that in the training
				/// set.
				if (temp_sum < min)
				{
					labels[i] = training_word_labels[j];
					min = temp_sum;
				}
			}

		}  // iterates over each key-point in the training images / dataset

	}  // end of outter loop iterates over each key-point

	for (int i = 0; i < feats_size; i++) cout << labels[i] << " ";
	int predicted_label = findMax(labels, feats_size);
	return predicted_label;
}

/************************************************************************************************
 *								Predict Label function *
 ************************************************************************************************/
int PlaceRecognition::predictLabel(
	CFeatureList* feats_testingAll, vector<float>* training_words,
	int* training_word_labels, int total_vocab_size, int current_image)
{
	CFeatureList feats_testing = feats_testingAll[current_image];
	int feats_size = feats_testing.size();

	/// PUT A CONDITION IF feats_size =0 OUTPUT A RANDOM CLASS INSTEAD OF GOING
	/// THROUGH BLANK STUFF, actually kinda doing that only currently
	int labels[feats_size];
	for (int i = 0; i < feats_size; i++) labels[i] = 0;

	for (int i = 0; i < feats_size;
		 i++)  // feat_size is the number of key-points
	{
		double min = std::numeric_limits<double>::max();  // 99999;
		vector<float> temp_feat;
		if (descriptor_selected == 1)
			temp_feat = feats_testing.getByID(i).get()->descriptors.SURF;
		else if (descriptor_selected == 2)
			temp_feat = feats_testing.getByID(i).get()->descriptors.SpinImg;

		long descriptor_size = temp_feat.size();

		for (int j = 0; j < total_vocab_size; j++)
		{
			double temp_sum = 0;
			// long descriptor_size = training_words[0].size();

			if (training_words[j].size() != 0)
			{
				for (int k = 0; k < descriptor_size; k++)
				{
					/// Use abs if better results
					// temp_sum = temp_sum + abs((temp_feat.at(k) -
					// training_words[j].at(k)));
					temp_sum =
						temp_sum +
						pow((temp_feat.at(k) - training_words[j].at(k)), 2);
				}

				/// computes the best descriptor match with that in the training
				/// set.
				if (temp_sum < min)
				{
					labels[i] = training_word_labels[j];
					min = temp_sum;
				}
			}
		}  // iterates over each key-point in the training images / dataset
	}  // end of outter loop iterates over each key-point

	for (int i = 0; i < feats_size; i++) cout << labels[i] << " ";

	int predicted_label = findMax(labels, feats_size);
	return predicted_label;
}

/************************************************************************************************
 *								Find Max function *
 ************************************************************************************************/
int PlaceRecognition::findMax(int* labels, int feats_size)
{
	int temp_labels[NUM_CLASSES];
	for (int i = 0; i < NUM_CLASSES; i++) temp_labels[i] = 0;

	for (int i = 0; i < feats_size; i++) temp_labels[labels[i] - 1]++;

	/// find out maximum out of the temp_labels array
	int max = 0, pos = 0;
	for (int i = 0; i < NUM_CLASSES; i++)
	{
		if (temp_labels[i] > max)
		{
			max = temp_labels[i];
			pos = i;
		}
	}
	// +1 to indicate the actual class
	return (pos + 1);
}

/************************************************************************************************
 *								Start Function function *
 ************************************************************************************************/
string PlaceRecognition::startPlaceRecognition(CFeatureExtraction fext)
{
	ofstream training_file;

	CTicTac feature_time;
	feature_time.Tic();

	/// stores the labels for the i'th image instance for training and testing
	/// images
	int training_labels[len_training];
	int testing_labels[len_testing];

	/// The training model is built here all features are extracted in this
	/// part, takes 30 seconds for 900+900 images
	if (!trained_flag)
	{
		for (int i = 0; i < len_training; i++)
		{
			training[i].loadFromFile(training_paths.at(i));
			fext.detectFeatures(training[i], feats_training[i], 0, numFeats);
			fext.computeDescriptors(
				training[i], feats_training[i], desc_to_compute);
		}
		for (int i = 0; i < len_testing; i++)
		{
			testing[i].loadFromFile(testing_paths.at(i));
			fext.detectFeatures(testing[i], feats_testing[i], 0, numFeats);
			fext.computeDescriptors(
				testing[i], feats_testing[i], desc_to_compute);
		}
		trained_flag = true;
		feats_testing_org = feats_testing;

	}  /// end of if feature extraction flag (trained flag)

	CTicTac label_time;
	label_time.Tic();

	computeLabels(training_paths, training_count, training_labels);
	computeLabels(testing_paths, testing_count, testing_labels);

	int len_train_words;
	len_train_words = 0;
	// int len_test_words = 0;
	for (int i = 0; i < len_training; i++)
		len_train_words += feats_training[i].size();

	if (!training_file_written_flag)
	{
		training_words2 = new vector<float>[len_train_words];
		training_words1 = new vector<uint8_t>[len_train_words];
	}
	int training_word_labels[len_train_words];

	CTicTac training_time;
	training_time.Tic();
	training_file.open("training_images_features.txt");

	if (!training_file_written_flag)
	{
		training_file.clear();
		int kount = 0;
		for (int i = 0; i < len_training; i++)
		{
			training_file << feats_training[i].size();
			for (unsigned int j = 0; j < feats_training[i].size(); j++, kount++)
			{
				if (descriptor_selected == 0)
				{
					vector<uint8_t> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.SIFT;
					training_words1[kount] =
						feats_training[i].getByID(j).get()->descriptors.SIFT;
					training_word_labels[kount] = training_labels[i];
					for (unsigned int k = 0; k < temp_feat.size(); k++)
						training_file << (int)temp_feat.at(k) << " ";
				}
				else if (descriptor_selected == 1)
				{
					vector<float> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.SURF;
					training_words2[kount] =
						feats_training[i].getByID(j).get()->descriptors.SURF;
					training_word_labels[kount] = training_labels[i];

					for (unsigned int k = 0; k < temp_feat.size(); k++)
					{
						training_file << temp_feat.at(k) << " ";
					}
				}
				else if (descriptor_selected == 2)
				{
					vector<float> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.SpinImg;
					training_words2[kount] =
						feats_training[i].getByID(j).get()->descriptors.SpinImg;
					training_word_labels[kount] = training_labels[i];

					for (unsigned int k = 0; k < temp_feat.size(); k++)
					{
						training_file << temp_feat.at(k) << " ";
					}
				}
				else if (descriptor_selected == 3)
					;
				// //!< Polar image descriptor
				else if (descriptor_selected == 4)
					;
				// //!< Log-Polar image descriptor
				else if (descriptor_selected == 5)
				{
					vector<uint8_t> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.ORB;
					training_words1[kount] =
						feats_training[i].getByID(j).get()->descriptors.ORB;
					training_word_labels[kount] = training_labels[i];

					for (unsigned int k = 0; k < temp_feat.size(); k++)
					{
						int temp_var;  //= (int) temp_feat.at(k);
						temp_var = (int)training_words1[kount].at(k);
						training_file << temp_var << " ";
					}
				}
				else if (descriptor_selected == 6)
				{
					vector<uint8_t> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.BLD;
					training_words1[kount] =
						feats_training[i].getByID(j).get()->descriptors.BLD;
					training_word_labels[kount] = training_labels[i];
					for (unsigned int k = 0; k < temp_feat.size(); k++)
						training_file << (int)temp_feat.at(k) << " ";
				}
				else if (descriptor_selected == 7)
				{
					vector<uint8_t> temp_feat;
					temp_feat =
						feats_training[i].getByID(j).get()->descriptors.LATCH;
					training_words1[kount] =
						feats_training[i].getByID(j).get()->descriptors.LATCH;
					training_word_labels[kount] = training_labels[i];
					for (unsigned int k = 0; k < temp_feat.size(); k++)
						training_file << (int)temp_feat.at(k) << " ";
				}
				training_file << " #" << training_labels[i] << " $"
							  << training_word_labels[kount] << endl;

			}  // end of inner for loop for number of key-points
		}  // end of outer for loop for number of images

		training_file.close();
		// testing_file.close();
		this->training_words_org = training_words2;
		this->training_words_org2 = training_words1;
		this->training_word_labels_org = training_word_labels;
		training_word_labels_org = new int[kount];
		for (int i = 0; i < kount; i++)
		{
			training_word_labels_org[i] = training_word_labels[i];
		}

		this->total_vocab_size_org = len_train_words;
		this->training_file_written_flag = true;
	}  // end of writting training features to a file

	CTicTac testing_time;
	testing_time.Tic();

	/// now extracting features for Place Recognition for testing dataset
	int predicted_classes[len_testing];

	CTicTac time_prediction;
	time_prediction.Tic();

	int predicted_Label = 1;
	if (descriptor_selected == 1)
		predicted_Label = predictLabel(
			feats_testing_org, training_words_org, training_word_labels_org,
			total_vocab_size_org, current_index_test_image);
	else
		predicted_Label = predictLabel2(
			feats_testing_org, training_words_org2, training_word_labels_org,
			total_vocab_size_org, current_index_test_image);

	current_index_test_image++;

	/// use a bag of words kind of framework here
	predicted_classes[current_index_test_image] = predicted_Label;
	if (predicted_classes[current_index_test_image] ==
		testing_labels[current_index_test_image])
		correct++;
	else
		incorrect++;

	stringstream output;
	output << endl
		   << endl
		   << "PLACE RECOGNITION RESULTS " << endl
		   << endl
		   << "actual label : "
		   << findPlaceName(
				  testing_labels[current_index_test_image % len_testing])
		   << ".\n"
		   << endl
		   << " predicted label : " << findPlaceName(predicted_Label) << ".\n"
		   << endl
		   << " correct = " << correct << "  incorrect = " << incorrect << ".\n"
		   << " Current Accuracy: "
		   << 100.00 * (double)correct / (double)(incorrect + correct) << " % "
		   << endl
		   << " image " << current_index_test_image << " of " << len_testing
		   << endl;

	return output.str();
}

/************************************************************************************************
 *								find PlaceName function *
 ************************************************************************************************/
string PlaceRecognition::findPlaceName(int type)
{
	if (type == 1)
		return "PRINTING AREA";
	else if (type == 2)
		return "CORRIDOR";
	else if (type == 3)
		return "BASIC OFFICE";
	else if (type == 4)
		return "KITCHEN";
	else
		return "EXECUTIVE OFFICE";
}
