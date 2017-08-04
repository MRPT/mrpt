//
// Created by raghavender on 02/08/17.
//

#ifndef MRPT_PLACERECOGNITION_H
#define MRPT_PLACERECOGNITION_H

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

#include <opencv2/line_descriptor.hpp>

#include <opencv2/plot.hpp>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>


#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#define NUM_CLASSES 5

using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt;

using namespace cv;
using namespace std;

class PlaceRecognition {

public:
    vector<string> training_paths;
    vector<string> testing_paths;
    //CFeatureExtraction fext;
    TDescriptorType desc_to_compute;
    int numFeats;
    int descriptor_selected;

public:
    PlaceRecognition(vector<string> training_paths,
                     vector<string> testing_paths,

                     TDescriptorType desc_to_compute,
                     int descriptor_selected,
                     int numFeats);
    //~PlaceRecognition();

    string startPlaceRecognition(CFeatureExtraction fext);

    void computeLabels(vector<string> file_paths, int counts[NUM_CLASSES], int *labels);

    int predictLabel(CFeatureList feats_testing, vector<float> *training_words, int *training_word_labels, int total_vocab_size);



};


#endif //MRPT_PLACERECOGNITION_H
