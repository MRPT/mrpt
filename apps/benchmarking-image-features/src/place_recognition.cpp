//
// Created by raghavender on 02/08/17.
//

#include "place_recognition.h"

PlaceRecognition::PlaceRecognition(vector<string> training_paths,
                                   vector<string> testing_paths,

                                   TDescriptorType desc_to_compute,
                                   int descriptor_selected,
                                   int numFeats)
{
    this->testing_paths = testing_paths;
    this->training_paths = training_paths;
    //this->fext = fext ;
    this->desc_to_compute = desc_to_compute;
    this->numFeats = numFeats;
    this->descriptor_selected = descriptor_selected;
}
void PlaceRecognition::computeLabels(vector<string> file_paths, int counts[NUM_CLASSES], int *labels)
{
    for(int i=0 ; i<NUM_CLASSES ; i++ )
        counts[i] = 0;
    for(int i=0; i<file_paths.size() ; i++)
    {
        if(file_paths.at(i).find("rPA") != std::string::npos) {
            labels[i] = 1;
            counts[0]++;
        }
        else if(file_paths.at(i).find("rCR") != std::string::npos) {
            labels[i] = 2;
            counts[1]++;
        }
        else if(file_paths.at(i).find("rBO") != std::string::npos) {
            labels[i] = 3;
            counts[2]++;
        }
        else if(file_paths.at(i).find("rKT") != std::string::npos) {
            labels[i] = 4;
            counts[3]++;
        }
        else if(file_paths.at(i).find("rEO") != std::string::npos) {
            labels[i] = 5;
            counts[4]++;
        }
    }

}
/**
 *
 * @param feats_testing this has the feature vector of size 100 keypoints * descriptor size for a single image
 * @param training_words this has the entire training_words inside the vector array
 * @param training_word_labels this has all the training_words labels respectively
 * @param total_vocab_size this is the total number of words/descriptors
 * @return
 */
int PlaceRecognition::predictLabel(CFeatureList feats_testing, vector<float> *training_words, int *training_word_labels, int total_vocab_size)
{
    cout << "in the function predictLabel " << endl;

    int feats_size = feats_testing.size();

    int labels[feats_size];
    double sum=0;
    double min=99999;
    for(int i=0 ; i<feats_size ; i++) // feat_size is the size of the descriptor
    {
        sum = 0;
        vector<float> temp_feat;
        temp_feat = feats_testing.getByID(i).get()->descriptors.SURF;
        long descriptor_size = temp_feat.size();

        for(int j=0 ; j<total_vocab_size ; j++)
        {
            double temp_sum = 0;
            //long descriptor_size = training_words[0].size();

            cout << descriptor_size << " desrcip" << endl;

            for(int k=0;k<descriptor_size ; k++)
            {
                temp_sum = temp_sum + pow( (temp_feat.at(k) - training_words[j].at(k)) , 2 );

                //cout << temp_feat.at(k) - training_words[j].at(k) << " result" << endl;
            }
            cout << temp_sum  << " temp_sum JJJJ =" << j  << " vocab size " << total_vocab_size   << endl;
            /// computes the best descriptor match with that in the training set.
            if(temp_sum < min)
            {
                cout << temp_sum << " temp_sum " << min << " MIN " << endl;
                labels[i] = training_word_labels[j];
                min = temp_sum;
            }
            cout << " printing_ min " << min << endl;

        }// iterates over each key-point in the training images / dataset

    }// end of outter loop iterates over each key-point

    cout << "outside the for loop" << endl;
    for (int i=0 ; i<feats_size ; i++)
    {
        cout << labels[i] << " " ;
    }
    cout << endl;

    cout << "finished the function predictLabel " << endl;
    return 2;
}


string PlaceRecognition::startPlaceRecognition(CFeatureExtraction fext)
{

    int len_training = training_paths.size();
    int len_testing = testing_paths.size();

    CImage training[len_training];
    CImage testing[len_testing];
    CFeatureList feats_training[len_training];
    CFeatureList feats_testing[len_testing];

    ofstream training_file;
    ofstream testing_file;

    CTicTac feature_time ;
    feature_time.Tic();


    cout << len_training <<" train "  << len_testing << " test"  << endl;
    for(int i=0 ; i < len_training ; i++)
    {
        training[i].loadFromFile(training_paths.at(i));
        fext.detectFeatures(training[i],feats_training[i],0,numFeats);
        fext.computeDescriptors(training[i], feats_training[i], desc_to_compute);
        //cout << (int) feats_training[i].getByID(0).get()->descriptors.ORB.at(0) << " ";
        //cout << feats_training[i].getByID(0).get()->descriptors.SURF.at(0) << " ";
        //cout << feats_training[i].getByID(0).get()->descriptors.LATCH.at(0) << " ";
        ///feats_training[i].getByID(j).get()->descriptors.SURF;
    }

    for(int i=0 ; i < len_testing ; i++)
    {
        testing[i].loadFromFile(testing_paths.at(i));
        fext.detectFeatures(testing[i],feats_testing[i],0,numFeats);
        fext.computeDescriptors(testing[i], feats_testing[i], desc_to_compute);
    }

    cout << "Feature Time (key-points+description): " << feature_time.Tac() << endl;
    // stores the number of i'th class instances at the position index 'i'
    int training_count[NUM_CLASSES];
    int testing_count[NUM_CLASSES];

    // stores the labels for the i'th image instance for training and testing images
    int training_labels[len_training];
    int testing_labels[len_testing];

    /// computing the labels for each image here, REDUCE THE FOLLOWING TO n CLASSES AND MAKE A FUNCTION WHICH DOES THE LABEL ASSIGNMENT
    CTicTac label_time;
    label_time.Tic();

    computeLabels(training_paths, training_count, training_labels );
    computeLabels(testing_paths, testing_count, testing_labels );

    cout << " Label computation time : " << label_time.Tac() << endl;


    //Mat training_words;
    //training_words.push_back(feats_training[0].getByID(0).get()->descriptors.SURF);

    int len_train_words=0;
    int len_test_words=0;
    for(int i=0 ; i<len_training ; i++)
        len_train_words += feats_training[i].size();

    //vector<uint8_t> training_words1[numFeats*len_training];
    //vector<float> training_words2[numFeats*len_training];
    //int training_word_labels[numFeats*len_training];

    vector<uint8_t> training_words1[len_train_words];
    vector<float> training_words2[len_train_words];
    int training_word_labels[len_train_words];

    CTicTac training_time;
    training_time.Tic();
    training_file.open ("training_images_features.txt");
    training_file.clear();
    //training_file << "Writing this to a file.\n";

    cout << "len training " << len_training << endl;
    cout << "feats_training[i].size() " << feats_training[99].size() <<endl;
    cout << "descriptor selected : " << descriptor_selected << endl ;

    int kount=0;
    for (int i=0 ; i<len_training ; i++)
    {
        training_file << feats_training[i].size();
        for (int j = 0; j < feats_training[i].size(); j++, kount++)
        {
            if (descriptor_selected == 0)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.SIFT;
                training_words1[kount] = feats_training[i].getByID(j).get()->descriptors.SIFT;
                training_word_labels[kount] = training_labels[i];
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << (int)temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.SIFT.data() << endl; //!< SIFT descriptors
            }
            else if (descriptor_selected == 1)
            {
                vector<float> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.SURF;
                training_words2[kount] = feats_training[i].getByID(j).get()->descriptors.SURF;
                training_word_labels[kount] = training_labels[i];

                for(int k=0 ; k < temp_feat.size() ; k++) {
                    training_file << temp_feat.at(k) << " ";
                    //training_file << training_words2[kount].at(k) << " " ;
                }

                //training_file << feats_testing[i].getByID(j).get()->descriptors.SURF.data() << endl; //!< SURF descriptors
            }
            else if (descriptor_selected == 2)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.SpinImg.size(); //!< Intensity-domain spin image descriptor
            else if (descriptor_selected == 3)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.PolarImg.size(); //!< Polar image descriptor
            else if (descriptor_selected == 4)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.LogPolarImg.size(); //!< Log-Polar image descriptor
            else if (descriptor_selected == 5)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.ORB;
                training_words1[kount] = feats_training[i].getByID(j).get()->descriptors.ORB;
                training_word_labels[kount] = training_labels[i];
                for(int k=0 ; k < temp_feat.size() ; k++)
                {
                    training_file << (int)temp_feat.at(k) << " ";
                    //cout << temp_feat.at(k) << " " ;

                    //cout << feats_training[i].getByID(j).get()->descriptors.ORB.at(k) << " ";
                }
                //training_file << feats_testing[i].getByID(j).get()->descriptors.ORB.data() << endl; //!< ORB image descriptor
            }
            else if (descriptor_selected == 6)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.BLD;
                training_words1[kount] = feats_training[i].getByID(j).get()->descriptors.BLD;
                training_word_labels[kount] = training_labels[i];
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << (int)temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.BLD.data() << endl; //!< BLD image descriptor
            }
            else if (descriptor_selected == 7)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.LATCH;
                training_words1[kount] = feats_training[i].getByID(j).get()->descriptors.LATCH;
                training_word_labels[kount] = training_labels[i];
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << (int)temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.LATCH.data() << endl; //!< LATCH image descriptor
            }
            //training_file << feats_testing[i].getByID(i).get()->descriptors.SURF;
            //cout << feats_testing[i].getFeatureX(i) << " , " << feats_testing[i].getFeatureY(i);
            training_file << " #" << training_labels[i] << " $" << training_word_labels[kount] << endl;

        } // end of inner for loop for number of key-points


    } // end of outer for loop for number of images





    cout << " elapsed time for training dataset: " << training_time.Tac() << " and kount is : " << kount << endl;

    CTicTac testing_time;
    testing_time.Tic();



    /// now extracting features for Place Recognition for testing dataset
    testing_file.open ("testing_images_features.txt");
    testing_file.clear();
    cout << "len testing " << len_testing << endl;


    for(int i=0 ; i<2 ; i++)
    {
        int predicted = predictLabel(feats_testing[i], training_words2, training_word_labels,len_train_words);
    }



    cout << " elapsed time for testing dataset: " << testing_time.Tac() << endl;

    //cout << training_words1[0].at(31) << endl;
    cout << training_words1[0].data() << endl;
    cout << training_words1[0].size() << endl;
    cout << training_words1->size() << endl;

    //cout << training_words2[0].at(31) << endl;
    cout << training_words2[0].data() << endl;
    cout << training_words2[0].size() << endl;
    cout << training_words2->size() << endl;
    cout << len_training * numFeats << endl ;


    training_file.close();
    testing_file.close();
    cout << "after reading testing images" << endl ;


    /// use a bag of words framework here

    return "";

}



/*for (int i=0, kount=0 ; i<len_testing ; i++)
    {
        testing_file << feats_testing[i].size();
        for (int j = 0; j < feats_testing[i].size(); j++, kount++)
        {
            if (descriptor_selected == 0)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.SIFT;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    testing_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.SIFT.data() << endl; //!< SIFT descriptors
            }
            else if (descriptor_selected == 1)
            {
                vector<float> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.SURF;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    testing_file << temp_feat.at(k) << " " ;


                //training_file << feats_testing[i].getByID(j).get()->descriptors.SURF.data() << endl; //!< SURF descriptors
            }
            else if (descriptor_selected == 2)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.SpinImg.size(); //!< Intensity-domain spin image descriptor
            else if (descriptor_selected == 3)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.PolarImg.size(); //!< Polar image descriptor
            else if (descriptor_selected == 4)
                ;//training_file << feats_testing[i].getByID(j).get()->descriptors.LogPolarImg.size(); //!< Log-Polar image descriptor
            else if (descriptor_selected == 5)
            {


                vector<uint8_t> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.ORB;
                //testing_words1[kount] = feats_testing[i].getByID(j).get()->descriptors.ORB;
                training_word_labels[kount] = training_labels[i];
                for(int k=0 ; k < temp_feat.size() ; k++)
                {
                    training_file << (int)temp_feat.at(k) << " ";
                    //cout << temp_feat.at(k) << " " ;

                    //cout << feats_training[i].getByID(j).get()->descriptors.ORB.at(k) << " ";
                }





                vector<uint8_t> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.ORB;

                for(int k=0 ; k < temp_feat.size() ; k++)
                    testing_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.ORB.data() << endl; //!< ORB image descriptor
            }
            else if (descriptor_selected == 6)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.BLD;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    testing_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.BLD.data() << endl; //!< BLD image descriptor
            }
            else if (descriptor_selected == 7)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_testing[i].getByID(j).get()->descriptors.LATCH;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    testing_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.LATCH.data() << endl; //!< LATCH image descriptor
            }
            //training_file << feats_testing[i].getByID(i).get()->descriptors.SURF;
            //cout << feats_testing[i].getFeatureX(i) << " , " << feats_testing[i].getFeatureY(i);

        }

        testing_file << endl;
        testing_file << testing_labels[i] << endl;
    }*/