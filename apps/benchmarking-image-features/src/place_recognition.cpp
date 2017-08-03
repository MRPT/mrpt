//
// Created by raghavender on 02/08/17.
//

#include "place_recognition.h"

PlaceRecognition::PlaceRecognition(vector<string> training_paths,
                                   vector<string> testing_paths,
                                   CFeatureExtraction fext,
                                   TDescriptorType desc_to_compute,
                                   int descriptor_selected,
                                   int numFeats)
{
    this->testing_paths = testing_paths;
    this->training_paths = training_paths;
    this->fext = fext ;
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
string PlaceRecognition::startPlaceRecognition()
{

    int len_training = training_paths.size();
    int len_testing = testing_paths.size();

    CImage training[len_training];
    CImage testing[len_testing];
    CFeatureList feats_training[len_training];
    CFeatureList feats_testing[len_testing];

    ofstream training_file;
    ofstream testing_file;



    cout << len_training <<" train "  << len_testing << " test"  << endl;
    for(int i=0 ; i < len_training ; i++)
    {
        training[i].loadFromFile(training_paths.at(i));
        fext.detectFeatures(training[i],feats_training[i],0,numFeats);
        fext.computeDescriptors(training[i], feats_training[i], desc_to_compute);
    }

    for(int i=0 ; i < len_testing ; i++)
    {
        testing[i].loadFromFile(testing_paths.at(i));
        fext.detectFeatures(testing[i],feats_testing[i],0,numFeats);
        fext.computeDescriptors(testing[i], feats_testing[i], desc_to_compute);
    }

    // stores the number of i'th class instances at the position index 'i'
    int training_count[NUM_CLASSES];
    int testing_count[NUM_CLASSES];

    // stores the labels for the i'th image instance for training and testing images
    int training_labels[len_training];
    int testing_labels[len_testing];

    /// computing the labels for each image here, REDUCE THE FOLLOWING TO n CLASSES AND MAKE A FUNCTION WHICH DOES THE LABEL ASSIGNMENT

    computeLabels(training_paths, training_count, training_labels );
    computeLabels(testing_paths, testing_count, testing_labels );

    CTicTac training_time;
    training_time.Tic();
    training_file.open ("training_images_features.txt");
    training_file.clear();
    //training_file << "Writing this to a file.\n";
    for (int i=0 ; i<len_training ; i++)
    {
        training_file << feats_training[i].size();
        for (int j = 0; j < feats_training[i].size(); j++)
        {
            if (descriptor_selected == 0)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.SIFT;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.SIFT.data() << endl; //!< SIFT descriptors
            }
            else if (descriptor_selected == 1)
            {
                vector<float> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.SURF;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << temp_feat.at(k) << " " ;


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
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.ORB.data() << endl; //!< ORB image descriptor
            }
            else if (descriptor_selected == 6)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.BLD;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.BLD.data() << endl; //!< BLD image descriptor
            }
            else if (descriptor_selected == 7)
            {
                vector<uint8_t> temp_feat;
                temp_feat = feats_training[i].getByID(j).get()->descriptors.LATCH;
                for(int k=0 ; k < temp_feat.size() ; k++)
                    training_file << temp_feat.at(k) << " " ;

                //training_file << feats_testing[i].getByID(j).get()->descriptors.LATCH.data() << endl; //!< LATCH image descriptor
            }
            //training_file << feats_testing[i].getByID(i).get()->descriptors.SURF;
            //cout << feats_testing[i].getFeatureX(i) << " , " << feats_testing[i].getFeatureY(i);

        }

        training_file << endl;
        training_file << training_labels[i] << endl;
    }
    cout << " elapsed time for training dataset: " << training_time.Tac() << end;

    CTicTac testing_time;
    testing_time.Tic();

    /// now extracting features for Place Recognition for testing dataset
    testing_file.open ("testing_images_features.txt");
    testing_file.clear();
    for (int i=0 ; i<len_testing ; i++)
    {
        testing_file << feats_testing[i].size();
        for (int j = 0; j < feats_testing[i].size(); j++)
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
    }

    cout << " elapsed time for testing dataset: " << testing_time.Tac() << end;


    testing_file.close();
    cout << "after reading testing images" << endl ;


    return "";

}