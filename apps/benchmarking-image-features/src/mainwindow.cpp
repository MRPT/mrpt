#include "mainwindow.h"
//#include "visualizedialog.h"
#include <QButtonGroup>
#include <QtWidgets>
#include <dirent.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"

#include <opencv2/plot.hpp>
#include <mrpt/utils/metaprogramming.h>

using namespace cv::line_descriptor;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

QImage qimage_1[MAX_DESC], qimage_2[MAX_DESC];
QLabel *images1[MAX_DESC], *images2[MAX_DESC];
/*
 * This button is used to visualize the descriptors
 */

void MainWindow::on_button_generate_clicked()
{
    next_desc->setVisible(true);
    //ReadInputFormat();
    //int numDescriptors = featsImage1.size();
    //visualize_dialog = new VisualizeDialog(this, inputFilePath->text(), detector_selected, descriptor_selected, numDescriptors,  featsImage1, featsImage2 );

    if(cnt >= numDesc1-9)
    {
        return;
    }

    for (unsigned int i1 = 0; i1 < numDesc1; i1++)
    {
        //tictac.Tic();

        // Display the current descriptor in its window and the best descriptor from the other image:
        switch (descriptor_selected) {
            case -1: // Patch, descAny
            case 3: // descPolarImages
            case 4: // descLogPolarImages
            case 2: { // descSpinImages

                CImage auxImg1, auxImg2;
                if (descriptor_selected == -1) // descAny
                {
                    auxImg1 = featsImage1[i1]->patch;

                } else if (descriptor_selected == 3) // descPolarImages
                {
                    auxImg1.setFromMatrix(featsImage1[i1]->descriptors.PolarImg);
                } else if (descriptor_selected == 4)  // descLogPolarImages
                {
                    auxImg1.setFromMatrix(featsImage1[i1]->descriptors.LogPolarImg);

                } else if (descriptor_selected == 2) // descSpinImages
                {

                    const size_t nR = featsImage1[i1]->descriptors.SpinImg_range_rows;
                    const size_t nC =
                            featsImage1[i1]->descriptors.SpinImg.size() / featsImage1[i1]->descriptors.SpinImg_range_rows;
                    CMatrixFloat M1(nR, nC);
                    for (size_t r = 0; r < nR; r++)
                        for (size_t c = 0; c < nC; c++)
                            M1(r, c) = featsImage1[i1]->descriptors.SpinImg[c + r * nC];
                    auxImg1.setFromMatrix(M1);

                }

                //cout << "before the while loop" << auxImg1.getWidth() << "  "<< auxImg1.getHeight() << "  " <<  endl;

                while (auxImg1.getWidth() < 100 && auxImg1.getHeight() < 100)
                    auxImg1.scaleImage(auxImg1.getWidth() * 2, auxImg1.getHeight() * 2, IMG_INTERP_NN);

                //cout << "after the while loop" << endl;

                cv::Mat cvImg1 = cv::cvarrToMat(auxImg1.getAs<IplImage>());
                cv::Mat temp1(cvImg1.cols, cvImg1.rows, cvImg1.type());
                cvtColor(cvImg1, temp1, CV_GRAY2RGB);
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);


                qimage_1[i1] = dest1.scaled(DESCRIPTOR_HEIGHT, DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                //cout << "after the image 1 setting, the while loop" << endl;

            }
                break;
            case 0: { // descSIFT
                vector<double> v1;
                mrpt::utils::metaprogramming::copy_container_typecasting(featsImage1[i1]->descriptors.SIFT, v1);

                cout << v1.size() << " SIFT Descriptor " << v1.at(2) << " " << v1.at(3) << " " << v1.at(4) << " "
                     << endl;

                cout << "before the plot" << endl;
                Mat data( v1.size(), 1, CV_64F );
                int gg = 0;


                // this does not work currently need to correct this
                /*for(int i=0 ; i<v1.size() ; i++)
                {
                    data.at(i,gg) = v1.at(i);
                }*/
                randu( data, 0, 500 ); // random values

                Mat plot_result;

                Ptr<plot::Plot2d> plot = plot::createPlot2d( data );
                plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) ); // i think it is not implemented yet
                plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
                plot->render( plot_result );

                //imshow( "plot", plot_result );
                //waitKey();

                cout << "after the plot" << endl;

                cout << plot_result.cols << " cols " << plot_result.rows << "  rows " << plot_result.type() << " type" << endl;
                cv::Mat temp1(plot_result.cols, plot_result.rows, plot_result.type());
                cvtColor(plot_result, temp1, CV_RGB2BGR);
                //imshow("temp ", temp1);
                //waitKey();
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);
                qimage_1[i1] = dest1.scaled(2*DESCRIPTOR_HEIGHT, 3*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));


                /*   vector<double> x;
                   for(int i=0; i <v1.size() ; i++)
                   {
                       x.at(i) = i;
                   }
                   QVector<double> vector_x;
                   vector_x.fromStdVector(x);

                   QVector<double> vector1;
                   vector1.fromStdVector(v1);
                   QCustomPlot *customPlot = new QCustomPlot;
                   customPlot->addGraph();
                   customPlot->graph(0)->setData(vector1,vector_x);*/



                //winptrPlot_descr1->plot( v1 );
                //winptrPlot_descr2->plot( v2 );

            }
                break;
            case 1: { // descSURF
                //winptrPlot_descr1->plot( feats1[i1]->descriptors.SURF );
                //winptrPlot_descr1->axis_fit();
            }
                break;
            default: {
                cerr << "Descriptor specified is not handled yet" << endl;
            }
                break;
        }
    }// end of for loop


    cv::Mat desc_Ref_img = imread(file_path1, IMREAD_ANYCOLOR); // cv::cvarrToMat(img1.getAs<IplImage>());
    // images1 have all the descriptors

    if(descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
    {
        for (int i = 0; i < DESCRIPTOR_GRID_SIZE; i++) {
            images_static[i] = images1[cnt];

            //desc_VisualizeGrid->replaceWidget(images_static[i],images1[cnt]);
            //images_static[i] = images1[cnt];


            //desc_VisualizeGrid->replaceWidget(images_static[i],images_static[i]);
            // now following lines are for computing the x and y coordinate
            images_label_coordinates[i] = new QLabel;
            float temp_x = featsImage1.getFeatureX(cnt); // get the descriptor x coordinate corresponding to images1[cnt]
            float temp_y = featsImage1.getFeatureY(cnt);
            stringstream ss;
            ss << temp_x << "," << temp_y << endl;
            string str = ss.str();


            cout << str << endl;
            stringstream ss2;
            ss2 << i ;
            string str2 = ss2.str();

            images_label_coordinates[i]->setText(QString::fromStdString(str2));
            circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(0,0,255), 2, 8, 0); // plot the circles on the appropriate points as per the shown descriptors
            putText(desc_Ref_img, str2, Point(temp_x,temp_y), 5, 2, Scalar(255,0,0),2,8,0);

            cnt++; // global counter for iterating over images and labels


            int i_t = 2*(i/DESCRIPTOR_ROW_SIZE);
            int j_t = 2*(i%DESCRIPTOR_ROW_SIZE);
            desc_VisualizeGrid->addWidget(images_static[i],i_t,j_t,1,1);
            desc_VisualizeGrid->addWidget(images_label_coordinates[i],i_t+1, j_t+1, 1,1);


        }
    }
    else if(descriptor_selected == 0 || descriptor_selected == 1)
    {
        for (int i = 0; i < DESCRIPTOR_GRID_SIZE2; i++)
        {
            images_static_sift_surf[i] = images1[cnt];
            images_label_coordinates_sift_surf[i] = new QLabel;

            float temp_x = featsImage1.getFeatureX(cnt); // get the descriptor x coordinate corresponding to images1[cnt]
            float temp_y = featsImage1.getFeatureY(cnt);

            stringstream ss;
            ss << temp_x << " , " << temp_y << endl;
            string str = ss.str();
            images_label_coordinates_sift_surf[i]->setText(QString::fromStdString(str));
            cout << str << endl;


            circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0); // plot the circles on the appropriate points as per the shown descriptors


            cnt++; // global counter for iterating over images and labels


            int i_t = 2*(i/DESCRIPTOR_ROW_SIZE);
            int j_t = 2*(i%DESCRIPTOR_ROW_SIZE);
            desc_VisualizeGrid->addWidget(images_static_sift_surf[i],i_t,j_t,1,1);
            desc_VisualizeGrid->addWidget(images_label_coordinates_sift_surf[i],i_t+1, j_t+1, 1,1);
        }
    }


    cv::Mat temp1 (desc_Ref_img.cols,desc_Ref_img.rows,desc_Ref_img.type());
    cvtColor(desc_Ref_img, temp1, CV_BGR2RGB);
    QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
    QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));


}


void MainWindow::button_close_clicked()
{
    window_gui->close();
    this->close();
    return;
}
void MainWindow::on_descriptor_choose(int choice)
{
    makeAllDescriptorParamsVisible(true);

    if (choice == 0) //!< SIFT descriptors
    {
        param1_desc->setText("Threshold: ");
        param2_desc->setText("Edge Threshold: ");
        param1_edit_desc->setText("0.04");
        param2_edit_desc->setText("10");
        param3_desc->setVisible(false);
        param4_desc->setVisible(false);
        param5_desc->setVisible(false);
        param3_edit_desc->setVisible(false);
        param4_edit_desc->setVisible(false);
        param5_edit_desc->setVisible(false);
    }
    else if (choice  == 1) //!< SURF descriptors
    {
        param1_desc->setText("hessianThreshold: ");
        param2_desc->setText("nLayersPerOctave: ");
        param3_desc->setText("nOctaves: ");
        param4_desc->setText("if rotation invariant: ");
        param1_edit_desc->setText("600");
        param2_edit_desc->setText("4");
        param3_edit_desc->setText("2");
        param4_edit_desc->setText("true");

        param5_desc->setVisible(false);
        param5_edit_desc->setVisible(false);
    }
    else if (choice == 2)  //!< Intensity-domain spin image descriptors
    {
        param1_desc->setText("Radius: ");
        param2_desc->setText("Hist size intensity: ");
        param3_desc->setText("Hist size distance: ");
        param4_desc->setText("Std distance: ");
        param5_desc->setText("Std intensity: ");
        param1_edit_desc->setText("20");
        param2_edit_desc->setText("10");
        param3_edit_desc->setText("10");
        param4_edit_desc->setText("0.4");
        param5_edit_desc->setText("20");
    }
    else if (choice == 3) // Polar Image descriptor
    {
        param1_desc->setText("Radius: ");
        param2_desc->setText("Bins Angle: ");
        param3_desc->setText("Bins Distance: ");
        param1_edit_desc->setText("20");
        param2_edit_desc->setText("8");
        param3_edit_desc->setText("6");

        param4_desc->setVisible(false);
        param5_desc->setVisible(false);
        param4_edit_desc->setVisible(false);
        param5_edit_desc->setVisible(false);
    }
    else if (choice == 4) // Log Polar Image descriptor
    {
        param1_desc->setText("Radius: ");
        param2_desc->setText("Number of Angles: ");
        param3_desc->setText("Rho Scale: ");
        param1_edit_desc->setText("30");
        param2_edit_desc->setText("16");
        param3_edit_desc->setText("5");

        param4_desc->setVisible(false);
        param5_desc->setVisible(false);
        param4_edit_desc->setVisible(false);
        param5_edit_desc->setVisible(false);
    }
    else
    {
        makeAllDescriptorParamsVisible(false);
        param1_desc->setText("Not yet Implemented");
        param1_desc->setVisible(true);
    }
}
void MainWindow::on_detector_choose(int choice)
{

    detector_selected = choice;
    makeAllDetectorParamsVisible(true);

    //KLT Features
    if(choice == 0)
    {
        param1->setText("Min distance : ");
        param2->setText("Radius : ");
        param3->setText("Threshold : ");
        param4->setText("Tile-image true/false : ");
        param1_edit->setText("7");
        param2_edit->setText("7");
        param3_edit->setText("0.1");
        param4_edit->setText("true");
        param5->setVisible(false);
        param5_edit->setVisible(false);
    }
        // for Harris Features
    else if (choice ==1) {
        param1->setText("Threshold : ");
        param2->setText("Sensitivity, k : ");
        param3->setText("Smoothing, sigma : ");
        param4->setText("Block size, radius : ");
        param5->setText("Tile_image (true/false) : ");
        param1_edit->setText("0.005");
        param2_edit->setText("0.04");
        param3_edit->setText("1.5");
        param4_edit->setText("3");
        param5_edit->setText("true");
    }
        // BCD Features not implemented yet in MRPT library
    else if(choice == 2)
    {
        makeAllDetectorParamsVisible(false);
    }
        //for SIFT Features
    else if (choice == 3)
    {
        param1->setText("Threshold: ");
        param2->setText("Edge Threshold: ");
        param1_edit->setText("0.04");
        param2_edit->setText("10");
        param3->setVisible(false);
        param4->setVisible(false);
        param5->setVisible(false);
        param3_edit->setVisible(false);
        param4_edit->setVisible(false);
        param5_edit->setVisible(false);

    }
        // for SURF Features
    else if(choice == 4)
    {
        param1->setText("HessianThreshold: ");
        param2->setText("nLayersPerOctave: ");
        param3->setText("nOctaves: ");
        param4->setText("if rotation invariant: ");
        param1_edit->setText("600");
        param2_edit->setText("4");
        param3_edit->setText("2");
        param4_edit->setText("true");

        param5->setVisible(false);
        param5_edit->setVisible(false);
    }
        // for FAST, FASTER9, FASTER10, FASTER12 features
    else if(choice == 5 || choice == 6 || choice==7 || choice == 8)
    {
        param1->setText("Threshold: ");
        param2->setText("MinimumDistance: ");
        param3->setText("Enable non maximal supression (true/false): ");
        param4->setText("Enable use KLT response (true/false): ");
        param1_edit->setText("20");
        param2_edit->setText("5");
        param3_edit->setText("true");
        param4_edit->setText("true");

        param5->setVisible(false);
        param5_edit->setVisible(false);


    }
        // ORB Features
    else if (choice == 9)
    {
        param1->setText("Min Distance: ");
        param2->setText("if extract patch true / false: ");
        param3->setText("Number of levels: ");
        param4->setText("Scale factor: ");
        param1_edit->setText("0");
        param2_edit->setText("false");
        param3_edit->setText("8");
        param4_edit->setText("1.2");

        param5->setVisible(false);
        param5_edit->setVisible(false);
    }
    else
    {
        makeAllDetectorParamsVisible(false);
        param1->setText("Not yet Implemented");
        param1->setVisible(true);
    }
}
void MainWindow::onStereoMatchingChecked(int state)
{
    if(stereo_matching->isChecked() && currentInputIndex == 1)
    {
        cout << " You clicked me .!!" << endl;
        CMatchedFeatureList mHarris, mSIFT, mHarris_SAD, mFAST_CC, mFAST_SAD;
        TMatchingOptions opts;

        CTicTac tic;
        tic.Tic();

        opts.matching_method = TMatchingOptions::mmSAD;
        matchFeatures(featsImage1, featsImage2, mHarris_SAD, opts);
        cout << "featsImage1 size" << featsImage1.size() << " featsImage2.seze " << featsImage2.size() << endl;
        cout << "SAD matches found: " << mHarris_SAD.size() <<" in seconds" << tic.Tac()*1000.0f << endl;

    }

}

void MainWindow::makeAllDetectorParamsVisible(bool flag)
{
    param1->setVisible(flag);
    param2->setVisible(flag);
    param3->setVisible(flag);
    param4->setVisible(flag);
    param5->setVisible(flag);
    param1_edit->setVisible(flag);
    param2_edit->setVisible(flag);
    param3_edit->setVisible(flag);
    param4_edit->setVisible(flag);
    param5_edit->setVisible(flag);
}
void MainWindow::makeAllDescriptorParamsVisible(bool flag)
{
    param1_desc->setVisible(flag);
    param2_desc->setVisible(flag);
    param3_desc->setVisible(flag);
    param4_desc->setVisible(flag);
    param5_desc->setVisible(flag);
    param1_edit_desc->setVisible(flag);
    param2_edit_desc->setVisible(flag);
    param3_edit_desc->setVisible(flag);
    param4_edit_desc->setVisible(flag);
    param5_edit_desc->setVisible(flag);
}

void MainWindow::on_file_input_choose(int choice)
{
    // HIDE input file path 2, browse button 2, image2 for cases : single image,  image raw log and single image dataset
    if (choice == 0 || choice == 2 || choice == 3)
    {
        inputFilePath2->setVisible(false);
        browse_button2->setVisible(false);
        image2->setVisible(false);
    }
    else
    {
        inputFilePath2->setVisible(true);
        browse_button2->setVisible(true);
        image2->setVisible(true);
    }
    // HIDE previous and next buttons for the cases : single image, stereo image, image raw log
    if(choice == 0 || choice == 1 || choice == 2)
    {
        next_button->setVisible(false);
        prev_button->setVisible(false);
    }
    else
    {
        next_button->setVisible(true);
        prev_button->setVisible(true);
    }
    currentInputIndex = inputs->currentIndex();
    switch (currentInputIndex)
    {
        case 0:
            groupBox_images->setTitle("Single Image");
            break;
        case 1:
            groupBox_images->setTitle("Stereo Image Pair");
            break;
        case 2:
            groupBox_images->setTitle("Image RawLog File <br> NOT IMPLEMENTED YET");
            break;
        case 3:
            groupBox_images->setTitle("Single Image Dataset");
            break;
        case 4:
            groupBox_images->setTitle("Stereo Image Dataset");
            break;
        default :
            groupBox_images->setTitle("Invalid Selection");
    }
}

void MainWindow::fillDetectorInfo()
{
    numFeats = numFeaturesLineEdit->text().toInt();

    cout << file_path1 << " File Path in fillDetectorInfo " << endl;

    img1.loadFromFile(file_path1);

    if(currentInputIndex == 1 || currentInputIndex ==4) // stereo image or stereo dataset
        img2.loadFromFile(file_path2);

    cout << file_path1 << " File Path in fillDetectorInfo AFTER IMAGE READ " << endl;

    if(detector_selected == 0) // 0 = KLT Detector
    {
        klt_opts.min_distance = param1_edit->text().toFloat();
        klt_opts.radius = param2_edit->text().toInt();
        klt_opts.threshold = param3_edit->text().toFloat();
        string temp_str = param4_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        klt_opts.tile_image = temp_bool;

        fext.options.featsType = featKLT;

        fext.options.KLTOptions.min_distance =klt_opts.min_distance;
        fext.options.KLTOptions.radius = klt_opts.radius;
        fext.options.KLTOptions.threshold = klt_opts.threshold;
        fext.options.KLTOptions.tile_image = klt_opts.tile_image;

        cout << "detecting KLT Features " << endl ;
    }
    else if(detector_selected == 1) //Harris Features
    {
        harris_opts.threshold = param1_edit->text().toFloat();
        harris_opts.k = param2_edit->text().toFloat();
        harris_opts.sigma = param3_edit->text().toFloat();
        harris_opts.radius = param4_edit->text().toFloat();
        string temp_str = param5_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") ? false : true;
        harris_opts.tile_image = temp_bool;

        fext.options.featsType = featHarris;

        fext.options.harrisOptions.threshold = harris_opts.threshold;//0.005;
        fext.options.harrisOptions.k =  harris_opts.k;  // default sensitivity
        fext.options.harrisOptions.sigma = harris_opts.sigma;  // default from matlab smoothing filter
        fext.options.harrisOptions.radius = harris_opts.radius;  // default block size
        //fext.options.harrisOptions.min_distance = 100;
        fext.options.harrisOptions.tile_image = harris_opts.tile_image;

        cout << "detecting Harris Features " << endl ;
    }
    else if(detector_selected == 2) // not implemented in MRPT yet
    {
        fext.options.featsType = featBCD;
        cout <<"detecting BCD Features" << endl;

    }
    else if(detector_selected == 3) // SIFT Detector
    {
        SIFT_opts.threshold = param1_edit->text().toFloat();
        SIFT_opts.edge_threshold = param2_edit->text().toFloat();

        fext.options.featsType = featSIFT;

        fext.options.SIFTOptions.threshold = SIFT_opts.threshold;
        fext.options.SIFTOptions.edgeThreshold = SIFT_opts.edge_threshold;
        fext.options.SIFTOptions.implementation = CFeatureExtraction::OpenCV;

        cout << "detecting SIFT Features " << endl ;

    }
    else if (detector_selected == 4) // 4= SURF Detector
    {
        fext.options.featsType = featSURF;
        SURF_opts.hessianThreshold = param1_edit->text().toInt();
        SURF_opts.nLayersPerOctave = param2_edit->text().toInt();
        SURF_opts.nOctaves = param3_edit->text().toInt();
        string temp_str = param4_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        SURF_opts.rotation_invariant = temp_bool;
        cout <<  temp_bool << endl;

        fext.options.SURFOptions.hessianThreshold = SURF_opts.hessianThreshold;
        fext.options.SURFOptions.nLayersPerOctave = SURF_opts.nLayersPerOctave;
        fext.options.SURFOptions.nOctaves = SURF_opts.nOctaves;
        fext.options.SURFOptions.rotation_invariant = SURF_opts.rotation_invariant;
    }
    else if(detector_selected == 5 || detector_selected == 6 || detector_selected == 7 || detector_selected == 8) //FAST detector and its variants
    {
        fast_opts.threshold = param1_edit->text().toFloat();
        fast_opts.min_distance = param2_edit->text().toFloat();
        string temp_str = param3_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        fast_opts.use_KLT_response = temp_bool;

        temp_str = param4_edit->text().toStdString();
        temp_bool = temp_str.compare("true") == 0;
        fast_opts.non_max_suppresion = temp_bool;

        if(detector_selected == 5)
            fext.options.featsType = featFAST;
        else if(detector_selected == 6)
            fext.options.featsType = featFASTER9;
        else if(detector_selected == 7)
            fext.options.featsType =featFASTER10;
        else
            fext.options.featsType = featFASTER12;

        fext.options.FASTOptions.threshold = fast_opts.threshold;
        fext.options.FASTOptions.min_distance = fast_opts.min_distance;
        fext.options.FASTOptions.use_KLT_response = fast_opts.use_KLT_response;
        fext.options.FASTOptions.nonmax_suppression = fast_opts.non_max_suppresion;
    }
    else if(detector_selected == 9) // ORB Feature detector
    {
        fext.options.featsType = featORB;

        ORB_opts.min_distance = param1_edit->text().toInt();
        string temp_str = param2_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        ORB_opts.extract_patch = temp_bool;
        ORB_opts.n_levels = param3_edit->text().toInt();
        ORB_opts.scale_factor = param4_edit->text().toFloat();


        fext.options.ORBOptions.min_distance = ORB_opts.min_distance;
        fext.options.ORBOptions.extract_patch = ORB_opts.extract_patch;
        fext.options.ORBOptions.n_levels = ORB_opts.n_levels;
        fext.options.ORBOptions.scale_factor = ORB_opts.scale_factor;

    }
}

void MainWindow::fillDescriptorInfo()
{
    //clear CDescriptor storage data variable before each button click
    //read inputs from user
    ReadInputFormat();

    numFeats = numFeaturesLineEdit->text().toInt();

    img1.loadFromFile(file_path1);

    if(currentInputIndex == 1 || currentInputIndex ==4) // stereo image or stereo dataset
        img2.loadFromFile(file_path2);

    if(descriptor_selected == 0) //!< SIFT Descriptors
    {
        SIFT_opts.threshold = param1_edit_desc->text().toFloat();
        SIFT_opts.edge_threshold = param2_edit_desc->text().toFloat();

        desc_to_compute = TDescriptorType (1); //!< SIFT descriptors

        fext.options.SIFTOptions.threshold = SIFT_opts.threshold;
        fext.options.SIFTOptions.edgeThreshold = SIFT_opts.edge_threshold;
        fext.options.SIFTOptions.implementation = CFeatureExtraction::LoweBinary;
    }
    else if(descriptor_selected == 1)
    {
        desc_to_compute = TDescriptorType (2); //!< SURF descriptors

        SURF_opts.hessianThreshold = param1_edit_desc->text().toInt();
        SURF_opts.nLayersPerOctave = param2_edit_desc->text().toInt();
        SURF_opts.nOctaves = param3_edit_desc->text().toInt();
        string temp_str = param4_edit_desc->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        SURF_opts.rotation_invariant = temp_bool;
        cout <<  temp_bool << endl;

        fext.options.SURFOptions.hessianThreshold =  SURF_opts.hessianThreshold;
        fext.options.SURFOptions.nLayersPerOctave =  SURF_opts.nLayersPerOctave;
        fext.options.SURFOptions.nOctaves = SURF_opts.nOctaves;
        fext.options.SURFOptions.rotation_invariant = SURF_opts.rotation_invariant;
    }
    else if(descriptor_selected == 2)
    {
        desc_to_compute = TDescriptorType(4); //!< Intensity-domain spin image descriptors

        spin_opts.radius = param1_edit_desc->text().toInt();
        spin_opts.hist_size_intensity = param2_edit_desc->text().toInt();
        spin_opts.hist_size_distance = param3_edit_desc->text().toInt();
        spin_opts.std_dist = param4_edit_desc->text().toFloat();
        spin_opts.std_intensity = param5_edit_desc->text().toFloat();


        fext.options.SpinImagesOptions.radius = spin_opts.radius;
        fext.options.SpinImagesOptions.hist_size_intensity = spin_opts.hist_size_intensity;
        fext.options.SpinImagesOptions.hist_size_distance = spin_opts.hist_size_distance ;
        fext.options.SpinImagesOptions.std_dist = spin_opts.std_dist;
        fext.options.SpinImagesOptions.std_intensity = spin_opts.std_intensity;

    }
    else if(descriptor_selected == 3)
    {
        desc_to_compute = TDescriptorType(8); //!< Polar image descriptor

        polar_opts.radius = param1_edit_desc->text().toInt();
        polar_opts.bins_angle = param2_edit_desc->text().toInt();
        polar_opts.bins_distance = param3_edit_desc->text().toInt();

        fext.options.PolarImagesOptions.radius = polar_opts.radius;
        fext.options.PolarImagesOptions.bins_angle = polar_opts.bins_angle;
        fext.options.PolarImagesOptions.bins_distance = polar_opts.bins_distance;
    }
    else if(descriptor_selected == 4)
    {
        desc_to_compute = TDescriptorType (16); //!< Log-Polar image descriptor

        log_polar_opts.radius = param1_edit_desc->text().toInt();
        log_polar_opts.num_angles = param2_edit_desc->text().toInt();
        log_polar_opts.rho_scale = param3_edit_desc->text().toFloat();

        fext.options.LogPolarImagesOptions.radius = log_polar_opts.radius;
        fext.options.LogPolarImagesOptions.num_angles = log_polar_opts.num_angles;
        fext.options.LogPolarImagesOptions.rho_scale = log_polar_opts.rho_scale;
    }
}

/*
 *
 * this function is called to show the performance of the selected detector on the input selected by the user
 * the performance metric displayed is in terms of repeatability, dispersion of image, computational cost, number of found points, etc.
 * */
void MainWindow::on_detector_button_clicked()
{
    //read inputs from user
    ReadInputFormat();
    if( detector_selected == -1 || file_path1.empty())
    {
        QMessageBox::information(this, "Image/Detector/Descriptor read error","Please specify a valid inputs for the image / detector..");
        return;
    }
    if(file_path1.empty() || file_path2.empty())
    {
        if(currentInputIndex == 3 || currentInputIndex == 4)
            readFilesFromFolder(2);
    }

    // Feature Extraction Starts here
    fillDetectorInfo();

    // Clearing the features list is very important to avoid mixing subsequent button clicks output
    featsImage1.clear();
    fext.detectFeatures(img1, featsImage1, 0, numFeats);
    //save to file
    featsImage1.saveToTextFile("./KeyPoints1.txt");

    cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());
    // Drawing a circle around corners for image 1
    //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    for(int i=0 ; i<featsImage1.size() ; i++)
    {
        int temp_x = (int) featsImage1.getFeatureX(i);
        int temp_y = (int) featsImage1.getFeatureY(i);
        circle(cvImg1, Point(temp_x, temp_y), 5, Scalar(0,255,0), 2, 8, 0);
    }


    // converting the cv::Mat to a QImage and changing the resolution of the output images
    cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
    cvtColor(cvImg1, temp1, CV_BGR2RGB);
    QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
    QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));

    cout << "Number of Features in image 1: " << featsImage1.size() << endl;


    if(currentInputIndex == 1 || currentInputIndex == 4)
    {
        featsImage2.clear();
        fext.detectFeatures(img2, featsImage2, 0, numFeats);
        featsImage2.saveToTextFile("./KeyPoints2.txt");
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());

        // Drawing a circle around corners for image 2
        //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        for (int i = 0; i < featsImage2.size(); i++) {
            int temp_x = (int) featsImage2.getFeatureX(i);
            int temp_y = (int) featsImage2.getFeatureY(i);
            circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(0,255,0), 2, 8, 0);
        }

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar *) cvImg2.data, cvImg2.cols, cvImg2.rows, cvImg2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));
        cout << "Number of Features in image 2: " << featsImage2.size() << endl;
    }

    /*if(detector_selected == 9)
    {
        computeBSD();
    }

    if(detector_selected == 10)
    {
        cv::Mat img = imread(file_path1);
        cv::Mat img2 = imread(file_path2);


        int scale = param1_edit->text().toInt();
        int numOctaves = param2_edit->text().toInt();

        cv::Mat ff = computeLSD(img,scale,numOctaves);
        cv::Mat ff2 = computeLSD(img2,scale,numOctaves);

        qimage1 = QImage((uchar*) ff.data, ff.cols, ff.rows, ff.step, QImage::Format_RGB888);
        //image1->setPixmap(QPixmap::fromImage(qimage1));

        QImage qsmall1 = qimage1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image1->setPixmap(QPixmap::fromImage(qsmall1));


        qimage2 = QImage((uchar*) ff2.data, ff2.cols, ff2.rows, ff2.step, QImage::Format_RGB888);
        QImage qsmall2 = qimage2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        //image2->setPixmap(QPixmap::fromImage(qimage2));

        image2->setPixmap(QPixmap::fromImage(qsmall2));


    }*/
}

/*
 *
 * this function is called to show the performance of the selected descriptor on the input selected by the user
 * the performance metric displayed is in terms of percentage of patches matched, descriptor distance between close matches, false positives/negatives, computational cost, etc.
 * */
void MainWindow::on_descriptor_button_clicked()
{
    CTicTac tic;
    tic.Tic();
    ReadInputFormat();

    //populate the selected descriptor
    fillDescriptorInfo();


    if(descriptor_selected == 0)
        desc_to_compute = TDescriptorType (1); //!< SIFT descriptors
    else if (descriptor_selected == 1)
        desc_to_compute = TDescriptorType  (2); //!< SURF descriptors
    else if (descriptor_selected == 2)
        desc_to_compute = TDescriptorType(4); //!< Intensity-domain spin image descriptor
    else if (descriptor_selected == 3)
        desc_to_compute = TDescriptorType(8); //!< Polar image descriptor
    else if (descriptor_selected == 4)
        desc_to_compute = TDescriptorType (16); //!< Log-Polar image descriptor


    if(desc_to_compute != descAny)
        fext.options.patchSize = 0;


    // find a way to clear off past detector/descriptor info stored in featsImage

    if(desc_to_compute != TDescriptorType(-1) && desc_to_compute !=descAny)
    {
        fext.computeDescriptors(img1, featsImage1, desc_to_compute);
        if (currentInputIndex == 1 || currentInputIndex == 4)
        {
            fext.computeDescriptors(img2, featsImage2, desc_to_compute);
            //featsImage2.saveToTextFile("/home/raghavender/Key_Descriptors2");
        }
        //featsImage1.saveToTextFile("/home/raghavender/Key_Descriptors1.txt");

        cout << featsImage1.size()<< endl;


    }
    // storing size of descriptors for visualizer
    numDesc1 = featsImage1.size();
    numDesc2 = featsImage2.size();

    cout << "Time Elapsed : " << tic.Tac() << endl;

}




/*
 *
 * this function browses for the image files that the user can select or the directory that the user can select
 * */
void MainWindow::on_browse_button_clicked()
{
    ReadInputFormat();

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::AnyFile);

    //0 = single image; 1 = stereo image; 2 = rawlog file ; 3 = image dataset folder ; 4 = stereo dataset folder
    if(currentInputIndex == 0 || currentInputIndex == 1)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 2 || currentInputIndex == 3 || currentInputIndex == 4)
    {
        dialog.setFileMode(QFileDialog::Directory);
    } else
        return;

    //dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg)"));
    dialog.setViewMode(QFileDialog::Detail);
    QStringList fileNames;
    if(dialog.exec())
        fileNames = dialog.selectedFiles();

    if(fileNames.size() != 0)
        inputFilePath->setText(fileNames.at(0));

    file_path1 = inputFilePath->text().toStdString();
    if(currentInputIndex == 0)
    {
        file_path1 = inputFilePath->text().toStdString();
        img1.loadFromFile(file_path1);
    }
    else if (currentInputIndex == 1) // only read the single images if a single image or stereo image input is specified
    {
        file_path1 = inputFilePath->text().toStdString();
        file_path2 = inputFilePath2->text().toStdString();

        img1.loadFromFile(file_path1);
        img2.loadFromFile(file_path2);
    }
}

void MainWindow::on_browse_button_clicked2()
{
    ReadInputFormat();

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::AnyFile);

    //0 = single image; 1 = stereo image; 2 = rawlog file ; 3 = image dataset folder
    if(currentInputIndex == 0 || currentInputIndex == 1)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 2 || currentInputIndex == 3 || currentInputIndex == 4)
    {
        dialog.setFileMode(QFileDialog::Directory);
    } else
        return;

    //dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg)"));
    dialog.setViewMode(QFileDialog::Detail);
    QStringList fileNames;
    if(dialog.exec())
        fileNames = dialog.selectedFiles();

    if(fileNames.size() != 0)
        inputFilePath2->setText(fileNames.at(0));


    file_path2 = inputFilePath2->text().toStdString();

    if(currentInputIndex == 1)
        img2.loadFromFile(file_path2);




}

/*
 * This function reads and stores the states of the user selections and can be used by other functions when required
 *
 **/
void MainWindow::ReadInputFormat()
{
    // store the input type here
    currentInputIndex = inputs->currentIndex();

    //store the detector chosen here
    detector_selected = detectors_select->currentIndex();

    //store the descriptor here
    descriptor_selected = descriptors_select->currentIndex();

    //numFeatures = numFeaturesLineEdit->text().toInt();

    /*if(currentInputIndex == 0)
    {
        file_path1 = inputFilePath->text().toStdString();
    }
    else if (currentInputIndex == 1) // only read the single images if a single image or stereo image input is specified
    {
        file_path1 = inputFilePath->text().toStdString();
        file_path2 = inputFilePath2->text().toStdString();
    }*/
}

/*
 * this function reads the files from a folder, the function is called when the user presses the next or previous button
 * WORKS CORRECTLY
 *  assumption both folders of the stereo image dataset have the same number of images
 */
void MainWindow::readFilesFromFolder(int next_prev) // indicate 1 for next and 0 for prev being pressed
{
    cout << " You clicked me" <<endl;
    ReadInputFormat();

    file_path1 = inputFilePath->text().toStdString();


    cout << currentInputIndex << endl;
    DIR *dir;
    dirent *pdir;
    vector<string> files;
    vector<string> files_fullpath;

    if (currentInputIndex == 3 || currentInputIndex == 4) //meaning stereo dataset or single image dataset
    {
        dir = opendir(file_path1.c_str());
        while(pdir = readdir(dir))
            files.push_back(pdir->d_name);
        for(int i=0,j=0 ; i<files.size() ; i++)
        {
            if(files.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
            {
                files_fullpath.push_back(file_path1 + "/" + files.at(i));
                cout << files_fullpath.at(j) << endl;
                j++;
            }
        } // end of for
    }
    file_path1 = files_fullpath.at(current_imageIndex);


    //DO the following only if user selects option for providing a STEREO image dataset
    if(currentInputIndex == 4)
    {
        file_path2 = inputFilePath2->text().toStdString();
        DIR *dir2;
        dirent *pdir2;
        vector<string> files2;
        vector<string> files_fullpath2;
        dir2 = opendir(file_path2.c_str());
        while (pdir2 = readdir(dir2))
            files2.push_back(pdir2->d_name);
        for (int i = 0, j = 0; i < files2.size(); i++)
        {
            if (files2.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
            {
                files_fullpath2.push_back(file_path2 + "/" + files2.at(i));
                cout << files_fullpath2.at(j) << endl;
                j++;
            }
        } // end of for
        file_path2 = files_fullpath2.at(current_imageIndex);
    }

    // add condition for the case when the current_imageIndex = the length of the total files start again from 0;
    // assumption both folders of the stereo image dataset have the same number of images
    if(next_prev == 1)
        current_imageIndex = (++current_imageIndex) %  files_fullpath.size();
    else if(next_prev == 2)
        current_imageIndex = current_imageIndex %files_fullpath.size();
    else
        current_imageIndex = (--current_imageIndex) %  files_fullpath.size();
}

/*
 * this function displays the images without detectors, function is called when NEXT or PREVIOUS buttons are pressed
 * WORKS CORRECTLY
 */
void MainWindow::displayImagesWithoutDetector()
{
    // DISPLAYING THE NEXT IMAGE AS A QIMAGE WITHOUT DETECTOR, DETECTOR WILL APPEAR ON THE IMAGE WHEN EVALUATE DETECTOR BUTTON IS CLICKED

    img1.loadFromFile(file_path1);

    cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());

    cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
    cvtColor(cvImg1, temp1, CV_BGR2RGB);
    QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
    QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));


    // DO the following only if user selects STEREO image dataset
    if(currentInputIndex == 4)
    {
        img2.loadFromFile(file_path2);

        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));
    }
}
void MainWindow::on_next_button_clicked()
{
    // read files from the folder and move to the next image accordingly
    readFilesFromFolder(1);

    //display the next images without the detectors on the screen
    displayImagesWithoutDetector();
}

void MainWindow::on_prev_button_clicked()
{
    // read files from the folder and move to the previous image accordingly
    readFilesFromFolder(0);

    //display the next images without the detectors on the screen
    displayImagesWithoutDetector();
}


void MainWindow::on_sample_clicked()
{
    float factor = decimateFactor->text().toFloat();
    sampling_rate += factor;
    ReadInputFormat();

    img1.loadFromFile(file_path1);
    cv::Mat cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());

    //Size(int(downsampled_clicked*cvImg1.cols),int(downsampled_clicked*cvImg1.rows);

    cv::Mat temp1 (cvImg1.cols,cvImg1.rows,cvImg1.type());
    cvtColor(cvImg1, temp1, CV_BGR2RGB);
    QImage disp1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);

    QImage qscaled1 = disp1.scaled(int(IMAGE_WIDTH*sampling_rate), int(IMAGE_HEIGHT*sampling_rate), Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));


    if(currentInputIndex == 1)
    {
        img2.loadFromFile(file_path2);
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());

        //Size(int(downsampled_clicked*cvImg1.cols),int(downsampled_clicked*cvImg1.rows);

        cv::Mat temp2 (cvImg2.cols,cvImg2.rows,cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage disp2 = QImage((uchar*) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        cout << "right before scaling the qimage" << endl;
        QImage qscaled2 = disp2.scaled(int(IMAGE_WIDTH*sampling_rate), int(IMAGE_HEIGHT*sampling_rate), Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));
    }
    // add qline edit to ask from the user the downsampling factor, here its 0.1


}
void MainWindow::initializeParameters()
{
    //detector information parameters fill in here from the user
    param1 = new QLabel("Parameter 1:");
    param1_edit = new QLineEdit;
    param1_edit->setText("1");
    param2 = new QLabel("Parameter 2:");
    param2_edit = new QLineEdit;
    param2_edit->setText("2");
    param3 = new QLabel("Parameter 3:");
    param3_edit = new QLineEdit;
    param3_edit->setText("3");
    param4 = new QLabel("Parameter 4:");
    param4_edit = new QLineEdit;
    param4_edit->setText("4");
    param5 = new QLabel("Parameter 5:");
    param5_edit = new QLineEdit;
    param5_edit->setText("5");

    param1_edit->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param2_edit->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param3_edit->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param4_edit->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param5_edit->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param1->setFixedHeight(PARAMS_HEIGHT);
    param2->setFixedHeight(PARAMS_HEIGHT);
    param3->setFixedHeight(PARAMS_HEIGHT);
    param4->setFixedHeight(PARAMS_HEIGHT);
    param5->setFixedHeight(PARAMS_HEIGHT);


    // Descriptor Information Fill in here from the user
    param1_desc = new QLabel("Parameter 1:");
    param1_edit_desc = new QLineEdit;
    param1_edit_desc->setText("1");
    param2_desc = new QLabel("Parameter 2:");
    param2_edit_desc = new QLineEdit;
    param2_edit_desc->setText("2");
    param3_desc = new QLabel("Parameter 3:");
    param3_edit_desc = new QLineEdit;
    param3_edit_desc->setText("3");
    param4_desc = new QLabel("Parameter 4:");
    param4_edit_desc = new QLineEdit;
    param4_edit_desc->setText("4");
    param5_desc = new QLabel("Parameter 5:");
    param5_edit_desc = new QLineEdit;
    param5_edit_desc->setText("5");

    param1_edit_desc->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param2_edit_desc->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param3_edit_desc->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param4_edit_desc->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param5_edit_desc->setFixedSize(PARAMS_WIDTH, PARAMS_HEIGHT);
    param1_desc->setFixedHeight(PARAMS_HEIGHT);
    param2_desc->setFixedHeight(PARAMS_HEIGHT);
    param3_desc->setFixedHeight(PARAMS_HEIGHT);
    param4_desc->setFixedHeight(PARAMS_HEIGHT);
    param5_desc->setFixedHeight(PARAMS_HEIGHT);
    param1_desc->setBaseSize(WIDGET_WIDTH, WIDGET_HEIGHT);
    param2_desc->setBaseSize(WIDGET_WIDTH, WIDGET_HEIGHT);
    param3_desc->setBaseSize(WIDGET_WIDTH, WIDGET_HEIGHT);
    param4_desc->setBaseSize(WIDGET_WIDTH, WIDGET_HEIGHT);
    param5_desc->setBaseSize(WIDGET_WIDTH, WIDGET_HEIGHT);
}
MainWindow::MainWindow(QWidget *window_gui) : QMainWindow(window_gui)
{
    current_imageIndex = 0;
    currentInputIndex = 0;
    detector_selected = 0;
    descriptor_selected = 0;
    sampling_rate = 1;
    cnt = 0;

    window_gui = new QWidget;
    window_gui->setWindowTitle("GUI app for benchmarking image detectors and descriptors");

    //Initialize the detectors here
    groupBox1 = new QGroupBox;
    string detector_names[] = {"KLT Detector", "Harris Corner Detector",
                               "BCD (Binary Corner Detector)", "SIFT",
                               "SURF", "FAST Detector",
                               "FASTER9 Detector", "FASTER10 Detector",
                               "FASTER12", "ORB Detector",
                               "AKAZE Detector", "LSD Detector"};

    detectors_select = new QComboBox;

    for(int i=0 ; i<NUM_DETECTORS ; i++)
        detectors_select->addItem(detector_names[i].c_str());
    connect(detectors_select, SIGNAL(currentIndexChanged(int)),this,SLOT(on_detector_choose(int)) );


    QPushButton *detector_button = new QPushButton;
    detector_button->setText("Evaluate Detector");
    detector_button->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(detector_button, SIGNAL(clicked(bool)),this, SLOT(on_detector_button_clicked()));




    //Initialize the descriptors here
    //groupBox2 = new QGroupBox(tr("Select your descriptor"));
    string descriptor_names[] = {"SIFT Descriptor", "SURF Descriptor",
                                 "Intensity-domain spin image descriptor", "Polar Images descriptor",
                                 "Log-polar image descriptor",
                                 "LATCH Descriptor", "BLD Descriptor"};
    //"BRIEF Descriptors"};

    descriptors_select = new QComboBox;
    for(int i=0 ; i<NUM_DESCRIPTORS ; i++)
        descriptors_select->addItem(descriptor_names[i].c_str());

    connect(descriptors_select, SIGNAL(currentIndexChanged(int)),this,SLOT(on_descriptor_choose(int)) );


    QPushButton *descriptor_button = new QPushButton;
    descriptor_button->setText("Evaluate Descriptor");
    descriptor_button->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(descriptor_button, SIGNAL(clicked(bool)),this, SLOT(on_descriptor_button_clicked()));




    button_generate = new QPushButton("Visualize Descriptors");
    button_generate->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(button_generate, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));
    next_desc = new QPushButton("Next Descriptor");
    next_desc->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(next_desc, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));


    /*  QVBoxLayout *vbox2 = new QVBoxLayout;

      vbox2->addWidget(descriptors_select);
      vbox2->addWidget(descriptor_button);
      groupBox2->setLayout(vbox2);
  */
    QLabel *selectDetector = new QLabel("<b>Select detector</b>");
    QLabel *selectDescriptor = new QLabel("<b>Select descriptor</b>");

    QGridLayout *vbox = new QGridLayout;

    vbox->addWidget(selectDetector,0,0);
    vbox->addWidget(selectDescriptor,0,1);
    vbox->addWidget(detectors_select,1,0);
    vbox->addWidget(detector_button,2,0);
    vbox->addWidget(descriptors_select,1,1);
    vbox->addWidget(descriptor_button,2,1);
    vbox->addWidget(button_generate,3,0);
    vbox->addWidget(next_desc,3,1);
    next_desc->setVisible(false);
    groupBox1->setLayout(vbox);



    //Displaying the pair of images here
    groupBox_images = new QGroupBox ("Single Image");
    image1 = new QLabel;
    qimage1.load("../../apps/benchmarking-image-features/images/1.png"); // replace this with initial image of select an image by specifying path
    QImage qscaled1 = qimage1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));

    image2 = new QLabel;
    qimage2.load("../../apps/benchmarking-image-features/images/2.png"); // replace this with initial image of select an image by specifying path
    QImage qscaled2 = qimage2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image2->setPixmap(QPixmap::fromImage(qscaled2));

    QGridLayout *hbox_images = new QGridLayout;
    hbox_images->addWidget(image1,0,0);
    hbox_images->addWidget(image2,0,1);
    groupBox_images->setLayout(hbox_images);


    //provide user input image options
    QGroupBox *inputGroupBox = new QGroupBox;
    QLabel *inputLabel = new QLabel("<b>Specify the input data format</b>");
    inputLabel->setFixedHeight(WIDGET_HEIGHT);
    inputs = new QComboBox;
    inputs->addItem("Single Image");
    inputs->addItem("Stereo Image");
    inputs->addItem("Image Rawlog file");
    inputs->addItem("Single Image Dataset");
    inputs->addItem("Stereo Image Dataset");

    inputs->setFixedSize(180,WIDGET_HEIGHT);

    connect(inputs, SIGNAL(currentIndexChanged(int)), this, SLOT(on_file_input_choose(int)));

    inputFilePath = new QLineEdit;
    inputFilePath->setFixedSize(300,WIDGET_HEIGHT);
    browse_button = new QPushButton("Browse1");
    browse_button->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(browse_button, SIGNAL(clicked()), this, SLOT(on_browse_button_clicked()));

    inputFilePath2 = new QLineEdit;
    inputFilePath2->setFixedSize(300,WIDGET_HEIGHT);
    browse_button2 = new QPushButton("Browse2");
    browse_button2->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);

    connect(browse_button2, SIGNAL(clicked()), this, SLOT(on_browse_button_clicked2()));
    //initially have the buttons hidden as single image selected by default
    inputFilePath2->setVisible(false);
    browse_button2->setVisible(false);





    QGridLayout *inputVbox = new QGridLayout;

    inputVbox->addWidget(inputLabel,0,0);
    inputVbox->addWidget(inputs,1,0);
    inputVbox->addWidget(inputFilePath,2,0);
    inputVbox->addWidget(browse_button,3,0);

    inputVbox->addWidget(inputFilePath2,4,0);
    inputVbox->addWidget(browse_button2,5,0);

    inputGroupBox->setLayout(inputVbox);

    //ask user for the number of feature
    QLabel *numFeaturesLabel = new QLabel("Enter the number of features to be detected: ");
    numFeaturesLineEdit = new QLineEdit;
    numFeaturesLineEdit->setFixedSize(PARAMS_WIDTH, WIDGET_HEIGHT);
    numFeaturesLineEdit->setText("100");

    inputVbox->addWidget(numFeaturesLabel,6,0);
    inputVbox->addWidget(numFeaturesLineEdit,6,1);


    // provide user with some additional functions
    QGroupBox *userOptionsGroupBox = new QGroupBox;
    stereo_matching = new QCheckBox;
    stereo_matching->setText("Activate Stereo Matching");
    connect(stereo_matching, SIGNAL(stateChanged(int)), this, SLOT(onStereoMatchingChecked(int)));
    QVBoxLayout *userOptionsVBox = new QVBoxLayout;
    userOptionsVBox->addWidget(stereo_matching);
    userOptionsGroupBox->setLayout(userOptionsVBox);



    QGroupBox *paramsGroupBox = new QGroupBox;
    QLabel *detector_param_label = new QLabel("<b>Detector Parameters: </b>");
    QLabel *descriptor_param_label = new QLabel("<b>Descriptor Parameters: </b>");
    detector_param_label->setFixedHeight(WIDGET_HEIGHT);
    descriptor_param_label->setFixedHeight(WIDGET_HEIGHT);

    initializeParameters();


    QGridLayout *paramVBox = new QGridLayout;



    paramVBox->addWidget(detector_param_label,0,0);
    paramVBox->addWidget(param1,1,0,1,1);
    paramVBox->addWidget(param1_edit,1,1,1,1);
    paramVBox->addWidget(param2,2,0,1,1);
    paramVBox->addWidget(param2_edit,2,1,1,1);
    paramVBox->addWidget(param3,3,0,1,1);
    paramVBox->addWidget(param3_edit,3,1,1,1);
    paramVBox->addWidget(param4,4,0,1,1);
    paramVBox->addWidget(param4_edit,4,1,1,1);
    paramVBox->addWidget(param5,5,0,1,1);
    paramVBox->addWidget(param5_edit,5,1,1,1);

    paramVBox->addWidget(descriptor_param_label,0,2);
    paramVBox->addWidget(param1_desc,1,2);
    paramVBox->addWidget(param1_edit_desc,1,3);
    paramVBox->addWidget(param2_desc,2,2);
    paramVBox->addWidget(param2_edit_desc,2,3);
    paramVBox->addWidget(param3_desc,3,2);
    paramVBox->addWidget(param3_edit_desc,3,3);
    paramVBox->addWidget(param4_desc,4,2);
    paramVBox->addWidget(param4_edit_desc,4,3);
    paramVBox->addWidget(param5_desc,5,2);
    paramVBox->addWidget(param5_edit_desc,5,3);
    paramVBox->setSizeConstraint(QLayout::SetMinimumSize);

    paramsGroupBox->setLayout(paramVBox);
    paramsGroupBox->setBaseSize(320,180);



    // initializing the buttons here
    QGroupBox *groupBox_buttons = new QGroupBox;
    button_generate = new QPushButton("Visualize Descriptors");
    button_generate->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(button_generate, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));

    button_close = new QPushButton("Close");
    button_close->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(button_close,SIGNAL(clicked(bool)),this,SLOT(button_close_clicked()));
    QGridLayout *hbox1 = new QGridLayout;
    hbox1->addWidget(button_close,0,0);
    hbox1->addWidget(button_generate,0,1);
    groupBox_buttons->setLayout(hbox1);


    QGroupBox *groupBox_buttons2 = new QGroupBox;
    QGridLayout *hbox2 = new QGridLayout;
    next_button = new QPushButton("Next");
    prev_button = new QPushButton("Previous");
    next_button->setFixedSize(PARAMS_WIDTH,PARAMS_HEIGHT);
    prev_button->setFixedSize(PARAMS_WIDTH,PARAMS_HEIGHT);
    next_button->setVisible(false);
    prev_button->setVisible(false);

    connect(next_button, SIGNAL(clicked(bool)), this, SLOT(on_next_button_clicked()));
    connect(prev_button, SIGNAL(clicked(bool)), this, SLOT(on_prev_button_clicked()));
    hbox2->addWidget(prev_button,0,0);
    hbox2->addWidget(next_button,0,1);
    groupBox_buttons2->setLayout(hbox2);


    QGroupBox *decimateImage = new QGroupBox;
    QGridLayout *vbox3 = new QGridLayout;
    QPushButton *sample = new QPushButton("Decimate");
    sample->setFixedSize(BUTTON_WIDTH, BUTTON_HEIGHT);


    decimateFactor = new QLineEdit;
    decimateFactor->setText("0.1");
    decimateFactor->setFixedSize(PARAMS_WIDTH,PARAMS_HEIGHT);

    QLabel *decimateLabel = new QLabel("<b>Image Decimation options </b>");
    connect(sample, SIGNAL(clicked(bool)), this, SLOT(on_sample_clicked()));


    vbox3->addWidget(decimateLabel,0,0);
    vbox3->addWidget(decimateFactor,1,0,1,1);
    vbox3->addWidget(sample,1,1,1,1);

    decimateImage->setLayout(vbox3);





    //ADD DESCRIPTOR IMAGES FOR VISUALIZATION
    QGroupBox *desc_images = new QGroupBox;
    desc_VisualizeGrid = new QGridLayout;

    for(int i=0 ; i< DESCRIPTOR_GRID_SIZE ; i++)
    {
        images_label_coordinates[i] = new QLabel("*");
        images_static[i] = new QLabel;
        descriptors[i].load("/home/raghavender/Downloads/images1.jpg"); // replace this with initial image of select an image by specifying path
        QImage qscaled1 = descriptors[i].scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_HEIGHT, Qt::KeepAspectRatio);
        images_static[i]->setPixmap(QPixmap::fromImage(qscaled1));
        int i_t = 2*(i/DESCRIPTOR_ROW_SIZE);
        int j_t = 2*(i%DESCRIPTOR_ROW_SIZE);
        //desc_VisualizeGrid->addWidget(images_static[i],i_t,j_t,1,1);
        //desc_VisualizeGrid->addWidget(images_label_coordinates[i],i_t+1, j_t+1, 1,1);
    }
    for(int i=0 ; i<DESCRIPTOR_GRID_SIZE2 ; i++)
        images_static_sift_surf[i] = new QLabel;

    desc_images->setLayout(desc_VisualizeGrid);

    numDesc1 = 0;
    numDesc2 = 0;



    layout_grid = new QGridLayout;

    //layout_grid->addWidget(groupBox2,0,1,1,1);


    layout_grid->addWidget(groupBox_images,0,0,12,4);
    layout_grid->addWidget(groupBox_buttons2,12,0,1,2);
    layout_grid->addWidget(desc_images,13,0,4,4);

    layout_grid->addWidget(groupBox1,2,4,3,2);
    layout_grid->addWidget(inputGroupBox,5,4,6,1);
    layout_grid->addWidget(decimateImage,11,4,2,1);
    layout_grid->addWidget(paramsGroupBox,13,4,12,2);

    //layout_grid->addWidget(groupBox_buttons2,3,0,1,1);
    //layout_grid->addWidget(groupBox_buttons,4,0,1,1);

    //layout_grid->addWidget(userOptionsGroupBox,1,2,1,1);
    //layout_grid->addWidget(paramsGroupBox,2,2,1,1);
    //layout_grid->addWidget(decimateImage,3,2,1,1);


    layout_grid->setSizeConstraint(QLayout::SetMinimumSize);


    window_gui->setLayout(layout_grid);
    window_gui->show();
}

