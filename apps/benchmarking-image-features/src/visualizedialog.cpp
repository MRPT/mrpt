#include "visualizedialog.h"
#include "mainwindow.h"

#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/system/threads.h>

#include <opencv2/plot.hpp>

#include <sstream>
#include <iostream>


using namespace std;

using namespace cv;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt;

int detector_selected;
int descriptor_selected;

QGridLayout *layout_grid;

void VisualizeDialog::on_next_button_clicked()
{
    cout <<"You clicked me next" << endl;

    int numDesc = feats1.size();
    cout << "In next button clicked, number of features : " << numDesc << endl;
    QImage qimage_1[numDesc], qimage_2[numDesc];
    QLabel *images1[numDesc], *images2[numDesc];


    //MAIN Descriptor description starts here
    CTicTac tictac;


    if (true)
    {

        for (unsigned int i1 = 0; i1 < numDesc; i1++) {
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
                        auxImg1 = feats1[i1]->patch;

                    } else if (descriptor_selected == 3) // descPolarImages
                    {
                        auxImg1.setFromMatrix(feats1[i1]->descriptors.PolarImg);
                    } else if (descriptor_selected == 4)  // descLogPolarImages
                    {
                        auxImg1.setFromMatrix(feats1[i1]->descriptors.LogPolarImg);

                    } else if (descriptor_selected == 2) // descSpinImages
                    {

                        const size_t nR = feats1[i1]->descriptors.SpinImg_range_rows;
                        const size_t nC =
                                feats1[i1]->descriptors.SpinImg.size() / feats1[i1]->descriptors.SpinImg_range_rows;
                        CMatrixFloat M1(nR, nC);
                        for (size_t r = 0; r < nR; r++)
                            for (size_t c = 0; c < nC; c++)
                                M1(r, c) = feats1[i1]->descriptors.SpinImg[c + r * nC];
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
                    vector<float> v1;
                    mrpt::utils::metaprogramming::copy_container_typecasting(feats1[i1]->descriptors.SIFT, v1);

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
        }
        flag = false;
    } // this switch case is executed only once to store images1


    cv::Mat desc_Ref_img = imread(filePath->toStdString(),IMREAD_COLOR);


    cout << "before the images for loop" << endl;

    if(descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
    {
        for (int i = 0; i < DESCRIPTOR_GRID_SIZE; i++) {
            images_static[i] = images1[cnt];
            images_label_coordinates[i] = new QLabel;

            float temp_x = feats1.getFeatureX(cnt); // get the descriptor x coordinate corresponding to images1[cnt]
            float temp_y = feats1.getFeatureY(cnt);

            stringstream ss;
            ss << temp_x << " , " << temp_y << endl;
            string str = ss.str();
            images_label_coordinates[i]->setText(QString::fromStdString(str));
            cout << str << endl;


            circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0); // plot the circles on the appropriate points as per the shown descriptors


            cnt++; // global counter for iterating over images and labels
        }
    }
    else if(descriptor_selected == 0 || descriptor_selected == 1)
    {
        for (int i = 0; i < DESCRIPTOR_GRID_SIZE2; i++)
        {
            images_static_sift_surf[i] = images1[cnt];
            images_label_coordinates_sift_surf[i] = new QLabel;

            float temp_x = feats1.getFeatureX(cnt); // get the descriptor x coordinate corresponding to images1[cnt]
            float temp_y = feats1.getFeatureY(cnt);

            stringstream ss;
            ss << temp_x << " , " << temp_y << endl;
            string str = ss.str();
            images_label_coordinates_sift_surf[i]->setText(QString::fromStdString(str));
            cout << str << endl;


            circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(1), 2, 8, 0); // plot the circles on the appropriate points as per the shown descriptors


            cnt++; // global counter for iterating over images and labels
        }
    }

    // cnt is a global counter update for the all the descriptors present in the image
    // need to add condition when cnt > total number of features, break or start from 0 again, should be pretty straightforward
    // do something like cnt % total_descriptors


    /// MAIN Descriptor description ends here



    // adding images of descriptor in a 5by5 image grid matrix

    cout << "before the adding to widgets for loop" << endl;
    int desc_row = 5;
    if(descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
        desc_row = 5;
    else if(descriptor_selected == 0 || descriptor_selected == 1)
        desc_row = 3;

    for (int i=0,k=0 ; i< desc_row ; i++)
    {
        for (int j=0 ; j<desc_row && bnt < numDesc ; j++, k++  ) // k runs from 0 to 24
        {
            if(descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
                layout_grid->addWidget(images_static[k], i, j, 1, 1);
            else if(descriptor_selected == 0 || descriptor_selected == 1)
                layout_grid->addWidget(images_static_sift_surf[k],i,j,3,4);
            //layout_grid->addWidget(images_label_coordinates[k],i+7,j,1,1);
        }
    }

    //layout_grid->addWidget(images_label_coordinates[3],12,12,1,1);
    // Display the reference image below


    cv::Mat temp2 (desc_Ref_img.cols, desc_Ref_img.rows, desc_Ref_img.type());
    cvtColor(desc_Ref_img, temp2, CV_BGR2RGB);
    QImage dest2 = QImage((uchar*) desc_Ref_img.data, desc_Ref_img.cols, desc_Ref_img.rows, desc_Ref_img.step, QImage::Format_RGB888);
    QImage referenceImg = dest2.scaled(IMAGE_HEIGHT, IMAGE_WIDTH, Qt::KeepAspectRatio);
    QLabel *reference_Img_label = new QLabel;
    reference_Img_label->setPixmap(QPixmap::fromImage(referenceImg));

    int offset = 2;
    if (descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
        offset = 2;
    else offset = 8;
    layout_grid->addWidget(reference_Img_label,0,desc_row+offset,4,4);

    next_button_clicked_counter++;
}
void VisualizeDialog::on_prev_button_clicked()
{
    cout <<"You clicked me" << endl;
}
void VisualizeDialog::oncloseDialog_clicked()
{
    feature_gui->setVisible(false);
    feature_gui->close();
    //this->close();
    return;
}

VisualizeDialog::VisualizeDialog(QWidget *parent, QString  filePath, int detector_id, int descriptor_id, int numDescriptors, CFeatureList desc_Image1, CFeatureList desc_Image2) :QDialog(feature_gui)
{
    cout << "started Visualize Dialog " << endl;
    feature_gui = new QWidget;
    feature_gui->setWindowTitle("Visualize Detector / Descriptor");
    layout_grid = new QGridLayout;

    descriptor_selected = descriptor_id;
    detector_selected = detector_id;










    flag = true;
    flag2 = true;
    next_button_clicked_counter= 0;
    bnt = 0;
    cnt = 0;
    this->filePath = new QString(filePath);


    string detector_names[] = {"KLT Detector", "Harris Corner Detector",
                               "BCD (Binary Corner Detector)", "SIFT",
                               "SURF", "FAST Detector",
                               "FASTER9 Detector", "FASTER10 Detector",
                               "FASTER12", "ORB Detector",
                               "AKAZE Detector", "LSD Detector"};

    string descriptor_names[] = {"SIFT Descriptor", "SURF Descriptor",
                                 "Intensity-domain spin image descriptor", "Polar Images descriptor",
                                 "Log-polar image descriptor",
                                 "LATCH Descriptor", "BLD Descriptor"};

    QString details("<b>DETECTOR Selected: </b>" +QString::fromStdString(detector_names[detector_id])  + "\n  <b>DESCRIPTOR Selected: </b> "+ QString::fromStdString(descriptor_names[descriptor_id]));

    QLabel *label = new QLabel(details);

    QPushButton *next_button = new QPushButton("Next Descriptor");
    connect(next_button, SIGNAL(clicked(bool)), this, SLOT(on_next_button_clicked()) );

    QPushButton *prev_button = new QPushButton("Prev Descriptor");
    connect(prev_button, SIGNAL(clicked(bool)),this, SLOT(on_prev_button_clicked()));

    //connect(detector_button, SIGNAL(clicked(bool)),this, SLOT(on_detector_button_clicked()));

    QPushButton *close_button = new QPushButton("Close");
    connect(close_button, SIGNAL(clicked()), this, SLOT(oncloseDialog_clicked()) );

    if( detector_id == -1 || descriptor_id == -1 || filePath.isEmpty())
    {
        QMessageBox::information(this,"Image/Detector/Descriptor read error","Please specify a valid inputs for the image / detector..");
        return;
    }

    feats1 = desc_Image1;
    feats2 = desc_Image2;

    int desc_row = 5;


    layout_grid->addWidget(close_button,desc_row,0,1,1);
    layout_grid->addWidget(prev_button,desc_row,1,1,1);
    layout_grid->addWidget(next_button,desc_row,2,1,1);
    layout_grid->addWidget(label,5,desc_row+2,1,1);

    feature_gui->setLayout(layout_grid);
    feature_gui->show();
}




