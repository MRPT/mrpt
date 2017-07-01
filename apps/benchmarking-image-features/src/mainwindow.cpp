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
#include <mrpt/math/data_utils.h>
#include <QEvent>
#include <QMouseEvent>


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

QImage qimage_1_plots_distances[MAX_DESC], qimage_2_plots_distances[MAX_DESC];
QLabel *images1_plots_distances[MAX_DESC], *images2_plots_distances[MAX_DESC];
QLabel *featureMatchingInfo[MAX_DESC];
int min_dist_indexes[MAX_DESC];




//MAX_DESC is 500
cv::Mat cvImg1;


void MainWindow::computeMinMax(CVectorDouble distances, int &min_idx, double &min_dist, int &max_idx, double &max_dist)
{
    int pos;
    double min = 10000;
    for(int i=0 ;i < distances.size() ;i++)
    {
        //if(distances.)

    }
}

/*
 * This button is used to visualize the descriptors
 */
void MainWindow::on_button_generate_clicked()
{

    visualize_descriptor_clicked = true;

    //ReadInputFormat();

    double min_dist = 0, max_dist = 0;
    size_t min_dist_idx = 0, max_dist_idx = 0;
    numDesc1 = featsImage1.size();
    for (unsigned int i1 = 0; i1 < numDesc1; i1++)
    {
        // do the following only if stereo images
        if(currentInputIndex == 1 || currentInputIndex == 4)
        {
            CVectorDouble distances(featsImage2.size());
            if (descriptor_selected != -1) {
                for (unsigned int i2 = 0; i2 < featsImage2.size(); i2++)
                    distances[i2] = featsImage1[i1]->descriptorDistanceTo(*featsImage2[i2]);
            } else{
                for(unsigned int i2=0 ; i2<featsImage2.size() ; i2++)
                    distances[i2] = featsImage1[i1]->patchCorrelationTo(*featsImage2[i2]);
            }


            distances.minimum_maximum(min_dist, max_dist, &min_dist_idx, &max_dist_idx);

            const double dist_std = mrpt::math::stddev(distances);

            cout << "Min Distance : " << min_dist << " for image2 feature # " << min_dist_idx << "Distances sigma " << dist_std << endl;
            min_dist_indexes[i1] = min_dist_idx;

            stringstream info;
            info << "<b>Feature (Image1) # " << i1 <<" matches Feature (Image2) # " << min_dist_idx <<  "<br/> with distance " << min_dist << ", Distances sigma <b> " << dist_std;
            string info_temp = info.str();
            featureMatchingInfo[i1] = new QLabel();
            featureMatchingInfo[i1]->setText(QString::fromStdString(info_temp));


            //circle(cvImg2, Point(featsImage2.getFeatureX(min_dist_idx), featsImage2.getFeatureY(min_dist_idx)), 5, Scalar(255,255,0), 3, 8, 0);

            for(int i=0 ; i< distances.size() ; i++)
               ;// cout << distances.row(i).x() << " Distances" << endl;




            Mat xData, yData, display;
            Ptr<plot::Plot2d> plot;
            int len = distances.size();
            xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
            yData.create(1, len, CV_64F);

            for(int i = 0; i<len; ++i)
            {
                xData.at<double>(i) = i;
                yData.at<double>(i) = distances.row(i).x();
                cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
            }
            plot = plot::createPlot2d(xData, yData);
            plot->setPlotSize(len, 1);
            plot->setMaxX(distances.size());
            plot->setMinX(-15);
            plot->setMaxY(max_dist*1.15);
            plot->setMinY(-0.15*max_dist);
            plot->setNeedPlotLine(false);
            plot->render(display);
            plot->setPlotLineWidth(10);
            plot->setPlotLineColor(Scalar(255,0,0));
            plot->setPlotBackgroundColor(Scalar(255,255,255));
            plot->setPlotGridColor(Scalar(255,255,0));
            //imshow("plots", display);
            //waitKey();


            cv::Mat temp1(display.cols, display.rows, display.type());
            cvtColor(display, temp1, CV_RGB2BGR);
            //imshow("temp ", temp1);
            //waitKey();
            QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                  QImage::Format_RGB888);
            qimage_1_plots_distances[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
            images1_plots_distances[i1] = new QLabel;
            images1_plots_distances[i1]->setPixmap(QPixmap::fromImage(qimage_1_plots_distances[i1]));

        }



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
                    if(currentInputIndex == 1 || currentInputIndex == 4) {
                        auxImg2 = featsImage2[min_dist_idx]->patch;
                    }
                } else if (descriptor_selected == 3) // descPolarImages
                {
                    auxImg1.setFromMatrix(featsImage1[i1]->descriptors.PolarImg);
                    if(currentInputIndex == 1 || currentInputIndex == 4) {
                        auxImg2.setFromMatrix(featsImage2[min_dist_idx]->descriptors.PolarImg);
                    }

                } else if (descriptor_selected == 4)  // descLogPolarImages
                {
                    auxImg1.setFromMatrix(featsImage1[i1]->descriptors.LogPolarImg);
                    if(currentInputIndex == 1 || currentInputIndex == 4) {
                        auxImg2.setFromMatrix(featsImage2[min_dist_idx]->descriptors.LogPolarImg);
                    }

                } else if (descriptor_selected == 2) // descSpinImages
                {

                    const size_t nR1 = featsImage1[i1]->descriptors.SpinImg_range_rows;
                    const size_t nC1 =
                            featsImage1[i1]->descriptors.SpinImg.size() / featsImage1[i1]->descriptors.SpinImg_range_rows;
                    CMatrixFloat M1(nR1, nC1);
                    for (size_t r = 0; r < nR1; r++)
                        for (size_t c = 0; c < nC1; c++)
                            M1(r, c) = featsImage1[i1]->descriptors.SpinImg[c + r * nC1];
                    auxImg1.setFromMatrix(M1);
                    if(currentInputIndex == 1 || currentInputIndex == 4)
                    {
                        const size_t nR = featsImage2[min_dist_idx]->descriptors.SpinImg_range_rows;
                        const size_t nC = featsImage2[min_dist_idx]->descriptors.SpinImg.size()/featsImage2[min_dist_idx]->descriptors.SpinImg_range_rows;
                        CMatrixFloat M2(nR,nC);
                        for (size_t r=0;r<nR;r++)
                            for (size_t c=0;c<nC;c++)
                                M2(r,c)=featsImage2[min_dist_idx]->descriptors.SpinImg[c+r*nC];
                        auxImg2.setFromMatrix( M2 );
                    }

                }
                //cout << "before the while loop" << auxImg1.getWidth() << "  "<< auxImg1.getHeight() << "  " <<  endl;
                while (auxImg1.getWidth() < 100 && auxImg1.getHeight() < 100)
                    auxImg1.scaleImage(auxImg1.getWidth() * 2, auxImg1.getHeight() * 2, IMG_INTERP_NN);
                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    while (auxImg2.getWidth() < 100 && auxImg2.getHeight() < 100)
                        auxImg2.scaleImage(auxImg2.getWidth() * 2, auxImg2.getHeight() * 2, IMG_INTERP_NN);
                }
                //cout << "after the while loop" << endl;

                cv::Mat cvImg1 = cv::cvarrToMat(auxImg1.getAs<IplImage>());
                cv::Mat temp1(cvImg1.cols, cvImg1.rows, cvImg1.type());
                cvtColor(cvImg1, temp1, CV_GRAY2RGB);
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);


                qimage_1[i1] = dest1.scaled(DESCRIPTOR_HEIGHT, DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    cv::Mat cvImg2 = cv::cvarrToMat(auxImg2.getAs<IplImage>());
                    cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
                    cvtColor(cvImg2, temp2, CV_GRAY2RGB);
                    QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step,
                                          QImage::Format_RGB888);


                    qimage_2[i1] = dest2.scaled(DESCRIPTOR_HEIGHT, DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                    images2[i1] = new QLabel;
                    images2[i1]->setPixmap(QPixmap::fromImage(qimage_2[i1]));
                }
            }
                break;
                // FOR THE FOLLOWING SIFT, SURF AND ORB CLEAN THE CODE LATER BY REDUCING SIZ TO 1/3RD AS MOST OF IT IS DUPLICATED
            case 0: { // descSIFT
                vector<uint8_t > v1,v2;
                //mrpt::utils::metaprogramming::copy_container_typecasting(featsImage1[i1]->descriptors.SIFT, v1);
                v1 = featsImage1[i1]->descriptors.SIFT;



                Mat xData, yData, display;
                Ptr<plot::Plot2d> plot;
                int len = v1.size();
                xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                yData.create(1, len, CV_64F);

                for(int i = 0; i<len; ++i)
                {
                    xData.at<double>(i) = i;
                    yData.at<double>(i) = v1.at(i);
                    //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                }
                plot = plot::createPlot2d(xData, yData);
                plot->setPlotSize(len, 1);
                plot->setMaxX(len);
                plot->setMinX(0);
                plot->setMaxY(1);
                plot->setMinY(-1);
                plot->render(display);


                cv::Mat temp1(display.cols, display.rows, display.type());
                cvtColor(display, temp1, CV_RGB2BGR);
                //imshow("temp ", temp1);
                //waitKey();
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);
                qimage_1[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    v2 = featsImage2[min_dist_idx]->descriptors.SIFT;
                    Mat xData, yData, display2;
                    Ptr<plot::Plot2d> plot;
                    int len = v2.size();
                    xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                    yData.create(1, len, CV_64F);

                    for(int i = 0; i<len; ++i)
                    {
                        xData.at<double>(i) = i;
                        yData.at<double>(i) = v2.at(i);
                        //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                    }
                    plot = plot::createPlot2d(xData, yData);
                    plot->setPlotSize(len, 1);
                    plot->setMaxX(len);
                    plot->setMinX(0);
                    plot->setMaxY(1);
                    plot->setMinY(-1);
                    plot->render(display2);


                    cv::Mat temp2(display2.cols, display2.rows, display2.type());
                    cvtColor(display2, temp2, CV_RGB2BGR);
                    //imshow("temp ", temp1);
                    //waitKey();
                    QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step,
                                          QImage::Format_RGB888);
                    qimage_2[i1] = dest2.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                    images2[i1] = new QLabel;
                    images2[i1]->setPixmap(QPixmap::fromImage(qimage_2[i1]));
                }

            }
                break;
            case 1: { // descSURF
                vector<float> v1, v2;
                //cout << "SURF Features size: " << featsImage1.size() << endl;
                //mrpt::utils::metaprogramming::copy_container_typecasting(featsImage1[i1]->descriptors.SIFT, v1);
                v1 = featsImage1[i1]->descriptors.SURF;
                if(currentInputIndex == 1 || currentInputIndex == 4)
                    v2 = featsImage2[min_dist_idx]->descriptors.SURF;

                Mat xData, yData, display;
                Ptr<plot::Plot2d> plot;
                int len = v1.size();
                xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                yData.create(1, len, CV_64F);

                for(int i = 0; i<len; ++i)
                {
                    xData.at<double>(i) = i;
                    yData.at<double>(i) = v1.at(i);
                    //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                }
                plot = plot::createPlot2d(xData, yData);
                plot->setPlotSize(len, 1);
                plot->setMaxX(len);
                plot->setMinX(0);
                plot->setMaxY(1);
                plot->setMinY(-1);
                plot->render(display);


                cv::Mat temp1(display.cols, display.rows, display.type());
                cvtColor(display, temp1, CV_RGB2BGR);
                //imshow("temp ", temp1);
                //waitKey();
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);
                qimage_1[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    Mat xData, yData, display;
                    Ptr<plot::Plot2d> plot;
                    int len = v2.size();
                    xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                    yData.create(1, len, CV_64F);

                    for(int i = 0; i<len; ++i)
                    {
                        xData.at<double>(i) = i;
                        yData.at<double>(i) = v2.at(i);
                        //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                    }
                    plot = plot::createPlot2d(xData, yData);
                    plot->setPlotSize(len, 1);
                    plot->setMaxX(len);
                    plot->setMinX(0);
                    plot->setMaxY(1);
                    plot->setMinY(-1);
                    plot->render(display);


                    cv::Mat temp1(display.cols, display.rows, display.type());
                    cvtColor(display, temp1, CV_RGB2BGR);
                    //imshow("temp ", temp1);
                    //waitKey();
                    QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                          QImage::Format_RGB888);
                    qimage_2[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                    images2[i1] = new QLabel;
                    images2[i1]->setPixmap(QPixmap::fromImage(qimage_2[i1]));
                }
            }
                break;
            case 5:
            {

                vector<uint8_t > v1,v2;
                v1 = featsImage1[i1]->descriptors.ORB;

                Mat xData, yData, display;
                Ptr<plot::Plot2d> plot;
                int len = v1.size();
                xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                yData.create(1, len, CV_64F);

                for(int i = 0; i<len; ++i)
                {
                    xData.at<double>(i) = i;
                    yData.at<double>(i) = v1.at(i);
                    //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                }
                plot = plot::createPlot2d(xData, yData);
                plot->setPlotSize(len, 1);
                plot->setMaxX(len);
                plot->setMinX(0);
                plot->setMaxY(350);
                plot->setMinY(-350);
                plot->render(display);


                cv::Mat temp1(display.cols, display.rows, display.type());
                cvtColor(display, temp1, CV_RGB2BGR);
                //imshow("temp ", temp1);
                //waitKey();
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);
                qimage_1[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    v2 = featsImage2[min_dist_idx]->descriptors.ORB;

                    Mat xData, yData, display;
                    Ptr<plot::Plot2d> plot;
                    int len = v2.size();
                    xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                    yData.create(1, len, CV_64F);

                    for(int i = 0; i<len; ++i)
                    {
                        xData.at<double>(i) = i;
                        yData.at<double>(i) = v2.at(i);
                        //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                    }
                    plot = plot::createPlot2d(xData, yData);
                    plot->setPlotSize(len, 1);
                    plot->setMaxX(len);
                    plot->setMinX(0);
                    plot->setMaxY(350);
                    plot->setMinY(-350);
                    plot->render(display);


                    cv::Mat temp1(display.cols, display.rows, display.type());
                    cvtColor(display, temp1, CV_RGB2BGR);
                    //imshow("temp ", temp1);
                    //waitKey();
                    QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                          QImage::Format_RGB888);
                    qimage_2[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                    images2[i1] = new QLabel;
                    images2[i1]->setPixmap(QPixmap::fromImage(qimage_2[i1]));
                }


            } break;

            case 6: { // descBLD
                vector<uint8_t > v1,v2;
                //mrpt::utils::metaprogramming::copy_container_typecasting(featsImage1[i1]->descriptors.SIFT, v1);
                v1 = featsImage1[i1]->descriptors.BLD;



                Mat xData, yData, display;
                Ptr<plot::Plot2d> plot;
                int len = v1.size();
                xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                yData.create(1, len, CV_64F);

                for(int i = 0; i<len; ++i)
                {
                    xData.at<double>(i) = i;
                    yData.at<double>(i) = v1.at(i);
                    //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                }
                plot = plot::createPlot2d(xData, yData);
                plot->setPlotSize(len, 1);
                plot->setMaxX(len);
                plot->setMinX(0);
                plot->setMaxY(257);
                plot->setMinY(-2);
                plot->render(display);


                cv::Mat temp1(display.cols, display.rows, display.type());
                cvtColor(display, temp1, CV_RGB2BGR);
                //imshow("temp ", temp1);
                //waitKey();
                QImage dest1 = QImage((uchar *) temp1.data, temp1.cols, temp1.rows, temp1.step,
                                      QImage::Format_RGB888);
                qimage_1[i1] = dest1.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                images1[i1] = new QLabel;
                images1[i1]->setPixmap(QPixmap::fromImage(qimage_1[i1]));

                if(currentInputIndex == 1 || currentInputIndex == 4)
                {
                    v2 = featsImage2[min_dist_idx]->descriptors.BLD;
                    Mat xData, yData, display2;
                    Ptr<plot::Plot2d> plot;
                    int len = v2.size();
                    xData.create(1, len, CV_64F);//1 Row, 100 columns, Double
                    yData.create(1, len, CV_64F);

                    for(int i = 0; i<len; ++i)
                    {
                        xData.at<double>(i) = i;
                        yData.at<double>(i) = v2.at(i);
                        //cout << yData.at<double>(i) << "  " << xData.at<double>(i) << endl;
                    }
                    plot = plot::createPlot2d(xData, yData);
                    plot->setPlotSize(len, 1);
                    plot->setMaxX(len);
                    plot->setMinX(0);
                    plot->setMaxY(257);
                    plot->setMinY(-2);
                    plot->render(display2);


                    cv::Mat temp2(display2.cols, display2.rows, display2.type());
                    cvtColor(display2, temp2, CV_RGB2BGR);
                    //imshow("temp ", temp1);
                    //waitKey();
                    QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step,
                                          QImage::Format_RGB888);
                    qimage_2[i1] = dest2.scaled(4*DESCRIPTOR_HEIGHT, 5*DESCRIPTOR_WIDTH, Qt::KeepAspectRatio);
                    images2[i1] = new QLabel;
                    images2[i1]->setPixmap(QPixmap::fromImage(qimage_2[i1]));
                }

            }
                break;



            default: {
                cerr << "Descriptor specified is not handled yet" << endl;
            }
                break;
        }
    }// end of for loop
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
    else if (choice == 5) // ORB Descriptor
    {
        param1_desc->setText("Extract Patch ");
        param2_desc->setText("Min Distance:: ");
        param3_desc->setText("Number of levels: ");
        param4_desc->setText("Scale factor: ");
        param1_edit_desc->setText("0");
        param2_edit_desc->setText("false");
        param3_edit_desc->setText("8");
        param4_edit_desc->setText("1.2");

        param5_desc->setVisible(false);
        param5_edit_desc->setVisible(false);
    }
    else if (choice == 6) // BLD Descriptor
    {
        int ksize_;
        int reductionRatio;
        int numOfOctave;
        int widthOfBand;
        param1_desc->setText("nOctaves ");
        param2_desc->setText("reduction Ratio: ");
        param3_desc->setText("width of Band: ");
        param4_desc->setText("ksize: ");
        param1_edit_desc->setText("1");
        param2_edit_desc->setText("2");
        param3_edit_desc->setText("7");
        param4_edit_desc->setText("1");

        param5_desc->setVisible(false);
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
    else if (choice == 10)
    {
        param1->setText("Descriptor Size: ");
        param2->setText("Descriptor Channels: ");
        param3->setText("Threshold: ");
        param4->setText("nOctaves: ");
        param5->setText("nOctave Layers");
        param1_edit->setText("0");
        param2_edit->setText("3");
        param3_edit->setText("0.001");
        param4_edit->setText("4");
        param5_edit->setText("4");

    }
    else if (choice == 11)
    {
        param1->setText("Scale: ");
        param4->setText("nOctaves: ");
        param1_edit->setText("2");
        param2_edit->setText("1");

        param3->setVisible(false);
        param4->setVisible(false);
        param5->setVisible(false);
        param3_edit->setVisible(false);
        param4_edit->setVisible(false);
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
    if(stereo_matching->isChecked() && (currentInputIndex == 1 || currentInputIndex == 4))
    {
        activate_stereo_matching = true;
        cout << " You clicked me .!!" << endl;
        /*
         * CMatchedFeatureList mHarris, mSIFT, mHarris_SAD, mFAST_CC, mFAST_SAD;
        TMatchingOptions opts;

        CTicTac tic;
        tic.Tic();

        opts.matching_method = TMatchingOptions::mmSAD;
        matchFeatures(featsImage1, featsImage2, mHarris_SAD, opts);
        cout << "featsImage1 size" << featsImage1.size() << " featsImage2.seze " << featsImage2.size() << endl;
        cout << "SAD matches found: " << mHarris_SAD.size() <<" in seconds" << tic.Tac()*1000.0f << endl;
*/
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
        klt_opts.min_distance       = param1_edit->text().toFloat();
        klt_opts.radius             = param2_edit->text().toInt();
        klt_opts.threshold          = param3_edit->text().toFloat();
        string temp_str             = param4_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        klt_opts.tile_image = temp_bool;

        fext.options.featsType = featKLT;

        fext.options.KLTOptions.min_distance    = klt_opts.min_distance;
        fext.options.KLTOptions.radius          = klt_opts.radius;
        fext.options.KLTOptions.threshold       = klt_opts.threshold;
        fext.options.KLTOptions.tile_image      = klt_opts.tile_image;

        cout << "detecting KLT Features " << endl ;
    }
    else if(detector_selected == 1) //Harris Features
    {
        harris_opts.threshold       = param1_edit->text().toFloat();
        harris_opts.k               = param2_edit->text().toFloat();
        harris_opts.sigma           = param3_edit->text().toFloat();
        harris_opts.radius          = param4_edit->text().toFloat();
        string temp_str             = param5_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") ? false : true;
        harris_opts.tile_image = temp_bool;

        fext.options.featsType = featHarris;

        fext.options.harrisOptions.threshold    = harris_opts.threshold;//0.005;
        fext.options.harrisOptions.k            =  harris_opts.k;  // default sensitivity
        fext.options.harrisOptions.sigma        = harris_opts.sigma;  // default from matlab smoothing filter
        fext.options.harrisOptions.radius       = harris_opts.radius;  // default block size
        //fext.options.harrisOptions.min_distance = 100;
        fext.options.harrisOptions.tile_image   = harris_opts.tile_image;

        cout << "detecting Harris Features " << endl ;
    }
    else if(detector_selected == 2) // not implemented in MRPT yet
    {
        fext.options.featsType = featBCD;
        cout <<"detecting BCD Features" << endl;

    }
    else if(detector_selected == 3) // SIFT Detector
    {
        SIFT_opts.threshold         = param1_edit->text().toFloat();
        SIFT_opts.edge_threshold    = param2_edit->text().toFloat();

        fext.options.featsType = featSIFT;

        fext.options.SIFTOptions.threshold      = SIFT_opts.threshold;
        fext.options.SIFTOptions.edgeThreshold  = SIFT_opts.edge_threshold;
        //fext.options.SIFTOptions.implementation = CFeatureExtraction::CSBinary;

        cout << "detecting SIFT Features " << endl ;

    }
    else if (detector_selected == 4) // 4= SURF Detector
    {
        fext.options.featsType = featSURF;
        SURF_opts.hessianThreshold      = param1_edit->text().toInt();
        SURF_opts.nLayersPerOctave      = param2_edit->text().toInt();
        SURF_opts.nOctaves              = param3_edit->text().toInt();
        string temp_str                 = param4_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        SURF_opts.rotation_invariant = temp_bool;
        cout <<  temp_bool << endl;

        fext.options.SURFOptions.hessianThreshold   = SURF_opts.hessianThreshold;
        fext.options.SURFOptions.nLayersPerOctave   = SURF_opts.nLayersPerOctave;
        fext.options.SURFOptions.nOctaves           = SURF_opts.nOctaves;
        fext.options.SURFOptions.rotation_invariant = SURF_opts.rotation_invariant;
    }
    else if(detector_selected == 5 || detector_selected == 6 || detector_selected == 7 || detector_selected == 8) //FAST detector and its variants
    {
        fast_opts.threshold         = param1_edit->text().toFloat();
        fast_opts.min_distance      = param2_edit->text().toFloat();
        string temp_str             = param3_edit->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        fast_opts.use_KLT_response = temp_bool;

        temp_str        = param4_edit->text().toStdString();
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

        fext.options.FASTOptions.threshold          = fast_opts.threshold;
        fext.options.FASTOptions.min_distance       = fast_opts.min_distance;
        fext.options.FASTOptions.use_KLT_response   = fast_opts.use_KLT_response;
        fext.options.FASTOptions.nonmax_suppression = fast_opts.non_max_suppresion;
    }
    else if(detector_selected == 9) // ORB Feature detector
    {
        fext.options.featsType = featORB;

        ORB_opts.min_distance       = param1_edit->text().toInt();
        string temp_str             = param2_edit->text().toStdString();
        bool temp_bool              = temp_str.compare("true") == 0;
        ORB_opts.extract_patch      = temp_bool;
        ORB_opts.n_levels           = param3_edit->text().toInt();
        ORB_opts.scale_factor       = param4_edit->text().toFloat();


        fext.options.ORBOptions.min_distance    = ORB_opts.min_distance;
        fext.options.ORBOptions.extract_patch   = ORB_opts.extract_patch;
        fext.options.ORBOptions.n_levels        = ORB_opts.n_levels;
        fext.options.ORBOptions.scale_factor    = ORB_opts.scale_factor;

    }
    else if(detector_selected == 10) // AKAZE Feature detector
    {
        fext.options.featsType = featAKAZE;

        AKAZE_opts.descriptor_size          = param1_edit->text().toInt();
        AKAZE_opts.descriptor_channels      = param2_edit->text().toInt();
        AKAZE_opts.threshold                = param3_edit->text().toFloat();
        AKAZE_opts.nOctaves                 = param4_edit->text().toInt();
        AKAZE_opts.nOctaveLayers            = param5_edit->text().toInt();

        fext.options.AKAZEOptions.descriptor_size       = AKAZE_opts.descriptor_size;
        fext.options.AKAZEOptions.descriptor_channels   = AKAZE_opts.descriptor_channels;
        fext.options.AKAZEOptions.threshold             = AKAZE_opts.threshold;
        fext.options.AKAZEOptions.nOctaves              = AKAZE_opts.nOctaves;
        fext.options.AKAZEOptions.nOctaveLayers         = AKAZE_opts.nOctaveLayers;

    }
    else if(detector_selected == 11) // LSD Feature detector
    {
        fext.options.featsType = featLSD;

        LSD_opts.scale      = param1_edit->text().toInt();
        LSD_opts.nOctaves   = param2_edit->text().toInt();


        fext.options.LSDOptions.scale       = LSD_opts.scale;
        fext.options.LSDOptions.nOctaves    = LSD_opts.nOctaves;

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
        SIFT_opts.threshold         = param1_edit_desc->text().toFloat();
        SIFT_opts.edge_threshold    = param2_edit_desc->text().toFloat();

        desc_to_compute = TDescriptorType (1); //!< SIFT descriptors

        fext.options.SIFTOptions.threshold      = SIFT_opts.threshold;
        fext.options.SIFTOptions.edgeThreshold  = SIFT_opts.edge_threshold;
        //fext.options.SIFTOptions.implementation = CFeatureExtraction::CSBinary;
    }
    else if(descriptor_selected == 1)
    {
        desc_to_compute = TDescriptorType (2); //!< SURF descriptors

        SURF_opts.hessianThreshold      = param1_edit_desc->text().toInt();
        SURF_opts.nLayersPerOctave      = param2_edit_desc->text().toInt();
        SURF_opts.nOctaves              = param3_edit_desc->text().toInt();
        string temp_str                 = param4_edit_desc->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        SURF_opts.rotation_invariant = temp_bool;
        cout <<  temp_bool << endl;

        fext.options.SURFOptions.hessianThreshold       =  SURF_opts.hessianThreshold;
        fext.options.SURFOptions.nLayersPerOctave       =  SURF_opts.nLayersPerOctave;
        fext.options.SURFOptions.nOctaves               = SURF_opts.nOctaves;
        fext.options.SURFOptions.rotation_invariant     = SURF_opts.rotation_invariant;
    }
    else if(descriptor_selected == 2)
    {
        desc_to_compute = TDescriptorType(4); //!< Intensity-domain spin image descriptors

        spin_opts.radius                = param1_edit_desc->text().toInt();
        spin_opts.hist_size_intensity   = param2_edit_desc->text().toInt();
        spin_opts.hist_size_distance    = param3_edit_desc->text().toInt();
        spin_opts.std_dist              = param4_edit_desc->text().toFloat();
        spin_opts.std_intensity         = param5_edit_desc->text().toFloat();


        fext.options.SpinImagesOptions.radius               = spin_opts.radius;
        fext.options.SpinImagesOptions.hist_size_intensity  = spin_opts.hist_size_intensity;
        fext.options.SpinImagesOptions.hist_size_distance   = spin_opts.hist_size_distance ;
        fext.options.SpinImagesOptions.std_dist             = spin_opts.std_dist;
        fext.options.SpinImagesOptions.std_intensity        = spin_opts.std_intensity;

    }
    else if(descriptor_selected == 3)
    {
        desc_to_compute = TDescriptorType(8); //!< Polar image descriptor

        polar_opts.radius           = param1_edit_desc->text().toInt();
        polar_opts.bins_angle       = param2_edit_desc->text().toInt();
        polar_opts.bins_distance    = param3_edit_desc->text().toInt();

        fext.options.PolarImagesOptions.radius          = polar_opts.radius;
        fext.options.PolarImagesOptions.bins_angle      = polar_opts.bins_angle;
        fext.options.PolarImagesOptions.bins_distance   = polar_opts.bins_distance;
    }
    else if(descriptor_selected == 4)
    {
        desc_to_compute = TDescriptorType (16); //!< Log-Polar image descriptor

        log_polar_opts.radius       = param1_edit_desc->text().toInt();
        log_polar_opts.num_angles   = param2_edit_desc->text().toInt();
        log_polar_opts.rho_scale    = param3_edit_desc->text().toFloat();

        fext.options.LogPolarImagesOptions.radius       = log_polar_opts.radius;
        fext.options.LogPolarImagesOptions.num_angles   = log_polar_opts.num_angles;
        fext.options.LogPolarImagesOptions.rho_scale    = log_polar_opts.rho_scale;
    }
    else if(descriptor_selected == 5)
    {
        desc_to_compute = TDescriptorType (32); //!< ORB image descriptor

        string temp_str = param1_edit_desc->text().toStdString();
        bool temp_bool = temp_str.compare("true") == 0;
        ORB_opts.extract_patch  = temp_bool;
        cout <<  temp_bool << endl;

        ORB_opts.min_distance       = param2_edit_desc->text().toInt();
        ORB_opts.n_levels           = param3_edit_desc->text().toInt();
        ORB_opts.scale_factor       = param4_edit_desc->text().toFloat();

        fext.options.ORBOptions.extract_patch       = ORB_opts.extract_patch;
        fext.options.ORBOptions.min_distance        = ORB_opts.min_distance;
        fext.options.ORBOptions.n_levels            = ORB_opts.n_levels;
        fext.options.ORBOptions.scale_factor        = ORB_opts.scale_factor;

    }
    else if(descriptor_selected == 6)
    {
        desc_to_compute = TDescriptorType (64); //!< BLD image descriptor


        BLD_opts.numOfOctave        = param1_edit_desc->text().toInt();
        BLD_opts.widthOfBand        = param2_edit_desc->text().toInt();
        BLD_opts.reductionRatio     = param3_edit_desc->text().toInt();
        BLD_opts.ksize_             = param4_edit_desc->text().toInt();

        fext.options.BLDOptions.numOfOctave         = BLD_opts.numOfOctave;
        fext.options.BLDOptions.widthOfBand         = BLD_opts.widthOfBand;
        fext.options.BLDOptions.reductionRatio      = BLD_opts.reductionRatio;
        fext.options.BLDOptions.ksize_              = BLD_opts.ksize_;
    }
}

/*
 *
 * this function is called to show the performance of the selected detector on the input selected by the user
 * the performance metric displayed is in terms of repeatability, dispersion of image, computational cost, number of found points, etc.
 * */
void MainWindow::on_detector_button_clicked()
{

    evaluate_detector_clicked = true;

    //read inputs from user
    ReadInputFormat();
    if( detector_selected == -1 || file_path1.empty())
    {
        QMessageBox::information(this, "Image/Detector/Descriptor read error","Please specify a valid inputs for the image / detector..");
        return;
    }
    //if(file_path1.empty() || file_path2.empty())
    {
        if(currentInputIndex == 3 || currentInputIndex == 4) // read files from folder for datasets
        {
            if(flag_read_files_bug)
            {
                readFilesFromFolder(2);
            }
        }
    }
    //cout << "in detector" << endl;

    // Feature Extraction Starts here
    fillDetectorInfo();

    // Clearing the features list is very important to avoid mixing subsequent button clicks output
    featsImage1.clear();
    fext.detectFeatures(img1, featsImage1, 0, numFeats);
    //save to file
    cout << "before saving to file" << endl;
    featsImage1.saveToTextFile("./KeyPoints1.txt");

    cout << "after saving to file" << endl;
    cvImg1 = cv::cvarrToMat(img1.getAs<IplImage>());
    // Drawing a circle around corners for image 1
    //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    for(int i=0 ; i<featsImage1.size() ; i++)
    {
        int temp_x = (int) featsImage1.getFeatureX(i);
        int temp_y = (int) featsImage1.getFeatureY(i);
        circle(cvImg1, Point(temp_x, temp_y), 5, Scalar(0,255,0), CIRCLE_THICKNESS, 8, 0);
    }
    drawLineLSD(cvImg1);


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

        //img2.loadFromFile(file_path2);
        cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());

        // Drawing a circle around corners for image 2
        //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        for (int i = 0; i < featsImage2.size(); i++) {
            int temp_x = (int) featsImage2.getFeatureX(i);
            int temp_y = (int) featsImage2.getFeatureY(i);
            circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(0,255,0), CIRCLE_THICKNESS, 8, 0);
        }
        drawLineLSD(cvImg2);

        cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
        cvtColor(cvImg2, temp2, CV_BGR2RGB);
        QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
        QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
        image2->setPixmap(QPixmap::fromImage(qscaled2));
        cout << "Number of Features in image 2: " << featsImage2.size() << endl;

    }
}

/*
 *
 * this function is called to show the performance of the selected descriptor on the input selected by the user
 * the performance metric displayed is in terms of percentage of patches matched, descriptor distance between close matches, false positives/negatives, computational cost, etc.
 * */
void MainWindow::on_descriptor_button_clicked()
{

    evaluate_descriptor_clicked = true;

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
    else if (descriptor_selected == 5)
        desc_to_compute = TDescriptorType (32); //!< ORB image descriptor
    else if (descriptor_selected == 6)
        desc_to_compute = TDescriptorType (64); //!< BLD image descriptor


    if(desc_to_compute != descAny)
        fext.options.patchSize = 0;


    // find a way to clear off past detector/descriptor info stored in featsImage

    if(desc_to_compute != TDescriptorType(-1) && desc_to_compute !=descAny)
    {
        fext.computeDescriptors(img1, featsImage1, desc_to_compute);
        if (currentInputIndex == 1 || currentInputIndex == 4)
        {
            fext.computeDescriptors(img2, featsImage2, desc_to_compute);
            featsImage2.saveToTextFile("./Key_Descriptors2");
        }
        featsImage1.saveToTextFile("./Key_Descriptors1.txt");

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
    flag_read_files_bug = true;
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
    flag_read_files_bug = true;
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

    if(currentInputIndex == 0)
    {
        file_path1 = inputFilePath->text().toStdString();
    }
    else if (currentInputIndex == 1) // only read the single images if a single image or stereo image input is specified
    {
        file_path1 = inputFilePath->text().toStdString();
        file_path2 = inputFilePath2->text().toStdString();
    }
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
    flag_read_files_bug = false;

    file_path1 = inputFilePath->text().toStdString();

    cout << file_path1 << endl;

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
    cout << current_imageIndex << "current IMage index" << endl;
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
        cout << "after while before for" << endl;
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
    cout << "yoyo" << endl;

    // add condition for the case when the current_imageIndex = the length of the total files start again from 0;
    // assumption both folders of the stereo image dataset have the same number of images
    if(next_prev == 1)
        current_imageIndex = (++current_imageIndex) %  files_fullpath.size();
    else if(next_prev == 2)
        current_imageIndex = current_imageIndex % files_fullpath.size();
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


    evaluate_detector_clicked = false;
    evaluate_descriptor_clicked = false;
    visualize_descriptor_clicked = false;
    flag_read_files_bug = true;


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
                                 "Log-polar image descriptor", "ORB Descriptor",
                                 "BLD Descriptor", "LATCH Descriptor"};
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

    /*next_desc = new QPushButton("Next Descriptor");
    next_desc->setFixedSize(BUTTON_WIDTH,BUTTON_HEIGHT);
    connect(next_desc, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));
*/

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
    //vbox->addWidget(next_desc,3,1);
    //next_desc->setVisible(false);
    groupBox1->setLayout(vbox);



    //Displaying the pair of images here
    groupBox_images = new QGroupBox ("Single Image");
    image1 = new my_qlabel;
    qimage1.load("../../apps/benchmarking-image-features/images/1.png"); // replace this with initial image of select an image by specifying path
    QImage qscaled1 = qimage1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));

    image2 = new QLabel;
    qimage2.load("../../apps/benchmarking-image-features/images/2.png"); // replace this with initial image of select an image by specifying path
    QImage qscaled2 = qimage2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image2->setPixmap(QPixmap::fromImage(qimage2));





    connect(image1, SIGNAL(Mouse_Pos()), this, SLOT(Mouse_current_pos()));
    connect(image1, SIGNAL(Mouse_Pressed()), this, SLOT(Mouse_Pressed()));
    connect(image1, SIGNAL(Mouse_Left()), this, SLOT(Mouse_left()));





    QGridLayout *hbox_images = new QGridLayout;
    hbox_images->addWidget(image1,0,0,1,1);
    hbox_images->addWidget(image2,0,1,1,1);
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
        images_label_coordinates = new QLabel("*");
        images_static = new QLabel;
        images_static2 = new QLabel;
        //descriptors.load("/home/raghavender/Downloads/images1.jpg"); // replace this with initial image of select an image by specifying path
        //QImage qscaled1 = descriptors.scaled(DESCRIPTOR_WIDTH, DESCRIPTOR_HEIGHT, Qt::KeepAspectRatio);
        //images_static->setPixmap(QPixmap::fromImage(qscaled1));
        //int i_t = 2*(i/DESCRIPTOR_ROW_SIZE);
        //int j_t = 2*(i%DESCRIPTOR_ROW_SIZE);
        //desc_VisualizeGrid->addWidget(images_static[i],i_t,j_t,1,1);
        //desc_VisualizeGrid->addWidget(images_label_coordinates[i],i_t+1, j_t+1, 1,1);
    }
    images_static_sift_surf = new QLabel;

    images_static_sift_surf2 = new QLabel;

    desc_images->setLayout(desc_VisualizeGrid);

    numDesc1 = 0;
    numDesc2 = 0;



    layout_grid = new QGridLayout;

    //layout_grid->addWidget(groupBox2,0,1,1,1);


    layout_grid->addWidget(groupBox_images,0,0,12,2);
    layout_grid->addWidget(groupBox_buttons2,12,0,1,2);
    layout_grid->addWidget(desc_images,13,0,4,4);

    layout_grid->addWidget(groupBox1,2,4,3,2);
    layout_grid->addWidget(inputGroupBox,5,4,6,1);
    layout_grid->addWidget(decimateImage,11,4,2,1);
    //layout_grid->addWidget(userOptionsGroupBox,13,4,1,1);
    layout_grid->addWidget(paramsGroupBox,14,4,12,2);




    //layout_grid->addWidget(groupBox_buttons2,3,0,1,1);
    //layout_grid->addWidget(groupBox_buttons,4,0,1,1);

    //layout_grid->addWidget(userOptionsGroupBox,1,2,1,1);
    //layout_grid->addWidget(paramsGroupBox,2,2,1,1);
    //layout_grid->addWidget(decimateImage,3,2,1,1);


    layout_grid->setSizeConstraint(QLayout::SetMinimumSize);
    flag_descriptor_match = false;


    //layout_grid->addWidget(sample2,14,0,2,2);


    window_gui->setLayout(layout_grid);
    //window_gui->show();

    QWidget* topLevelWidget = 0;


    topLevelWidget = window_gui;

    QScrollArea* scroller = new QScrollArea;
    scroller->setWidget(window_gui);
    topLevelWidget = scroller;

    //topLevelWidget->show();
    topLevelWidget->showMaximized();
    topLevelWidget->raise();
    //return app.exec();


}

void MainWindow::drawLineLSD(Mat img)
{
    if(detector_selected == 11)
    {
        for (int i = 0; i < featsImage1.size(); i++)
        {
            float temp_x1 = featsImage1.getByID(i).get()->x2[0];
            float temp_x2 = featsImage1.getByID(i).get()->x2[1];
            float temp_y1 = featsImage1.getByID(i).get()->y2[0];
            float temp_y2 = featsImage1.getByID(i).get()->y2[1];
            /* get a random color */
            int R = ( rand() % (int) ( 255 + 1 ) );
            int G = ( rand() % (int) ( 255 + 1 ) );
            int B = ( rand() % (int) ( 255 + 1 ) );
            line(img,Point(temp_x1,temp_y1), Point(temp_x2,temp_y2),Scalar(R,G,B), 3);
        }
    }
}

void MainWindow::Mouse_Pressed()
{


    if (!(evaluate_detector_clicked && evaluate_descriptor_clicked && visualize_descriptor_clicked) )
    {
        QMessageBox::information(this, "All Buttons need to be clicked","Please click evaluate detector, evaluate descriptor and visualize descriptor before clicking any key-Point for Visualization !!");
        return;
    }
    //cout << "Mouse current_pos "<< endl ;
    stringstream ss;
    ss << "x : " << image1->x << " y : "<< image1->y ;
    string str = ss.str();
    //cout << str << "Mouse current_pos "<< endl ;
    QString ss2 = QString::fromStdString(str);
    //param1_desc->setText(ss2);



    mouse_x = image1->x;
    mouse_y = image1->y -40 ; // -40 as it is the padding added due to a hidden reason

    double x[numDesc1],y[numDesc1];
    for(int i=0 ; i <numDesc1 ; i++)
    {
        x[i] = featsImage1.getFeatureX(i);
        y[i] = featsImage1.getFeatureY(i);
    }

    QLabel *plotInfo = new QLabel("<b>Descriptor Distances from selected descriptor<br/> in Image 1 to all other descriptors in Image 2 <b/>");

    cv::Mat desc_Ref_img = imread(file_path1, IMREAD_ANYCOLOR); // cv::cvarrToMat(img1.getAs<IplImage>());
    // images1 have all the descriptors

    images_static_sift_surf->setVisible(false);
    images_static->setVisible(false);

    images_static2->setVisible(false);
    images_static_sift_surf2->setVisible(false);
    plotInfo->setVisible(false);

    if(currentInputIndex == 1 || currentInputIndex == 4)
    {
        images_static2->setVisible(false);
        images_static_sift_surf2->setVisible(false);
    }
     int pos = findClosest(mouse_x, mouse_y, x, y, numDesc1);
    if(descriptor_selected == 2 || descriptor_selected == 3 || descriptor_selected == 4)
    {


        float temp_x = featsImage1.getFeatureX(pos); // get the descriptor x coordinate corresponding to images1[cnt]
        float temp_y = featsImage1.getFeatureY(pos);
        //cout << mouse_x << " mouse _x " << mouse_y << " mouse _y" <<endl;
        //cout << temp_x << " desc _x " << temp_y << " desc _y" <<endl;
        stringstream ss;
        ss << temp_x << "," << temp_y << endl;
        string str = ss.str();
        //cout << str << endl;
        stringstream ss2;
        ss2 << "#" << endl;
        string str2 = ss2.str();




        // THE VAriable images_plots_sift_surf is common for both classes SIFT/SURF/ORB and Polar/LogPolar/SpinImage
        images_plots_sift_surf = images1_plots_distances[pos];
        images_static= images1[pos];
        if(currentInputIndex == 1 || currentInputIndex == 4) {
            cout << "going to set the iamges2[pos] " << endl;
            images_static2 = images2[pos];
            cout<<" successfully set images2[pos]" << endl;
        }
        //featureMatched = new QLabel;

        // this is done to fix the overlaying of labels on top of each other.
        if(flag_descriptor_match)
        {
            featureMatched->setVisible(false);

        }

        flag_descriptor_match = true;
        featureMatched = new QLabel;
        if(currentInputIndex == 1 || currentInputIndex == 4)
        {
            featureMatched = featureMatchingInfo[pos];
            featureMatched->setVisible(true);


            // PLOT THE BEST MATCHING FEATURE IN THE SECOND IMAGE
            int temp_idx = min_dist_indexes[pos];
            //featsImage2.clear();
            //fext.detectFeatures(img2, featsImage2, 0, numFeats);
            //featsImage2.saveToTextFile("./KeyPoints2.txt");
            cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());


            cout << " Before the for loop " << endl;
            // Drawing a circle around corners for image 2
            //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
            for (int i = 0; i < featsImage2.size(); i++) {
                int temp_x = (int) featsImage2.getFeatureX(i);
                int temp_y = (int) featsImage2.getFeatureY(i);
                circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(0,255,0), CIRCLE_THICKNESS, 8, 0);
            }
            drawLineLSD(cvImg2);

            circle(cvImg2, Point(featsImage2.getFeatureX(temp_idx), featsImage2.getFeatureY(temp_idx)), 5, Scalar(255,0,0), CIRCLE_THICKNESS, 8, 0);


            cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
            cvtColor(cvImg2, temp2, CV_BGR2RGB);
            QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
            QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
            cout << "right before setting  "<<endl;
            image2->setPixmap(QPixmap::fromImage(qscaled2));
        }

        // now following lines are for computing the x and y coordinate
        images_label_coordinates = new QLabel;

        images_label_coordinates->setText(QString::fromStdString(str2));
        circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(0,0,255), CIRCLE_THICKNESS, 8, 0); // plot the circles on the appropriate points as per the shown descriptors
        //putText(desc_Ref_img, str2, Point(temp_x,temp_y), 5, 2, Scalar(255,0,0),2,8,0);
        //cnt++; // global counter for iterating over images and labels

        cout << "before static" << endl;



        cout << "after static" << endl;


        if(currentInputIndex == 1 || currentInputIndex == 4)
        {

            images_static2->setVisible(true);
            desc_VisualizeGrid->addWidget(images_static2, 0, 1, 1, 1);
            desc_VisualizeGrid->addWidget(plotInfo,1,2,1,1);
            plotInfo->setVisible(true);

        }
        desc_VisualizeGrid->addWidget(images_static,0,0,1,1);

        images_static->setVisible(true);

        desc_VisualizeGrid->addWidget(images_plots_sift_surf,0,2,1,1);  // add the image distance plot with respect to other features in image 1
        QLabel *detector_info  = new QLabel("Evaluation Metrics shown here");
        if(currentInputIndex == 0 || currentInputIndex == 3)
            desc_VisualizeGrid->addWidget(detector_info,0,1,1,1);

        if(currentInputIndex == 1 || currentInputIndex == 4)
            desc_VisualizeGrid->addWidget(featureMatched, 1,0,1,1); // add the label telling about the feature matching


    }
    else if(descriptor_selected == 0 || descriptor_selected == 1 || descriptor_selected == 5 || descriptor_selected == 6)
    {

        //featureMatched->clear();
        //featureMatched->destroy(true,true);
        //featureMatched->destroyed();
        images_plots_sift_surf = images1_plots_distances[pos];
        images_static_sift_surf = images1[pos];
        if(currentInputIndex == 1 || currentInputIndex == 4) {
            images_static_sift_surf2 = images2[pos];
        }
        //featureMatched = new QLabel;

        // this is done to fix the overlaying of labels on top of each other.
        if(flag_descriptor_match)
        {
            featureMatched->setVisible(false);

        }

        flag_descriptor_match = true;
        featureMatched = new QLabel;
        if(currentInputIndex == 1 || currentInputIndex == 4)
        {
            featureMatched = featureMatchingInfo[pos];
            featureMatched->setVisible(true);


            // PLOT THE BEST MATCHING FEATURE IN THE SECOND IMAGE
            int temp_idx = min_dist_indexes[pos];
            //featsImage2.clear();
            //fext.detectFeatures(img2, featsImage2, 0, numFeats);
            //featsImage2.saveToTextFile("./KeyPoints2.txt");
            cv::Mat cvImg2 = cv::cvarrToMat(img2.getAs<IplImage>());



            // Drawing a circle around corners for image 2
            //C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
            for (int i = 0; i < featsImage2.size(); i++) {
                int temp_x = (int) featsImage2.getFeatureX(i);
                int temp_y = (int) featsImage2.getFeatureY(i);
                circle(cvImg2, Point(temp_x, temp_y), 5, Scalar(0,255,0), CIRCLE_THICKNESS, 8, 0);
            }
            drawLineLSD(cvImg2);
            circle(cvImg2, Point(featsImage2.getFeatureX(temp_idx), featsImage2.getFeatureY(temp_idx)), 5, Scalar(255,0,0), CIRCLE_THICKNESS, 8, 0);

            cv::Mat temp2(cvImg2.cols, cvImg2.rows, cvImg2.type());
            cvtColor(cvImg2, temp2, CV_BGR2RGB);
            QImage dest2 = QImage((uchar *) temp2.data, temp2.cols, temp2.rows, temp2.step, QImage::Format_RGB888);
            QImage qscaled2 = dest2.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
            image2->setPixmap(QPixmap::fromImage(qscaled2));
        }


        images_label_coordinates_sift_surf = new QLabel;

        float temp_x = featsImage1.getFeatureX(pos); // get the descriptor x coordinate corresponding to images1[cnt]
        float temp_y = featsImage1.getFeatureY(pos);

        stringstream ss;
        ss << temp_x << " , " << temp_y << endl;
        string str = ss.str();

        //images_label_coordinates_sift_surf->setText(QString::fromStdString(str));
        cout << str << endl;



        circle(desc_Ref_img, Point(temp_x, temp_y), 5, Scalar(255,0,0), CIRCLE_THICKNESS, 8, 0); // plot the circles on the appropriate points as per the shown descriptors

        images_static_sift_surf->setVisible(true);
        if(currentInputIndex == 1 || currentInputIndex == 4) {
            images_static_sift_surf2->setVisible(true);
            desc_VisualizeGrid->addWidget(images_static_sift_surf2,0,1,1,1);
            desc_VisualizeGrid->addWidget(plotInfo,1,2,1,1);
            plotInfo->setVisible(true);

        }
        desc_VisualizeGrid->removeWidget(featureMatched);
        desc_VisualizeGrid->addWidget(images_static_sift_surf,0,0,1,1);
        desc_VisualizeGrid->addWidget(images_plots_sift_surf,0,2,1,1);  // add the image distance plot with respect to other features in image 1

        QLabel *detector_info  = new QLabel("Evaluation Metrics shown here");
        if(currentInputIndex == 0 || currentInputIndex == 3)
            desc_VisualizeGrid->addWidget(detector_info,0,1,1,1);
        if(currentInputIndex == 1 || currentInputIndex == 4)
            desc_VisualizeGrid->addWidget(featureMatched, 1,0,1,1); // add the label telling about the feature matching

        //featureMatched->clear();

        //desc_VisualizeGrid->addWidget(images_label_coordinates_sift_surf,i_t+1, j_t+1, 1,1);

        //cnt++; // global counter for iterating over images and labels
    }

    cv::Mat temp1 (desc_Ref_img.cols,desc_Ref_img.rows,desc_Ref_img.type());
    cvtColor(desc_Ref_img, temp1, CV_BGR2RGB);
    QImage dest1 = QImage((uchar*) temp1.data, temp1.cols, temp1.rows, temp1.step, QImage::Format_RGB888);
    QImage qscaled1 = dest1.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image1->setPixmap(QPixmap::fromImage(qscaled1));



}
void MainWindow::Mouse_current_pos(){
    //param2_desc->setText("Mouse Pressed");
    cout << "Mouse Pressed" << endl;
    cout << image1->x << " x " << image1->y << endl;


}
void MainWindow::Mouse_left(){

    //param2_desc->setText("Mouse Pressed");

    cout <<  "Mouse left "<< endl ;


}
int MainWindow::findClosest(double x, double y, double X[], double Y[], int n)
{
    double dist = 10000;
    double temp_dist ;
    int pos = 0;
    for(int i=0 ; i<n ; i++)
    {
        temp_dist = sqrt(pow((X[i]-x),2)+pow((Y[i] -y),2));

        if(temp_dist < dist)
        {
            dist = temp_dist;
            pos = i;
        }
    }
    return pos;

}