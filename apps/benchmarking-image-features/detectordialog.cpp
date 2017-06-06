#include "detectordialog.h"
#include <QtGui>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <QMessageBox>

using namespace cv;

void DetectorDialog::onshow_results_clicked()
{

}
void DetectorDialog::oncloseDialog_clicked()
{
    detector_gui->setVisible(false);
    detector_gui->close();
    //this->close();
    return;
}

DetectorDialog::DetectorDialog(QWidget *detector_gui, QString str, int detector) :QDialog(detector_gui)
{
    detector_gui = new QWidget;
    detector_gui->setWindowTitle("Detector Evaluation");
    QGridLayout *layout_grid = new QGridLayout;

    QLabel *label = new QLabel("I am a label detector");
    QLabel *label2 = new QLabel;
    label2->setText(str);// + "I am a label");

    QPushButton *showResults = new QPushButton("Show Results");
    connect(showResults, SIGNAL(clicked(bool)), this, SLOT(onshow_results_clicked()) );
    QPushButton *closeDialog = new QPushButton("Close");
    connect(closeDialog, SIGNAL(clicked()), this, SLOT(oncloseDialog_clicked()) );


    cv::Mat img = imread(str.toStdString(), IMREAD_COLOR);
    if(img.empty())
    {
        QMessageBox::information(this,"Image read error","Please specify a valid path for the image..");
        return;
    }



    layout_grid->addWidget(label,0,1,1,1);
    layout_grid->addWidget(label2,1,1,1,1);
    layout_grid->addWidget(showResults,2,1,1,1);
    layout_grid->addWidget(closeDialog,3,1,1,1);




    detector_gui->setLayout(layout_grid);
    detector_gui->show();

    //cv::imshow("sample image",img);
    //cv::waitKey(0);
}

void computeDetector(int detector_id)
{

}
