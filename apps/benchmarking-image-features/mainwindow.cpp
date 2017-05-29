#include "mainwindow.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <string.h>
#include <iostream>

using namespace std;
void MainWindow::on_button_generate_clicked()
{
    ReadInputFormat();
    if(detector_selected == -1 || descriptor_selected == -1)
    {
        QMessageBox::information(this,"Detector / Descriptor Evaluation Metric"," Please select both the detector and the descriptor before viewing the performance results");
        return;
    }


    QString details("You have selected the DETECTOR" + QString::number(detector_selected) + " and DESCRIPTOR " + QString::number(descriptor_selected) + "\nThe selected image "+ QString::number(currentInputIndex) + " has the following characteristics");
    QMessageBox::information(this,"Detector Characteristics for the selected input", details);

}

void MainWindow::button_close_clicked()
{
    window_gui->close();
    this->close();
    return;
}


/*
 *
 * this function is called to show the performance of the selected detector on the input selected by the user
 * the performance metric displayed is in terms of repeatability, dispersion of image, computational cost, number of found points, etc.
 * */
void MainWindow::on_detector_button_clicked()
{
    ReadInputFormat();
    if(detector_selected == -1)
    {
        QMessageBox::information(this,"Warning", "Please select the detector");
        return;
    }


    // CALL DETECTOR DIALOG HERE TO CREATE DIALOG FOR PARAMETERS/EVALUATION

    QString details("You have selected the DETECTOR" + QString::number(detector_selected) + " ,The selected image "+ QString::number(currentInputIndex) + " has the following characteristics");
    QMessageBox::information(this,"Detector Characteristics for the selected input", details);
}

/*
 *
 * this function is called to show the performance of the selected descriptor on the input selected by the user
 * the performance metric displayed is in terms of percentage of patches matched, descriptor distance between close matches, false positives/negatives, computational cost, etc.
 * */
void MainWindow::on_descriptor_button_clicked()
{
    ReadInputFormat();
    if(descriptor_selected == -1)
    {
        QMessageBox::information(this,"Warning", "Please select the descriptor before viewing the performance metrics");
        return;
    }

    // CALL DESCRIPTOR DIALOG HERE TO CREATE DIALOG FOR PARAMETERS/EVALUATION

    QString details("You have selected the DESCRIPTOR" + QString::number(descriptor_selected) + "The selected image"+ QString::number(currentInputIndex) +"has the following characteristics");
    QMessageBox::information(this,"Descriptor Characteristics for the selected input", details);
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

    //0 = single image; 1 = stereo image; 2 = rawlog file ; 3 = image dataset folder
    if(currentInputIndex == 0 || currentInputIndex == 2)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 1 || currentInputIndex == 3)
    {
        dialog.setFileMode(QFileDialog::Directory);
    }

    //dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg)"));
    dialog.setViewMode(QFileDialog::Detail);
    QStringList fileNames;
    if(dialog.exec())
        fileNames = dialog.selectedFiles();
    inputFilePath->setText(fileNames.at(0));



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
    for(int i=0 ; i<NUM_DETECTORS ; i++)
    {
        detector_selected = -1;
        if(detectors[i]->isChecked())
        {
            detector_selected = i;
            break;
        }
    }

    //store the descriptor here
    for(int i=0 ; i<NUM_DESCRIPTORS ; i++)
    {
        descriptor_selected = -1;
        if(descriptors[i]->isChecked())
        {
            descriptor_selected = i;
            break;
        }
    }
}

MainWindow::MainWindow(QWidget *window_gui) : QMainWindow(window_gui)
{
    inputFormat = 0;
    currentInputIndex = 0;
    detector_selected = 0;
    descriptor_selected = 0;

    window_gui = new QWidget;
    window_gui->setWindowTitle("GUI app for benchmarking image detectors and descriptord");

    //Initialize the detectors here
    QGroupBox *groupBox = new QGroupBox(tr("Select your detector"));
    string detector_names[] = {"Harris Corner Detector", "Detectorf",
                     "FAST Detector", "LSD Detector",
                     "AKAZE Detector"};
    for(int i=0 ; i<NUM_DETECTORS ; i++)
    {
        detectors[i] = new QRadioButton(this);
        detectors[i]->setText(detector_names[i].c_str());
    }
    QPushButton *detector_button = new QPushButton;
    detector_button->setText("Evaluate Detector");
    connect(detector_button, SIGNAL(clicked(bool)),this, SLOT(on_detector_button_clicked()));
    QVBoxLayout *vbox = new QVBoxLayout;
    for(int i=0 ; i<NUM_DETECTORS ; i++)
    {
        vbox->addWidget(detectors[i]);
    }
    vbox->addWidget(detector_button);
    groupBox->setLayout(vbox);




    //initialize the descriptors here
    QGroupBox *groupBox2 = new QGroupBox(tr("Select your descriptor"));
    string descriptor_names[] = {"SIFT Descriptor", "SURF Descriptor",
                                  "LATCH Descriptor", "BLD Descriptor",
                                  "BRIEF Descriptors"};
    for(int i=0 ; i<NUM_DETECTORS ; i++)
    {
        descriptors[i] = new QRadioButton(this);
        descriptors[i]->setText(descriptor_names[i].c_str());
    }
    QPushButton *descriptor_button = new QPushButton;
    descriptor_button->setText("Evaluate Descriptor");
    connect(descriptor_button, SIGNAL(clicked(bool)),this, SLOT(on_descriptor_button_clicked()));
    QVBoxLayout *vbox2 = new QVBoxLayout;
    for(int i=0 ; i<NUM_DESCRIPTORS ; i++)
    {
        vbox2->addWidget(descriptors[i]);
    }
    vbox2->addWidget(descriptor_button);
    groupBox2->setLayout(vbox2);



    //provide user input image options
    QGroupBox *inputGroupBox = new QGroupBox;
    QLabel *inputLabel = new QLabel("<b>Specify the input data format</b>");
    inputs = new QComboBox;
    inputs->addItem("Single Image");
    inputs->addItem("Stereo Image");
    inputs->addItem("Image Rawlog file");
    inputs->addItem("Image Dataset");
    inputFilePath = new QLineEdit;
    QPushButton *browse_button = new QPushButton("Browse");
    connect(browse_button, SIGNAL(clicked()), this, SLOT(on_browse_button_clicked()));

    QVBoxLayout *inputVbox = new QVBoxLayout;
    inputVbox->addWidget(inputLabel);
    inputVbox->addWidget(inputs);
    inputVbox->addWidget(inputFilePath);
    inputVbox->addWidget(browse_button);
    inputGroupBox->setLayout(inputVbox);



    // initializing the buttons here
    QGroupBox *groupBox_buttons = new QGroupBox;
    button_generate = new QPushButton("Generate Detectors / Descriptors");
    connect(button_generate, SIGNAL(clicked(bool)), this,SLOT(on_button_generate_clicked()));
    button_close = new QPushButton("Close");
    connect(button_close,SIGNAL(clicked(bool)),this,SLOT(button_close_clicked()));
    QHBoxLayout *hbox1 = new QHBoxLayout;
    hbox1->addWidget(button_close);
    hbox1->addWidget(button_generate);
    groupBox_buttons->setLayout(hbox1);


    layout_grid = new QGridLayout;
    layout_grid->addWidget(groupBox,1,0,1,1);
    layout_grid->addWidget(groupBox2,1,1,1,1);
    layout_grid->addWidget(groupBox_buttons,2,1,1,1);
    layout_grid->addWidget(inputGroupBox,1,3,1,1);
    //layout_grid->addWidget(dialog,1,4,1,1);


    window_gui->setLayout(layout_grid);
    window_gui->show();
}
