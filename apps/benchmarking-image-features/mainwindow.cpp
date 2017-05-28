#include "mainwindow.h"


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

    if(currentInputIndex == 0)
    {
        dialog.setNameFilter(tr("Images (*.png *.xpm *.jpg *.tiff *.gif)"));
    }
    else if(currentInputIndex == 1 || currentInputIndex == 2)
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
    currentInputIndex = inputs->currentIndex();

    if(detector1->isChecked())
        detector_selected = 1;
    else if(detector2->isChecked())
        detector_selected = 2;
    else if(detector3->isChecked())
        detector_selected = 3;
    else if(detector4->isChecked())
        detector_selected = 4;
    else if(detector5->isChecked())
        detector_selected = 5;
    else
        detector_selected = -1;


    if(descriptor1->isChecked())
        descriptor_selected = 1;
    else if(descriptor2->isChecked())
        descriptor_selected = 2;
    else if(descriptor3->isChecked())
        descriptor_selected = 3;
    else if(descriptor4->isChecked())
        descriptor_selected = 4;
    else if(descriptor5->isChecked())
        descriptor_selected = 5;
    else
        descriptor_selected = -1;
}

MainWindow::MainWindow(QWidget *window_gui) : QMainWindow(window_gui)
{
    inputFormat = 0;
    currentInputIndex = 0;
    detector_selected = 0;
    descriptor_selected = 0;

    window_gui = new QWidget;
    window_gui->setWindowTitle("GUI app for benchmarking image detectors and descriptord");

    //Select the Detector here
    QGroupBox *groupBox = new QGroupBox(tr("Select your detector"));
    detector1 = new QRadioButton("Harris Corner Detector");
    detector2 = new QRadioButton("Detector 2");
    detector3 = new QRadioButton("FAST Detector");
    detector4 = new QRadioButton("LSD Detector");
    detector5 = new QRadioButton("AKAZE Detector");
    QPushButton *detector_button = new QPushButton;
    detector_button->setText("Evaluate Detector");
    connect(detector_button, SIGNAL(clicked(bool)),this, SLOT(on_detector_button_clicked()));


    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(detector1);
    vbox->addWidget(detector2);
    vbox->addWidget(detector3);
    vbox->addWidget(detector4);
    vbox->addWidget(detector5);
    vbox->addWidget(detector_button);
    groupBox->setLayout(vbox);

    //Select the descriptor here
    QGroupBox *groupBox2 = new QGroupBox(tr("Select your descriptor"));
    descriptor1 = new QRadioButton("SIFT Descriptor");
    descriptor2 = new QRadioButton("SURF Descriptor");
    descriptor3 = new QRadioButton("LATCH Descriptor");
    descriptor4 = new QRadioButton("BLD Descriptor");
    descriptor5 = new QRadioButton("BRIEF Descriptors");
    QPushButton *descriptor_button = new QPushButton;
    descriptor_button->setText("Evaluate Descriptor");
    connect(descriptor_button, SIGNAL(clicked(bool)),this, SLOT(on_descriptor_button_clicked()));


    QVBoxLayout *vbox2 = new QVBoxLayout;
    vbox2->addWidget(descriptor1);
    vbox2->addWidget(descriptor2);
    vbox2->addWidget(descriptor3);
    vbox2->addWidget(descriptor4);
    vbox2->addWidget(descriptor5);
    vbox2->addWidget(descriptor_button);
    groupBox2->setLayout(vbox2);

    //provide user input image options




    QGroupBox *inputGroupBox = new QGroupBox;
    QLabel *inputLabel = new QLabel("<b>Specify the input data format</b>");
    inputs = new QComboBox;
    inputs->addItem("Single Image");
    inputs->addItem("Stereo Image");
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


    //QFileDialog fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/raghavender", tr("Image Files (*.png *.jpg *.bmp)"));;


    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);

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
