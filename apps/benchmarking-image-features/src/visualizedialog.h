#ifndef VISUALIZEDIALOG_H
#define VISUALIZEDIALOG_H

#define DESCRIPTOR_WIDTH 100
#define DESCRIPTOR_HEIGHT 100
#define DESCRIPTOR_GRID_SIZE 25
#define DESCRIPTOR_GRID_SIZE2 9

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QDialog>
#include <string>
#include <QString>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>

#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


#include <mrpt/vision/CFeatureExtraction.h>

using namespace cv;
using namespace std;
using namespace mrpt::vision;
using namespace mrpt::utils;

using namespace std;
class VisualizeDialog: public QDialog
{
Q_OBJECT
public:
    VisualizeDialog(QWidget *parent = 0, QString  filePath="/home", int detector_id=-1, int descriptor_id=-1, int numDescriptors=-1, CFeatureList desc_Image1={}, CFeatureList desc_Image2={});

public:
    QWidget *feature_gui;
    CFeatureList feats1, feats2;

    QLabel *images_static[DESCRIPTOR_GRID_SIZE];
    QLabel *images_static_sift_surf[DESCRIPTOR_GRID_SIZE2];

    QLabel *images_label_coordinates[DESCRIPTOR_GRID_SIZE];
    QLabel *images_label_coordinates_sift_surf[DESCRIPTOR_GRID_SIZE2];
    QString *filePath;


    int global_cnt ;
    bool flag;
    bool flag2;
    int next_button_clicked_counter;

    int cnt; // global counter for iterating over images
    int bnt; // global counter for iterating over x and y coordinates
    //int ds, dt

signals:

public slots:
    void on_next_button_clicked();
    void on_prev_button_clicked();
    void oncloseDialog_clicked();

};

#endif // VISUALIZEDIALOG_H



