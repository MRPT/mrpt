#include <QApplication>
#include <QtGui>
#include <QtCore>
#include "mainwindow.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;
int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    MainWindow w;

    return app.exec();
}
