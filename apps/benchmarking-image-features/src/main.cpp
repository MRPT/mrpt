#include <QApplication>
#include <QtGui>
#include "mainwindow.h"
#include <QObject>
#include <QThread>


using namespace std;
using namespace cv;
int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    MainWindow w;


    return app.exec();
}
