#include <QApplication>
#include <QtGui>
#include <QtCore>
#include "mainwindow.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow w;

    return app.exec();
}
