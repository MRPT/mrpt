#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define NUM_DETECTORS 5
#define NUM_DESCRIPTORS 5

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QRadioButton>
#include <QtGui>
#include <QtCore>
#include <QLineEdit>
#include <QMessageBox>
#include <QGroupBox>
#include <QListWidget>
#include <QComboBox>
#include <QFileDialog>

#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=0);
    void ReadInputFormat();

public:

    QWidget *window_gui;
    QLabel *detector_label;
    QPushButton *button_generate;
    QPushButton *button_close;
    QGridLayout *layout_grid;
    int inputFormat;
    int currentInputIndex;
    int detector_selected;
    int descriptor_selected;

    QComboBox *inputs;
    QLineEdit *inputFilePath;

    QRadioButton *detector1;
    QRadioButton *detector2;
    QRadioButton *detector3;
    QRadioButton *detector4;
    QRadioButton *detector5;

    QRadioButton *detectors[NUM_DETECTORS];
    QRadioButton *descriptors[NUM_DESCRIPTORS];


    QRadioButton *descriptor1;
    QRadioButton *descriptor2;
    QRadioButton *descriptor3;
    QRadioButton *descriptor4;
    QRadioButton *descriptor5;
signals:

public slots:
    void on_button_generate_clicked();
    void button_close_clicked();
    void on_detector_button_clicked();
    void on_descriptor_button_clicked();
    void on_browse_button_clicked();

};

#endif // MAINWINDOW_H
