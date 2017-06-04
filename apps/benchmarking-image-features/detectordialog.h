#ifndef DIALOGDETECTOR_H
#define DIALOGDETECTOR_H

#include <QDialog>
#include <string>
#include <QString>


using namespace std;
class DetectorDialog : public QDialog
{
    Q_OBJECT
public:

    explicit DetectorDialog(QWidget *parent = 0, QString  str="/home", int detector=0);
    void computeDetector(int detector_id);

public:
    QWidget *detector_gui;
    string file_path;

public slots:
    void onshow_results_clicked();
    void oncloseDialog_clicked();

};

#endif // DIALOGDETECTOR_H
