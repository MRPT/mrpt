#ifndef DIALOGDETECTOR_H
#define DIALOGDETECTOR_H

#include <QDialog>

class DialogDetector : public QDialog
{
    Q_OBJECT
public:
    DialogDetector();
    explicit MyDialog(QWidget *parent = 0);
    ~MyDialog();
    
};

#endif // DIALOGDETECTOR_H
