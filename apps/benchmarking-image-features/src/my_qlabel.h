//
// Created by raghavender on 26/06/17.
//

#ifndef BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
#define BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QDebug>
#include <QEvent>

class my_qlabel: public QLabel {

    Q_OBJECT

public :
    explicit my_qlabel(QWidget *parent = 0);


    void mousePressEvent(QMouseEvent *ev);
    void leaveEvent(QEvent *);

    void mouseMoveEvent(QMouseEvent *ev);
    int x,y;

signals:
    void Mouse_Pressed();
    void Mouse_Pos();
    void Mouse_Left();
public slots:


};


#endif //BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
