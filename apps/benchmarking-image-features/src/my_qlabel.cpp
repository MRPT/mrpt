//
// Created by raghavender on 26/06/17.
//

#include "my_qlabel.h"

my_qlabel::my_qlabel(QWidget *parent) : QLabel(parent)
{

}

void my_qlabel::mouseMoveEvent(QMouseEvent *ev) {
    this->x = ev->x();
    this->y = ev->y();
    emit Mouse_Pos();
}

void my_qlabel::mousePressEvent(QMouseEvent *ev)
{
    this->x = ev->x();
    this->y = ev->y();
    emit Mouse_Pressed();
}

void my_qlabel::leaveEvent(QEvent *)
{
    emit Mouse_Left();
}

