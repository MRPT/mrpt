/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: benchmarkingImageFeatures_gui
	FILE: my_qlabel.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/
#include "my_qlabel.h"

my_qlabel::my_qlabel(QWidget* parent) : QLabel(parent) {}
void my_qlabel::mouseMoveEvent(QMouseEvent* ev)
{
	this->x = ev->x();
	this->y = ev->y();
	emit Mouse_Pos();
}

void my_qlabel::mousePressEvent(QMouseEvent* ev)
{
	this->x = ev->x();
	this->y = ev->y();
	emit Mouse_Pressed();
}

void my_qlabel::leaveEvent(QEvent*) { emit Mouse_Left(); }
