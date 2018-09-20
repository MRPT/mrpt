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

#ifndef BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
#define BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QDebug>
#include <QEvent>

class my_qlabel : public QLabel
{
	Q_OBJECT

   public:
	explicit my_qlabel(QWidget* parent = nullptr);

	void mousePressEvent(QMouseEvent* ev) override;
	void leaveEvent(QEvent*) override;
	void mouseMoveEvent(QMouseEvent* ev) override;
	int x, y;

   signals:
	void Mouse_Pressed();
	void Mouse_Pos();
	void Mouse_Left();

   public slots:
};

#endif  // BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
