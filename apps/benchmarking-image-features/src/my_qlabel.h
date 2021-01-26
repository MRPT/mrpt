/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: benchmarkingImageFeatures_gui
	FILE: my_qlabel.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

#ifndef BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
#define BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H

#include <QDebug>
#include <QEvent>
#include <QLabel>
#include <QMouseEvent>

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

#endif	// BENCHMARKINGIMAGEFEATURES_GUI_MY_QLABEL_H
