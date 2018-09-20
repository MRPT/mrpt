/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <QWidget>

#include <memory>

#include "ui_CPoseDirection.h"

class CPoseDirection : public QWidget
{
	Q_OBJECT
   public:
	CPoseDirection(QWidget* parent = nullptr);
	~CPoseDirection() override;
	void setDirection(double yaw, double pitch, double roll);
	double getYaw() const;
	double getPitch() const;
	double getRoll() const;
	void setIndex(size_t index);

   signals:
	void updateDirection(size_t index, double yaw, double pitch, double roll);

   private slots:
	void dataChanged();

   private:
	std::unique_ptr<Ui::CPoseDirection> m_ui;
	int m_index{0};
};
