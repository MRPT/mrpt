/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QWidget>

#include <memory>


namespace Ui
{
class CPointsConfig;
}
class CPointsConfig: public QWidget
{
public:
	CPointsConfig(QWidget *parent = nullptr);
	virtual ~CPointsConfig();

private:
	std::unique_ptr<Ui::CPointsConfig> m_ui;
};

