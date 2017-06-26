/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "COccupancyConfig.h"
#include "ui_COccupancyConfig.h"



COccupancyConfig::COccupancyConfig(QWidget *parent)
	: QWidget(parent)
	, m_ui(std::make_unique<Ui::COccupancyConfig>())
{
	m_ui->setupUi(this);
}

COccupancyConfig::~COccupancyConfig()
{

}
