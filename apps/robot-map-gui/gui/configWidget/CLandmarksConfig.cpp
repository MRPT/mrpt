/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CLandmarksConfig.h"
#include "ui_CLandmarksConfig.h"
#include "TypeOfConfig.h"


CLandmarksConfig::CLandmarksConfig(QWidget *parent)
	: CBaseConfig(parent)
	, m_ui(std::make_unique<Ui::CLandmarksConfig>())
{
	m_ui->setupUi(this);
}


CLandmarksConfig::~CLandmarksConfig()
{

}

const QString CLandmarksConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::Landmarks));
}

void CLandmarksConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer *options)
{
	Q_UNUSED(options);
}
