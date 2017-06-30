/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CGasGridConfig.h"
#include "ui_CGasGridConfig.h"
#include "TypeOfConfig.h"


CGasGridConfig::CGasGridConfig(QWidget *parent)
	: CBaseConfig(parent)
	, m_ui(std::make_unique<Ui::CGasGridConfig>())
{
	m_ui->setupUi(this);
}

CGasGridConfig::~CGasGridConfig()
{

}

const QString CGasGridConfig::getName()
{
	return QString::fromStdString(typeToName(TypeOfConfig::GasGrid));
}

void CGasGridConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer *options)
{
	Q_UNUSED(options);
}

void CGasGridConfig::setCreationOpt(float min_x, float max_x, float min_y, float max_y, float resolution)
{

}

void CGasGridConfig::setInsertOpt(const mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions &insertOpt)
{

}

void CGasGridConfig::setMapTypeOpt(const mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation &mapType)
{

}
