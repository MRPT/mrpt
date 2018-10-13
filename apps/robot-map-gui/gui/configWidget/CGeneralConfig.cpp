/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CGeneralConfig.h"

#include <QColorDialog>

CGeneralConfig::CGeneralConfig()
	: CBaseConfig(),
	  m_ui(std::make_unique<Ui::CGeneralConfig>()),
	  m_generalSetting()
{
	m_ui->setupUi(this);
	connect(
		m_ui->m_gridColor, &QPushButton::released, this,
		&CGeneralConfig::openGridColor);
	connect(
		m_ui->m_backgroundColor, &QPushButton::released, this,
		&CGeneralConfig::openBackgroundColor);
}

const QString CGeneralConfig::getName() { return "General"; }
void CGeneralConfig::updateConfiguration(
	mrpt::maps::TMetricMapInitializer* options)
{
	Q_UNUSED(options);
}

TypeOfConfig CGeneralConfig::type() const { return TypeOfConfig::General; }
const SGeneralSetting& CGeneralConfig::generalSetting()
{
	m_generalSetting.robotPosesSize = m_ui->m_robotPosesSize->value();
	m_generalSetting.selectedRobotPosesSize =
		m_ui->m_selectedRobotPosesSize->value();
	m_generalSetting.robotPosesColor = m_ui->m_robotPosesColor->currentIndex();
	m_generalSetting.selectedRobotPosesColor =
		m_ui->m_selectedRobotPosesColor->currentIndex();
	m_generalSetting.isGridVisibleChanged = m_ui->m_enableGrid->isChecked();
	m_generalSetting.currentBot = m_ui->m_bots->currentIndex();

	return m_generalSetting;
}

void CGeneralConfig::openGridColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);

	if (col.isValid())
	{
		QString qss = QString("background-color: %1").arg(col.name());
		m_ui->m_gridColor->setStyleSheet(qss);
		m_generalSetting.gridColor = col;
	}
}

void CGeneralConfig::openBackgroundColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);

	if (col.isValid())
	{
		QString qss = QString("background-color: %1").arg(col.name());
		m_ui->m_backgroundColor->setStyleSheet(qss);
		m_generalSetting.backgroundColor = col;
	}
}

SGeneralSetting::SGeneralSetting()
	: backgroundColor(Qt::white), gridColor(Qt::black)

{
}
