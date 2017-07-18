/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CGeneralConfig.h"

#include <QColorDialog>


CGeneralConfig::CGeneralConfig()
	: CBaseConfig()
	, m_ui(std::make_unique<Ui::CGeneralConfig>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->m_gridColor, SIGNAL(released()), SLOT(openGridColor()));
	QObject::connect(m_ui->m_backgroundColor, SIGNAL(released()), SLOT(openBackgroundColor()));
	QObject::connect(m_ui->m_enableGrid, SIGNAL(stateChanged(int)), SLOT(stateChanged(int)));
	QObject::connect(m_ui->m_bots, SIGNAL(currentIndexChanged(int)), SIGNAL(currentBotChanged(int)));
}

const QString CGeneralConfig::getName()
{
	return "General";
}

void CGeneralConfig::updateConfiguration(mrpt::maps::TMetricMapInitializer *options)
{
	Q_UNUSED(options);
}

TypeOfConfig CGeneralConfig::type() const
{
	return TypeOfConfig::General;
}

void CGeneralConfig::openGridColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);

	if(col.isValid())
	{
		QString qss = QString("background-color: %1").arg(col.name());
		m_ui->m_gridColor->setStyleSheet(qss);
		emit gridColorChanged(col);
	}
}

void CGeneralConfig::openBackgroundColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);

	if(col.isValid())
	{
		QString qss = QString("background-color: %1").arg(col.name());
		m_ui->m_backgroundColor->setStyleSheet(qss);
		emit backgroundColorChanged(col);
	}
}

void CGeneralConfig::stateChanged(int state)
{
	emit gridVisibleChanged(state == Qt::Checked);
}
