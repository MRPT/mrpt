/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CPoseDirection.h"
#include <QDoubleSpinBox>

CPoseDirection::CPoseDirection(QWidget* parent)
	: QWidget(parent), m_ui(std::make_unique<Ui::CPoseDirection>())
{
	m_ui->setupUi(this);

	connect(m_ui->yaw, SIGNAL(valueChanged(double)), this, SLOT(dataChanged()));
	connect(
		m_ui->pitch, SIGNAL(valueChanged(double)), this, SLOT(dataChanged()));
	connect(
		m_ui->roll, SIGNAL(valueChanged(double)), this, SLOT(dataChanged()));
}

CPoseDirection::~CPoseDirection() = default;
void CPoseDirection::setDirection(double yaw, double pitch, double roll)
{
	blockSignals(true);
	m_ui->yaw->setValue(yaw);
	m_ui->pitch->setValue(pitch);
	m_ui->roll->setValue(roll);
	blockSignals(true);
}

double CPoseDirection::getYaw() const { return m_ui->yaw->value(); }
double CPoseDirection::getPitch() const { return m_ui->pitch->value(); }
double CPoseDirection::getRoll() const { return m_ui->roll->value(); }
void CPoseDirection::setIndex(size_t index) { m_index = index; }
void CPoseDirection::dataChanged()
{
	emit updateDirection(
		m_index, m_ui->yaw->value(), m_ui->pitch->value(), m_ui->roll->value());
}
