/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CSelectType.h"
#include "ui_CSelectType.h"

#include <QListWidget>

CSelectType::CSelectType(QWidget* parent)
	: QDialog(parent), m_ui(std::make_unique<Ui::CSelectType>())
{
	m_ui->setupUi(this);

	addItem(tr("Points Map"), TypeOfConfig::PointsMap);
	addItem(tr("Occupancy"), TypeOfConfig::Occupancy);
	addItem(tr("Landmarks"), TypeOfConfig::Landmarks);
	addItem(tr("Beacon"), TypeOfConfig::Beacon);
	addItem(tr("GasGrid"), TypeOfConfig::GasGrid);
}

CSelectType::~CSelectType() = default;
int CSelectType::selectedItem() const
{
	QListWidgetItem* item = m_ui->m_typeList->currentItem();
	if (!item) return -1;

	return item->data(Qt::UserRole).toInt();
}

void CSelectType::addItem(const QString& name, TypeOfConfig type)
{
	QListWidgetItem* item = new QListWidgetItem(name, m_ui->m_typeList);
	item->setData(Qt::UserRole, type);
	m_ui->m_typeList->addItem(item);
}
