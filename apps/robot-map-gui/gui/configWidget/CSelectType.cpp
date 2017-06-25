/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CSelectType.h"
#include "ui_CSelectType.h"

#include <QListWidget>


CSelectType::CSelectType(QWidget *parent)
	: QDialog(parent)
	, m_ui(std::make_unique<Ui::CSelectType>())
{
	m_ui->setupUi(this);

	addItem(tr("Points Map"), TypeOfMaps::PointsMap);
	addItem(tr("Occupancy"), TypeOfMaps::Occupancy);
	addItem(tr("Landmarks"), TypeOfMaps::Landmarks);
	addItem(tr("Beacon"), TypeOfMaps::Beacon);
	addItem(tr("GasGrid"), TypeOfMaps::GasGrid);
}

CSelectType::~CSelectType()
{

}

CSelectType::TypeOfMaps CSelectType::selectedItem() const
{
	QListWidgetItem * item = m_ui->m_typeList->currentItem();
	if (!item)
		return None;

	return static_cast<TypeOfMaps>(item->data(Qt::UserRole).toInt());
}

void CSelectType::addItem(const QString &name, CSelectType::TypeOfMaps type)
{
	QListWidgetItem *item = new QListWidgetItem(name, m_ui->m_typeList);
	item->setData(Qt::UserRole, type);
	m_ui->m_typeList->addItem(item);
}

