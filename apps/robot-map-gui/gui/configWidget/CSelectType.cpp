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
#include "CConfigWidget.h"

#include <QListWidget>


CSelectType::CSelectType(QWidget *parent)
	: QDialog(parent)
	, m_ui(std::make_unique<Ui::CSelectType>())
{
	m_ui->setupUi(this);

	addItem(tr("Points Map"), CConfigWidget::PointsMap);
	addItem(tr("Occupancy"), CConfigWidget::Occupancy);
	addItem(tr("Landmarks"), CConfigWidget::Landmarks);
	addItem(tr("Beacon"), CConfigWidget::Beacon);
	addItem(tr("GasGrid"), CConfigWidget::GasGrid);
}

CSelectType::~CSelectType()
{

}

int CSelectType::selectedItem() const
{
	QListWidgetItem * item = m_ui->m_typeList->currentItem();
	if (!item)
		return -1;

	return item->data(Qt::UserRole).toInt();
}

void CSelectType::addItem(const QString &name, CConfigWidget::TypeOfConfig type)
{
	QListWidgetItem *item = new QListWidgetItem(name, m_ui->m_typeList);
	item->setData(Qt::UserRole, type);
	m_ui->m_typeList->addItem(item);
}

