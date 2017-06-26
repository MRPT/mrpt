/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CConfigWidget.h"
#include "ui_CConfigWidget.h"
#include "CSelectType.h"
#include "COccupancyConfig.h"
#include "CPointsConfig.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QDebug>


CConfigWidget::CConfigWidget(QWidget *parent)
	: QWidget(parent)
	, m_ui(std::make_unique<Ui::CConfigWidget>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->m_loadConfig, SIGNAL(released()), SLOT(openConfig()));
	QObject::connect(m_ui->m_saveConfig, SIGNAL(released()), SLOT(saveConfig()));
	QObject::connect(m_ui->m_add, SIGNAL(released()), SLOT(addMap()));


	QListWidgetItem *item = new QListWidgetItem("General", m_ui->m_config);
	item->setData(Qt::UserRole, TypeOfConfig::General);
	m_ui->m_config->addItem(item);

	QListWidgetItem *item2 = new QListWidgetItem("Occupancy", m_ui->m_config);
	item2->setData(Qt::UserRole, TypeOfConfig::Occupancy);
	m_ui->m_config->addItem(item2);

	QListWidgetItem *item3 = new QListWidgetItem("PointsMap", m_ui->m_config);
	item3->setData(Qt::UserRole, TypeOfConfig::PointsMap);
	m_ui->m_config->addItem(item3);

	QWidget* w = new QWidget();
	w->setLayout(new QHBoxLayout(w));
	w->layout()->addWidget(new QCheckBox("test", w));
	m_ui->stackedWidget->addWidget(w);

	m_ui->stackedWidget->addWidget(new COccupancyConfig(m_ui->stackedWidget));
	m_ui->stackedWidget->addWidget(new CPointsConfig(m_ui->stackedWidget));

	QObject::connect(m_ui->m_config, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
					 this, SLOT(currentConfigChanged(QListWidgetItem *, QListWidgetItem *)));

	m_ui->m_config->setCurrentItem(item);
}

CConfigWidget::~CConfigWidget()
{

}

void CConfigWidget::openConfig()
{
	QString configName = QFileDialog::getOpenFileName(this, tr("Open Config File"), "", tr("Files (*.ini)"));
	if (configName.isEmpty())
		return;
	QFile file(configName);
	if (!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::information(this, tr("Unable to open file"), file.errorString());
		return;
	}


}

void CConfigWidget::saveConfig()
{
	QString configName = QFileDialog::getSaveFileName(this, QObject::tr("Save Config File"), "", "Files (*.ini)");
	if (configName.isEmpty())
		return;

	QFile file(configName);
	if (!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::information(this, tr("Unable to open file"), file.errorString());
		return;
	}

	QDataStream out(&file);
	out.setVersion(QDataStream::Qt_5_8);
}

void CConfigWidget::addMap()
{
	std::unique_ptr<CSelectType> dialog = std::make_unique<CSelectType>();
	int result = dialog->exec();
	if (result == QDialog::Accepted)
	{
		CSelectType::TypeOfMaps type = dialog->selectedItem();
		qDebug() << type;
	}
}

void CConfigWidget::currentConfigChanged(QListWidgetItem *current, QListWidgetItem */*previous*/)
{
	if (!current)
		return;
	TypeOfConfig type = static_cast<TypeOfConfig>(current->data(Qt::UserRole).toInt());
	if (type == General)
	{
		m_ui->stackedWidget->setCurrentIndex(0);
	}
	else if (type == Occupancy)
	{
		m_ui->stackedWidget->setCurrentIndex(1);
	}
	else if (type == PointsMap)
	{
		m_ui->stackedWidget->setCurrentIndex(2);
	}
}
