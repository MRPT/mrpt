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
#include "CBeaconConfig.h"

#include <mrpt/utils/CFileOutputStream.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QDebug>


using namespace mrpt;
using namespace maps;

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

	QWidget* w = new QWidget();
	w->setLayout(new QHBoxLayout(w));
	w->layout()->addWidget(new QCheckBox("test", w));
	m_ui->stackedWidget->addWidget(w);


	QObject::connect(m_ui->m_config, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
					 this, SLOT(currentConfigChanged(QListWidgetItem *, QListWidgetItem *)));
	QObject::connect(m_ui->m_apply, SIGNAL(released()), this, SIGNAL(updatedConfig()));

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
	emit openedConfig(configName.toStdString());
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

}

void CConfigWidget::addMap()
{
	std::unique_ptr<CSelectType> dialog = std::make_unique<CSelectType>();
	int result = dialog->exec();
	if (result == QDialog::Accepted)
	{
		int type = dialog->selectedItem();
		TypeOfConfig typeOfConfig = static_cast<TypeOfConfig>(type);
		switch (typeOfConfig)
		{
		case PointsMap:
		{
			addWidget(TypeOfConfig::PointsMap, "pointsMap", new CPointsConfig(m_ui->stackedWidget));
			break;
		}
		case Occupancy:
		{
			addWidget(TypeOfConfig::Occupancy, "occupancyGrid", new COccupancyConfig(m_ui->stackedWidget));
			break;
		}
		case Beacon:
		{
			addWidget(TypeOfConfig::Beacon, "beaconMap", new CBeaconConfig(m_ui->stackedWidget));
			break;
		}
		default:
			break;
		}
	}
}

void CConfigWidget::currentConfigChanged(QListWidgetItem *current, QListWidgetItem */*previous*/)
{
	if (!current)
		return;

	m_ui->stackedWidget->setCurrentIndex(m_ui->m_config->currentRow());
}

TSetOfMetricMapInitializers CConfigWidget::updateConfig()
{

	using internal::TMetricMapTypesRegistry;
	TMetricMapTypesRegistry & mmr = TMetricMapTypesRegistry::Instance();
	TSetOfMetricMapInitializers mapCfg;
	MRPT_START
	mapCfg.clear();
	for (auto &it: m_configs)
	{
		int index = 0;
		for (auto &map : it.second)
		{
			const std::string sMapName = map->getName();
			TMetricMapInitializer *mi = mmr.factoryMapDefinition(sMapName);
			ASSERT_(mi);

			map->updateConfiguration(mi);
			mapCfg.push_back(TMetricMapInitializer::Ptr(mi));
			++index;
		}
	}
	MRPT_END
	return mapCfg;
}

void CConfigWidget::addWidget(CConfigWidget::TypeOfConfig type, const QString &name, CBaseConfig *w)
{
	auto it = m_configs.find(type);
	if (it == m_configs.end())
		it = m_configs.emplace(type, std::vector<CBaseConfig *>()).first;


	int numberOfType = it->second.size();

	QListWidgetItem *item = new QListWidgetItem( name + QString::number(numberOfType), m_ui->m_config);
	item->setData(Qt::UserRole, type);
	m_ui->m_config->addItem(item);
	m_ui->stackedWidget->addWidget(w);
	it->second.push_back(w);

	emit addedMap();
}
