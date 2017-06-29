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
#include "CLandmarksConfig.h"
#include "CGasGridConfig.h"

#include <mrpt/utils/CFileOutputStream.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QDebug>


using namespace mrpt;
using namespace maps;
using namespace opengl;

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
	mrpt::utils::CFileOutputStream f(configName.toStdString(), true);

	for (int i = 0; i < m_ui->stackedWidget->count(); ++i)
	{
		QWidget *w = m_ui->stackedWidget->widget(i);
		COccupancyConfig *o = dynamic_cast<COccupancyConfig*>(w);
		Q_UNUSED(o);
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
		CBaseConfig *w = configByType(typeOfConfig);
		if (w)
			addWidget(typeOfConfig, w);
	}
}

void CConfigWidget::currentConfigChanged(QListWidgetItem *current, QListWidgetItem */*previous*/)
{
	if (!current)
		return;

	m_ui->stackedWidget->setCurrentIndex(m_ui->m_config->currentRow());
}

CBaseConfig *CConfigWidget::configByType(TypeOfConfig type) const
{
	switch (type)
	{
	case TypeOfConfig::PointsMap:
	{
		return new CPointsConfig(m_ui->stackedWidget);
	}
	case TypeOfConfig::Occupancy:
	{
		return new COccupancyConfig(m_ui->stackedWidget);
	}
	case TypeOfConfig::Beacon:
	{
		return new CBeaconConfig(m_ui->stackedWidget);
	}
	case TypeOfConfig::GasGrid:
	{
		return new CGasGridConfig(m_ui->stackedWidget);
	}
	case TypeOfConfig::Landmarks:
	{
		return new CLandmarksConfig(m_ui->stackedWidget);
	}
	default:
		break;
	}
	return nullptr;

}

TSetOfMetricMapInitializers CConfigWidget::config()
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
			const std::string sMapName = map->getName().toStdString();
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

void CConfigWidget::setConfig(const CMultiMetricMap::TListMaps &config)
{
	for (auto iter = config.begin(); iter != config.end(); ++iter)
	{
		bool found = false;
		{
			CSimplePointsMap::Ptr ptr = std::dynamic_pointer_cast<CSimplePointsMap>(iter->get_ptr());
			if (ptr.get())
			{
				CPointsConfig *pConfig = new CPointsConfig(m_ui->stackedWidget);
				addWidget(TypeOfConfig::PointsMap, pConfig);
				pConfig->updateConfiguration(ptr->MapDefinition());
				found = true;
			}
		}
		if (!found)
		{
			COccupancyGridMap2D::Ptr ptr = std::dynamic_pointer_cast<COccupancyGridMap2D>(iter->get_ptr());
			if (ptr.get())
			{
				COccupancyConfig *pConfig = new COccupancyConfig(m_ui->stackedWidget);
				addWidget(TypeOfConfig::Occupancy, pConfig);
				pConfig->updateConfiguration(ptr->MapDefinition());
				found = true;
			}
		}
		if (!found)
		{
			CGasConcentrationGridMap2D::Ptr ptr = std::dynamic_pointer_cast<CGasConcentrationGridMap2D>(iter->get_ptr());
			if (ptr.get())
			{
				CGasGridConfig *pConfig = new CGasGridConfig(m_ui->stackedWidget);
				addWidget(TypeOfConfig::GasGrid, pConfig);
				pConfig->updateConfiguration(ptr->MapDefinition());
				found = true;
			}
		}
		if (!found)
		{
			CBeaconMap::Ptr ptr = std::dynamic_pointer_cast<CBeaconMap>(iter->get_ptr());
			if (ptr.get())
			{
				CBeaconConfig *pConfig = new CBeaconConfig(m_ui->stackedWidget);
				addWidget(TypeOfConfig::Beacon, pConfig);
				pConfig->updateConfiguration(ptr->MapDefinition());
				found = true;
			}
		}
		if (!found)
		{
			CLandmarksMap::Ptr ptr = std::dynamic_pointer_cast<CLandmarksMap>(iter->get_ptr());
			if (ptr.get())
			{
				CLandmarksConfig *pConfig = new CLandmarksConfig(m_ui->stackedWidget);
				addWidget(TypeOfConfig::Landmarks, pConfig);
				pConfig->updateConfiguration(ptr->MapDefinition());
				found = true;
			}
		}
	}
}

int CConfigWidget::addWidget(TypeOfConfig type, CBaseConfig *w)
{
	auto it = m_configs.find(type);
	if (it == m_configs.end())
		it = m_configs.emplace(type, std::vector<CBaseConfig *>()).first;


	int numberOfType = it->second.size();

	QListWidgetItem *item = new QListWidgetItem( w->getName() + QString::number(numberOfType), m_ui->m_config);
	item->setData(Qt::UserRole, type);
	m_ui->m_config->addItem(item);
	m_ui->stackedWidget->addWidget(w);

	int index = it->second.size();
	it->second.push_back(w);

	emit addedMap();
	return index;
}
