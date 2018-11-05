/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CConfigWidget.h"
#include "ui_CConfigWidget.h"
#include "CSelectType.h"
#include "COccupancyConfig.h"
#include "CGeneralConfig.h"
#include "CPointsConfig.h"
#include "CBeaconConfig.h"
#include "CLandmarksConfig.h"
#include "CGasGridConfig.h"

#include "mrpt/io/CFileOutputStream.h"
#include "mrpt/gui/error_box.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QCheckBox>

using namespace mrpt;
using namespace maps;
using namespace opengl;

CConfigWidget::CConfigWidget(QWidget* parent)
	: QWidget(parent),
	  m_general(new CGeneralConfig()),
	  m_ui(std::make_unique<Ui::CConfigWidget>())
{
	m_ui->setupUi(this);

	addWidget(TypeOfConfig::General, m_general);

	connect(
		m_ui->m_loadConfig, &QPushButton::released, this,
		&CConfigWidget::openConfig);
	connect(
		m_ui->m_saveConfig, &QPushButton::released, this,
		&CConfigWidget::saveConfig);
	connect(m_ui->m_add, &QPushButton::released, this, &CConfigWidget::addMap);
	connect(
		m_ui->m_remove, &QPushButton::released, this,
		&CConfigWidget::removeMap);
	connect(
		m_ui->m_apply, &QPushButton::released, this,
		&CConfigWidget::applyConfigurationForCurrentMaps);

	connect(
		m_ui->m_config, &QListWidget::currentItemChanged, this,
		&CConfigWidget::currentConfigChanged);
}

CConfigWidget::~CConfigWidget() { clearConfig(true); }
void CConfigWidget::openConfig()
{
	mrpt::gui::tryCatch(
		[this]() {
			QString configName = QFileDialog::getOpenFileName(
				this, tr("Open Config File"), "", tr("Files (*.ini)"));
			if (configName.isEmpty()) return;
			QFile file(configName);
			if (!file.open(QIODevice::ReadOnly)) throw "";

			emit openedConfig(configName.toStdString());
		},
		"The file is corrupted and cannot be opened!");
}

void CConfigWidget::saveConfig()
{
	QString configName = QFileDialog::getSaveFileName(
		this, QObject::tr("Save Config File"), "", "Files (*.ini)");
	if (configName.isEmpty()) return;

	QFile file(configName);
	if (!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::information(
			this, tr("Unable to open file"), file.errorString());
		return;
	}
	mrpt::io::CFileOutputStream f(configName.toStdString(), true);

	for (int i = 0; i < m_ui->stackedWidget->count(); ++i)
	{
		QWidget* w = m_ui->stackedWidget->widget(i);
		COccupancyConfig* o = dynamic_cast<COccupancyConfig*>(w);
		Q_UNUSED(o);
		// TODO: saving configurations
	}
}

void CConfigWidget::addMap()
{
	std::unique_ptr<CSelectType> dialog = std::make_unique<CSelectType>();
	int result = dialog->exec();
	if (result == QDialog::Accepted)
	{
		int type = dialog->selectedItem();
		auto typeOfConfig = static_cast<TypeOfConfig>(type);
		CBaseConfig* w = configByType(typeOfConfig);
		if (w) addWidget(typeOfConfig, w);
	}
}

void CConfigWidget::removeMap()
{
	int currentRow = m_ui->m_config->currentRow();
	if (currentRow <= 0) return;

	QWidget* w = m_ui->stackedWidget->widget(currentRow);
	ASSERT_(w);

	auto* base = dynamic_cast<CBaseConfig*>(w);
	ASSERT_(base);

	TypeOfConfig type = base->type();

	auto it = m_configs.find(type);
	if (it != m_configs.end())
	{
		for (auto vectorIter = it->second.begin();
			 vectorIter != it->second.end(); ++vectorIter)
			if (*vectorIter == base)
			{
				it->second.erase(vectorIter);
				break;
			}
		delete (m_ui->m_config->takeItem(m_ui->m_config->currentRow()));
		emit removedMap();
	}
	delete base;
}

void CConfigWidget::currentConfigChanged(
	QListWidgetItem* current, QListWidgetItem* /*previous*/)
{
	if (!current) return;

	m_ui->stackedWidget->setCurrentIndex(m_ui->m_config->currentRow());
}

CBaseConfig* CConfigWidget::configByType(TypeOfConfig type) const
{
	switch (type)
	{
		case TypeOfConfig::PointsMap:
		{
			return new CPointsConfig();
		}
		case TypeOfConfig::Occupancy:
		{
			return new COccupancyConfig();
		}
		case TypeOfConfig::Beacon:
		{
			return new CBeaconConfig();
		}
		case TypeOfConfig::GasGrid:
		{
			return new CGasGridConfig();
		}
		case TypeOfConfig::Landmarks:
		{
			return new CLandmarksConfig();
		}
		default:
			break;
	}
	return nullptr;
}

void CConfigWidget::clearConfig(bool deleteGeneral)
{
	for (int i = 0; i < m_ui->stackedWidget->count();)
	{
		QWidget* w = m_ui->stackedWidget->widget(i);

		if (deleteGeneral || i != 0)
		{
			m_ui->stackedWidget->removeWidget(w);
			delete w;
			QListWidgetItem* item = m_ui->m_config->takeItem(i);
			delete item;
		}
		else
			++i;
	}

	m_configs.clear();
}

TSetOfMetricMapInitializers CConfigWidget::config()
{
	using mrpt::maps::internal::TMetricMapTypesRegistry;
	TMetricMapTypesRegistry& mmr = TMetricMapTypesRegistry::Instance();
	TSetOfMetricMapInitializers mapCfg;
	MRPT_START
	mapCfg.clear();
	for (auto& it : m_configs)
	{
		int index = 0;
		for (auto& map : it.second)
		{
			const std::string sMapName = map->getName().toStdString();
			TMetricMapInitializer* mi = mmr.factoryMapDefinition(sMapName);
			ASSERT_(mi);

			map->updateConfiguration(mi);
			mapCfg.push_back(TMetricMapInitializer::Ptr(mi));
			++index;
		}
	}
	MRPT_END
	return mapCfg;
}

void CConfigWidget::setConfig(const CMultiMetricMap::TListMaps& config)
{
	clearConfig();

	for (auto iter = config.begin(); iter != config.end(); ++iter)
	{
		bool found = false;
		{
			CSimplePointsMap::Ptr ptr =
				std::dynamic_pointer_cast<CSimplePointsMap>(iter->get_ptr());
			if (ptr.get())
			{
				CPointsConfig* pConfig = new CPointsConfig();
				addWidget(TypeOfConfig::PointsMap, pConfig);
				pConfig->setInsertOpt(ptr->insertionOptions);
				pConfig->setLikelihoodOpt(ptr->likelihoodOptions);
				found = true;
			}
		}
		if (!found)
		{
			COccupancyGridMap2D::Ptr ptr =
				std::dynamic_pointer_cast<COccupancyGridMap2D>(iter->get_ptr());
			if (ptr.get())
			{
				COccupancyConfig* pConfig = new COccupancyConfig();
				addWidget(TypeOfConfig::Occupancy, pConfig);
				pConfig->setCreationOpt(
					ptr->getXMin(), ptr->getXMax(), ptr->getYMin(),
					ptr->getYMax(), ptr->getResolution());
				pConfig->setInsertOpt(ptr->insertionOptions);
				pConfig->setLikelihoodOpt(ptr->likelihoodOptions);
				found = true;
			}
		}
		if (!found)
		{
			CGasConcentrationGridMap2D::Ptr ptr =
				std::dynamic_pointer_cast<CGasConcentrationGridMap2D>(
					iter->get_ptr());
			if (ptr.get())
			{
				CGasGridConfig* pConfig = new CGasGridConfig();
				addWidget(TypeOfConfig::GasGrid, pConfig);
				pConfig->setCreationOpt(
					ptr->getXMin(), ptr->getXMax(), ptr->getYMin(),
					ptr->getYMax(), ptr->getResolution());
				pConfig->setInsertOpt(ptr->insertionOptions);
				// pConfig->setMapTypeOpt(ptr->mapType);
				found = true;
			}
		}
		if (!found)
		{
			CBeaconMap::Ptr ptr =
				std::dynamic_pointer_cast<CBeaconMap>(iter->get_ptr());
			if (ptr.get())
			{
				CBeaconConfig* pConfig = new CBeaconConfig();
				addWidget(TypeOfConfig::Beacon, pConfig);
				pConfig->setInsertOpt(ptr->insertionOptions);
				pConfig->setLikelihoodOpt(ptr->likelihoodOptions);
				found = true;
			}
		}
		if (!found)
		{
			CLandmarksMap::Ptr ptr =
				std::dynamic_pointer_cast<CLandmarksMap>(iter->get_ptr());
			if (ptr.get())
			{
				CLandmarksConfig* pConfig = new CLandmarksConfig();
				addWidget(TypeOfConfig::Landmarks, pConfig);
				pConfig->setInsertOpt(ptr->insertionOptions);
				pConfig->setLikelihoodOpt(ptr->likelihoodOptions);
				found = true;
			}
		}
	}
}

const SGeneralSetting& CConfigWidget::generalSetting()
{
	return m_general->generalSetting();
}

int CConfigWidget::addWidget(TypeOfConfig type, CBaseConfig* w)
{
	QString nameListWidgetItem = w->getName();

	auto it = m_configs.find(type);
	if (type != TypeOfConfig::General)
	{
		if (it == m_configs.end())
			it = m_configs.emplace(type, std::vector<CBaseConfig*>()).first;

		nameListWidgetItem += QString::number(it->second.size());
	}

	QListWidgetItem* item =
		new QListWidgetItem(nameListWidgetItem, m_ui->m_config);
	item->setData(Qt::UserRole, type);
	m_ui->m_config->addItem(item);
	m_ui->stackedWidget->addWidget(w);

	int index = 0;
	if (type != TypeOfConfig::General)
	{
		index = it->second.size();
		it->second.push_back(w);
	}

	emit addedMap();
	return index;
}
