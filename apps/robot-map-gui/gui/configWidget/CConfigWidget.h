/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <QWidget>

#include "CBaseConfig.h"
#include "CGeneralConfig.h"
#include "TypeOfConfig.h"

#include <memory>

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/opengl/CSetOfObjects.h>

class QListWidgetItem;
class CGeneralConfig;
namespace Ui
{
class CConfigWidget;
}

/** This class contains configuration*/
class CConfigWidget : public QWidget
{
	Q_OBJECT
   public:
	CConfigWidget(QWidget* parent = nullptr);
	~CConfigWidget() override;
	mrpt::maps::TSetOfMetricMapInitializers config();
	void setConfig(const mrpt::maps::CMultiMetricMap::TListMaps& config);

	const SGeneralSetting& generalSetting();

   signals:
	void addedMap();
	void removedMap();
	void updatedConfig();
	void openedConfig(const std::string& str);
	void applyConfigurationForCurrentMaps();

   public slots:
	void openConfig();

   private slots:
	void saveConfig();
	void addMap();
	void removeMap();
	void currentConfigChanged(QListWidgetItem* current, QListWidgetItem*);

   private:
	CBaseConfig* configByType(TypeOfConfig type) const;
	void clearConfig(bool deleteGeneral = false);

	CGeneralConfig* m_general;
	std::unique_ptr<Ui::CConfigWidget> m_ui;
	std::map<TypeOfConfig, std::vector<CBaseConfig*>> m_configs;

	int addWidget(TypeOfConfig type, CBaseConfig* w);
};
