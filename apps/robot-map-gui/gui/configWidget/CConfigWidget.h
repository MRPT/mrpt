/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QWidget>

#include "CBaseConfig.h"
#include "TypeOfConfig.h"

#include <memory>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/CMultiMetricMap.h>


class QListWidgetItem;
namespace Ui
{
class CConfigWidget;
}

class CConfigWidget: public QWidget
{
	Q_OBJECT
public:
	CConfigWidget(QWidget *parent = nullptr);
	virtual ~CConfigWidget();
	mrpt::maps::TSetOfMetricMapInitializers config();
	void setConfig(const mrpt::maps::CMultiMetricMap::TListMaps &config);


signals:
	void addedMap();
	void removedMap();
	void updatedConfig();
	void openedConfig(const std::string str);

	void applyConfigurationForCurrentMaps();

private slots:
	void openConfig();
	void saveConfig();
	void addMap();
	void removeMap();
	void currentConfigChanged(QListWidgetItem *current, QListWidgetItem *);

private:
	CBaseConfig *configByType(TypeOfConfig type) const;

	std::unique_ptr<Ui::CConfigWidget> m_ui;
	std::map<TypeOfConfig, std::vector< CBaseConfig *>> m_configs;

	int addWidget(TypeOfConfig type, CBaseConfig* w);
};
