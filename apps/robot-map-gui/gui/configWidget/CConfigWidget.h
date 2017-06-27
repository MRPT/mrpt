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

#include <memory>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/TMetricMapInitializer.h>


class QListWidgetItem;
namespace Ui
{
class CConfigWidget;
}

class CConfigWidget: public QWidget
{
	Q_OBJECT
public:
	enum TypeOfConfig
	{
		None = -1,
		General = 0,
		PointsMap = 1,
		Occupancy = 2,
		Landmarks = 3,
		Beacon = 4,
		GasGrid = 5
	};
	CConfigWidget(QWidget *parent = nullptr);
	virtual ~CConfigWidget();
	mrpt::maps::TSetOfMetricMapInitializers updateConfig();


signals:
	void addedMap();
	void updatedConfig();

private slots:
	void openConfig();
	void saveConfig();
	void addMap();
	void currentConfigChanged(QListWidgetItem *current, QListWidgetItem *);

private:
	std::unique_ptr<Ui::CConfigWidget> m_ui;
	std::map<TypeOfConfig, std::vector< CBaseConfig *>> m_configs;

	void addWidget(TypeOfConfig type, const QString &name, CBaseConfig* w);
};
