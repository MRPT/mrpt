/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include "CBaseConfig.h"
#include "ui_CGeneralConfig.h"

#include <memory>

struct SGeneralSetting
{
	SGeneralSetting();
	QColor backgroundColor;
	QColor gridColor;
	double robotPosesSize;
	double selectedRobotPosesSize;
	int robotPosesColor;
	int selectedRobotPosesColor;
	bool isGridVisibleChanged{true};
	int currentBot{0};
};

class CGeneralConfig : public CBaseConfig
{
	Q_OBJECT
   public:
	CGeneralConfig();
	~CGeneralConfig() override = default;

	const QString getName() override;
	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	TypeOfConfig type() const override;
	const SGeneralSetting& generalSetting();

   private slots:
	void openGridColor();
	void openBackgroundColor();

   private:
	std::unique_ptr<Ui::CGeneralConfig> m_ui;
	SGeneralSetting m_generalSetting;
};
