/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include "CBaseConfig.h"

#include <memory>


namespace Ui
{
class CPointsConfig;
}
class CPointsConfig: public CBaseConfig
{
public:
	CPointsConfig(QWidget *parent = nullptr);
	virtual ~CPointsConfig();

	virtual const std::string getName() override;
	virtual void updateConfiguration(mrpt::maps::TMetricMapInitializer *options) override;

private:
	std::unique_ptr<Ui::CPointsConfig> m_ui;
};

