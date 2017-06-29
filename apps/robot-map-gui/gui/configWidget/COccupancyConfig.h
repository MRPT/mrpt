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
#include <mrpt/maps/COccupancyGridMap2D.h>


namespace Ui{class COccupancyConfig;}
namespace mrpt
{
namespace utils{class CFileOutputStream;}
}

class COccupancyConfig: public CBaseConfig
{
public:
	COccupancyConfig(QWidget *parent);
	virtual ~COccupancyConfig();

	virtual const QString getName() override;
	virtual void updateConfiguration(mrpt::maps::TMetricMapInitializer *options) override;
private:
	std::unique_ptr<Ui::COccupancyConfig> m_ui;
};
