/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <mrpt/maps/COccupancyGridMap2D.h>

#include <memory>

#include "CBaseConfig.h"

namespace Ui
{
class COccupancyConfig;
}
namespace mrpt
{
namespace io
{
class CFileOutputStream;
}
}  // namespace mrpt

class COccupancyConfig : public CBaseConfig
{
   public:
	COccupancyConfig();
	~COccupancyConfig() override = default;

	const QString getName() override;
	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	TypeOfConfig type() const override;

	void setCreationOpt(
		float min_x, float max_x, float min_y, float max_y, float resolution);
	void setInsertOpt(
		const mrpt::maps::COccupancyGridMap2D::TInsertionOptions& insertOpt =
			mrpt::maps::COccupancyGridMap2D::TInsertionOptions());
	void setLikelihoodOpt(
		const mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions&
			likelihoodOpt =
				mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions());

   private:
	std::unique_ptr<Ui::COccupancyConfig> m_ui;
};
