/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once

#include <mrpt/maps/CBeaconMap.h>

#include <memory>

#include "CBaseConfig.h"

namespace Ui
{
class CBeaconConfig;
}
class CBeaconConfig : public CBaseConfig
{
   public:
	CBeaconConfig();
	~CBeaconConfig() override = default;

	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	const QString getName() override;
	TypeOfConfig type() const override;

	void setInsertOpt(
		const mrpt::maps::CBeaconMap::TInsertionOptions& insertOpt =
			mrpt::maps::CBeaconMap::TInsertionOptions());
	void setLikelihoodOpt(
		const mrpt::maps::CBeaconMap::TLikelihoodOptions& likelihoodOpt =
			mrpt::maps::CBeaconMap::TLikelihoodOptions());

   private:
	std::unique_ptr<Ui::CBeaconConfig> m_ui;
};
