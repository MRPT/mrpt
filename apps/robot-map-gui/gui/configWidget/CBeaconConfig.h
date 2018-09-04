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

#include <memory>

#include <mrpt/maps/CBeaconMap.h>

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
