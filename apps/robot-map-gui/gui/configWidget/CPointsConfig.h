/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <mrpt/maps/CSimplePointsMap.h>

#include <memory>

#include "CBaseConfig.h"

namespace Ui
{
class CPointsConfig;
}
class CPointsConfig : public CBaseConfig
{
   public:
	CPointsConfig();
	~CPointsConfig() override;

	const QString getName() override;
	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	TypeOfConfig type() const override;

	void setInsertOpt(
		const mrpt::maps::CSimplePointsMap::TInsertionOptions& insertOpt =
			mrpt::maps::CSimplePointsMap::TInsertionOptions());
	void setLikelihoodOpt(
		const mrpt::maps::CSimplePointsMap::TLikelihoodOptions& likelihoodOpt =
			mrpt::maps::CSimplePointsMap::TLikelihoodOptions());

   private:
	std::unique_ptr<Ui::CPointsConfig> m_ui;
};
