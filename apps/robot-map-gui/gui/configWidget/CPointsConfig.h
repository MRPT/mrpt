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

#include <mrpt/maps/CSimplePointsMap.h>

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
