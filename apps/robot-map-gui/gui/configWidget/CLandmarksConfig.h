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
#include <mrpt/maps/CLandmarksMap.h>

#include <memory>

#include "CBaseConfig.h"

namespace Ui
{
class CLandmarksConfig;
}
class CLandmarksConfig : public CBaseConfig
{
   public:
	CLandmarksConfig();
	~CLandmarksConfig() override = default;

	const QString getName() override;
	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	TypeOfConfig type() const override;

	void setInsertOpt(
		const mrpt::maps::CLandmarksMap::TInsertionOptions& insertOpt =
			mrpt::maps::CLandmarksMap::TInsertionOptions());
	void setLikelihoodOpt(
		const mrpt::maps::CLandmarksMap::TLikelihoodOptions& likelihoodOpt =
			mrpt::maps::CLandmarksMap::TLikelihoodOptions());

   private:
	std::unique_ptr<Ui::CLandmarksConfig> m_ui;
};
