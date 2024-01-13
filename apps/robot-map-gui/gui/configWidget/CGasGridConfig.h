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
#include <mrpt/maps/CGasConcentrationGridMap2D.h>

#include <memory>

#include "CBaseConfig.h"

namespace Ui
{
class CGasGridConfig;
}
class CGasGridConfig : public CBaseConfig
{
   public:
	CGasGridConfig();
	~CGasGridConfig() override = default;

	const QString getName() override;
	void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) override;
	TypeOfConfig type() const override;

	void setCreationOpt(
		float min_x, float max_x, float min_y, float max_y, float resolution);
	void setInsertOpt(
		const mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions&
			insertOpt =
				mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions());
	void setMapTypeOpt(
		const mrpt::maps::CGasConcentrationGridMap2D::TMapRepresentation&);

   private:
	std::unique_ptr<Ui::CGasGridConfig> m_ui;
};
