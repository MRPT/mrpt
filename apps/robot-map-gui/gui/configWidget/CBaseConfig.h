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
#include <QWidget>

#include "TypeOfConfig.h"

namespace mrpt
{
namespace maps
{
class TMapGenericParams;
struct TMetricMapInitializer;
}  // namespace maps
namespace config
{
class CLoadableOptions;
}
}  // namespace mrpt

class CBaseConfig : public QWidget
{
   public:
	CBaseConfig();
	~CBaseConfig() override = default;

	virtual const QString getName() = 0;
	virtual void updateConfiguration(
		mrpt::maps::TMetricMapInitializer* options) = 0;
	virtual TypeOfConfig type() const = 0;
};
