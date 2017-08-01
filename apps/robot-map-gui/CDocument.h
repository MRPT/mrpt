/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <string>

#include "mrpt/maps/CSimpleMap.h"
#include "mrpt/maps/CMultiMetricMap.h"
#include "mrpt/opengl/CRenderizable.h"
#include "mrpt/utils/CConfigFile.h"

#include "gui/configWidget/CConfigWidget.h"
#include "TypeOfConfig.h"

typedef mrpt::utils::poly_ptr_ptr<mrpt::maps::CMetricMap::Ptr> MetricPolyPtr;
typedef std::map<SType, mrpt::opengl::CSetOfObjects::Ptr> RenderizableMaps;
typedef std::map<TypeOfConfig, std::vector<MetricPolyPtr>> TypeConfig;

class CDocument
{
   public:
	CDocument();
	~CDocument();

	void loadSimpleMap(const std::string& fileName);

	void setListOfMaps(mrpt::maps::TSetOfMetricMapInitializers& mapCfg);
	void setConfig(const std::string& config);

	const RenderizableMaps renderizableMaps() const;

	const mrpt::maps::CSimpleMap& simplemap() const;
	const mrpt::maps::CMultiMetricMap::TListMaps& config() const;

	const TypeConfig& typeConfig() const;

	void remove(const std::vector<int>& idx);

   private:
	void addMapToRenderizableMaps(
		TypeOfConfig type, RenderizableMaps& renderMaps) const;

	mrpt::maps::CSimpleMap m_simplemap;
	mrpt::maps::CMultiMetricMap m_metricmap;
	TypeConfig m_typeConfigs;
};
