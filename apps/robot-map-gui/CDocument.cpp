/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CDocument.h"

#include "mrpt/utils/CFileGZInputStream.h"
#include "mrpt/utils/CFileOutputStream.h"
#include "mrpt/utils/CConfigFile.h"

#include <QDebug>

const std::string METRIC_MAP_CONFIG_SECTION = "MappingApplication";

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::utils;

CDocument::CDocument()
	: m_simplemap(CSimpleMap()), m_metricmap(CMultiMetricMap())
{
}

CDocument::~CDocument() {}
void CDocument::loadSimpleMap(const std::string& fileName)
{
	CFileGZInputStream file(fileName.c_str());
	file >> m_simplemap;
}

void CDocument::setListOfMaps(TSetOfMetricMapInitializers& mapCfg)
{
	m_metricmap.setListOfMaps(&mapCfg);
	updateMetricMap();
}

void CDocument::setConfig(const std::string& config)
{
	TSetOfMetricMapInitializers mapCfg;
	mapCfg.loadFromConfigFile(CConfigFile(config), METRIC_MAP_CONFIG_SECTION);
	setListOfMaps(mapCfg);

	//	mrpt::utils::CFileOutputStream f("/home/lisgein/tmp/test.ini");
	//	mapCfg.dumpToTextStream(f);
}

const RenderizableMaps CDocument::renderizableMaps() const
{
	RenderizableMaps renderizable;

	addMapToRenderizableMaps(TypeOfConfig::PointsMap, renderizable);
	addMapToRenderizableMaps(TypeOfConfig::Occupancy, renderizable);
	addMapToRenderizableMaps(TypeOfConfig::Landmarks, renderizable);
	addMapToRenderizableMaps(TypeOfConfig::Beacon, renderizable);
	addMapToRenderizableMaps(TypeOfConfig::GasGrid, renderizable);
	return renderizable;
}

const CSimpleMap& CDocument::simplemap() const { return m_simplemap; }
const CMultiMetricMap::TListMaps& CDocument::config() const
{
	return m_metricmap.maps;
}

const TypeConfig& CDocument::typeConfig() const { return m_typeConfigs; }
void CDocument::remove(const std::vector<int>& idx)
{
	for (auto& it : idx) m_simplemap.remove(it);
	updateMetricMap();
}

void CDocument::insert(const std::vector<int> &idx, CSimpleMap::TPosePDFSensFramePairList &posesObsPairs)
{
	assert(idx.size() == posesObsPairs.size());
	for (size_t i = 0; i < idx.size(); ++i)
		m_simplemap.insertToPos(idx[i], posesObsPairs[i].first, posesObsPairs[i].second);

	updateMetricMap();
}

CSimpleMap::TPosePDFSensFramePairList CDocument::get(const std::vector<int> &idx)
{
	CSimpleMap::TPosePDFSensFramePairList posesObsPairs;
	for (auto& it : idx)
	{
		CSimpleMap::TPosePDFSensFramePair pair;
		m_simplemap.get(it, pair.first, pair.second);
		posesObsPairs.push_back(pair);
	}
	return posesObsPairs;
}

void CDocument::addMapToRenderizableMaps(
	TypeOfConfig type, RenderizableMaps& renderMaps) const
{
	auto iter = m_typeConfigs.find(type);
	if (iter != m_typeConfigs.end())
	{
		int index = 0;
		for (auto& map : iter->second)
		{
			CMetricMap::Ptr ptr =
				std::dynamic_pointer_cast<CMetricMap>(map.get_ptr());
			if (ptr.get())
			{
				auto obj = mrpt::make_aligned_shared<CSetOfObjects>();
				ptr->getAs3DObject(obj);
				renderMaps.emplace(SType(type, index), obj);
			}
			++index;
		}
	}
}

void CDocument::updateMetricMap()
{
	m_metricmap.loadFromProbabilisticPosesAndObservations(m_simplemap);

	m_typeConfigs.clear();
	m_typeConfigs.emplace(
		TypeOfConfig::PointsMap, std::vector<MetricPolyPtr>());
	m_typeConfigs.emplace(
		TypeOfConfig::Occupancy, std::vector<MetricPolyPtr>());
	m_typeConfigs.emplace(
		TypeOfConfig::Landmarks, std::vector<MetricPolyPtr>());
	m_typeConfigs.emplace(TypeOfConfig::Beacon, std::vector<MetricPolyPtr>());
	m_typeConfigs.emplace(TypeOfConfig::GasGrid, std::vector<MetricPolyPtr>());

	for (auto iter = m_metricmap.begin(); iter != m_metricmap.end(); ++iter)
	{
		TypeOfConfig type = TypeOfConfig::None;
		{
			CSimplePointsMap::Ptr ptr =
				std::dynamic_pointer_cast<CSimplePointsMap>(iter->get_ptr());
			if (ptr.get()) type = TypeOfConfig::PointsMap;
		}
		if (type == TypeOfConfig::None)
		{
			COccupancyGridMap2D::Ptr ptr =
				std::dynamic_pointer_cast<COccupancyGridMap2D>(iter->get_ptr());
			if (ptr.get()) type = TypeOfConfig::Occupancy;
		}
		if (type == TypeOfConfig::None)
		{
			CGasConcentrationGridMap2D::Ptr ptr =
				std::dynamic_pointer_cast<CGasConcentrationGridMap2D>(
					iter->get_ptr());
			if (ptr.get()) type = TypeOfConfig::GasGrid;
		}
		if (type == TypeOfConfig::None)
		{
			CBeaconMap::Ptr ptr =
				std::dynamic_pointer_cast<CBeaconMap>(iter->get_ptr());
			if (ptr.get()) type = TypeOfConfig::Beacon;
		}
		if (type == TypeOfConfig::None)
		{
			CLandmarksMap::Ptr ptr =
				std::dynamic_pointer_cast<CLandmarksMap>(iter->get_ptr());
			if (ptr.get()) type = TypeOfConfig::Landmarks;
		}
		if (type != TypeOfConfig::None)
		{
			m_typeConfigs.find(type)->second.push_back(iter->get_ptr());
		}
	}
}
