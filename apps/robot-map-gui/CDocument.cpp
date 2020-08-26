/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CDocument.h"

#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/serialization/CArchive.h>

#include "mrpt/config/CConfigFile.h"
#include "mrpt/io/CFileGZInputStream.h"
#include "mrpt/io/CFileGZOutputStream.h"
#include "mrpt/io/CFileOutputStream.h"

const std::string METRIC_MAP_CONFIG_SECTION = "MappingApplication";

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::io;
using namespace mrpt::config;
using namespace mrpt::maps;

bool CDocument::isFileChanged() const { return m_changedFile; }
void CDocument::loadSimpleMap(const std::string& fileName)
{
	m_fileName = fileName;
	CFileGZInputStream f(fileName.c_str());
	mrpt::serialization::archiveFrom(f) >> m_simplemap;
}

void CDocument::saveSimpleMap()
{
	m_changedFile = !m_simplemap.saveToFile(m_fileName);
}

void CDocument::saveMetricMapRepresentationToFile(
	const std::string& fileName, const std::string& mapName) const
{
	TypeOfConfig type = nameToType(mapName);
	if (type == TypeOfConfig::None) return;

	std::string number = mapName.substr(typeToName(type).size());
	int index = std::atoi(number.c_str());

	auto iter = m_typeConfigs.find(type);
	if (iter == m_typeConfigs.end() || iter->second.empty()) return;

	auto mapIter = iter->second.begin() + index;
	(*mapIter)->saveMetricMapRepresentationToFile(fileName);
}

void CDocument::saveMetricmapInBinaryFormat(
	const std::string& fileName, const std::string& mapName) const
{
	TypeOfConfig type = nameToType(mapName);
	if (type == TypeOfConfig::None) return;

	std::string number = mapName.substr(typeToName(type).size());
	int index = std::atoi(number.c_str());

	auto iter = m_typeConfigs.find(type);
	if (iter == m_typeConfigs.end() || iter->second.empty()) return;

	auto mapIter = iter->second.begin() + index;

	mrpt::io::CFileGZOutputStream fil(fileName);
	mrpt::serialization::archiveFrom(fil) << **mapIter;
}

void CDocument::saveAsPng(const std::string& fileName) const
{
	std::string str = typeToName(Occupancy) + "0";
	saveMetricMapRepresentationToFile(fileName, str);
}

bool CDocument::hasPointsMap() const { return m_hasPointsMap; }
void CDocument::saveAsText(const std::string& fileName) const
{
	for (auto& m : m_metricmap)
	{
		auto ptr = std::dynamic_pointer_cast<CSimplePointsMap>(m);
		if (ptr)
		{
			ptr->save3D_to_text_file(fileName);
			break;
		}
	}
}

const std::string& CDocument::getFileName() const { return m_fileName; }
void CDocument::setListOfMaps(TSetOfMetricMapInitializers& mapCfg)
{
	m_metricmap.setListOfMaps(mapCfg);
	updateMetricMap();
}

void CDocument::setConfig(const std::string& config)
{
	TSetOfMetricMapInitializers mapCfg;
	mapCfg.loadFromConfigFile(CConfigFile(config), METRIC_MAP_CONFIG_SECTION);
	setListOfMaps(mapCfg);

	//	mrpt::io::CFileOutputStream f("/home/lisgein/tmp/test.ini");
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
std::vector<size_t> CDocument::remove(const std::vector<size_t>& indexes)
{
	std::vector<size_t> idx = indexes;
	std::vector<size_t> v;
	for (auto it = idx.begin(); it != idx.end(); ++it)
	{
		auto current = *it;
		m_simplemap.remove(current);
		v.push_back(current);
		for (auto iter = it; iter != idx.end(); ++iter)
		{
			if (*iter > current)
			{
				int checkedNumb = *iter;
				auto pos = idx.erase(iter);
				idx.insert(pos, checkedNumb - 1);
			}
		}
	}
	updateMetricMap();
	m_changedFile = true;
	return v;
}

void CDocument::move(
	const std::vector<size_t>& indexes,
	const CSimpleMap::TPosePDFSensFramePairList& posesObsPairs)
{
	for (size_t i = 0; i < indexes.size(); ++i)
		move(indexes[i], posesObsPairs[i], true);

	m_changedFile = true;
	updateMetricMap();
}

void CDocument::move(
	size_t index, const CSimpleMap::TPosePDFSensFramePair& posesObsPair,
	bool disableUpdateMetricMap)
{
	m_simplemap.remove(index);
	m_simplemap.insertToPos(index, posesObsPair.first, posesObsPair.second);

	m_changedFile = true;
	if (!disableUpdateMetricMap) updateMetricMap();
}

void CDocument::insert(
	const std::vector<size_t>& idx,
	CSimpleMap::TPosePDFSensFramePairList& posesObsPairs)
{
	ASSERT_(idx.size() == posesObsPairs.size());
	for (size_t i = 0; i < idx.size(); ++i)
		m_simplemap.insertToPos(
			idx[i], posesObsPairs[i].first, posesObsPairs[i].second);

	updateMetricMap();
}

CSimpleMap::TPosePDFSensFramePairList CDocument::get(
	const std::vector<size_t>& idx) const
{
	CSimpleMap::TPosePDFSensFramePairList posesObsPairs;
	for (auto& it : idx)
	{
		CSimpleMap::TPosePDFSensFramePair pair = get(it);
		posesObsPairs.push_back(pair);
	}
	return posesObsPairs;
}

CSimpleMap::TPosePDFSensFramePair CDocument::get(size_t idx) const
{
	CSimpleMap::TPosePDFSensFramePair posesObsPair;
	m_simplemap.get(idx, posesObsPair.first, posesObsPair.second);
	return posesObsPair;
}

CSimpleMap::TPosePDFSensFramePairList CDocument::getReverse(
	const std::vector<size_t>& idx) const
{
	CSimpleMap::TPosePDFSensFramePairList posesObsPairs;
	for (int i = idx.size() - 1; i >= 0; --i)
	{
		CSimpleMap::TPosePDFSensFramePair pair;
		m_simplemap.get(idx[i], pair.first, pair.second);
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
			CMetricMap::Ptr ptr = std::dynamic_pointer_cast<CMetricMap>(map);
			if (ptr.get())
			{
				auto obj = std::make_shared<CSetOfObjects>();
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
	m_typeConfigs.emplace(TypeOfConfig::PointsMap, std::vector<MetricPtr>());
	m_typeConfigs.emplace(TypeOfConfig::Occupancy, std::vector<MetricPtr>());
	m_typeConfigs.emplace(TypeOfConfig::Landmarks, std::vector<MetricPtr>());
	m_typeConfigs.emplace(TypeOfConfig::Beacon, std::vector<MetricPtr>());
	m_typeConfigs.emplace(TypeOfConfig::GasGrid, std::vector<MetricPtr>());

	bool addedPointsMap = false;
	for (auto& m : m_metricmap)
	{
		TypeOfConfig type = TypeOfConfig::None;
		{
			auto ptr = std::dynamic_pointer_cast<CSimplePointsMap>(m);
			if (ptr.get())
			{
				type = TypeOfConfig::PointsMap;
				addedPointsMap = true;
			}
		}
		if (type == TypeOfConfig::None)
		{
			auto ptr = std::dynamic_pointer_cast<COccupancyGridMap2D>(m);
			if (ptr.get()) type = TypeOfConfig::Occupancy;
		}
		if (type == TypeOfConfig::None)
		{
			auto ptr = std::dynamic_pointer_cast<CGasConcentrationGridMap2D>(m);
			if (ptr.get()) type = TypeOfConfig::GasGrid;
		}
		if (type == TypeOfConfig::None)
		{
			auto ptr = std::dynamic_pointer_cast<CBeaconMap>(m);
			if (ptr.get()) type = TypeOfConfig::Beacon;
		}
		if (type == TypeOfConfig::None)
		{
			auto ptr = std::dynamic_pointer_cast<CLandmarksMap>(m);
			if (ptr.get()) type = TypeOfConfig::Landmarks;
		}
		if (type != TypeOfConfig::None)
		{
			m_typeConfigs.find(type)->second.push_back(m);
		}
	}

	m_hasPointsMap = addedPointsMap;
}
