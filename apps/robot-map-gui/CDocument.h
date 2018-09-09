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
#include <string>

#include "mrpt/maps/CSimpleMap.h"
#include "mrpt/maps/CMultiMetricMap.h"
#include "mrpt/opengl/CRenderizable.h"
#include "mrpt/config/CConfigFile.h"

#include "gui/configWidget/CConfigWidget.h"
#include "TypeOfConfig.h"

/** This class gets *.simplemap and *.ini, and convert it to name of the map -
CSetOfObjects.
Also, it provides work with *.simplemap.
*/

using MetricPolyPtr =
	mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr>;
using RenderizableMaps = std::map<SType, mrpt::opengl::CSetOfObjects::Ptr>;
using TypeConfig = std::map<TypeOfConfig, std::vector<MetricPolyPtr>>;

class CDocument
{
   public:
	CDocument() = default;
	~CDocument() = default;

	bool isFileChanged() const;

	void loadSimpleMap(const std::string& fileName);
	void saveSimpleMap();

	void saveMetricMapRepresentationToFile(
		const std::string& fileName, const std::string& mapName) const;

	void saveMetricmapInBinaryFormat(
		const std::string& fileName, const std::string& mapName) const;

	void saveAsPng(const std::string& fileName) const;

	bool hasPointsMap() const;
	void saveAsText(const std::string& fileName) const;

	const std::string& getFileName() const;

	void setListOfMaps(mrpt::maps::TSetOfMetricMapInitializers& mapCfg);
	void setConfig(const std::string& config);

	const RenderizableMaps renderizableMaps() const;

	const mrpt::maps::CSimpleMap& simplemap() const;
	const mrpt::maps::CMultiMetricMap::TListMaps& config() const;

	const TypeConfig& typeConfig() const;

	std::vector<size_t> remove(const std::vector<size_t>& indexes);

	void move(
		const std::vector<size_t>& indexes,
		const mrpt::maps::CSimpleMap::TPosePDFSensFramePairList& posesObsPairs);
	void move(
		size_t index,
		const mrpt::maps::CSimpleMap::TPosePDFSensFramePair& posesObsPair,
		bool disableUpdateMetricMap = false);

	void insert(
		const std::vector<size_t>& idx,
		mrpt::maps::CSimpleMap::TPosePDFSensFramePairList& posesObsPairs);

	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList get(
		const std::vector<size_t>& idx) const;
	mrpt::maps::CSimpleMap::TPosePDFSensFramePair get(size_t idx) const;

	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList getReverse(
		const std::vector<size_t>& idx) const;

   private:
	void addMapToRenderizableMaps(
		TypeOfConfig type, RenderizableMaps& renderMaps) const;
	void updateMetricMap();

	mrpt::maps::CSimpleMap m_simplemap;
	mrpt::maps::CMultiMetricMap m_metricmap;
	TypeConfig m_typeConfigs;
	std::string m_fileName;
	bool m_changedFile{false};
	bool m_hasPointsMap{false};
};
