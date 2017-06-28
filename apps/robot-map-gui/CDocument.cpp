/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CDocument.h"

#include "mrpt/utils/CFileGZInputStream.h"
#include "mrpt/utils/CFileOutputStream.h"
#include "mrpt/utils/CConfigFile.h"


const std::string METRIC_MAP_CONFIG_SECTION  =  "MappingApplication";

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::utils;

CDocument::CDocument(const std::string &fileName)
	: m_simplemap(CSimpleMap())
	, m_metricmap(CMultiMetricMap())
{
	CFileGZInputStream file(fileName.c_str());
	file >> m_simplemap;

}

CDocument::~CDocument()
{
}

void CDocument::setListOfMaps(TSetOfMetricMapInitializers &mapCfg)
{
	m_metricmap.setListOfMaps( &mapCfg );
	m_metricmap.loadFromProbabilisticPosesAndObservations(m_simplemap);
}

void CDocument::setConfig(const std::string &config)
{
	TSetOfMetricMapInitializers mapCfg;
	mapCfg.loadFromConfigFile( CConfigFile(config), METRIC_MAP_CONFIG_SECTION);

	m_metricmap.setListOfMaps( &mapCfg );
	m_metricmap.loadFromProbabilisticPosesAndObservations(m_simplemap);
}

const std::map<std::string, CSetOfObjects::Ptr> CDocument::renderizableMaps() const
{
	std::map<std::string, CSetOfObjects::Ptr> renderizable;

	{
		CSimplePointsMap::Ptr ptr = m_metricmap.getMapByClass<CSimplePointsMap>();
		if (ptr.get())
		{
			CSetOfObjects::Ptr obj = CSetOfObjects::Create();
			ptr->getAs3DObject(obj);
			renderizable.emplace("Points map", obj);
		}
	}
	{
		COccupancyGridMap2D::Ptr ptr = m_metricmap.getMapByClass<COccupancyGridMap2D>();
		if (ptr.get())
		{
			CSetOfObjects::Ptr obj = CSetOfObjects::Create();
			ptr->getAs3DObject(obj);
			renderizable.emplace("Occupancy grid", obj);

			CFileOutputStream f("/home/lisgein/tmp/test.ini", false);
			ptr->insertionOptions.dumpToTextStream(f);
			ptr->likelihoodOptions.dumpToTextStream(f);
		}
	}
	{
		CGasConcentrationGridMap2D::Ptr ptr = m_metricmap.getMapByClass<CGasConcentrationGridMap2D>();
		if (ptr.get())
		{
			CSetOfObjects::Ptr obj = CSetOfObjects::Create();
			ptr->getAs3DObject(obj);
			renderizable.emplace("Gas concentration grid", obj);
		}
	}
	{
		CBeaconMap::Ptr ptr = m_metricmap.getMapByClass<CBeaconMap>();
		if (ptr.get())
		{
			CSetOfObjects::Ptr obj = CSetOfObjects::Create();
			ptr->getAs3DObject(obj);
			renderizable.emplace("Beacon", obj);
		}
	}
	{
		CLandmarksMap::Ptr ptr = m_metricmap.getMapByClass<CLandmarksMap>();
		if (ptr.get())
		{
			CSetOfObjects::Ptr obj = CSetOfObjects::Create();
			ptr->getAs3DObject(obj);
			renderizable.emplace("Landmarks", obj);
		}
	}



	return renderizable;
}

const CSimpleMap &CDocument::simplemap() const
{
	return m_simplemap;
}
