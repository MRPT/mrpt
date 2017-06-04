#include "CDocument.h"

#include "mrpt/utils/CFileGZInputStream.h"
#include "mrpt/utils/CConfigFile.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::utils;

CDocument::CDocument(const std::string &fileName, const std::string &config)
	: m_simplemap(CSimpleMap())
	, m_metricmap(CMultiMetricMap())
{
	CFileGZInputStream file(fileName.c_str());
	file >> m_simplemap;

	std::string METRIC_MAP_CONFIG_SECTION  =  "MappingApplication";
	TSetOfMetricMapInitializers mapCfg;
	mapCfg.loadFromConfigFile( CConfigFile(config), METRIC_MAP_CONFIG_SECTION);

	m_metricmap.setListOfMaps( &mapCfg );
	m_metricmap.loadFromProbabilisticPosesAndObservations(m_simplemap);
}

CDocument::~CDocument()
{

}

const std::map<std::string, CRenderizable::Ptr> CDocument::renderizableMaps() const
{
	std::map<std::string, CRenderizable::Ptr> renderizable;

	CSimplePointsMap::Ptr prt = m_metricmap.getMapByClass<CSimplePointsMap>();
	if (prt.get())
	{
		CSetOfObjects::Ptr obj = CSetOfObjects::Create();
		prt->getAs3DObject(obj);
		renderizable.emplace("Points map", obj);
	}

	return renderizable;
}
