#pragma once
#include<string>

#include "mrpt/maps/CSimpleMap.h"
#include "mrpt/maps/CMultiMetricMap.h"
#include "mrpt/opengl/CRenderizable.h"


class CDocument
{
public:
	CDocument(const std::string& fileName, const std::string& config);
	~CDocument();

	const std::map<std::string, mrpt::opengl::CRenderizable::Ptr> renderizableMaps() const;

private:
	mrpt::maps::CSimpleMap m_simplemap;
	mrpt::maps::CMultiMetricMap m_metricmap;
};
