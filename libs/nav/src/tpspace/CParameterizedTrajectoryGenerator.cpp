/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/utils/CStream.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

using namespace mrpt::nav;

std::string CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX = "./reactivenav.logs";

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CParameterizedTrajectoryGenerator, CSerializable, mrpt::nav)


CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator() :
	refDistance(.0),
	m_alphaValuesCount(0),
	m_score_priority(1.0)
{ }

void CParameterizedTrajectoryGenerator::setParamsCommon(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection,  const std::string &sKeyPrefix)
{
	this->refDistance        = cfg.read_double  (sSection, sKeyPrefix+std::string("refDistance"), .0, true );
	this->m_alphaValuesCount = cfg.read_uint64_t(sSection, sKeyPrefix+std::string("num_paths"), 0, true );
	this->m_score_priority   = cfg.read_double  (sSection, sKeyPrefix+std::string("score_priority"), m_score_priority );
}

void CParameterizedTrajectoryGenerator::internal_readFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;
	switch (version)
	{
	case 0:
		in >> refDistance >> m_alphaValuesCount >> m_score_priority;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CParameterizedTrajectoryGenerator::internal_writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 0;
	out << version;

	out << refDistance << m_alphaValuesCount << m_score_priority;
}

uint16_t CParameterizedTrajectoryGenerator::alpha2index( double alpha ) const 
{
	mrpt::math::wrapToPi(alpha);
	int k = mrpt::utils::round(0.5*(m_alphaValuesCount*(1.0+alpha/M_PI) - 1.0));
	if (k<0) k=0;
	if (k>=m_alphaValuesCount) k=m_alphaValuesCount-1;
	return (uint16_t)k;
}
