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

void CParameterizedTrajectoryGenerator::setParamsCommon(const mrpt::utils::TParameters<double> &params)
{
	refDistance        = params["ref_distance"];
	m_alphaValuesCount = params["num_paths"];
}

void CParameterizedTrajectoryGenerator::internal_readFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;
	switch (version)
	{
	case 0:
		in >> refDistance >> m_alphaValuesCount;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CParameterizedTrajectoryGenerator::internal_writeToStream(mrpt::utils::CStream &out) const
{
	const uint8_t version = 0;
	out << version;

	out << refDistance << m_alphaValuesCount;
}

