/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers

#include <mrpt/apps/DataSourceRawlog.h>
#include <mrpt/obs/CRawlog.h>

using namespace mrpt::apps;

bool DataSourceRawlog::impl_get_next_observations(
	mrpt::obs::CActionCollection::Ptr& action,
	mrpt::obs::CSensoryFrame::Ptr& observations,
	mrpt::obs::CObservation::Ptr& observation)
{
	MRPT_START

	// 1st time? Open rawlog:
	if (!m_rawlog_arch)
	{
		std::string err_msg;
		if (!m_rawlog_io.open(m_rawlogFileName, err_msg))
		{
			THROW_EXCEPTION_FMT(
				"Error opening rawlog file: `%s`", err_msg.c_str());
		}
		m_rawlog_arch = mrpt::serialization::archiveUniquePtrFrom(m_rawlog_io);

		MRPT_LOG_INFO_FMT("RAWLOG file: `%s`", m_rawlogFileName.c_str());
	}

	// Read:

	for (;;)
	{
		if (!mrpt::obs::CRawlog::getActionObservationPairOrObservation(
				*m_rawlog_arch, action, observations, observation,
				m_rawlogEntry))
			return false;

		// Optional skip of first N entries
		if (m_rawlogEntry < m_rawlog_offset) continue;

		MRPT_LOG_DEBUG_STREAM("Processing rawlog entry #" << m_rawlogEntry);

		// Ok, accept this new observations:
		return true;
	};

	return false;

	MRPT_END
}
