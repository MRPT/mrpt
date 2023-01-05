/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::apps
{
/** Implementation of BaseAppDataSource for reading from a rawlog file
 *
 * \ingroup mrpt_apps_grp
 */
class DataSourceRawlog : virtual public BaseAppDataSource,
						 virtual public mrpt::system::COutputLogger
{
   public:
	DataSourceRawlog() = default;
	virtual ~DataSourceRawlog() override = default;

   protected:
	bool impl_get_next_observations(
		mrpt::obs::CActionCollection::Ptr& action,
		mrpt::obs::CSensoryFrame::Ptr& observations,
		mrpt::obs::CObservation::Ptr& observation) override;

	std::string m_rawlogFileName = "UNDEFINED.rawlog";
	std::size_t m_rawlog_offset = 0;
	std::size_t m_rawlogEntry = 0;
	mrpt::io::CFileGZInputStream m_rawlog_io;
	mrpt::serialization::CArchive::UniquePtr m_rawlog_arch;
};

}  // namespace mrpt::apps
