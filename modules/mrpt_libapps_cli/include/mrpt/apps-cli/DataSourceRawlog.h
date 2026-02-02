/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/apps-cli/BaseAppDataSource.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::apps
{
/** Implementation of BaseAppDataSource for reading from a rawlog file
 *
 * \ingroup mrpt_apps_grp
 */
class DataSourceRawlog :
    virtual public BaseAppDataSource,
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
  mrpt::io::CCompressedInputStream m_rawlog_io;
  mrpt::serialization::CArchive::UniquePtr m_rawlog_arch;
};

}  // namespace mrpt::apps
