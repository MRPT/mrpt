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

#include <mrpt/apps_cli/rawlog_ops.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>
#include <type_traits>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// Operations that need the whole rawlog in memory: load it, transform it
// with mrpt::apps::rawlog_ops, and save the result to the output file.
namespace
{
CRawlog loadWholeRawlog(CCompressedInputStream& in, bool verbose)
{
  CRawlog rawlog;
  auto arch = mrpt::serialization::archiveFrom(in);
  for (;;)
  {
    try
    {
      rawlog.insert(arch.ReadObject());
    }
    catch (const mrpt::serialization::CExceptionEOF&)
    {
      break;
    }
  }
  VERBOSE_COUT << "Loaded " << rawlog.size() << " entries.\n";
  return rawlog;
}

void saveWholeRawlog(const CRawlog& rawlog, bool verbose)
{
  TOutputRawlogCreator out;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    *out.out_rawlog << *rawlog.getAsGeneric(i);
  }
  VERBOSE_COUT << "Saved " << rawlog.size() << " entries to: " << out.out_rawlog_filename << "\n";
}

template <typename T>
T requiredValue(CLI::App& cmdline, const std::string& name)
{
  std::string s;
  if (!getArgValue<std::string>(cmdline, name, s) || s.empty())
  {
    THROW_EXCEPTION_FMT("Missing value for --%s", name.c_str());
  }
  // The whole text must be a number, and not negative for unsigned types:
  std::istringstream ss(s);
  T v{};
  const bool negativeUnsigned = std::is_unsigned_v<T> && s.find('-') != std::string::npos;
  if (negativeUnsigned || !(ss >> v) || !(ss >> std::ws).eof())
  {
    THROW_EXCEPTION_FMT("Invalid value for --%s: '%s'", name.c_str(), s.c_str());
  }
  return v;
}
}  // namespace

DECLARE_OP_FUNCTION(op_decimate)
{
  const auto ratio = requiredValue<size_t>(cmdline, "decimate");
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  const auto kept = rawlog_ops::decimate(rawlog, ratio);
  VERBOSE_COUT << "Sensory frames kept: " << kept << "\n";
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_lossless_decimate)
{
  const auto ratio = requiredValue<size_t>(cmdline, "lossless-decimate");
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  const auto kept = rawlog_ops::losslessDecimate(rawlog, ratio);
  VERBOSE_COUT << "Sensory frames in the output: " << kept << "\n";
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_compact)
{
  const bool onePerLabel = isFlagSet(cmdline, "one-per-label");
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  const auto st = rawlog_ops::compact(rawlog, onePerLabel);
  VERBOSE_COUT << "Actions removed: " << st.actionsRemoved
               << ", sensory frames removed: " << st.sensoryFramesRemoved << "\n";
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_to_observations_only)
{
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  rawlog_ops::toObservationsOnly(rawlog);
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_to_sensory_frames)
{
  const auto maxLen = requiredValue<double>(cmdline, "to-sensory-frames");
  const bool onePerLabel = isFlagSet(cmdline, "one-per-label");
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  rawlog_ops::toSensoryFrames(rawlog, maxLen, onePerLabel);
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_sort_by_timestamp)
{
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  rawlog_ops::sortByTimestamp(rawlog, !isFlagSet(cmdline, "use-pc-timestamp"));
  saveWholeRawlog(rawlog, verbose);
}

DECLARE_OP_FUNCTION(op_regenerate_sf_timestamps)
{
  auto rawlog = loadWholeRawlog(in_rawlog, verbose);
  const auto n = rawlog_ops::regenerateTimestampsInSensoryFrames(rawlog);
  VERBOSE_COUT << "Timestamps regenerated: " << n << "\n";
  saveWholeRawlog(rawlog, verbose);
}
