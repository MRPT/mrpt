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

#include <mrpt/apps-cli/CRawlogProcessor.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/system/filesystem.h>

// Declarations:
#define VERBOSE_COUT \
  if (verbose) std::cout << "[rawlog-edit] "

#define DECLARE_OP_FUNCTION(_NAME) \
  void _NAME(mrpt::io::CCompressedInputStream& in_rawlog, TCLAP::CmdLine& cmdline, bool verbose)

/** Auxiliary struct that performs all the checks and create the
   output rawlog stream, publishing it as "out_rawlog"
*/
struct TOutputRawlogCreator
{
  mrpt::io::CFileGZOutputStream out_rawlog_io;
  std::unique_ptr<mrpt::serialization::CArchive> out_rawlog;
  std::string out_rawlog_filename;

  TOutputRawlogCreator();
};

// ======================================================================
//  Search for a specific command-line argument.
// Return false if not not set, an exception if args doesn't exist
// ======================================================================
template <typename T>
bool getArgValue(TCLAP::CmdLine& cmdline, const std::string& arg_name, T& out_val);
bool isFlagSet(TCLAP::CmdLine& cmdline, const std::string& arg_name);
