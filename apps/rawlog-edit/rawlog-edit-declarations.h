/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef RAWLOG_EDIT_DECLS_H
#define RAWLOG_EDIT_DECLS_H

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservation.h>
#include "CRawlogProcessor.h"

// Declarations:
#define VERBOSE_COUT \
	if (verbose) std::cout << "[rawlog-edit] "

#define DECLARE_OP_FUNCTION(_NAME)                                        \
	void _NAME(                                                           \
		mrpt::io::CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline, \
		bool verbose)

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
bool getArgValue(
	TCLAP::CmdLine& cmdline, const std::string& arg_name, T& out_val);
bool isFlagSet(TCLAP::CmdLine& cmdline, const std::string& arg_name);

#endif
