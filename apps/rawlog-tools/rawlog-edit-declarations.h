/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef RAWLOG_EDIT_DECLS_H
#define RAWLOG_EDIT_DECLS_H

#include <mrpt/obs.h>
#include "CRawlogProcessor.h"

// Declarations:
#define VERBOSE_COUT	if (verbose) std::cout << "[rawlog-edit] "

#define DECLARE_OP_FUNCTION(_NAME) void _NAME(mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose)


/** Auxiliary struct that performs all the checks and create the
     output rawlog stream, publishing it as "out_rawlog"
*/
struct TOutputRawlogCreator
{
	mrpt::utils::CFileGZOutputStream out_rawlog;
	std::string out_rawlog_filename;

	TOutputRawlogCreator();
};

// ======================================================================
//  Search for a specific command-line argument.
// Return false if not not set, an exception if args doesn't exist
// ======================================================================
template <typename T>
bool getArgValue(TCLAP::CmdLine &cmdline, const std::string &arg_name, T &out_val);


#endif


