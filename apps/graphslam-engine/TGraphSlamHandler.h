/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TGRAPHSLAMHANDLER_H
#define TGRAPHSLAMHANDLER_H

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/COutputLogger.h>

#include <string>
#include <sstream>
#include <iostream>

// Class to manage variables and methods general to the graphslam-engine application
class TGraphSlamHandler : mrpt::utils::COutputLogger
{
public:
	TGraphSlamHandler();
	~TGraphSlamHandler();

	void readConfigFname(const std::string& fname);
	void printParams() const;
	void getParamsAsString(std::string* str) const;
	std::string getParamsAsString() const;
	void setOutputLoggerPtr(mrpt::utils::COutputLogger* logger);

	bool save_graph;
	bool save_3DScene;

	std::string save_graph_fname;
	std::string save_3DScene_fname;

	mrpt::utils::COutputLogger* logger;
};


#endif /* end of include guard: TGRAPHSLAMHANDLER_H */
