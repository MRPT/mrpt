/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "config-precomp.h"  // Precompiled headers

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <iostream>
#include <mrpt/core/format.h>

using namespace mrpt::config;

const int LOADABLEOPTS_COLUMN_WIDTH = 41;  // Until the "=" in each row.

void CLoadableOptions::loadFromConfigFileName(
	const std::string& config_file, const std::string& section)
{
	CConfigFile f(config_file);
	this->loadFromConfigFile(f, section);
}

void CLoadableOptions::saveToConfigFile(
	CConfigFileBase& target, const std::string& section) const
{
	MRPT_UNUSED_PARAM(target);
	MRPT_UNUSED_PARAM(section);
	throw std::logic_error("The child class does not implement this method.");
}

void CLoadableOptions::saveToConfigFileName(
	const std::string& config_file, const std::string& section) const
{
	CConfigFile f(config_file);
	this->saveToConfigFile(f, section);
}

void CLoadableOptions::dumpToConsole() const { dumpToTextStream(std::cout); }
void CLoadableOptions::dumpVar_int(
	std::ostream& out, const char* varName, int v)
{
	out << mrpt::format("%-*s= %i\n", LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_float(
	std::ostream& out, const char* varName, float v)
{
	out << mrpt::format("%-*s= %f\n", LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_double(
	std::ostream& out, const char* varName, double v)
{
	out << mrpt::format("%-*s= %f\n", LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_bool(
	std::ostream& out, const char* varName, bool v)
{
	out << mrpt::format(
		"%-*s= %s\n", LOADABLEOPTS_COLUMN_WIDTH, varName, v ? "YES" : "NO");
}

void CLoadableOptions::dumpVar_string(
	std::ostream& out, const char* varName, const std::string& v)
{
	out << mrpt::format(
		"%-*s= %s\n", LOADABLEOPTS_COLUMN_WIDTH, varName, v.c_str());
}

void CLoadableOptions::dumpToTextStream(std::ostream& out) const
{
	CConfigFileMemory cfg;
	this->saveToConfigFile(cfg, "");
	out << cfg.getContent();
}
