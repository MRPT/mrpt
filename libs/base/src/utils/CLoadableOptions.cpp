/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>

using namespace mrpt::utils;

CStdOutStream	loadable_opts_my_cout;
const int LOADABLEOPTS_COLUMN_WIDTH   = 41;  // Until the "=" in each row.

void  CLoadableOptions::loadFromConfigFileName(
	const std::string		&config_file,
	const std::string			&section)
{
	CConfigFile f(config_file);
	this->loadFromConfigFile(f,section);
}

void  CLoadableOptions::saveToConfigFile(
	mrpt::utils::CConfigFileBase &target,
	const std::string            &section) const
{
	MRPT_UNUSED_PARAM(target); MRPT_UNUSED_PARAM(section);
	throw std::logic_error("The child class does not implement this method.");
}

void  CLoadableOptions::saveToConfigFileName(
	const std::string		&config_file,
	const std::string		&section) const
{
	CConfigFile f(config_file);
	this->saveToConfigFile(f,section);
}

void  CLoadableOptions::dumpToConsole() const
{
	dumpToTextStream( loadable_opts_my_cout );
}

void CLoadableOptions::dumpVar_int( CStream &out, const char *varName, int v )
{
	out.printf("%-*s= %i\n",LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_float( CStream &out, const char *varName, float v )
{
	out.printf("%-*s= %f\n",LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_double( CStream &out, const char *varName, double v )
{
	out.printf("%-*s= %f\n",LOADABLEOPTS_COLUMN_WIDTH, varName, v);
}

void CLoadableOptions::dumpVar_bool( CStream &out, const char *varName, bool v )
{
	out.printf("%-*s= %s\n",LOADABLEOPTS_COLUMN_WIDTH, varName, v ? "YES":"NO" );
}

void CLoadableOptions::dumpVar_string( CStream &out, const char *varName, const std::string &v )
{
	out.printf("%-*s= %s\n",LOADABLEOPTS_COLUMN_WIDTH, varName, v.c_str() );
}

/** This method should clearly display all the contents of the structure in textual form, sending it to a CStream.
  * The default implementation in this base class relies on \a saveToConfigFile() to generate a plain text representation of all the parameters.
  */
void CLoadableOptions::dumpToTextStream(mrpt::utils::CStream &out) const
{
	CConfigFileMemory cfg;
	this->saveToConfigFile(cfg,"");
	out.printf("%s", cfg.getContent().c_str() );
}
