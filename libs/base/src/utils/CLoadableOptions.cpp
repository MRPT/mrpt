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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/CConfigFile.h>

using namespace mrpt::utils;

CStdOutStream	loadable_opts_my_cout;
const int LOADABLEOPTS_COLUMN_WIDTH   = 41;  // Until the "=" in each row.

//MRPT_TODO("Reimplement with an internal registry of variables, etc..")

void  CLoadableOptions::loadFromConfigFileName(
	const std::string		&config_file,
	const std::string			&section)
{
	CConfigFile f(config_file);
	this->loadFromConfigFile(f,section);
}

void  CLoadableOptions::saveToConfigFileName(
	const std::string		&config_file,
	const std::string		&section)
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
