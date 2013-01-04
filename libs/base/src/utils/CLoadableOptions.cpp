/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>

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
void CLoadableOptions::dumpToTextStream(CStream &out) const
{
	CConfigFileMemory cfg;
	this->saveToConfigFile(cfg,"");
	out.printf("%s", cfg.getContent().c_str() );
}
