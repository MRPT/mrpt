/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#include <mrpt/slam.h>  // Precompiled header 



#include <mrpt/slam/TKLDParams.h>

using namespace mrpt::slam; 
using namespace mrpt::utils; 

/*---------------------------------------------------------------
					TKLDParams
 ---------------------------------------------------------------*/
TKLDParams::TKLDParams() :
	KLD_binSize_XY			( 0.2f ),
	KLD_binSize_PHI			( DEG2RAD(5) ),
	KLD_delta				( 0.01f ),
	KLD_epsilon				( 0.02f ),
	KLD_minSampleSize		( 250 ),
	KLD_maxSampleSize		( 100000 ),
	KLD_minSamplesPerBin	( 0 )
{
}


/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  TKLDParams::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [TKLDParams] ------------ \n\n");

	out.printf("KLD_minSampleSize                       = %i\n", KLD_minSampleSize );
	out.printf("KLD_maxSampleSize                       = %i\n", KLD_maxSampleSize );
	out.printf("KLD_binSize_XY                          = %f m\n", KLD_binSize_XY );
	out.printf("KLD_binSize_PHI                         = %f deg\n", RAD2DEG(KLD_binSize_PHI) );
	out.printf("KLD_delta                               = %f\n", KLD_delta);
	out.printf("KLD_epsilon                             = %f\n", KLD_epsilon);
	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  TKLDParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR( KLD_minSampleSize, int,   iniFile,section );
	MRPT_LOAD_CONFIG_VAR( KLD_maxSampleSize, int,   iniFile,section );

	MRPT_LOAD_CONFIG_VAR( KLD_binSize_XY, double,   iniFile,section );
	MRPT_LOAD_CONFIG_VAR_DEGREES( KLD_binSize_PHI,  iniFile,section );
	MRPT_LOAD_CONFIG_VAR( KLD_delta, double,   iniFile,section );
	MRPT_LOAD_CONFIG_VAR( KLD_epsilon, double,   iniFile,section );
	MRPT_LOAD_CONFIG_VAR( KLD_minSamplesPerBin, double,   iniFile,section );
	
}


