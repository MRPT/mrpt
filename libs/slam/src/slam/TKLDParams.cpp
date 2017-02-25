/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers 

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CConfigFileBase.h>
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
void  TKLDParams::dumpToTextStream(mrpt::utils::CStream	&out) const
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


