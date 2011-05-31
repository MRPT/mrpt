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


