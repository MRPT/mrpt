/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CArchive.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/slam/TKLDParams.h>

using namespace mrpt::slam;

/*---------------------------------------------------------------
					TKLDParams
 ---------------------------------------------------------------*/
TKLDParams::TKLDParams() : KLD_binSize_PHI(DEG2RAD(5)) {}
/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void TKLDParams::dumpToTextStream(std::ostream& out) const
{
	out << mrpt::format("\n----------- [TKLDParams] ------------ \n\n");

	out << mrpt::format(
		"KLD_minSampleSize                       = %i\n", KLD_minSampleSize);
	out << mrpt::format(
		"KLD_maxSampleSize                       = %i\n", KLD_maxSampleSize);
	out << mrpt::format(
		"KLD_binSize_XY                          = %f m\n", KLD_binSize_XY);
	out << mrpt::format(
		"KLD_binSize_PHI                         = %f deg\n",
		RAD2DEG(KLD_binSize_PHI));
	out << mrpt::format(
		"KLD_delta                               = %f\n", KLD_delta);
	out << mrpt::format(
		"KLD_epsilon                             = %f\n", KLD_epsilon);
	out << mrpt::format("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void TKLDParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	MRPT_LOAD_CONFIG_VAR(KLD_minSampleSize, int, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(KLD_maxSampleSize, int, iniFile, section);

	MRPT_LOAD_CONFIG_VAR(KLD_binSize_XY, double, iniFile, section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(KLD_binSize_PHI, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(KLD_delta, double, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(KLD_epsilon, double, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(KLD_minSamplesPerBin, double, iniFile, section);
}
