/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/bayes/CKalmanFilterCapable.h>


using namespace mrpt::bayes;

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  TKF_options::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [TKF_options] ------------ \n\n");
	out.printf("method                                  = ");
	switch (method)
	{
	case kfEKFNaive:
		out.printf("EKF\n");
		break;
	case kfEKFAlaDavison:
		out.printf("EKF a la Davison\n");
		break;
	case kfIKFFull:
		out.printf("Full IKF\n");
		break;
	case kfIKF:
		out.printf("IKF observation-by-observation\n");
		break;
	default:
		out.printf("NON VALID VALUE!\n");
		break;
	}

	out.printf("verbose                                 = %c\n", verbose ? 'Y':'N');
	out.printf("IKF_iterations                          = %i\n", IKF_iterations);
	out.printf("enable_profiler                         = %c\n", enable_profiler ? 'Y':'N');

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  TKF_options::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(method,int, TKFMethod, iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( verbose, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( IKF_iterations, int , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( enable_profiler, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( use_analytic_transition_jacobian, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( use_analytic_observation_jacobian, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( debug_verify_analytic_jacobians, bool    , iniFile, section  );
}

/*---------------------------------------------------------------
			TKF_options
  ---------------------------------------------------------------*/
TKF_options::TKF_options() :
	method      	( kfEKFNaive),
	verbose     	( false ),
	IKF_iterations 	( 5 ),
	enable_profiler	(false),
	use_analytic_transition_jacobian	(true),
	use_analytic_observation_jacobian	(true),
	debug_verify_analytic_jacobians		(false)
{

}

