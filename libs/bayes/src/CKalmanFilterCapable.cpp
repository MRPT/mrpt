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

#include <mrpt/bayes.h>  // Precompiled headers

#include <mrpt/bayes/CKalmanFilterCapable.h>


using namespace mrpt::bayes;

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  TKF_options::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [TKF_options] ------------ \n\n");
	out.printf("method                                  = %s\n", mrpt::utils::TEnumType<TKFMethod>::value2name(method).c_str() );
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
	method = iniFile.read_enum<TKFMethod>(section,"method", method );
	MRPT_LOAD_CONFIG_VAR( verbose, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( IKF_iterations, int , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( enable_profiler, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( use_analytic_transition_jacobian, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( use_analytic_observation_jacobian, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( debug_verify_analytic_jacobians, bool    , iniFile, section  );
	MRPT_LOAD_CONFIG_VAR( debug_verify_analytic_jacobians_threshold, double, iniFile, section );
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
	debug_verify_analytic_jacobians		(false),
	debug_verify_analytic_jacobians_threshold	(1e-2)
{

}

