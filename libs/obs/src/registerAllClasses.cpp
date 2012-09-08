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

#include <mrpt/obs.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_obs_H
#	include <mrpt/obs.h>
#endif


#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CStartUpClassesRegister.h>


using namespace mrpt::slam;
using namespace mrpt::utils;


void registerAllClasses_mrpt_obs();

CStartUpClassesRegister  mrpt_obs_class_reg(&registerAllClasses_mrpt_obs);


/*---------------------------------------------------------------
					registerAllClasses_mrpt_obs
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_obs()
{
	registerClass( CLASS_ID( CSensoryFrame ) );
	registerClassCustomName( "CSensorialFrame", CLASS_ID( CSensoryFrame ) );

	registerClass( CLASS_ID( CObservation ) );
	registerClass( CLASS_ID( CObservation2DRangeScan ) );
	registerClass( CLASS_ID( CObservation3DRangeScan ) );
	registerClass( CLASS_ID( CObservationBatteryState ) );
	registerClass( CLASS_ID( CObservationWirelessPower ) );
	registerClass( CLASS_ID( CObservationRFID ) );
	registerClass( CLASS_ID( CObservationBeaconRanges ) );
	registerClass( CLASS_ID( CObservationBearingRange ) );
	registerClass( CLASS_ID( CObservationComment ) );
	registerClass( CLASS_ID( CObservationGasSensors ) );
	registerClass( CLASS_ID( CObservationGPS ) );
	registerClass( CLASS_ID( CObservationImage ) );
	registerClass( CLASS_ID( CObservationIMU ) );
	registerClass( CLASS_ID( CObservationOdometry ) );
	registerClass( CLASS_ID( CObservationRange ) );
	registerClass( CLASS_ID( CObservationReflectivity ) );
	registerClass( CLASS_ID( CObservationStereoImages ) );
	registerClass( CLASS_ID( CObservationStereoImagesFeatures ) );
	//registerClass( CLASS_ID( CObservationVisualLandmarks ) );

	registerClass( CLASS_ID( CSimpleMap ) );
	registerClassCustomName( "CSensFrameProbSequence", CLASS_ID( CSimpleMap ) );

	registerClass( CLASS_ID( CMetricMap ) );
	registerClass( CLASS_ID( CRawlog ) );

	registerClass( CLASS_ID( CAction ) );
	registerClass( CLASS_ID( CActionCollection ) );
	registerClass( CLASS_ID( CActionRobotMovement2D ) );
	registerClass( CLASS_ID( CActionRobotMovement3D ) );
}

