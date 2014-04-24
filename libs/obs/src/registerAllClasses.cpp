/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/obs.h>

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
	registerClass( CLASS_ID( CObservationWindSensor ) );
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

	registerClass( CLASS_ID( CObservationCANBusJ1939 ) );
	registerClass( CLASS_ID( CObservationRawDAQ ) );
}

