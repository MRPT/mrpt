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

