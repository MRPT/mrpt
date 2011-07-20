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
#ifndef mrpt_obs_H
#define mrpt_obs_H

#include <mrpt/config.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_obs_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS  || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

// Observations:
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CObservationImage.h>
// #include <mrpt/slam/CObservationVisualLandmarks.h>  // This one is in mrpt-core
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/slam/CObservationStereoImagesFeatures.h>
#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationBatteryState.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationBearingRange.h>
#include <mrpt/slam/CObservationComment.h>
#include <mrpt/slam/CObservationReflectivity.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/slam/CSensoryFrame.h>


// Observations:
#include <mrpt/slam/CAction.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>


// Others:
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/carmen_log_tools.h>

// Very basic classes for maps:
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CSimpleMap.h>


#endif // end precomp.headers


#endif
