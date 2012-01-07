/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#ifndef _mrpt_maps_H
#define _mrpt_maps_H

#include <mrpt/config.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_maps_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/slam/CBeacon.h>
#include <mrpt/slam/CBeaconMap.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CWirelessPowerGridMap2D.h>
#include <mrpt/slam/CHeightGridMap2D.h>
#include <mrpt/slam/CReflectivityGridMap2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CWeightedPointsMap.h>
#include <mrpt/slam/CPointsMap.h>

//#include <mrpt/slam/PCL_adapters.h>  // NOTE: This file must be included from the user 
                                       // code only if he has already #include'd PCL headers.

#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

#endif // end precomp.headers

#endif
