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

#include <mrpt/maps.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef _mrpt_maps_H
#	include <mrpt/maps.h>
#endif

#include <mrpt/utils/CStartUpClassesRegister.h>


using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::opengl;

void registerAllClasses_mrpt_maps();

CStartUpClassesRegister  mrpt_maps_class_reg(&registerAllClasses_mrpt_maps);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_maps
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_maps()
{
	registerClass( CLASS_ID( CBeacon ) );
	registerClass( CLASS_ID( CBeaconMap ) );

	registerClass( CLASS_ID( CPointsMap ) );
	registerClass( CLASS_ID( CSimplePointsMap ) );
	registerClass( CLASS_ID( CColouredPointsMap ) );
	registerClass( CLASS_ID( COccupancyGridMap2D ) );
	registerClass( CLASS_ID( CGasConcentrationGridMap2D ) );
	registerClass( CLASS_ID( CWirelessPowerGridMap2D ) );
	registerClass( CLASS_ID( CHeightGridMap2D ) );
	registerClass( CLASS_ID( CReflectivityGridMap2D ) );


	registerClass( CLASS_ID( CAngularObservationMesh ) );
	registerClass( CLASS_ID( CPlanarLaserScan ) ) ;
}

