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

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_slam_H
#	include <mrpt/slam.h>
#endif

#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::opengl;


void registerAllClasses_mrpt_core();

CStartUpClassesRegister  mrpt_slam_class_reg(&registerAllClasses_mrpt_core);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_core
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_core()
{
//   Hack to enable compatibility with an older name of this class:
	registerClass( CLASS_ID( CMultiMetricMap ) );
	registerClassCustomName( "CHybridMetricMap", CLASS_ID( CMultiMetricMap ) );

	registerClass( CLASS_ID( CIncrementalMapPartitioner ) );
	registerClass( CLASS_ID( CMultiMetricMapPDF ) );


}

