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

#include <mrpt/hmtslam.h>
#include <mrpt/utils.h>


using namespace mrpt::utils;
using namespace mrpt::hmtslam;


void registerAllClasses_mrpt_hmtslam();

CStartUpClassesRegister  mrpt_hmtslam_class_reg(&registerAllClasses_mrpt_hmtslam);


/*---------------------------------------------------------------
					registerAllClasses_mrpt_hmtslam
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_hmtslam()
{
	registerClass( CLASS_ID(CHMTSLAM) );
	registerClass( CLASS_ID(CLSLAMParticleData) );
	registerClass( CLASS_ID(CHierarchicalMHMap) );
	registerClass( CLASS_ID(CHMHMapArc) );
	registerClass( CLASS_ID(CHMHMapNode) );
	registerClass( CLASS_ID(CRobotPosesGraph ) );
	registerClass( CLASS_ID(THypothesisIDSet) );
	registerClass( CLASS_ID(CLocalMetricHypothesis) );
}


