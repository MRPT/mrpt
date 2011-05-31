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

#include <mrpt/obs.h>   // Precompiled headers



#include <mrpt/slam/CObservation.h>
#include <mrpt/system/os.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CObservation, CSerializable, mrpt::slam)


extern CStartUpClassesRegister  mrpt_obs_class_reg;

const volatile int dumm = mrpt_obs_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

/*---------------------------------------------------------------
					CONSTRUCTOR
  ---------------------------------------------------------------*/
CObservation::CObservation() :
	timestamp( mrpt::system::now() ),
	sensorLabel()
{
}


void CObservation::getSensorPose( mrpt::math::TPose3D &out_sensorPose ) const
{
	CPose3D  p;
	getSensorPose(p);
	out_sensorPose = TPose3D(p);
}

void CObservation::setSensorPose( const mrpt::math::TPose3D &newSensorPose )
{
	setSensorPose(CPose3D(newSensorPose));
}


void CObservation::swap(CObservation &o)
{
	std::swap(timestamp, o.timestamp);
	std::swap(sensorLabel, o.sensorLabel);
}
