/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

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
