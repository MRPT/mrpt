/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservation.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/poses/CPose3D.h>
#include <iomanip>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CObservation, CSerializable, mrpt::obs)


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

void CObservation::getDescriptionAsText(std::ostream &o) const
{
	o << "Timestamp (UTC): " << mrpt::system::dateTimeToString(timestamp) << std::endl;
	o << "  (as time_t): " <<  std::fixed << std::setprecision(5) << mrpt::system::timestampTotime_t(timestamp) << std::endl;
	o << "  (as TTimestamp): " << timestamp << std::endl;
	o << "Sensor label: '" << sensorLabel << "'" << std::endl;
	o << std::endl;
}
