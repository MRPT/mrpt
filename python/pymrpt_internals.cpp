/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/CSensoryFrame.h>

bool mrpt::pymrpt_internal::insertObs(
	const mrpt::obs::CSensoryFrame& sf,
    mrpt::maps::CMetricMap* map,
	const mrpt::poses::CPose3D* robotPose)
{
    ASSERT_(map);
    bool any=false;
    for (const auto &obs : sf)
    {
        if (!obs) continue;
        bool done = map->insertObs(*obs,robotPose);
        any = any || done;
    }
    return any;
}
