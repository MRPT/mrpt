/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
