/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

#include <cstdio>
#include <string>
#include <vector>

namespace mrpt::hwdrivers::testing
{
inline constexpr char STX = 0x02;
inline constexpr char ETX = 0x03;

/** A "sRA LMDscandata" telegram with the given ranges (in millimeters), status
 * word and data channel name. Tokens 1..26 follow the layout the driver
 * expects, then one hexadecimal number per range. */
inline std::string scanTelegram(
    const std::vector<unsigned>& rangesMm,
    const std::string& status = "0",
    const std::string& channel = "DIST1")
{
  std::string s;
  s += STX;
  s += "sRA LMDscandata";
  for (int tok = 3; tok <= 25; tok++)
  {
    s += ' ';
    if (tok == 6)
    {
      s += status;
    }
    else if (tok == 21)
    {
      s += channel;
    }
    else
    {
      s += "0";
    }
  }
  char buf[16];
  std::snprintf(buf, sizeof(buf), " %X", static_cast<unsigned>(rangesMm.size()));
  s += buf;
  for (const auto r : rangesMm)
  {
    std::snprintf(buf, sizeof(buf), " %X", r);
    s += buf;
  }
  s += ETX;
  return s;
}

/** The drivers give the device only a few tens of milliseconds to answer each
 * scan request, and a busy machine may miss one: real users call the driver
 * repeatedly, so do the same, up to a limit. */
template <class SENSOR>
void pollScan(
    SENSOR& sensor, bool& thereIsObs, mrpt::obs::CObservation2DRangeScan& obs, bool& hwError)
{
  for (int i = 0; i < 25 && !thereIsObs; i++)
  {
    sensor.doProcessSimple(thereIsObs, obs, hwError);
  }
}

/** Like pollScan(), through the generic sensor interface. */
template <class SENSOR>
mrpt::hwdrivers::CGenericSensor::TListObservations pollObservations(SENSOR& sensor)
{
  for (int i = 0; i < 25; i++)
  {
    sensor.doProcess();
    auto lst = sensor.getObservations();
    if (!lst.empty())
    {
      return lst;
    }
  }
  return {};
}

}  // namespace mrpt::hwdrivers::testing
