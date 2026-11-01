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

#include <mrpt/core/exceptions.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h>

#include <cmath>
#include <iostream>
#include <utility>

using namespace mrpt::hwdrivers;

namespace
{
grabber_dc1394_framerate_t toDc1394Framerate(double hz)
{
  constexpr std::pair<double, grabber_dc1394_framerate_t> RATES[] = {
      {1.875, FRAMERATE_1_875},
      { 3.75,  FRAMERATE_3_75},
      {  7.5,   FRAMERATE_7_5},
      {   15,    FRAMERATE_15},
      {   30,    FRAMERATE_30},
      {   60,    FRAMERATE_60},
      {  120,   FRAMERATE_120},
      {  240,   FRAMERATE_240}
  };
  for (const auto& [rate, value] : RATES)
  {
    if (std::abs(rate - hz) < 1e-3)
    {
      return value;
    }
  }
  THROW_EXCEPTION_FMT(
      "Unsupported frame rate: %f Hz (valid: 1.875, 3.75, 7.5, 15, 30, 60, 120, 240)", hz);
}
}  // namespace

CStereoGrabber_Bumblebee_libdc1394::CStereoGrabber_Bumblebee_libdc1394(
    uint64_t cameraGUID, uint16_t cameraUnit, double frameRate, bool bayerDecoding)
{
  MRPT_START

  TCaptureOptions_dc1394 opt1394;
  opt1394.mode7 = 3;                  // stereo cameras are captured with MODE7-3
  opt1394.deinterlace_stereo = true;  // It is stereo.
  opt1394.bayer_decoding = bayerDecoding;
  opt1394.framerate = toDc1394Framerate(frameRate);

  m_firewire_capture = std::make_unique<CImageGrabber_dc1394>(cameraGUID, cameraUnit, opt1394);

  if (!m_firewire_capture->isOpen())
  {
    std::cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open\n";
  }

  MRPT_END
}

CStereoGrabber_Bumblebee_libdc1394::~CStereoGrabber_Bumblebee_libdc1394() = default;

bool CStereoGrabber_Bumblebee_libdc1394::getStereoObservation(
    mrpt::obs::CObservationStereoImages& out_observation)
{
  if (!isOpen())
  {
    std::cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open\n";
    return false;
  }

  auto grabbed = m_firewire_capture->grabStereoFrame();
  if (!grabbed)
  {
    return false;
  }
  out_observation = std::move(*grabbed);
  return true;
}
