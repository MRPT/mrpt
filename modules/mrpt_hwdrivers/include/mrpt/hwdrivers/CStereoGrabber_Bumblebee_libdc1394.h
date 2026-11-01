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

#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/config.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include <memory>

namespace mrpt::hwdrivers
{
/** Grabs from a "Bumblebee" or "Bumblebee2" stereo camera using raw access to
 * the libdc1394 library.
 *
 * Only raw, unrectified images can be captured with this class, which can be
 * rectified offline given the camera calibration. Images are either RGB
 * (demosaiced here) or the raw 8-bit Bayer mosaics sent by the camera
 * (lossless; see bayerPattern()).
 *
 * Once connected to a camera, you can call `getStereoObservation()` to
 * retrieve the stereo images.
 *
 * \sa You'll probably want to use instead the most generic camera grabber in
 * MRPT: mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CStereoGrabber_Bumblebee_libdc1394
{
 public:
  /** Constructor. Parameters have the same meaning as in
   * CImageGrabber_dc1394::CImageGrabber_dc1394().
   * \param frameRate One of the rates of mrpt::hwdrivers::grabber_dc1394_framerate_t
   *        (1.875, 3.75, 7.5, 15, 30, 60, 120 or 240 Hz).
   * \param bayerDecoding If false, images are the raw Bayer mosaics.
   * \exception std::exception On an unsupported frame rate.
   */
  CStereoGrabber_Bumblebee_libdc1394(
      uint64_t cameraGUID, uint16_t cameraUnit, double frameRate, bool bayerDecoding = true);

  CStereoGrabber_Bumblebee_libdc1394(const CStereoGrabber_Bumblebee_libdc1394&) = delete;
  CStereoGrabber_Bumblebee_libdc1394& operator=(const CStereoGrabber_Bumblebee_libdc1394&) = delete;

  virtual ~CStereoGrabber_Bumblebee_libdc1394();

  /** Whether the camera was opened successfully. */
  [[nodiscard]] bool isOpen() const { return m_firewire_capture && m_firewire_capture->isOpen(); }

  /** The color filter of the raw images (see CImageGrabber_dc1394::bayerPattern()). */
  [[nodiscard]] mrpt::img::BayerPattern bayerPattern() const
  {
    return m_firewire_capture->bayerPattern();
  }

  /** Grab stereo images.
   * \param out_observation The object to be filled with sensed data.
   *
   * \note The member "CObservationStereoImages::refCameraPose" must be set on
   * the return of this method by the user, since we don't know here the robot
   * physical structure.
   *
   * \return false on any error, true if all go fine.
   */
  bool getStereoObservation(mrpt::obs::CObservationStereoImages& out_observation);

 protected:
  /** The actual capture object. */
  std::unique_ptr<mrpt::hwdrivers::CImageGrabber_dc1394> m_firewire_capture;
};  // End of class
static_assert(
    !std::is_copy_constructible_v<CStereoGrabber_Bumblebee_libdc1394> &&
        !std::is_copy_assignable_v<CStereoGrabber_Bumblebee_libdc1394>,
    "Copy Check");
}  // namespace mrpt::hwdrivers
