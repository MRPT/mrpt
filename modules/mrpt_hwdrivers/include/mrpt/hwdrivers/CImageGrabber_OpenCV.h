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

#include <mrpt/core/pimpl.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/hwdrivers/config.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/typemeta/TEnumType.h>

#include <optional>

namespace mrpt::hwdrivers
{
/** These capture types are like their OpenCV equivalents. */
enum TCameraType
{
  CAMERA_CV_AUTODETECT = 0,
  CAMERA_CV_DC1394 = 1,
  CAMERA_CV_VFL = 2,
  // CAMERA_CV_VFW = 3, // removed feb-2024
  // CAMERA_CV_MIL = 4, // removed feb-2024
  CAMERA_CV_DSHOW = 5
};

/** Options used when creating an OpenCV capture object
 *  Some options apply to IEEE1394 cameras only.
 * \sa CImageGrabber_OpenCV
 * \ingroup mrpt_hwdrivers_grp
 */
struct TCaptureCVOptions
{
  TCaptureCVOptions() = default;

  /** (All cameras) Capture resolution (0: Leave the default) */
  int frame_width{0}, frame_height{0};
  /** (All cameras) Camera gain (0: Leave the default) */
  double gain{0};
  /** (IEEE1394 cameras) Frame rate for the capture (0: Leave the default). */
  double ieee1394_fps{0};
  /** (IEEE1394 cameras) Whether to grab grayscale images (Default=false). */
  bool ieee1394_grayscale{false};
};

/** \brief Grabs images from any OpenCV-compatible camera or AVI/video file.
 *
 * Wraps OpenCV's VideoCapture to provide MRPT observation objects. Supports
 * USB webcams, Video4Linux (V4L) devices, DirectShow (Windows), IEEE-1394
 * (dc1394 via OpenCV), and AVI/MPEG video files.
 *
 * Produced observations are of type mrpt::obs::CObservationImage.
 *
 * Unless playback from AVI files is required, the more generic class
 * mrpt::hwdrivers::CCameraSensor is recommended because it transparently
 * handles many additional backends.
 *
 * \note Requires MRPT to be built with OpenCV (MRPT_HAS_OPENCV).
 * \note Some code is based on the class CaptureCamera from the Orocos project.
 * \sa mrpt::hwdrivers::CCameraSensor, CImageGrabber_dc1394
 * \ingroup mrpt_hwdrivers_grp
 */
class CImageGrabber_OpenCV
{
 protected:
  /** Set to false if we could not initialize the camera.
   */
  bool m_bInitialized;

  struct Impl;
  mrpt::pimpl<Impl> m_capture;

 public:
  /** \brief Opens a live camera.
   *
   * \param[in] cameraIndex Device index passed to OpenCV VideoCapture.
   *            Use -1 when \a cameraType is CAMERA_CV_AUTODETECT.
   * \param[in] cameraType Camera backend selector (see TCameraType).
   * \param[in] options Capture options such as resolution and frame rate.
   */
  CImageGrabber_OpenCV(
      int cameraIndex = -1,
      TCameraType cameraType = CAMERA_CV_AUTODETECT,
      const TCaptureCVOptions& options = TCaptureCVOptions());

  /** \brief Opens an AVI or other video file for frame-by-frame playback.
   *
   * \param[in] AVI_fileName Path to the video file.
   */
  CImageGrabber_OpenCV(const std::string& AVI_fileName);

  /** \brief Destructor. Releases the OpenCV capture object. */
  virtual ~CImageGrabber_OpenCV();

  /** \brief Returns true if the camera or video file was opened successfully. */
  bool isOpen() const { return m_bInitialized; }

  /** \brief Grabs one frame from the opened camera.
   *
   * \param[out] out_observation Filled with the captured image and timestamp.
   * \return false on any error, true on success.
   * \deprecated Use grabFrame() instead.
   */
  [[deprecated("Use grabFrame() instead")]]
  bool getObservation(mrpt::obs::CObservationImage& out_observation);

  /** \brief Grabs one frame from the opened camera, returning by value.
   *
   * \return std::nullopt on any error, or the captured observation on success.
   */
  [[nodiscard]] std::optional<mrpt::obs::CObservationImage> grabFrame()
  {
    mrpt::obs::CObservationImage obs;
    if (!getObservation(obs)) { return std::nullopt; }
    return obs;
  }

};  // End of class

}  // namespace mrpt::hwdrivers
MRPT_ENUM_TYPE_BEGIN(mrpt::hwdrivers::TCameraType)
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_AUTODETECT);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_DC1394);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_VFL);
MRPT_FILL_ENUM_MEMBER(mrpt::hwdrivers, CAMERA_CV_DSHOW);
MRPT_ENUM_TYPE_END()
