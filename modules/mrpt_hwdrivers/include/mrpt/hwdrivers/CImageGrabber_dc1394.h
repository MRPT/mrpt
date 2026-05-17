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

#include <mrpt/hwdrivers/config.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include <list>
#include <optional>

namespace mrpt::hwdrivers
{
typedef enum
{
  FRAMERATE_1_875 = 32,
  FRAMERATE_3_75,
  FRAMERATE_7_5,
  FRAMERATE_15,
  FRAMERATE_30,
  FRAMERATE_60,
  FRAMERATE_120,
  FRAMERATE_240
} grabber_dc1394_framerate_t;

typedef enum
{
  COLOR_CODING_MONO8 = 352,
  COLOR_CODING_YUV411,
  COLOR_CODING_YUV422,
  COLOR_CODING_YUV444,
  COLOR_CODING_RGB8,
  COLOR_CODING_MONO16
} grabber_dc1394_color_coding_t;

/** Options used when creating an dc1394 capture object
 *   All but the frame size, framerate, and color_coding can be changed
 * dynamically by CImageGrabber_dc1394::changeCaptureOptions
 * \sa CImageGrabber_dc1394
 * \ingroup mrpt_hwdrivers_grp
 */
struct TCaptureOptions_dc1394
{
  TCaptureOptions_dc1394() = default;

  /** Capture resolution (Default: 640x480) */
  int frame_width{640}, frame_height{480};
  grabber_dc1394_framerate_t framerate{FRAMERATE_15};
  grabber_dc1394_color_coding_t color_coding{COLOR_CODING_YUV422};

  /** -1: Normal mode, i>=0: use MODE7_i, then frame_width/height and
   * color_coding are ignored. */
  int mode7{-1};

  /** Shutter, -1=default:Do not change */
  int shutter{-1};
  /** Gain, -1=default:Do not change */
  int gain{-1};
  /** Gamma, -1=default:Do not change */
  int gamma{-1};
  /** Brightness, -1=default:Do not change */
  int brightness{-1};
  /** Exposure, -1=default:Do not change */
  int exposure{-1};
  /** Sharpness, -1=default:Do not change */
  int sharpness{-1};
  /** White balance, -1=default:Do not change */
  int white_balance{-1};
  /** Shutter mode, -1=default:Do not change */
  int shutter_mode{-1};
  /** Gain mode, -1=default:Do not change */
  int gain_mode{-1};
  /** Gamma mode, -1=default:Do not change */
  int gamma_mode{-1};
  /** Brightness mode, -1=default:Do not change */
  int brightness_mode{-1};
  /** Exposure mode, -1=default:Do not change */
  int exposure_mode{-1};
  /** Sharpness mode, -1=default:Do not change */
  int sharpness_mode{-1};
  /** White balance mode, -1=default:Do not change */
  int white_balance_mode{-1};
  /** For stereo cameras (eg PR Bumblebee) */
  bool deinterlace_stereo{false};
  int trigger_power{-1};
  int trigger_mode{-1};
  int trigger_source{-1};
  int trigger_polarity{-1};
  /** Size of the libdc1394 ring buffer */
  int ring_buffer_size{15};
};

/** \brief Captures images from IEEE 1394 (FireWire) cameras using libdc1394-2.
 *
 * Supports any IEEE 1394 camera (DCAM/IIDC standard), including stereo pairs
 * such as the Point Grey Bumblebee. Use enumerateCameras() to discover
 * connected devices and their GUIDs before opening.
 *
 * Produced observations are:
 *  - mrpt::obs::CObservationImage for monocular cameras (grabFrame()).
 *  - mrpt::obs::CObservationStereoImages for stereo cameras
 *    (grabStereoFrame()).
 *
 * \note Requires libdc1394-2 and OpenCV. Currently Linux-only.
 * \note You may need to set permissions: chmod 666 /dev/video1394/ * and
 *       chmod 666 /dev/raw1394 to allow non-root access to FireWire devices.
 * \note Ring buffer length is configurable via
 *       TCaptureOptions_dc1394::ring_buffer_size (new in MRPT 1.3.0).
 * \sa mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CImageGrabber_dc1394
{
 protected:
  /** Set to false if we could not initialize the camera.
   */
  bool m_bInitialized{false};

  /** Internal use: */
  void /* dc1394_t * */* m_dc1394_lib_context{nullptr};
  void /* dc1394camera_t* */* m_dc1394camera{nullptr};
  int m_desired_mode;

  TCaptureOptions_dc1394 m_options;

 public:
  /** Constructor: open an ieee1394 camera.
   * \param cameraGUID Set the camera GUID to open, or 0 to open the first
   * found camera.
   * \param cameraUnit (Ignored if cameraGUID=0). The number of camera to open
   * within the device with the given GUID: In a stereo camera this may be 0
   * or 1. Normally this is 0.
   * \param options Capture options, defined in
   * mrpt::hwdrivers::TCaptureOptions_dc1394.
   * \param verbose  Displays a lot of information about the camera to be open
   * and its valid video modes.
   */
  CImageGrabber_dc1394(
      uint64_t cameraGUID = 0,
      uint16_t cameraUnit = 0,
      const TCaptureOptions_dc1394& options = TCaptureOptions_dc1394(),
      bool verbose = false);

  /** Destructor
   */
  virtual ~CImageGrabber_dc1394();

  /** Check whether the camera has been open successfully. */
  bool isOpen() const { return m_bInitialized; }
  /** \brief Changes live capture properties (brightness, gain, shutter, etc.).
   *
   * Frame size, frame rate, and color coding cannot be changed after opening
   * the camera; those fields in \a options are silently ignored.
   * \param[in] options The new capture property values to apply.
   * \return false on any error.
   */
  bool changeCaptureOptions(const TCaptureOptions_dc1394& options);

  /** \brief Grabs one frame from a monocular FireWire camera.
   *
   * \param[out] out_observation Filled with the captured image.
   * \return false on any error, true on success.
   * \note May block when using software trigger if no frame is pending.
   * \deprecated Use grabFrame() instead.
   */
  [[deprecated("Use grabFrame() instead")]] bool getObservation(
      mrpt::obs::CObservationImage& out_observation);

  /** \brief Grabs one frame from a monocular FireWire camera, returning by
   * value.
   *
   * \return std::nullopt on any error, or the captured observation on success.
   */
  [[nodiscard]] std::optional<mrpt::obs::CObservationImage> grabFrame()
  {
    mrpt::obs::CObservationImage obs;
    if (!getObservation(obs))
    {
      return std::nullopt;
    }
    return obs;
  }

  /** \brief Grabs a stereo frame from a FireWire stereo camera.
   *
   * \param[out] out_observation Filled with left and right images.
   * \return false on any error, true on success.
   * \deprecated Use grabStereoFrame() instead.
   */
  [[deprecated("Use grabStereoFrame() instead")]] bool getObservation(
      mrpt::obs::CObservationStereoImages& out_observation);

  /** \brief Grabs a stereo frame from a FireWire stereo camera, returning by
   * value.
   *
   * \return std::nullopt on any error, or the captured stereo observation on
   * success.
   */
  [[nodiscard]] std::optional<mrpt::obs::CObservationStereoImages> grabStereoFrame()
  {
    mrpt::obs::CObservationStereoImages obs;
    if (!getObservation(obs))
    {
      return std::nullopt;
    }
    return obs;
  }

  /** \brief Sets the software trigger signal level (ON/OFF).
   *
   * Allows controlling camera triggering via software rather than hardware.
   * \param[in] level true to assert (ON), false to de-assert (OFF).
   * \return false on any error.
   */
  bool setSoftwareTriggerLevel(bool level);

  /** \brief Information about a single FireWire camera found on the bus. */
  struct TCameraInfo
  {
    uint64_t guid;
    int unit;
    uint32_t unit_spec_ID;
    uint32_t unit_sw_version;
    uint32_t unit_sub_sw_version;
    uint32_t command_registers_base;
    uint32_t unit_directory;
    uint32_t unit_dependent_directory;
    uint64_t advanced_features_csr;
    uint64_t PIO_control_csr;
    uint64_t SIO_control_csr;
    uint64_t strobe_control_csr;
    uint64_t format7_csr[16];
    int iidc_version;
    std::string vendor;
    std::string model;
    uint32_t vendor_id;
    uint32_t model_id;
    bool bmode_capable;
    bool one_shot_capable;
    bool multi_shot_capable;
    bool can_switch_on_off;
    bool has_vmode_error_status;
    bool has_feature_error_status;
    int max_mem_channel;
  };

  using TCameraInfoList = std::list<TCameraInfo>;

  /** \brief Enumerates all FireWire cameras currently visible on the bus.
   *
   * \param[out] out_list Populated with one entry per discovered camera.
   * \exception std::runtime_error On any error from libdc1394.
   */
  static void enumerateCameras(TCameraInfoList& out_list);

};  // End of class

}  // namespace mrpt::hwdrivers
