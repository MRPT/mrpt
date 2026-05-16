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
#include <mrpt/img/TCamera.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <optional>

namespace mrpt::hwdrivers
{
/** Open parameters for CMyntEyeCamera
 * \sa CMyntEyeCamera
 * \ingroup mrpt_hwdrivers_grp
 */
struct TMyntEyeCameraParameters : public mrpt::config::CLoadableOptions
{
  std::uint8_t ir_intensity = 4;  //!< IR (Infrared), range [0,10], default 0.

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override;
};

/** \brief Captures stereo depth+RGB data from MYNT EYE-D cameras.
 *
 * Wraps the MYNT EYE SDK to expose the sensor as a standard MRPT grabber.
 * Produced observations are of type mrpt::obs::CObservation3DRangeScan
 * containing the left RGB image and the corresponding depth map.
 *
 * \note Requires the MYNT EYE SDK (libmynteye) to be installed.
 * \sa mrpt::hwdrivers::CCameraSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CMyntEyeCamera
{
 public:
  /** \brief Opens the MYNT EYE camera with the given parameters.
   *
   * \param[in] params Camera open parameters (IR intensity, etc.).
   */
  CMyntEyeCamera(const TMyntEyeCameraParameters& params);

  /** \brief Destructor. Closes the camera. */
  virtual ~CMyntEyeCamera();

  /** \brief Returns true if the camera was opened successfully. */
  bool isOpen() const { return m_bInitialized; }

  /** \brief Grabs one frame from the camera.
   *
   * \param[out] out Filled with the RGB+depth observation on success.
   * \return false on any error, true on success.
   * \deprecated Use grabFrame() instead.
   */
  [[deprecated("Use grabFrame() instead")]]
  bool getObservation(mrpt::obs::CObservation3DRangeScan& out);

  /** \brief Grabs one frame from the camera, returning by value.
   *
   * \return std::nullopt if no new data is available or on error, or the
   * observation on success.
   */
  [[nodiscard]] std::optional<mrpt::obs::CObservation3DRangeScan> grabFrame()
  {
    mrpt::obs::CObservation3DRangeScan obs;
    if (!getObservation(obs)) { return std::nullopt; }
    return obs;
  }

 protected:
  /** Set to false if we could not initialize the camera.
   */
  bool m_bInitialized = false;

  struct Impl;
  mrpt::pimpl<Impl> m_capture;

  mrpt::img::TCamera m_intrinsics_left, m_intrinsics_right;

};  // End of class

}  // namespace mrpt::hwdrivers
