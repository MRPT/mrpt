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

#include <mrpt/core/config.h>  // MRPT_OS_*()
#include <mrpt/hwdrivers/config.h>

#include <vector>

/*---------------------------------------------------------------
  Class
  ---------------------------------------------------------------*/
namespace mrpt::hwdrivers
{
/** \brief Reads axis positions and button states from joysticks and gamepads.
 *
 * Supports an arbitrary number of axes and buttons. Enumerates connected
 * devices via getJoysticksCount() and reads state with getJoystickPosition().
 *
 * Platform support: Linux (via /dev/input/jsX) and Windows (WinMM joystick
 * API).
 *
 * \note As of MRPT 2.13.0 the API was extended to support an arbitrary number
 *       of input axes (previously only x, y, z were exposed).
 * \ingroup mrpt_hwdrivers_grp
 */
class CJoystick
{
 private:
  /** The axis limits for normalization. */
  std::vector<int> m_minPerAxis, m_maxPerAxis;

#if defined(MRPT_OS_LINUX)
  /** File FD for the joystick, or -1 if not open (Linux only) */
  int m_joy_fd{-1};
  /** The index of the joystick open in m_joy_fd (Linux only) */
  int m_joy_index{-1};
  /** Using an event system we only have deltas, need to keep the whole
   * joystick state (Linux only) */
  std::vector<bool> m_joystate_btns;
  /** Using an event system we only have deltas, need to keep the whole
   * joystick state (Linux only) */
  std::vector<int> m_joystate_axes;
#endif

 public:
  /** \brief Default constructor. */
  CJoystick();

  /** \brief Destructor. Closes any open device handle. */
  virtual ~CJoystick();

  /** \brief Returns the number of joystick/gamepad devices currently
   * connected to the system.
   * \return Device count (>= 0).
   */
  static int getJoysticksCount();

  struct State
  {
    std::vector<bool> buttons;
    std::vector<float> axes;    ///!< Normalized position values [0,1]
    std::vector<int> axes_raw;  ///!< Raw position values
  };

  /** \brief Reads the current state of a joystick.
   *
   * Attempts to open the device if not already open, so it is safe to call
   * while joysticks are plugged or unplugged.
   *
   * \param[in]  nJoy   Zero-based index of the joystick to query.
   * \param[out] output Filled with normalized axis values [0,1] in
   *             output.axes, raw axis values in output.axes_raw, and button
   *             states in output.buttons.
   * \return true on success; false if the device is absent or an error occurs.
   * \sa setLimits
   */
  bool getJoystickPosition(int nJoy, State& output);

  /** \brief Sets the raw ADC range for each axis, used to normalize to [-1,1].
   *
   * Required only when calibrating analog joysticks whose default range
   * differs from the platform default (Windows: [0, 0xFFFF];
   * Linux: [-32767, 32767]).
   *
   * \param[in] minPerAxis Vector of minimum raw values per axis.
   * \param[in] maxPerAxis Vector of maximum raw values per axis.
   * \sa getJoystickPosition
   */
  void setLimits(const std::vector<int>& minPerAxis, const std::vector<int>& maxPerAxis);

};  // End of class def.

}  // namespace mrpt::hwdrivers
