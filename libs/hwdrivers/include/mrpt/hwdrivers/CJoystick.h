/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>

#include <vector>

/*---------------------------------------------------------------
  Class
  ---------------------------------------------------------------*/
namespace mrpt::hwdrivers
{
/** Access to joysticks and gamepads (read buttons and position), and request
 * number of joysticks in the system.
 *
 * \note New in MRPT 2.13.0: the API changed to support an arbitrary number of
 *       input axes.
 *
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
  /** Constructor
   */
  CJoystick();

  /** Destructor
   */
  virtual ~CJoystick();

  /** Returns the number of Joysticks in the computer.
   */
  static int getJoysticksCount();

  struct State
  {
    std::vector<bool> buttons;
    std::vector<float> axes;    ///!< Normalized position values [0,1]
    std::vector<int> axes_raw;  ///!< Raw position values
  };

  /** Gets joystick information.
   *
   *   This method will try first to open the joystick, so you can safely call
   * it while the joystick is plugged and removed arbitrarly.
   *
   * \param nJoy The index of the joystick to query: The first one is 0, the
   * second 1, etc... See CJoystick::getJoysticksCount to discover the number
   * of joysticks in the system.
   * \param x The x axis position, range [-1,1]
   * \param y The y axis position, range [-1,1]
   * \param z The z axis position, range [-1,1]
   * \param buttons Each element will hold true if buttons are pressed. The
   * size of the vector will be set automatically to the number of buttons.
   * \param raw_x_pos If it is desired the raw integer measurement from
   * JoyStick, set this pointer to a desired placeholder.
   * \param raw_y_pos If it is desired the raw integer measurement from
   * JoyStick, set this pointer to a desired placeholder.
   * \param raw_z_pos If it is desired the raw integer measurement from
   * JoyStick, set this pointer to a desired placeholder.
   *
   * \return Returns true if successfull, false on error, for example, if
   * joystick is not present.
   *
   * \sa setLimits
   */
  bool getJoystickPosition(int nJoy, State& output);

  /** Set the axis limit values, for computing a [-1,1] position index easily
   * Only required to calibrate analog joystick.
   * It seems that these values must been calibrated for each joystick
   * model.
   *
   * Default values:
   * - Windows: [0, 0xFFFF]
   * - Linux: [-32767,32767]
   *
   * \sa getJoystickPosition
   */
  void setLimits(const std::vector<int>& minPerAxis, const std::vector<int>& maxPerAxis);

};  // End of class def.

}  // namespace mrpt::hwdrivers
