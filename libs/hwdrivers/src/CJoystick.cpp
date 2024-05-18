/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <mmsystem.h>

#if !defined(__GNUC__)
#pragma comment(lib, "WINMM.LIB")
#endif
#endif

#if defined(MRPT_OS_LINUX) || defined(__APPLE__)
// Linux
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(MRPT_OS_LINUX) && defined(HAVE_LINUX_INPUT_H)
#include <linux/input.h>
#include <linux/joystick.h>
#endif
#endif

#include <mrpt/hwdrivers/CJoystick.h>

using namespace mrpt::hwdrivers;

CJoystick::CJoystick() = default;

/*---------------------------------------------------------------
          Destructor
 ---------------------------------------------------------------*/
CJoystick::~CJoystick()
{
#if defined(MRPT_OS_LINUX) && defined(HAVE_LINUX_INPUT_H)
  // Close joystick, if open:
  if (m_joy_fd > 0)
  {
    close(m_joy_fd);
    m_joy_fd = -1;
  }
#endif
}

/*---------------------------------------------------------------
          getJoysticksCount
  Returns the number of Joysticks in the computer.
 ---------------------------------------------------------------*/
int CJoystick::getJoysticksCount()
{
  MRPT_START
#ifdef _WIN32
  return joyGetNumDevs();
#elif defined(MRPT_OS_LINUX) && defined(HAVE_LINUX_INPUT_H)
  // Try to open several joy devs:
  int joy_fd = -1;
  int nJoys = 0;

  do
  {
    if (-1 != (joy_fd = open(format("/dev/input/js%i", nJoys).c_str(), O_RDONLY)))
    {
      nJoys++;
      close(joy_fd);
    }
  } while (joy_fd != -1);

  return nJoys;
#else
  // Apple:
  return 0;
#endif
  MRPT_END
}

/*---------------------------------------------------------------
  Gets joystick information.

  \return Returns true if successfull, false on error, for example, if joystick
 is not present.
 ---------------------------------------------------------------*/
bool CJoystick::getJoystickPosition(int nJoy, State& output)
{
  MRPT_START
#ifdef _WIN32
  JOYINFO jinfo;

  int ID = JOYSTICKID1 + nJoy;

  // Get joy pos:
  if (JOYERR_NOERROR != joyGetPos(ID, &jinfo)) return false;  // Error.

  int calib_min[3], calib_max[3];

  for (int i = 0; i < 3; i++)
  {
    calib_min[i] = m_minPerAxis.size() > i ? m_minPerAxis[i] : 0;
    calib_max[i] = m_maxPerAxis.size() > i ? m_maxPerAxis[i] : 0xffff;
  }

  // Output data:
  float x = (jinfo.wXpos - calib_min[0]) / (float)(calib_max[0] - calib_min[0]);
  float y = (jinfo.wYpos - calib_min[1]) / (float)(calib_max[1] - calib_min[1]);
  float z = (jinfo.wZpos - calib_min[2]) / (float)(calib_max[2] - calib_min[2]);

  x = 2 * x - 1;
  y = 2 * y - 1;
  z = 2 * z - 1;

  output.buttons.resize(4);

  output.buttons[0] = 0 != (jinfo.wButtons & JOY_BUTTON1);
  output.buttons[1] = 0 != (jinfo.wButtons & JOY_BUTTON2);
  output.buttons[2] = 0 != (jinfo.wButtons & JOY_BUTTON3);
  output.buttons[3] = 0 != (jinfo.wButtons & JOY_BUTTON4);

  output.axes_raw.resize(3);
  output.axes_raw[0] = jinfo.wXpos;
  output.axes_raw[1] = jinfo.wYpos;
  output.axes_raw[2] = jinfo.wZpos;

  output.axes.resize(3);
  output.axes[0] = x;
  output.axes[1] = y;
  output.axes[2] = z;

  return true;
#elif defined(MRPT_OS_LINUX) && defined(HAVE_LINUX_INPUT_H)
  // Already open?
  if (m_joy_index == nJoy && m_joy_fd != -1)
  {
    // Ok
  }
  else
  {
    // Close previous opened joy?
    if (m_joy_fd != -1) close(m_joy_fd);

    // Go, try open joystick:
    if ((m_joy_fd = open(format("/dev/input/js%i", nJoy).c_str(), O_RDONLY)) < 0) return false;

    // Perfect!
    m_joy_index = nJoy;

    // Read in non-blocking way: **** Refer to sources of "jstest"!!!! ***
    fcntl(m_joy_fd, F_SETFL, O_NONBLOCK);
  }

  struct js_event js;

  while (read(m_joy_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))
  {
    // js.number: Button number
    const size_t jsNum = static_cast<size_t>(js.number);

    // Button?
    if (js.type & JS_EVENT_BUTTON)
    {
      if (m_joystate_btns.size() < jsNum + 1) m_joystate_btns.resize(jsNum + 1);

      m_joystate_btns[jsNum] = (js.value != 0);
    }

    // Axes?
    if (js.type & JS_EVENT_AXIS)
    {
      if (m_joystate_axes.size() < jsNum + 1) m_joystate_axes.resize(jsNum + 1);

      m_joystate_axes[jsNum] = static_cast<int>(js.value);
    }
  }

  if (errno != EAGAIN)
  {
    // Joystick disconnected?
    m_joy_fd = -1;
    m_joy_index = -1;
    return false;
  }

  // Fill out data:
  output.buttons = m_joystate_btns;

  const size_t nAxis = m_joystate_axes.size();
  output.axes_raw.resize(nAxis);
  output.axes.resize(nAxis);

  for (size_t i = 0; i < nAxis; i++)
  {
    output.axes_raw[i] = m_joystate_axes[i];

    const int calib_min = m_minPerAxis.size() > i ? m_minPerAxis[i] : -32767;
    const int calib_max = m_maxPerAxis.size() > i ? m_maxPerAxis[i] : 32767;

    output.axes[i] =
        -1.0f + 2.0f * (m_joystate_axes[i] - calib_min) / static_cast<float>(calib_max - calib_min);
  }

  return true;
#else
  // Apple.
  return false;
#endif
  MRPT_END
}

void CJoystick::setLimits(const std::vector<int>& minPerAxis, const std::vector<int>& maxPerAxis)
{
  m_minPerAxis = minPerAxis;
  m_maxPerAxis = maxPerAxis;
}
