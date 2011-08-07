/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers


#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
	#include <mmsystem.h>

	#if !defined(__GNUC__)
	#	pragma comment (lib,"WINMM.LIB")
	#endif
#endif

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	// Linux
	#include <sys/ioctl.h>
	#include <sys/time.h>
	#include <sys/types.h>
	#include <stdlib.h>
	#include <fcntl.h>
	#include <unistd.h>
	#include <stdio.h>
	#include <errno.h>
	#include <string.h>
	#include <stdlib.h>
	#include <stdint.h>

	#if defined(MRPT_OS_LINUX)
		#include <linux/input.h>
		#include <linux/joystick.h>
	#endif
#endif

#include <mrpt/hwdrivers/CJoystick.h>

using namespace mrpt::hwdrivers;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CJoystick::CJoystick()
#ifdef MRPT_OS_LINUX
   :
	m_joy_fd		(-1),
	m_joy_index		(-1)
#endif
{
	setLimits();
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CJoystick::~CJoystick()
{
#ifdef MRPT_OS_LINUX
	// Close joystick, if open:
	if (m_joy_fd>0)
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
int  CJoystick::getJoysticksCount()
{
	MRPT_START
#ifdef MRPT_OS_WINDOWS
	return joyGetNumDevs();
#elif defined(MRPT_OS_LINUX)
	// Try to open several joy devs:
	int joy_fd=-1;
	int nJoys = 0;

	do
	{
		if ( -1 != (joy_fd=open(format("/dev/input/js%i",nJoys).c_str(),O_RDONLY)) )
		{
			nJoys++;
			close(joy_fd);
		}
	} while (joy_fd!=-1);

	return nJoys;
#else
	// Apple:
	return 0;
#endif
	MRPT_END
}


/*---------------------------------------------------------------
  Gets joystick information.

  \return Returns true if successfull, false on error, for example, if joystick is not present.
 ---------------------------------------------------------------*/
bool CJoystick::getJoystickPosition(
	int 	nJoy,
	float 	&x,
	float 	&y,
	float 	&z,
	std::vector<bool>	&buttons,
	int *raw_x_pos,
	int	*raw_y_pos,
	int *raw_z_pos )
{
	MRPT_START
#ifdef MRPT_OS_WINDOWS
	JOYINFO		jinfo;

	int			ID = JOYSTICKID1 + nJoy;

	// Get joy pos:
	if (JOYERR_NOERROR != joyGetPos( ID, &jinfo ) )
		return false;	// Error.

	// Output data:
	x = ( jinfo.wXpos - m_x_min ) / (float)( m_x_max-m_x_min );
	y = ( jinfo.wYpos - m_y_min ) / (float)( m_y_max-m_y_min );
	z = ( jinfo.wZpos - m_z_min ) / (float)( m_z_max-m_z_min );

	x = 2*x-1;
	y = 2*y-1;
	z = 2*z-1;

	buttons.resize(4);

	buttons[0] = 0 != (jinfo.wButtons & JOY_BUTTON1);
	buttons[1] = 0 != (jinfo.wButtons & JOY_BUTTON2);
	buttons[2] = 0 != (jinfo.wButtons & JOY_BUTTON3);
	buttons[3] = 0 != (jinfo.wButtons & JOY_BUTTON4);

	if (raw_x_pos) *raw_x_pos=jinfo.wXpos;
	if (raw_y_pos) *raw_y_pos=jinfo.wYpos;
	if (raw_z_pos) *raw_z_pos=jinfo.wZpos;

	return true;
#elif defined(MRPT_OS_LINUX)
	// Already open?
	if ( m_joy_index == nJoy && m_joy_fd!=-1)
	{
		// Ok
	}
	else
	{
		// Close previous opened joy?
		if (m_joy_fd!=-1)
			close(m_joy_fd);

		// Go, try open joystick:
		if ((m_joy_fd = open( format("/dev/input/js%i",nJoy).c_str() , O_RDONLY)) < 0)
			return false;

		// Perfect!
		m_joy_index = nJoy;

		// Read in non-blocking way: **** Refer to sources of "jstest"!!!! ***
		fcntl(m_joy_fd, F_SETFL, O_NONBLOCK);
	}

	struct js_event js;

	while (read(m_joy_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))
	{
		// Button?
		if (js.type & JS_EVENT_BUTTON)
		{
			// js.number: Button number
			if (m_joystate_btns.size() < (size_t)js.number+1) m_joystate_btns.resize( js.number+1 );
			m_joystate_btns[js.number] = js.value!=0;
		}

		// Axes?
		if (js.type & JS_EVENT_AXIS)
		{
			//std::cout << "joy: event axis" << std::endl;
			if (m_joystate_axes.size()<(size_t)js.number+1) m_joystate_axes.resize( js.number+1 );
			m_joystate_axes[js.number] = js.value;
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
	const size_t nAxis = m_joystate_axes.size();
	if (nAxis>=1)
	{
		x = -1+2*( m_joystate_axes[0] - m_x_min ) / (float)( m_x_max-m_x_min );
		if (raw_x_pos) *raw_x_pos = m_joystate_axes[0];
	}

	if (nAxis>=2)
	{
		y = -1+2*( m_joystate_axes[1] - m_y_min ) / (float)( m_y_max-m_y_min );
		if (raw_y_pos) *raw_x_pos = m_joystate_axes[1];
	}

	if (nAxis>=3)
	{
		z = -1+2*( m_joystate_axes[2] - m_z_min ) / (float)( m_z_max-m_z_min );
		if (raw_z_pos) *raw_x_pos = m_joystate_axes[2];
	}
	else
	{
		z=0;
	}

	buttons = m_joystate_btns;

	return true;
#else
	// Apple.
	return false;
#endif
	MRPT_END
}

/** Set the axis limit values, for computing a [-1,1] position index easily.
*   It seems that these values must been calibrated for each joystick model.
*
* \sa getJoystickPosition
*/
void  CJoystick::setLimits( int x_min ,int x_max , int y_min,int y_max ,int z_min,int z_max )
{
	m_x_max = x_max;
	m_x_min = x_min;

	m_y_max = y_max;
	m_y_min = y_min;

	m_z_max = z_max;
	m_z_min = z_min;
}

