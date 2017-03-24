/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSBAUDCODE_H
#define XSBAUDCODE_H

/*! \brief Internal baud rate configuration codes
*/

enum XSNOCOMEXPORT XsBaudCode {
	// Baudrate codes for SetBaudrate message
	XBC_4k8           = 0x0B,		//!< 4k8 (4800 bps)
	XBC_9k6           = 0x09,		//!< 9k6 (9600 bps)
//	XBC_14k4          = 0x08,
	XBC_19k2          = 0x07,		//!< 19k2 (19200 bps)
//	XBC_28k8          = 0x06,
	XBC_38k4          = 0x05,		//!< 38k4 (38400 bps)
	XBC_57k6          = 0x04,		//!< 57k6 (57600 bps)
//	XBC_76k8          = 0x03,
	XBC_115k2         = 0x02,		//!< 115k2 (115200 bps)
	XBC_230k4         = 0x01,		//!< 230k4 (230400 bps)
	XBC_460k8         = 0x00,		//!< 460k8 (460800 bps)
	XBC_921k6         = 0x0A,		//!< 921k6 (921600 bps). Only usable from MTi/x FW 2.4.6
	XBC_921k6Legacy   = 0x80,		//!< 921k6 (921600 bps)
	XBC_2MegaBaud     = 0x0C,		//!< 2000k0 (2000000 bps)
	XBC_4MegaBaud	  = 0x0D,		//!< 4000k0 (4000000 bps)
	XBC_Invalid       = 0xFF		//!< Not a valid baud rate
};

#endif
