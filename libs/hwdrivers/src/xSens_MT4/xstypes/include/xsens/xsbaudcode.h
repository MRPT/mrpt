/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSBAUDCODE_H
#define XSBAUDCODE_H

/*! \brief Internal baud rate configuration codes
 */

enum XSNOCOMEXPORT XsBaudCode
{
	// Baudrate codes for SetBaudrate message
	/** 4k8 (4800 bps) */
	XBC_4k8 = 0x0B,
	/** 9k6 (9600 bps) */
	XBC_9k6 = 0x09,
	//	XBC_14k4          = 0x08,
	/** 19k2 (19200 bps) */
	XBC_19k2 = 0x07,
	//	XBC_28k8          = 0x06,
	/** 38k4 (38400 bps) */
	XBC_38k4 = 0x05,
	/** 57k6 (57600 bps) */
	XBC_57k6 = 0x04,
	//	XBC_76k8          = 0x03,
	/** 115k2 (115200 bps) */
	XBC_115k2 = 0x02,
	/** 230k4 (230400 bps) */
	XBC_230k4 = 0x01,
	/** 460k8 (460800 bps) */
	XBC_460k8 = 0x00,
	/** 921k6 (921600 bps). Only usable from MTi/x FW 2.4.6 */
	XBC_921k6 = 0x0A,
	/** 921k6 (921600 bps) */
	XBC_921k6Legacy = 0x80,
	/** 2000k0 (2000000 bps) */
	XBC_2MegaBaud = 0x0C,
	/** 4000k0 (4000000 bps) */
	XBC_4MegaBaud = 0x0D,
	/** Not a valid baud rate */
	XBC_Invalid = 0xFF
};

#endif
