/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSBAUDRATE_H
#define XSBAUDRATE_H

#ifdef _WIN32
#	include <windows.h>
#else
#	include <termios.h>
#	ifndef	B460800
#		undef	B230400
#		define	B230400	0010003
#		define  B460800 0010004
#		define  B921600 0010007
#	endif
#endif

/*! \brief Communication speed. */
#ifdef _WIN32

enum XsBaudRate {
	XBR_Invalid					= 0,			//!< Not a valid baud rate
	XBR_4800					= CBR_4800,		//!< 4k8 (4800 bps)
	XBR_9600					= CBR_9600,		//!< 9k6 (9600 bps)
//	XBR_14k4					= CBR_14400,
	XBR_19k2					= CBR_19200,	//!< 19k2 (19200 bps)
//	XBR_28k8					= 28800,
	XBR_38k4					= CBR_38400,	//!< 38k4 (38400 bps)
	XBR_57k6					= CBR_57600,	//!< 57k6 (57600 bps)
	XBR_115k2					= CBR_115200,	//!< 115k2 (115200 bps)
	XBR_230k4					= 230400,		//!< 230k4 (230400 bps)
	XBR_460k8					= 460800,		//!< 460k8 (460800 bps)
	XBR_921k6					= 921600,		//!< 921k6 (921600 bps)
	XBR_2000k					= 2000000,		//!< 2000k0 (2000000 bps)
	XBR_4000k					= 4000000		//!< 4000k0 (4000000 bps)
};

#else

enum XSNOCOMEXPORT XsBaudRate {
	XBR_Invalid					= 0,			//!< Not a valid baud rate

	#ifndef	B2000000
	#define B2000000	2000000
	#endif
	#ifndef	B4000000
	#define B4000000	4000000
	#endif

	XBR_4800					= B4800,		//!< 4k8 (4800 bps)
	XBR_9600					= B9600,		//!< 9k6 (9600 bps)
//	XBR_14k4					= B14400,
	XBR_19k2					= B19200,		//!< 19k2 (19200 bps)
//	XBR_28k8					= B28800,
	XBR_38k4					= B38400,		//!< 38k4 (38400 bps)
	XBR_57k6					= B57600,		//!< 57k6 (57600 bps)
	XBR_115k2					= B115200,		//!< 115k2 (115200 bps)
	XBR_230k4					= B230400,		//!< 230k4 (230400 bps)
	XBR_460k8					= B460800,		//!< 460k8 (460800 bps)
	XBR_921k6					= B921600,		//!< 921k6 (921600 bps)
	XBR_2000k					= B2000000,		//!< 2000k0 (2000000 bps)
	XBR_4000k					= B4000000		//!< 4000k0 (4000000 bps)
};

#endif

#endif
