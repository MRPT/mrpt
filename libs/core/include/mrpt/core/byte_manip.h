/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#define SELBYTE0(v) (v & 0xff)
#define SELBYTE1(v) ((v >> 8) & 0xff)
#define SELBYTE2(v) ((v >> 16) & 0xff)
#define SELBYTE3(v) ((v >> 24) & 0xff)

#define MAKEWORD16B(__LOBYTE, __HILOBYTE) ((__LOBYTE) | ((__HILOBYTE) << 8))
#define MAKEWORD32B(__LOWORD16, __HIWORD16) \
	((__LOWORD16) | ((__HIWORD16) << 16))
#define MAKEWORD64B(__LOWORD32, __HIWORD32) \
	((__LOWORD32) | ((__HIWORD32) << 32))
