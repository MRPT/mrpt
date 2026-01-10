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

#define SELBYTE0(v) (v & 0xff)
#define SELBYTE1(v) ((v >> 8) & 0xff)
#define SELBYTE2(v) ((v >> 16) & 0xff)
#define SELBYTE3(v) ((v >> 24) & 0xff)

#define MAKEWORD16B(__LOBYTE, __HILOBYTE)   ((__LOBYTE) | ((__HILOBYTE) << 8))
#define MAKEWORD32B(__LOWORD16, __HIWORD16) ((__LOWORD16) | ((__HIWORD16) << 16))
#define MAKEWORD64B(__LOWORD32, __HIWORD32) ((__LOWORD32) | ((__HIWORD32) << 32))
