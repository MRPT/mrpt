/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#ifdef _MSC_VER
#pragma warning(disable : 4251)  // DLL export private fields
#endif

#if defined(MRPT_EXPR_STATIC_DEFINE) || !defined(_WIN32)
#define MRPT_EXPR_EXPORT
#define MRPT_EXPR_NO_EXPORT
#else
#ifndef MRPT_EXPR_EXPORT
#ifdef expr_EXPORTS
/* We are building this library */
#define MRPT_EXPR_EXPORT __declspec(dllexport)
#else
/* We are using this library */
#define MRPT_EXPR_EXPORT __declspec(dllimport)
#endif
#endif

#ifndef MRPT_EXPR_NO_EXPORT
#define MRPT_EXPR_NO_EXPORT
#endif
#endif
