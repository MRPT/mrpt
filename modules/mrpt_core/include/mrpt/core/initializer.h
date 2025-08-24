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

/** \file initializer.h
 * Usage: MRPT_INITIALIZER(func) { ... }
 * MRPT_INITIALIZER() should be used only once per library in its present
 * cross-platform implementation.
 */
#if defined(_MSC_VER)
#include <mrpt/core/config.h>
#if defined(MRPT_BUILT_AS_DLL)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define MRPT_INITIALIZER(f)                                                            \
  static void f();                                                                     \
  BOOL WINAPI DllMain(HINSTANCE /*hinstDLL*/, DWORD fdwReason, LPVOID /*lpvReserved*/) \
  {                                                                                    \
    switch (fdwReason)                                                                 \
    {                                                                                  \
      case DLL_PROCESS_ATTACH:                                                         \
        f();                                                                           \
    }                                                                                  \
    return TRUE;                                                                       \
  }                                                                                    \
  static void f(void)
#else
// Static libs in Windows: the hardest case, subject to be optimized out
#define MRPT_INITIALIZER(f) \
  static void f(void);      \
  struct f##_t_             \
  {                         \
    f##_t_(void) { f(); }   \
  };                        \
  static f##_t_ f##_;       \
  static void f(void)
#endif
#else
#define MRPT_INITIALIZER(f)                                               \
  static void f(void) __attribute__((constructor)) __attribute__((used)); \
  static void f(void)
#endif
