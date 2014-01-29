/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef LIBUSBEMU_FAIL_GUARD_H
#define LIBUSBEMU_FAIL_GUARD_H

namespace libusbemu
{
  namespace failguard
  {
    const bool Check();

    void WaitDecision();

    const bool Abort();
  }
}

#endif//LIBUSBEMU_FAIL_GUARD_H
