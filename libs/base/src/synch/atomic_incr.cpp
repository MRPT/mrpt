/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "base-precomp.h" // Precompiled headers

#include <mrpt/synch/atomic_incr.h>

#include <atomic>

using namespace mrpt::synch;

#define MY_ATOMIC reinterpret_cast<std::atomic_long *>(m_atomic)
#define MY_ATOMIC_CONST reinterpret_cast<const std::atomic_long *>(m_atomic)

CAtomicCounter::CAtomicCounter() {
  static_assert(sizeof(m_atomic) >= sizeof(std::atomic_long));
  new (MY_ATOMIC) std::atomic_long();
}
CAtomicCounter::~CAtomicCounter() { (MY_ATOMIC)->~atomic<long>(); }
CAtomicCounter::CAtomicCounter(long v) { new (MY_ATOMIC) std::atomic_long(v); }

void CAtomicCounter::operator++() { (*MY_ATOMIC)++; }
long CAtomicCounter::operator--() { return (*MY_ATOMIC)--; }
CAtomicCounter::operator long() const { return (*MY_ATOMIC_CONST); }
