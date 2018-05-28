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

using namespace mrpt::synch;

#include <atomic>

#define MY_ATOMIC (*reinterpret_cast<std::atomic_long *>(m_atomic))

CAtomicCounter::CAtomicCounter()
    : m_atomic(reinterpret_cast<long *>(new std::atomic_long)) {}
CAtomicCounter::CAtomicCounter(long v)
    : m_atomic(reinterpret_cast<long *>(new std::atomic_long)) {
  MY_ATOMIC = v;
}
CAtomicCounter::~CAtomicCounter() {
  delete reinterpret_cast<std::atomic_long *>(m_atomic);
  m_atomic = NULL;
}

void CAtomicCounter::operator++() { MY_ATOMIC++; }
long CAtomicCounter::operator--() { return MY_ATOMIC--; }
CAtomicCounter::operator long() const { return MY_ATOMIC; }
