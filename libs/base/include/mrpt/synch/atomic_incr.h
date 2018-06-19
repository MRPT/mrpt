/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */
#ifndef mrpt_synch_atomicincr_H
#define mrpt_synch_atomicincr_H

#include <mrpt/base/link_pragmas.h> // DLL import/export definitions
#include <mrpt/config.h>
#include <mrpt/utils/compiler_fixes.h>

namespace mrpt {
namespace synch {

/** This class acts exactly as an int (or long) variable, but with atomic
 * increment and decrement operators. This is a useful component of thread-safe
 * smart pointers. \ingroup synch_grp
 */
class BASE_IMPEXP CAtomicCounter {
public:
  CAtomicCounter();
  ~CAtomicCounter();
  explicit CAtomicCounter(long v);

  void operator++();     //!< Atomic increment of value.
  long operator--();     //!< Atomic decrement of value and return new value.
  operator long() const; //!< Get current value

private:
  long m_atomic[3];

  CAtomicCounter(CAtomicCounter const &);            //!< Forbidden method
  CAtomicCounter &operator=(CAtomicCounter const &); //!< Forbidden method
};                                                   // end of CAtomicCounter

} // namespace synch
} // namespace mrpt

#endif
