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

#include <cstdint>

namespace mrpt::system
{
/** A high-performance stopwatch, with typical resolution of nanoseconds.
 *
 * This always uses the system MONOTONIC clock, despite the setting in
 * mrpt::Clock.
 *
 *  \note The class is named after the Spanish equivalent of "Tic-Toc" ;-)
 * \ingroup mrpt_system_grp
 */
class CTicTac
{
 public:
  /** Default constructor. Implicitly calls Tic() */
  CTicTac() noexcept;
  /** Starts the stopwatch. \sa Tac() */
  void Tic() noexcept;
  /** Stops the stopwatch.  \return Returns the ellapsed time in seconds.
   * \sa Tic() */
  [[nodiscard]] double Tac() const noexcept;

 private:
  /** Opaque storage, reinterpreted as `struct timespec` (POSIX) or
   * `LARGE_INTEGER` (Windows). `uint64_t` is used rather than
   * `unsigned long` so that the 8-byte alignment both of those require holds
   * on every platform: `unsigned long` is only 4 bytes (and 4-byte aligned)
   * on MSVC, while `LARGE_INTEGER` contains a `LONGLONG`.
   * `CTicTac.cpp` asserts size and alignment for the actual platform type.
   *
   * Deliberately *not* over-aligned beyond that: CTicTac is embedded in
   * CTimeLogger, which in turn is a member of classes used as virtual bases
   * (e.g. mrpt::graphslam::CRegistrationDeciderOrOptimizer). GCC then
   * compiles member functions of such a class assuming `this` has the
   * over-aligned alignment, while the virtual-base subobject inside a derived
   * class is only placed at its natural alignment, so the resulting aligned
   * SIMD accesses crash. See the equivalent note in
   * mrpt::math::CMatrixFixed. */
  std::uint64_t largeInts[4]{};
};  // End of class def.

static_assert(
    alignof(CTicTac) <= alignof(std::uint64_t),
    "CTicTac must not be over-aligned: it is embedded in classes used as "
    "virtual bases, where GCC then emits aligned SIMD accesses on "
    "subobjects that are only naturally aligned.");

}  // namespace mrpt::system
