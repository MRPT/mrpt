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

#include <mrpt/math/CMatrixD.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::nav
{
/** \addtogroup nav_holo Holonomic navigation methods
 * \ingroup mrpt_nav_grp
 * @{ */

/** A base class for log records for different holonomic navigation methods.
 *
 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
 */
class CHolonomicLogFileRecord : public mrpt::serialization::CSerializable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CHolonomicLogFileRecord, mrpt::nav)
 public:
  /** Final [N-1] and earlier stages [0...N-1] evaluation scores for each
   * direction, in the same order of TP-Obstacles. May be not filled by all
   * methods. */
  std::vector<std::vector<double>> dirs_eval;

  virtual const mrpt::math::CMatrixD* getDirectionScores() const { return nullptr; }
};

/** @} */
}  // namespace mrpt::nav
