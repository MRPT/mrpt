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

#include <string>

namespace mrpt
{
/** Interface for classes whose state can be represented as a human-friendly
 * text.
 *
 * \ingroup mrpt_core_grp
 */
class Stringifyable
{
 public:
  Stringifyable() = default;
  virtual ~Stringifyable() = default;

  /** Returns a human-friendly textual description of the object. For classes
   * with a large/complex internal state, only a summary should be returned
   * instead of the exhaustive enumeration of all data members.
   */
  virtual std::string asString() const = 0;
};

}  // namespace mrpt
