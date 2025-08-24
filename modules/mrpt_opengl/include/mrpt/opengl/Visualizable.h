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

#include <mrpt/opengl/opengl_frwds.h>

#include <memory>  // shared_ptr

namespace mrpt::opengl
{
/** Interface for classes visualizable as an mrpt::opengl::CSetOfObjects.
 *
 * \ingroup mrpt_opengl_grp
 */
class Visualizable
{
 public:
  Visualizable() = default;
  ~Visualizable() = default;

  /** Inserts 3D primitives representing this object into the provided
   * container.
   * Note that the former contents of `o` are not cleared.
   *
   * \sa getVisualization()
   */
  virtual void getVisualizationInto(mrpt::opengl::CSetOfObjects& o) const = 0;

  /** Creates 3D primitives representing this objects.
   * This is equivalent to getVisualizationInto() but creating, and returning
   * by value, a new rpt::opengl::CSetOfObjects::Ptr shared pointer.
   * \sa getVisualizationInto()
   */
  std::shared_ptr<mrpt::opengl::CSetOfObjects> getVisualization() const;
};

}  // namespace mrpt::opengl
