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

#include <mrpt/viz/viz_frwds.h>

#include <memory>  // shared_ptr

namespace mrpt::viz
{
/** Interface for classes visualizable as an mrpt::viz::CSetOfObjects.
 *
 * \ingroup mrpt_viz_grp
 */
class Visualizable
{
 public:
  Visualizable() = default;
  virtual ~Visualizable() = default;

  Visualizable(const Visualizable&) = default;
  Visualizable& operator=(const Visualizable&) = default;
  Visualizable(Visualizable&&) = default;
  Visualizable& operator=(Visualizable&&) = default;

  /** Inserts 3D primitives representing this object into the provided
   * container.
   * Note that the former contents of `o` are not cleared.
   *
   * \sa getVisualization()
   */
  virtual void getVisualizationInto(mrpt::viz::CSetOfObjects& o) const = 0;

  /** Creates 3D primitives representing this object.
   * This is equivalent to getVisualizationInto() but creating, and returning
   * by value, a new rpt::opengl::CSetOfObjects::Ptr shared pointer.
   * \sa getVisualizationInto()
   */
  [[nodiscard]] std::shared_ptr<mrpt::viz::CSetOfObjects> getVisualization() const;
};

}  // namespace mrpt::viz
