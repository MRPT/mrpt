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

#include <mrpt/viz/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::viz
{
/** An especial "ellipsoid" in 2D computed as the uncertainty iso-surfaces of a
 * (range,bearing) variable.
 *  The parameter space of this ellipsoid comprises these variables (in this
 * order):
 *   - range: Distance from sensor to feature.
 *   - bearing: Angle from +X to the line that goes from the sensor towards the
 * feature.
 *
 *  This class expects you to provide a mean vector of length 2 and a 2x2
 * covariance matrix, set with \a setCovMatrixAndMean().
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 *  the mathematical details about setting the desired confidence interval.
 *
 * ![mrpt::viz::CEllipsoidRangeBearing2D](preview_CEllipsoidRangeBearing2D.png)
 *
 * \ingroup mrpt_viz_grp
 */
class CEllipsoidRangeBearing2D :
    public CGeneralizedEllipsoidTemplate<2>,
    virtual public CVisualObject
{
  using BASE = CGeneralizedEllipsoidTemplate<2>;
  DEFINE_SERIALIZABLE(CEllipsoidRangeBearing2D, mrpt::viz)
 protected:
  /** To be implemented by derived classes: maps, using some arbitrary space
   * transformation, a list of points
   *  defining an ellipsoid in parameter space into their corresponding
   * points in 2D/3D space.
   */
  void transformFromParameterSpace(
      const std::vector<BASE::array_parameter_t>& in_pts,
      std::vector<BASE::array_point_t>& out_pts) const override;
  /** Constructor
   */
 public:
  CEllipsoidRangeBearing2D() = default;
  /** Private, virtual destructor: only can be deleted from smart pointers */
  ~CEllipsoidRangeBearing2D() override = default;
};

}  // namespace mrpt::viz
