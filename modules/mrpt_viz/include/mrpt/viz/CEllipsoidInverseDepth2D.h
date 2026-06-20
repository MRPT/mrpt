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

#include <mrpt/viz/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::viz
{
/** An especial "ellipsoid" in 3D computed as the uncertainty iso-surfaces of a
 * (inv_range,yaw) variable.
 *  The parameter space of this ellipsoid comprises these variables (in this
 * order):
 *   - inv_range: The inverse distance from the sensor to the feature.
 *   - yaw: Angle for the rotation around +Z ("azimuth").
 *
 *  This parameterization is a 2D version of that presented in the paper:
 *   - Civera, J. and Davison, A.J. and Montiel, J., "Inverse depth
 * parametrization for monocular SLAM", T-RO, 2008.
 *
 *  This class expects you to provide a mean vector of length 4 and a 4x4
 * covariance matrix, set with \a setCovMatrixAndMean().
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 *  the mathematical details about setting the desired confidence interval.
 *
 * ![mrpt::viz::CEllipsoidInverseDepth2D](preview_CEllipsoidInverseDepth2D.png)
 *
 * \ingroup mrpt_viz_grp
 */
class CEllipsoidInverseDepth2D : public CGeneralizedEllipsoidTemplate<2>
{
  using BASE = CGeneralizedEllipsoidTemplate<2>;
  DEFINE_SERIALIZABLE(CEllipsoidInverseDepth2D, mrpt::viz)

 public:
  CEllipsoidInverseDepth2D() = default;
  virtual ~CEllipsoidInverseDepth2D() override = default;

  /** The maximum range to be used as a correction when a point of the
   * ellipsoid falls in the negative ranges (default: 1e6) */
  void setUnderflowMaxRange(const double maxRange) { m_underflowMaxRange = maxRange; }
  double getUnderflowMaxRange() const { return m_underflowMaxRange; }

 protected:
  /** To be implemented by derived classes: maps, using some arbitrary space
   * transformation, a list of points
   *  defining an ellipsoid in parameter space into their corresponding
   * points in 2D/3D space.
   */
  void transformFromParameterSpace(
      const std::vector<BASE::array_parameter_t>& in_pts,
      std::vector<BASE::array_point_t>& out_pts) const override;

 private:
  double m_underflowMaxRange{1e6};
};

}  // namespace mrpt::viz
