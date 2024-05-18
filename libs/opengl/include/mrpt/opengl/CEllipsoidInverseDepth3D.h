/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::opengl
{
/** An especial "ellipsoid" in 3D computed as the uncertainty iso-surfaces of a
 * (inv_range,yaw,pitch) variable.
 *  The parameter space of this ellipsoid comprises these variables (in this
 * order):
 *   - inv_range: The inverse distance from the sensor to the feature.
 *   - yaw: Angle for the rotation around +Z ("azimuth").
 *   - pitch: Angle for the rotation around +Y ("elevation"). Positive means
 * pointing below the XY plane.
 *
 *  This parameterization is based on the paper:
 *   - Civera, J. and Davison, A.J. and Montiel, J., "Inverse depth
 * parametrization for monocular SLAM", T-RO, 2008.
 *
 *  This class expects you to provide a mean vector of length 3 and a 3x3
 * covariance matrix, set with \a setCovMatrixAndMean().
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 *  the mathematical details about setting the desired confidence interval.
 *
 * ![mrpt::opengl::CEllipsoidInverseDepth3D](preview_CEllipsoidInverseDepth3D.png)
 *
 * \ingroup mrpt_opengl_grp
 */
class CEllipsoidInverseDepth3D :
    public CGeneralizedEllipsoidTemplate<3>,
    virtual public CRenderizable
{
  using BASE = CGeneralizedEllipsoidTemplate<3>;
  DEFINE_SERIALIZABLE(CEllipsoidInverseDepth3D, mrpt::opengl)

 public:
  /** The maximum range to be used as a correction when a point of the
   * ellipsoid falls in the negative ranges (default: 1e6) */
  void setUnderflowMaxRange(const float maxRange) { m_underflowMaxRange = maxRange; }
  float getUnderflowMaxRange() const { return m_underflowMaxRange; }

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
  float m_underflowMaxRange = 1e6f;

 public:
  /** Constructor
   */
  CEllipsoidInverseDepth3D() = default;
  /** Private, virtual destructor: only can be deleted from smart pointers */
  ~CEllipsoidInverseDepth3D() override = default;
};

}  // namespace mrpt::opengl
