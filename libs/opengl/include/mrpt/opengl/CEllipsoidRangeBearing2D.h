/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::opengl
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
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CEllipsoidRangeBearing2D </td> <td> \image html
 * preview_CEllipsoidRangeBearing2D.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CEllipsoidRangeBearing2D : public CGeneralizedEllipsoidTemplate<2>,
								 virtual public CRenderizable
{
	using BASE = CGeneralizedEllipsoidTemplate<2>;
	DEFINE_SERIALIZABLE(CEllipsoidRangeBearing2D, mrpt::opengl)
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

}  // namespace mrpt::opengl
