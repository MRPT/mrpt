/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CEllipsoidRangeBearing2D_H
#define opengl_CEllipsoidRangeBearing2D_H

#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>

namespace mrpt
{
	namespace opengl
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CEllipsoidRangeBearing2D, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** An especial "ellipsoid" in 2D computed as the uncertainty iso-surfaces of a (range,bearing) variable.
		  *  The parameter space of this ellipsoid comprises these variables (in this order):
		  *   - range: Distance from sensor to feature.
		  *   - bearing: Angle from +X to the line that goes from the sensor towards the feature.
		  *
		  *  This class expects you to provide a mean vector of length 2 and a 2x2 covariance matrix, set with \a setCovMatrixAndMean().
		  *
		  * Please read the documentation of CGeneralizedEllipsoidTemplate::setQuantiles() for learning
		  *  the mathematical details about setting the desired confidence interval.
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CEllipsoidRangeBearing2D </td> <td> \image html preview_CEllipsoidRangeBearing2D.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CEllipsoidRangeBearing2D : public CGeneralizedEllipsoidTemplate<2>
		{
			typedef CGeneralizedEllipsoidTemplate<2> BASE;
			DEFINE_SERIALIZABLE( CEllipsoidRangeBearing2D )
		protected:
			/** To be implemented by derived classes: maps, using some arbitrary space transformation, a list of points 
			  *  defining an ellipsoid in parameter space into their corresponding points in 2D/3D space.
			  */
			virtual void transformFromParameterSpace(
				const std::vector<BASE::array_parameter_t> &in_pts,
				std::vector<BASE::array_point_t> & out_pts) const;
		private:
			/** Constructor
			  */
			CEllipsoidRangeBearing2D()
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CEllipsoidRangeBearing2D() { }
		};

	} // end namespace

} // End of namespace

#endif