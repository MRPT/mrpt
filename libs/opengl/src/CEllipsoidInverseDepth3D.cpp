/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CEllipsoidInverseDepth3D.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CEllipsoidInverseDepth3D, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							transformFromParameterSpace
  ---------------------------------------------------------------*/
  void CEllipsoidInverseDepth3D::transformFromParameterSpace(
	const std::vector<BASE::array_parameter_t> &in_pts,
	std::vector<BASE::array_point_t> & out_pts) const
{
	MRPT_START

	// (inv_range,yaw,pitch) --> (x,y,z)
	const size_t N = in_pts.size();
	out_pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		const double inv_range   = in_pts[i][0];
		const double yaw   = in_pts[i][1];
		const double pitch = in_pts[i][2];

		const double range = inv_range<0 ? m_underflowMaxRange : (inv_range!=0 ? 1./inv_range : 0);

		out_pts[i][0] = range * cos(yaw)*cos(pitch);
		out_pts[i][1] = range * sin(yaw)*cos(pitch);
		out_pts[i][2] =-range * sin(pitch);
	}

	MRPT_END
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CEllipsoidInverseDepth3D::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		BASE::thisclass_writeToStream(out);
			
		out << m_underflowMaxRange;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CEllipsoidInverseDepth3D::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			BASE::thisclass_readFromStream(in);

			in >> m_underflowMaxRange;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
