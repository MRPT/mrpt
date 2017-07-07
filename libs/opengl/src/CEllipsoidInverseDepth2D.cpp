/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CEllipsoidInverseDepth2D.h>
#include <mrpt/math/matrix_serialization.h>  // for << >> ops of matrices
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(
	CEllipsoidInverseDepth2D, CRenderizableDisplayList, mrpt::opengl)

/*---------------------------------------------------------------
							transformFromParameterSpace
  ---------------------------------------------------------------*/
void CEllipsoidInverseDepth2D::transformFromParameterSpace(
	const std::vector<BASE::array_parameter_t>& in_pts,
	std::vector<BASE::array_point_t>& out_pts) const
{
	MRPT_START

	// (inv_range,yaw) --> (x,y)
	const size_t N = in_pts.size();
	out_pts.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		const double inv_range = in_pts[i][0];
		const double yaw = in_pts[i][1];
		const double range = inv_range < 0
								 ? m_underflowMaxRange
								 : (inv_range != 0 ? 1. / inv_range : 0);
		out_pts[i][0] = range * cos(yaw);
		out_pts[i][1] = range * sin(yaw);
	}

	MRPT_END
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
	 CSerializable objects
  ---------------------------------------------------------------*/
void CEllipsoidInverseDepth2D::writeToStream(
	mrpt::utils::CStream& out, int* version) const
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
void CEllipsoidInverseDepth2D::readFromStream(
	mrpt::utils::CStream& in, int version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			BASE::thisclass_readFromStream(in);

			in >> m_underflowMaxRange;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}
