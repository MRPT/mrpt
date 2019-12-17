/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/matrix_serialization.h>  // for << >> ops of matrices
#include <mrpt/opengl/CEllipsoidInverseDepth2D.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(
	CEllipsoidInverseDepth2D, CRenderizable, mrpt::opengl)

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

uint8_t CEllipsoidInverseDepth2D::serializeGetVersion() const { return 0; }
void CEllipsoidInverseDepth2D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	BASE::thisclass_writeToStream(out);

	out << m_underflowMaxRange;
}
void CEllipsoidInverseDepth2D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
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
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}
