/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CEllipsoidInverseDepth3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(
	CEllipsoidInverseDepth3D, CRenderizable, mrpt::opengl)

/*---------------------------------------------------------------
							transformFromParameterSpace
  ---------------------------------------------------------------*/
void CEllipsoidInverseDepth3D::transformFromParameterSpace(
	const std::vector<BASE::array_parameter_t>& in_pts,
	std::vector<BASE::array_point_t>& out_pts) const
{
	MRPT_START

	// (inv_range,yaw,pitch) --> (x,y,z)
	const size_t N = in_pts.size();
	out_pts.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		const double inv_range = in_pts[i][0];
		const double yaw = in_pts[i][1];
		const double pitch = in_pts[i][2];

		const double range = inv_range < 0
								 ? m_underflowMaxRange
								 : (inv_range != 0 ? 1. / inv_range : 0);

		out_pts[i][0] = range * cos(yaw) * cos(pitch);
		out_pts[i][1] = range * sin(yaw) * cos(pitch);
		out_pts[i][2] = -range * sin(pitch);
	}

	MRPT_END
}

uint8_t CEllipsoidInverseDepth3D::serializeGetVersion() const { return 0; }
void CEllipsoidInverseDepth3D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	BASE::thisclass_writeToStream(out);

	out << m_underflowMaxRange;
}

void CEllipsoidInverseDepth3D::serializeFrom(
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
