/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(
	CEllipsoid3D, CRenderizableShaderWireFrame, mrpt::opengl)

void CEllipsoid3D::transformFromParameterSpace(
	const std::vector<BASE::array_parameter_t>& in_pts,
	std::vector<BASE::array_point_t>& out_pts) const
{
	// Euclidean space:
	out_pts = in_pts;
}

uint8_t CEllipsoid3D::serializeGetVersion() const { return 0; }
void CEllipsoid3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_cov << m_drawSolid3D << m_quantiles << (uint32_t)m_numSegments
		<< m_lineWidth;
}

void CEllipsoid3D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			in >> m_cov;
			in >> m_drawSolid3D >> m_quantiles;
			m_numSegments = in.ReadAs<uint32_t>();
			in >> m_lineWidth;

			// Update cov. matrix cache:
			setCovMatrix(m_cov);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

#if 0
static bool quickSolveEqn(double a, double b_2, double c, double& t)
{
	double delta = square(b_2) - a * c;
	if (delta == 0)
		return (t = -b_2 / a) >= 0;
	else if (delta > 0)
	{
		delta = sqrt(delta);
		if ((t = (-b_2 - delta) / a) >= 0)
			return true;
		else
			return (t = (-b_2 + delta) / a) >= 0;
	}
	else
		return false;
}
#endif

bool CEllipsoid3D::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
#if 0  // Update, someday...
	if (m_cov.rows() != 3) return false;
	TLine3D lin, lin2;
	createFromPoseX((o - this->m_pose).asTPose(), lin);
	lin.unitarize();  // By adding this line, distance from any point of the
	// line to its base is exactly equal to the "t".
	for (size_t i = 0; i < 3; i++)
	{
		lin2.pBase[i] = 0;
		lin2.director[i] = 0;
		for (size_t j = 0; j < 3; j++)
		{
			double vji = m_eigVec(j, i);
			lin2.pBase[i] += vji * lin.pBase[j];
			lin2.director[i] += vji * lin.director[j];
		}
	}
	double a = 0, b_2 = 0, c = -square(m_quantiles);
	for (size_t i = 0; i < 3; i++)
	{
		double ev = m_eigVal(i, i);
		a += square(lin2.director[i] / ev);
		b_2 += lin2.director[i] * lin2.pBase[i] / square(ev);
		c += square(lin2.pBase[i] / ev);
	}
	return quickSolveEqn(a, b_2, c, dist);
#endif
	return false;
}
