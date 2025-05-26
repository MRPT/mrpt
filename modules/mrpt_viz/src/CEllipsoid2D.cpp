/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CEllipsoid2D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CEllipsoid2D, CVisualObject, mrpt::viz)

void CEllipsoid2D::transformFromParameterSpace(
    const std::vector<BASE::array_parameter_t>& in_pts,
    std::vector<BASE::array_point_t>& out_pts) const
{
  // Euclidean space:
  out_pts = in_pts;
}

uint8_t CEllipsoid2D::serializeGetVersion() const { return 1; }
void CEllipsoid2D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_cov << m_drawSolid3D << m_quantiles << (uint32_t)m_numSegments;
  VisualObjectParams_Lines::params_serialize(out);
}

void CEllipsoid2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      readFromStreamRender(in);
      in >> m_cov;
      in >> m_drawSolid3D >> m_quantiles;
      m_numSegments = in.ReadAs<uint32_t>();

      if (version >= 1)
      {
        VisualObjectParams_Lines::params_deserialize(in);
      }
      else
      {
        VisualObjectParams_Lines::setLineWidth(in.ReadAs<float>());
      }

      // Update cov. matrix cache:
      setCovMatrix(m_cov);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
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

bool CEllipsoid2D::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
#if 0  // Update, someday...
	if (m_cov.rows() != 3) return false;
	TLine3D lin, lin2;
	createFromPoseX((o - getCPose()).asTPose(), lin);
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
