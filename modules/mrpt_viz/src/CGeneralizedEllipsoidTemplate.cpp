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
#include <mrpt/viz/CGeneralizedEllipsoidTemplate.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

template <>
void CGeneralizedEllipsoidTemplate<2>::generatePoints(
    const CGeneralizedEllipsoidTemplate<2>::cov_matrix_t& U,
    std::vector<CGeneralizedEllipsoidTemplate<2>::array_parameter_t>& pts) const
{
  pts.clear();
  pts.reserve(m_numSegments);
  const double Aa = 2 * M_PI / m_numSegments;
  for (double ang = 0; ang < 2 * M_PI; ang += Aa)
  {
    const double ccos = cos(ang);
    const double ssin = sin(ang);

    pts.resize(pts.size() + 1);

    auto& pt = pts.back();

    pt[0] = d2f(m_mean[0] + ccos * U(0, 0) + ssin * U(0, 1));
    pt[1] = d2f(m_mean[1] + ccos * U(1, 0) + ssin * U(1, 1));
  }
}

void aux_add3DpointWithEigenVectors(
    const double x,
    const double y,
    const double z,
    std::vector<mrpt::math::CMatrixFixed<float, 3, 1>>& pts,
    const mrpt::math::CMatrixFixed<double, 3, 3>& M,
    const mrpt::math::CMatrixFixed<double, 3, 1>& mean)
{
  pts.resize(pts.size() + 1);
  mrpt::math::CMatrixFixed<float, 3, 1>& pt = pts.back();
  pt[0] = d2f(mean[0] + x * M(0, 0) + y * M(0, 1) + z * M(0, 2));
  pt[1] = d2f(mean[1] + x * M(1, 0) + y * M(1, 1) + z * M(1, 2));
  pt[2] = d2f(mean[2] + x * M(2, 0) + y * M(2, 1) + z * M(2, 2));
}

template <>
void CGeneralizedEllipsoidTemplate<3>::generatePoints(
    const CGeneralizedEllipsoidTemplate<3>::cov_matrix_t& U,
    std::vector<CGeneralizedEllipsoidTemplate<3>::array_parameter_t>& pts) const
{
  MRPT_START
  const auto slices = m_numSegments, stacks = m_numSegments;
  ASSERT_GE_(slices, 3);
  ASSERT_GE_(stacks, 3);
  // sin/cos cache --------
  // Slices: [0,pi]
  std::vector<double> slice_cos(slices), slice_sin(slices);
  for (uint32_t i = 0; i < slices; i++)
  {
    double angle = M_PI * i / double(slices - 1);
    slice_sin[i] = sin(angle);
    slice_cos[i] = cos(angle);
  }
  // Stacks: [0,2*pi]
  std::vector<double> stack_sin(stacks), stack_cos(stacks);
  for (uint32_t i = 0; i < stacks; i++)
  {
    double angle = 2 * M_PI * i / double(stacks);
    stack_sin[i] = sin(angle);
    stack_cos[i] = cos(angle);
  }

  // Points in the ellipsoid:
  //  * "#slices" slices, with "#stacks" points each, but for the two ends
  //  * 1 point at each end slice
  // #total points = stacks*(slices-2) + 2
  pts.clear();
  pts.reserve((slices - 2) * stacks + 2);

  for (uint32_t i = 0; i < slices; i++)
  {
    if (i == 0)
      aux_add3DpointWithEigenVectors(1, 0, 0, pts, U, m_mean);
    else if (i == (slices - 1))
      aux_add3DpointWithEigenVectors(-1, 0, 0, pts, U, m_mean);
    else
    {
      const double x = slice_cos[i];
      const double R = slice_sin[i];

      for (uint32_t j = 0; j < stacks; j++)
      {
        const double y = R * stack_cos[j];
        const double z = R * stack_sin[j];
        aux_add3DpointWithEigenVectors(x, y, z, pts, U, m_mean);
      }
    }
  }

  MRPT_END
}
