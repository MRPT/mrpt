/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CSetOfLines.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSetOfLines, CVisualObject, mrpt::viz)

/** Constructor */
CSetOfLines::CSetOfLines()
{
  // Override default pointsize=1 in points shader:
  VisualObjectParams_Points::setPointSize(0);
}

/** Constructor with a initial set of lines. */
CSetOfLines::CSetOfLines(const std::vector<TSegment3D>& sgms, bool antiAliasing) : m_Segments(sgms)
{
  // Override default pointsize=1 in points shader:
  VisualObjectParams_Points::setPointSize(0);

  VisualObjectParams_Lines::setLineWidth(1);
  VisualObjectParams_Lines::enableAntiAliasing(antiAliasing);
}

/*---------------------------------------------------------------
              setLineByIndex
  ---------------------------------------------------------------*/
void CSetOfLines::setLineByIndex(size_t index, const mrpt::math::TSegment3D& segm)
{
  MRPT_START
  if (index >= m_Segments.size()) THROW_EXCEPTION("Index out of bounds");
  CVisualObject::notifyChange();
  m_Segments[index] = segm;
  MRPT_END
}

uint8_t CSetOfLines::serializeGetVersion() const { return 5; }
void CSetOfLines::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_Segments;
  VisualObjectParams_Lines::params_serialize(out);   // v5
  VisualObjectParams_Points::params_serialize(out);  // v4
}

void CSetOfLines::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      readFromStreamRender(in);
      CVectorFloat x0, y0, z0, x1, y1, z1;
      in >> x0 >> y0 >> z0 >> x1 >> y1 >> z1;
      if (version >= 1) VisualObjectParams_Lines::setLineWidth(in.ReadAs<float>());
      size_t N = x0.size();
      m_Segments.resize(N);
      for (size_t i = 0; i < N; i++)
      {
        m_Segments[i][0][0] = x0[i];
        m_Segments[i][0][1] = y0[i];
        m_Segments[i][0][2] = z0[i];
        m_Segments[i][1][0] = x1[i];
        m_Segments[i][1][1] = y1[i];
        m_Segments[i][1][2] = z1[i];
      }
    }
    break;
    case 2:
    case 3:
    case 4:
    {
      readFromStreamRender(in);
      in >> m_Segments;
      VisualObjectParams_Lines::setLineWidth(in.ReadAs<float>());
      if (version >= 3) VisualObjectParams_Lines::enableAntiAliasing(in.ReadAs<bool>());

      if (version >= 4)
        VisualObjectParams_Points::params_deserialize(in);
      else
        VisualObjectParams_Points::setPointSize(0);
    }
    break;
    case 5:
    {
      readFromStreamRender(in);
      in >> m_Segments;
      VisualObjectParams_Lines::params_deserialize(in);   // v5
      VisualObjectParams_Points::params_deserialize(in);  // v4
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

auto CSetOfLines::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  auto bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

  for (const auto& s : m_Segments)
  {
    for (size_t p = 0; p < 2; p++)
    {
      const TPoint3D& pt = s[p];
      bb.updateWithPoint(pt);
    }
  }

  return bb;
}

void CSetOfLines::getLineByIndex(
    size_t index, double& x0, double& y0, double& z0, double& x1, double& y1, double& z1) const
{
  ASSERT_(index < m_Segments.size());
  const mrpt::math::TPoint3D& p0 = m_Segments[index].point1;
  const mrpt::math::TPoint3D& p1 = m_Segments[index].point2;
  x0 = p0.x;
  y0 = p0.y;
  z0 = p0.z;
  x1 = p1.x;
  y1 = p1.y;
  z1 = p1.z;
}
