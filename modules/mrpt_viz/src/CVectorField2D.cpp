/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CVectorField2D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField2D, CVisualObject, mrpt::viz)

/** Constructor */
CVectorField2D::CVectorField2D() : xcomp(0, 0), ycomp(0, 0)
{
  m_point_color = getColor_u8();
  m_field_color = getColor_u8();
}

/** Constructor with a initial set of lines. */
CVectorField2D::CVectorField2D(
    [[maybe_unused]] CMatrixFloat Matrix_x,
    [[maybe_unused]] CMatrixFloat Matrix_y,
    [[maybe_unused]] float xmin,
    [[maybe_unused]] float xmax,
    [[maybe_unused]] float ymin,
    [[maybe_unused]] float ymax)
{
  m_point_color = getColor_u8();
  m_field_color = getColor_u8();
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
   CSerializable objects
  ---------------------------------------------------------------*/
uint8_t CVectorField2D::serializeGetVersion() const { return 2; }
void CVectorField2D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << xcomp << ycomp;
  out << xMin << xMax << yMin << yMax;
  out << m_point_color;
  out << m_field_color;
  VisualObjectParams_Lines::params_serialize(out);
  VisualObjectParams_Points::params_serialize(out);
  VisualObjectParams_Triangles::params_serialize(out);
}

void CVectorField2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
      THROW_EXCEPTION("Unsupported old serialized version");
      break;

    case 2:
      readFromStreamRender(in);

      in >> xcomp >> ycomp;
      in >> xMin >> xMax >> yMin >> yMax;
      in >> m_point_color;
      in >> m_field_color;

      VisualObjectParams_Lines::params_deserialize(in);
      VisualObjectParams_Points::params_deserialize(in);
      VisualObjectParams_Triangles::params_deserialize(in);

      break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
      break;
  };
  CVisualObject::notifyChange();
}

auto CVectorField2D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return verticesBoundingBox();
}

void CVectorField2D::adjustVectorFieldToGrid()
{
  ASSERT_(xcomp.size() > 0);

  const float ratio_xp = xcomp.maxCoeff() * (xcomp.cols() - 1) / (xMax - xMin);
  const float ratio_xn = xcomp.minCoeff() * (xcomp.cols() - 1) / (xMax - xMin);
  const float ratio_yp = ycomp.maxCoeff() * (ycomp.rows() - 1) / (yMax - yMin);
  const float ratio_yn = ycomp.minCoeff() * (ycomp.rows() - 1) / (yMax - yMin);
  const float norm_factor =
      0.85f / max(max(ratio_xp, std::abs(ratio_xn)), max(ratio_yp, std::abs(ratio_yn)));

  xcomp *= norm_factor;
  ycomp *= norm_factor;
  CVisualObject::notifyChange();
}
