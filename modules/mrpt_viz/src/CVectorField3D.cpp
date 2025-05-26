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
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CVectorField3D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField3D, CVisualObject, mrpt::viz)

/** Constructor */
CVectorField3D::CVectorField3D() :
    x_vf(0, 0), y_vf(0, 0), z_vf(0, 0), x_p(0, 0), y_p(0, 0), z_p(0, 0)
{
  m_point_color = m_field_color = m_still_color = m_maxspeed_color = getColor_u8();
  m_maxspeed = 1.f;
}

/** Constructor with a initial set of lines. */
CVectorField3D::CVectorField3D(
    CMatrixFloat x_vf_ini,
    CMatrixFloat y_vf_ini,
    CMatrixFloat z_vf_ini,
    CMatrixFloat x_p_ini,
    CMatrixFloat y_p_ini,
    CMatrixFloat z_p_ini) :
    m_colorFromModule(false), m_showPoints(true)
{
  x_vf = x_vf_ini;
  y_vf = y_vf_ini;
  z_vf = z_vf_ini;
  x_p = x_p_ini;
  y_p = y_p_ini;
  z_p = z_p_ini;
  m_point_color = m_field_color = m_still_color = m_maxspeed_color = getColor_u8();
  m_maxspeed = 1.f;
}

uint8_t CVectorField3D::serializeGetVersion() const { return 1; }
void CVectorField3D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);

  out << x_vf << y_vf << z_vf;
  out << x_p << y_p << z_p;
  out << m_point_color;
  out << m_field_color;
  VisualObjectParams_Lines::params_serialize(out);   // v1
  VisualObjectParams_Points::params_serialize(out);  // v1
}
void CVectorField3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
      THROW_EXCEPTION("Importing from old version not supported");
      break;

    case 1:
      readFromStreamRender(in);

      in >> x_vf >> y_vf >> z_vf;
      in >> x_p >> y_p >> z_p;
      in >> m_point_color;
      in >> m_field_color;
      VisualObjectParams_Lines::params_deserialize(in);   // v1
      VisualObjectParams_Points::params_deserialize(in);  // v1
      break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
      break;
  };
  CVisualObject::notifyChange();
}

auto CVectorField3D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return verticesBoundingBox();
}
