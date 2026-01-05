/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CText3D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CAxis, CVisualObject, mrpt::viz)

CAxis::CAxis(
    float xmin,
    float ymin,
    float zmin,
    float xmax,
    float ymax,
    float zmax,
    float frecuency,
    float lineWidth,
    bool marks) :
    m_xmin(xmin),
    m_ymin(ymin),
    m_zmin(zmin),
    m_xmax(xmax),
    m_ymax(ymax),
    m_zmax(zmax),
    m_frequency(frecuency)
{
  VisualObjectParams_Lines::setLineWidth(lineWidth);

  m_marks.fill(marks);

  // x:180, 0, 90
  m_textRot[0][0] = 180.f;
  m_textRot[0][1] = 0.f;
  m_textRot[0][2] = 90.f;
  // y:90, 0, 90
  m_textRot[1][0] = 90.f;
  m_textRot[1][1] = 0.f;
  m_textRot[1][2] = 90.f;
  // z:180, 0, 90
  m_textRot[2][0] = 180.f;
  m_textRot[2][1] = 0.f;
  m_textRot[2][2] = 90.f;
}

void CAxis::setTickMarksLength(float len)
{
  m_markLen = len;
  CVisualObject::notifyChange();
}

uint8_t CAxis::serializeGetVersion() const { return 3; }
void CAxis::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_xmin << m_ymin << m_zmin;
  out << m_xmax << m_ymax << m_zmax;
  out << m_frequency;
  // v1:
  out << m_marks[0] << m_marks[1] << m_marks[2] << m_textScale;
  for (auto i : m_textRot)
    for (int j = 0; j < 3; j++) out << i[j];
  // v2:
  out << m_markLen;
  // v3:
  this->VisualObjectParams_Lines::params_serialize(out);
}

void CAxis::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      readFromStreamRender(in);
      in >> m_xmin >> m_ymin >> m_zmin;
      in >> m_xmax >> m_ymax >> m_zmax;
      in >> m_frequency;
      if (version >= 1)
      {
        in >> m_marks[0] >> m_marks[1] >> m_marks[2] >> m_textScale;
        for (auto& i : m_textRot)
          for (int j = 0; j < 3; j++) in >> i[j];
      }
      else
      {
        bool v;
        in >> v;
        m_marks.fill(v);
        m_textScale = 0.25f;
      }
      if (version >= 2) in >> m_markLen;
      if (version >= 3) this->VisualObjectParams_Lines::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

auto CAxis::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {m_xmin, m_ymin, m_zmin}, {m_xmax, m_ymax, m_zmax});
}

void CAxis::setFrequency(float f)
{
  ASSERT_(f > 0);
  m_frequency = f;
  CVisualObject::notifyChange();
}
float CAxis::getFrequency() const { return m_frequency; }
void CAxis::enableTickMarks(bool v)
{
  m_marks.fill(v);
  CVisualObject::notifyChange();
}
void CAxis::enableTickMarks(bool show_x, bool show_y, bool show_z)
{
  m_marks[0] = show_x;
  m_marks[1] = show_y;
  m_marks[2] = show_z;
  CVisualObject::notifyChange();
}
void CAxis::setTextScale(float f)
{
  ASSERT_(f > 0);
  m_textScale = f;
  CVisualObject::notifyChange();
}
float CAxis::getTextScale() const { return m_textScale; }
void CAxis::setAxisLimits(float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
  m_xmin = xmin;
  m_ymin = ymin;
  m_zmin = zmin;
  m_xmax = xmax;
  m_ymax = ymax;
  m_zmax = zmax;
  CVisualObject::notifyChange();
}
void CAxis::setTextLabelOrientation(int axis, float yaw_deg, float pitch_deg, float roll_deg)
{
  ASSERT_(axis >= 0 && axis < 3);
  m_textRot[axis][0] = yaw_deg;
  m_textRot[axis][1] = pitch_deg;
  m_textRot[axis][2] = roll_deg;
}
void CAxis::getTextLabelOrientation(
    int axis, float& yaw_deg, float& pitch_deg, float& roll_deg) const
{
  ASSERT_(axis >= 0 && axis < 3);
  yaw_deg = m_textRot[axis][0];
  pitch_deg = m_textRot[axis][1];
  roll_deg = m_textRot[axis][2];
}
