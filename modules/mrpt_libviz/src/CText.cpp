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
#include <mrpt/containers/yaml.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CText.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CText, CVisualObject, mrpt::viz)

CText::~CText() = default;

#if 0
constexpr double text_spacing = 1.5;
constexpr double text_kerning = 0.1;
#endif

std::pair<double, double> CText::computeTextExtension() const
{
#if 0
  mrpt::viz::internal::glSetFont(m_fontName);
  const auto [textW, textH] = mrpt::viz::internal::glGetExtends(m_str, text_spacing, text_kerning);
  return {textW, textH};
#endif
  THROW_EXCEPTION("TODO");
  MRPT_TODO("Refactor!");
}

uint8_t CText::serializeGetVersion() const { return 2; }
void CText::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_str;
  out << m_fontName;
  out << (uint32_t)m_fontHeight;
}

void CText::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      uint32_t i;
      readFromStreamRender(in);
      in >> m_str;
      if (version >= 1)
      {
        in >> m_fontName;
        in >> i;
        m_fontHeight = i;

        if (version < 2)
        {
          in >> i;
          // dummy, removed in v2: m_fontWidth = i;
        }
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

auto CText::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return {
      {0.f, 0.f, 0.f},
      {0.f, 0.f, 0.f}
  };
}

void CText::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
  CVisualObject::toYAMLMap(propertiesMap);
  propertiesMap["text"] = m_str;
}
