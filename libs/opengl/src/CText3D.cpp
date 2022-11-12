/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/containers/yaml.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/serialization/CArchive.h>

#include "gltext.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CText3D, CRenderizable, mrpt::opengl)

CText3D::CText3D(
	const std::string& str, const std::string& fontName, const float scale,
	const mrpt::opengl::TOpenGLFontStyle text_style, const double text_spacing,
	const double text_kerning)
	: m_str(str),
	  m_fontName(fontName),
	  m_text_style(text_style),
	  m_text_spacing(text_spacing),
	  m_text_kerning(text_kerning)
{
	this->setScale(scale);
}

CText3D::~CText3D() = default;

void CText3D::onUpdateBuffers_Text()
{
	std::unique_lock<std::shared_mutex> writeLock(m_textDataMtx.data);

	auto& vbd = CRenderizableShaderText::m_vertex_buffer_data;
	auto& tris = CRenderizableShaderText::m_triangles;
	auto& cbd = CRenderizableShaderText::m_color_buffer_data;
	vbd.clear();
	tris.clear();
	cbd.clear();

	mrpt::opengl::internal::glSetFont(m_fontName);
	mrpt::opengl::internal::glDrawText(
		m_str, tris, vbd, m_text_style, m_text_spacing, m_text_kerning);

	// All lines & triangles, the same color:
	cbd.assign(vbd.size(), m_color);
	for (auto& tri : m_triangles)
		tri.setColor(m_color);
}

uint8_t CText3D::serializeGetVersion() const { return 0; }
void CText3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_str << m_fontName << (uint32_t)m_text_style << m_text_spacing
		<< m_text_kerning;
}

void CText3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);

			uint32_t i;
			in >> m_str >> m_fontName >> i >> m_text_spacing >> m_text_kerning;

			m_text_style = TOpenGLFontStyle(i);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

auto CText3D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
	return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
		{0.f, 0.f, 0.f}, {m_str.size() * m_scale_x, 1.0f * m_scale_y, 0.f});
}

void CText3D::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
	CRenderizable::toYAMLMap(propertiesMap);
	propertiesMap["text"] = m_str;
}
