/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CColorBar.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl/gl_utils.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CColorBar, CRenderizableDisplayList, mrpt::opengl)

CColorBar::CColorBar(
	const mrpt::utils::TColormap colormap, //!< The colormap to represent.
	double width, double height,   //!< size of the color bar
	double min_col, double max_col,  //!< limits for [0,1] colormap indices
	double min_value, double max_value, //!< limits for values associated to extreme colors
	const std::string &label_format, //!< sprintf-like format string for values
	double label_font_size //!< Label text font size
) :
	m_colormap(colormap),
	m_min_col(min_col), m_max_col(max_col),
	m_min_value(min_value), m_max_value(max_value),
	m_label_format(label_format),
	m_label_font_size(label_font_size)
{

}

CColorBarPtr CColorBar::Create(
	const mrpt::utils::TColormap colormap, //!< The colormap to represent.
	double width, double height,   //!< size of the color bar
	double min_col, double max_col,  //!< limits for [0,1] colormap indices
	double min_value, double max_value, //!< limits for values associated to extreme colors
	const std::string &label_format, //!< sprintf-like format string for values
	double label_font_size //!< Label text font size
)
{ 
	return CColorBarPtr(new CColorBar(colormap,width,height,min_col,max_col, min_value, max_value, label_format, label_font_size)); 
}

void CColorBar::setColormap(const mrpt::utils::TColormap colormap)
{
	m_colormap = colormap; 
	CRenderizableDisplayList::notifyChange();
}

void CColorBar::setColorAndValueLimits(double col_min, double col_max, double value_min, double value_max)
{
	m_min_col = col_min;
	m_max_col = col_max;
	m_min_value = value_min;
	m_max_value = value_max;
	CRenderizableDisplayList::notifyChange();
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CColorBar::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	glDisable(GL_DEPTH_TEST); // colobars are typically displayed on-top of the rest of objects!
	glDisable(GL_LIGHTING);

	// solid:
	glEnable(GL_NORMALIZE);

	glBegin(GL_TRIANGLES);
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

	// Front face:
	//gl_utils::renderTriangleWithNormal(
	//	TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_min.z),
	//	TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
	//	TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_max.z) );

	glEnd();

	glDisable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CColorBar::writeToStream(mrpt::utils::CStream &out,int *version) const	{
	if (version) *version=0;
	else	{
		writeToStreamRender(out);
		//version 0
		out <<
			uint32_t(m_colormap) <<
			m_min_col << m_max_col <<
			m_min_value << m_max_value <<
			m_label_format <<
			m_label_font_size;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CColorBar::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);

			in.ReadAsAndCastTo<uint32_t, mrpt::utils::TColormap>(m_colormap);
			in >>
				m_min_col >> m_max_col >>
				m_min_value >> m_max_value >>
				m_label_format >>
				m_label_font_size;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}


void CColorBar::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max.x = m_width;
	bb_max.y = m_height;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
