/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/color_maps.h>

namespace mrpt
{
namespace opengl
{
/** A colorbar indicator. This class renders a colorbar as a 3D object, in the
 * XY plane.
  * For an overlay indicator that can be easily added to any display, see
 * COpenGLScene::addColorBar()
  *
  * \sa opengl::COpenGLScene,opengl::CRenderizable, COpenGLScene::addColorBar()
  *
  *  <div align="center">
  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
  *   <tr> <td> mrpt::opengl::CColorBar </td> <td> \image html
 * preview_CColorBar.png </td> </tr>
  *  </table>
  *  </div>
  *
  * \ingroup mrpt_opengl_grp
  */
class OPENGL_IMPEXP CColorBar : public CRenderizableDisplayList
{
	DEFINE_SERIALIZABLE(CColorBar)

   protected:
	mrpt::utils::TColormap m_colormap;
	double m_width, m_height;
	std::string m_label_format;
	double m_min_col, m_max_col, m_min_value, m_max_value;
	double m_label_font_size;
	bool m_disable_depth_test;

   public:
	/** Constructor returning a smart pointer to the newly created object. */
	static CColorBar::Ptr Create(
		/** The colormap to represent. */
		const mrpt::utils::TColormap colormap,
		/** size of the color bar */
		double width, double height,
		/** limits for [0,1] colormap indices */
		double min_col, double max_col,
		/** limits for values associated to extreme colors */
		double min_value, double max_value,
		/** sprintf-like format string for values */
		const std::string& label_format = std::string("%7.02f"),
		/** Label text font size */
		double label_font_size = .05);

	/** Render
	  * \sa mrpt::opengl::CRenderizable
	  */
	void render_dl() const override;

	void setColormap(const mrpt::utils::TColormap colormap);
	void setColorAndValueLimits(
		double col_min, double col_max, double value_min, double value_max);
	void enableDepthTest(bool enable);

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Basic empty constructor. Set all parameters to default. */
	CColorBar(
		/** The colormap to represent. */
		const mrpt::utils::TColormap colormap = mrpt::utils::cmGRAYSCALE,
		/** size of the color bar */
		double width = 0.2, double height = 1.0,
		/** limits for [0,1] colormap indices */
		double min_col = .0, double max_col = 1.0,
		/** limits for values associated to extreme colors */
		double min_value = .0, double max_value = 1.0,
		/** sprintf-like format string for values */
		const std::string& label_format = std::string("%7.02f"),
		/** Label text font size */
		double label_font_size = .05);
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(
	CColorBar, CRenderizableDisplayList, OPENGL_IMPEXP)
}
}
