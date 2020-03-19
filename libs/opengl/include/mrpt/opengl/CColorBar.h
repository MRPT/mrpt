/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/color_maps.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
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
class CColorBar : public CRenderizableShaderTriangles,
				  public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CColorBar, mrpt::opengl)

   public:
	CColorBar(/** The colormap to represent. */
			  const mrpt::img::TColormap colormap = mrpt::img::cmGRAYSCALE,
			  /** size of the color bar */
			  double width = 0.2, double height = 1.0,
			  /** limits for [0,1] colormap indices */
			  float min_col = .0, float max_col = 1.0,
			  /** limits for values associated to extreme colors */
			  float min_value = .0, float max_value = 1.0,
			  /** sprintf-like format string for values */
			  const std::string& label_format = std::string("%7.02f"),
			  /** Label text font size */
			  float label_font_size = .05f);

	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	virtual shader_list_t requiredShaders() const override
	{
		// May use up to two shaders (triangles and lines):
		return {DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	void onUpdateBuffers_all();
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
	}
	/** @} */

	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	void setColormap(const mrpt::img::TColormap colormap);
	void setColorAndValueLimits(
		float col_min, float col_max, float value_min, float value_max);

   protected:
	mrpt::img::TColormap m_colormap;
	double m_width, m_height;
	std::string m_label_format;
	float m_min_col, m_max_col, m_min_value, m_max_value;
	float m_label_font_size;
};
}  // namespace mrpt::opengl
