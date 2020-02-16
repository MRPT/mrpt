/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the triangles shader.
 *
 *  \sa opengl::COpenGLScene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderTriangles : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderTriangles)

   public:
	CRenderizableShaderTriangles() = default;
	virtual ~CRenderizableShaderTriangles();

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TRIANGLES};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Triangles() = 0;

	/** Enables or disables transparency (default=false) */
	inline void enableTransparency(bool v)
	{
		m_enableTransparency = v;
		CRenderizable::notifyChange();
	}

	inline bool transparencyEnabled() const { return m_enableTransparency; }

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;

	// Computed in renderUpdateBuffers()
	// Note: a normal per vertex, not per triangle.
	mutable std::vector<mrpt::math::TVector3Df> m_trianglesNormals;

	bool m_enableTransparency = false;  //!< Transparency enabling.

   private:
	mutable unsigned int m_trianglesBuffer = 0, m_vao = 0;
	mutable unsigned int m_normalsBuffer = 0;
	mutable unsigned int m_colorBuffer = 0;
};

}  // namespace mrpt::opengl
