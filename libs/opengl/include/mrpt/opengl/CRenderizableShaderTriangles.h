/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
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
	virtual ~CRenderizableShaderTriangles() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TRIANGLES};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Triangles() = 0;

	// See base docs
	void freeOpenGLResources() override
	{
		m_trianglesBuffer.destroy();
		m_vao.destroy();
	}

	bool isLightEnabled() const { return m_enableLight; }
	void enableLight(bool enable = true) { m_enableLight = enable; }

	/** Control whether to render the FRONT, BACK, or BOTH (default) set of
	 * faces. Refer to docs for glCullFace() */
	void cullFaces(const TCullFace& cf) { m_cullface = cf; }
	TCullFace cullFaces() const { return m_cullface; }

	/** @name Raw access to triangle shader buffer data
	 * @{ */
	const auto& shaderTexturedTrianglesBuffer() const { return m_triangles; }
	/** @} */

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;

	/** Returns the bounding box of m_triangles, or (0,0,0)-(0,0,0) if empty. */
	const mrpt::math::TBoundingBox trianglesBoundingBox() const;

	void params_serialize(mrpt::serialization::CArchive& out) const;
	void params_deserialize(mrpt::serialization::CArchive& in);

   private:
	mutable COpenGLBuffer m_trianglesBuffer;
	mutable COpenGLVertexArrayObject m_vao;

	bool m_enableLight = true;
	TCullFace m_cullface = TCullFace::NONE;
};

}  // namespace mrpt::opengl
