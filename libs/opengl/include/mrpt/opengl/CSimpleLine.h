/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
{
/** A line segment
 *  \sa opengl::Scene
 * \ingroup mrpt_opengl_grp
 */
class CSimpleLine : public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CSimpleLine, mrpt::opengl)

   protected:
	float m_x0, m_y0, m_z0;
	float m_x1, m_y1, m_z1;

   public:
	void setLineCoords(
		const mrpt::math::TPoint3Df& p0, const mrpt::math::TPoint3Df& p1)
	{
		m_x0 = p0.x;
		m_y0 = p0.y;
		m_z0 = p0.z;
		m_x1 = p1.x;
		m_y1 = p1.y;
		m_z1 = p1.z;
	}

	mrpt::math::TPoint3Df getLineStart() const { return {m_x0, m_y0, m_z0}; }
	mrpt::math::TPoint3Df getLineEnd() const { return {m_x1, m_y1, m_z1}; }

	/// \deprecated (MRPT 2.3.1)
	void setLineCoords(
		float x0, float y0, float z0, float x1, float y1, float z1)
	{
		m_x0 = x0;
		m_y0 = y0;
		m_z0 = z0;
		m_x1 = x1;
		m_y1 = y1;
		m_z1 = z1;
		CRenderizable::notifyChange();
	}

	/// \deprecated (MRPT 2.3.1)
	void getLineCoords(
		float& x0, float& y0, float& z0, float& x1, float& y1, float& z1) const
	{
		x0 = m_x0;
		y0 = m_y0;
		z0 = m_z0;
		x1 = m_x1;
		y1 = m_y1;
		z1 = m_z1;
	}

	void onUpdateBuffers_Wireframe() override;

	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

	/** Constructor
	 */
	CSimpleLine(
		float x0 = 0, float y0 = 0, float z0 = 0, float x1 = 0, float y1 = 0,
		float z1 = 0, float lineWidth = 1, bool antiAliasing = true);

	/** Private, virtual destructor: only can be deleted from smart pointers
	 */
	~CSimpleLine() override = default;
};

}  // namespace mrpt::opengl
