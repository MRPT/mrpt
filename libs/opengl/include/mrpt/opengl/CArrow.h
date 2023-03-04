/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableShaderTriangles.h>

namespace mrpt::opengl
{
/** A 3D arrow
 *
 *  ![mrpt::opengl::CArrow](preview_CArrow.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_opengl_grp
 */
class CArrow : public CRenderizableShaderTriangles
{
	DEFINE_SERIALIZABLE(CArrow, mrpt::opengl)
	DEFINE_SCHEMA_SERIALIZABLE()

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void onUpdateBuffers_Triangles() override;
	/** @} */

	void setArrowEnds(
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
	template <typename Vector3Like>
	void setArrowEnds(const Vector3Like& start, const Vector3Like& end)
	{
		m_x0 = start[0];
		m_y0 = start[1];
		m_z0 = start[2];
		m_x1 = end[0];
		m_y1 = end[1];
		m_z1 = end[2];
		CRenderizable::notifyChange();
	}
	void setHeadRatio(float rat)
	{
		m_headRatio = rat;
		CRenderizable::notifyChange();
	}
	void setSmallRadius(float rat)
	{
		m_smallRadius = rat;
		CRenderizable::notifyChange();
	}
	void setLargeRadius(float rat)
	{
		m_largeRadius = rat;
		CRenderizable::notifyChange();
	}
	/** Number of radial divisions  */
	inline void setSlicesCount(uint32_t slices)
	{
		m_slices = slices;
		CRenderizable::notifyChange();
	}

	/** Number of radial divisions  */
	inline uint32_t getSlicesCount() const { return m_slices; }

	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

	/** Constructor */
	CArrow(
		float x0 = 0, float y0 = 0, float z0 = 0, float x1 = 1, float y1 = 1,
		float z1 = 1, float headRatio = 0.2f, float smallRadius = 0.05f,
		float largeRadius = 0.2f)
		: m_x0(x0),
		  m_y0(y0),
		  m_z0(z0),
		  m_x1(x1),
		  m_y1(y1),
		  m_z1(z1),
		  m_headRatio(headRatio),
		  m_smallRadius(smallRadius),
		  m_largeRadius(largeRadius)
	{
	}

	CArrow(
		const mrpt::math::TPoint3Df& from, const mrpt::math::TPoint3Df& to,
		float headRatio = 0.2f, float smallRadius = 0.05f,
		float largeRadius = 0.2f)
		: m_x0(from.x),
		  m_y0(from.y),
		  m_z0(from.z),
		  m_x1(to.x),
		  m_y1(to.y),
		  m_z1(to.z),
		  m_headRatio(headRatio),
		  m_smallRadius(smallRadius),
		  m_largeRadius(largeRadius)
	{
	}
	~CArrow() override = default;

   protected:
	mutable float m_x0, m_y0, m_z0;
	mutable float m_x1, m_y1, m_z1;
	float m_headRatio;
	float m_smallRadius, m_largeRadius;
	/** Number of radial divisions. */
	uint32_t m_slices = 10;
};

}  // namespace mrpt::opengl
