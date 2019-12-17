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

namespace mrpt::opengl
{
/** A 3D arrow
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CArrow </td> <td> \image html preview_CArrow.png
 * </td> </tr>
 *  </table>
 *  </div>
 * \ingroup mrpt_opengl_grp
 *
 */
class CArrow : public CRenderizable
{
	DEFINE_SERIALIZABLE(CArrow, mrpt::opengl)
	DEFINE_SCHEMA_SERIALIZABLE()
   protected:
	mutable float m_x0, m_y0, m_z0;
	mutable float m_x1, m_y1, m_z1;
	float m_headRatio;
	float m_smallRadius, m_largeRadius;
	// For version 2 in stream
	float m_arrow_roll;
	float m_arrow_pitch;
	float m_arrow_yaw;

   public:
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
	void setArrowYawPitchRoll(float yaw, float pitch, float roll)
	{
		m_arrow_yaw = yaw;
		m_arrow_pitch = pitch;
		m_arrow_roll = roll;
		CRenderizable::notifyChange();
	}

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor
	 */
	CArrow(
		float x0 = 0, float y0 = 0, float z0 = 0, float x1 = 1, float y1 = 1,
		float z1 = 1, float headRatio = 0.2f, float smallRadius = 0.05f,
		float largeRadius = 0.2f, float arrow_roll = -1.0f,
		float arrow_pitch = -1.0f, float arrow_yaw = -1.0f)
		: m_x0(x0),
		  m_y0(y0),
		  m_z0(z0),
		  m_x1(x1),
		  m_y1(y1),
		  m_z1(z1),
		  m_headRatio(headRatio),
		  m_smallRadius(smallRadius),
		  m_largeRadius(largeRadius),
		  m_arrow_roll(arrow_roll),
		  m_arrow_pitch(arrow_pitch),
		  m_arrow_yaw(arrow_yaw)
	{
	}

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CArrow() override = default;
};

}  // namespace mrpt::opengl
