/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::opengl
{
/** A planar disk in the XY plane.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CDisk : public CRenderizable
{
	DEFINE_SERIALIZABLE(CDisk, mrpt::opengl)

   protected:
	float m_radiusIn{0}, m_radiusOut{1};
	uint32_t m_nSlices{50}, m_nLoops{4};

   public:
	void setDiskRadius(float outRadius, float inRadius = 0)
	{
		m_radiusIn = inRadius;
		m_radiusOut = outRadius;
		CRenderizable::notifyChange();
	}

	float getInRadius() const { return m_radiusIn; }
	float getOutRadius() const { return m_radiusOut; }
	/** Default=50 */
	void setSlicesCount(uint32_t N)
	{
		m_nSlices = N;
		CRenderizable::notifyChange();
	}
	/** Default=4 */
	void setLoopsCount(uint32_t N)
	{
		m_nLoops = N;
		CRenderizable::notifyChange();
	}

	void render() const override;
	void renderUpdateBuffers() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Ray tracing
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/** Constructor
	 */
	CDisk() = default;
	CDisk(float rOut, float rIn, uint32_t slices = 50, uint32_t loops = 4)
		: m_radiusIn(rIn), m_radiusOut(rOut), m_nSlices(slices), m_nLoops(loops)
	{
	}

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CDisk() override = default;
};

}  // namespace mrpt::opengl
