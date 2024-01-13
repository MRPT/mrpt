/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt
{
/** \ingroup mrpt_maps_grp */
namespace opengl
{
/** This object renders a 2D laser scan by means of three elements: the points,
 * the line along end-points and the 2D scanned surface.
 *
 *  By default, all those three elements are drawn, but you can individually
 * switch them on/off with:
 *    - CPlanarLaserScan::enablePoints()
 *    - CPlanarLaserScan::enableLine()
 *    - CPlanarLaserScan::enableSurface()
 *
 *  To change the final result, more methods allow further customization of the
 * 3D object (color of each element, etc.).
 *
 *  The scan is passed or updated through CPlanarLaserScan::setScan()
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CPlanarLaserScan </td> <td> \image html
 * preview_CPlanarLaserScan.png </td> </tr>
 *  </table>
 *  </div>
 *
 *  \note The laser points are projected at the sensor pose as given in the
 * "scan" object, so this CPlanarLaserScan object should be placed at the exact
 * pose of the robot coordinates origin.
 *
 *  \sa mrpt::opengl::CPointCloud, opengl::Scene
 * \ingroup mrpt_maps_grp
 */
class CPlanarLaserScan : public CRenderizableShaderPoints,
						 public CRenderizableShaderTriangles,
						 public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CPlanarLaserScan, mrpt::opengl)

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
		CRenderizableShaderPoints::freeOpenGLResources();
	}

	virtual shader_list_t requiredShaders() const override
	{
		return {
			DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES_NO_LIGHT,
			DefaultShaderID::POINTS};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	void onUpdateBuffers_Points() override;
	mrpt::math::TPoint3Df getLocalRepresentativePoint() const override;
	/** @} */

	CPlanarLaserScan() = default;
	~CPlanarLaserScan() override = default;

	/** Clear the scan */
	void clear();

	/** Show or hides the scanned points \sa sePointsWidth, setPointsColor*/
	inline void enablePoints(bool enable = true)
	{
		m_enable_points = enable;
		CRenderizable::notifyChange();
	}

	/** Show or hides lines along all scanned points \sa setLineWidth,
	 * setLineColor*/
	inline void enableLine(bool enable = true)
	{
		m_enable_line = enable;
		CRenderizable::notifyChange();
	}

	/** Show or hides the scanned area as a 2D surface \sa setSurfaceColor */
	inline void enableSurface(bool enable = true)
	{
		m_enable_surface = enable;
		CRenderizable::notifyChange();
	}

	void setLineColor(float R, float G, float B, float A = 1.0f)
	{
		m_line_R = R;
		m_line_G = G;
		m_line_B = B;
		m_line_A = A;
	}
	void setPointsColor(float R, float G, float B, float A = 1.0f)
	{
		m_points_R = R;
		m_points_G = G;
		m_points_B = B;
		m_points_A = A;
	}
	void setSurfaceColor(float R, float G, float B, float A = 1.0f)
	{
		m_plane_R = R;
		m_plane_G = G;
		m_plane_B = B;
		m_plane_A = A;
	}

	void setScan(const mrpt::obs::CObservation2DRangeScan& scan)
	{
		CRenderizable::notifyChange();
		m_cache_valid = false;
		m_scan = scan;
	}

	auto internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf override;

   protected:
	mrpt::obs::CObservation2DRangeScan m_scan;
	mutable mrpt::maps::CSimplePointsMap m_cache_points;
	mutable bool m_cache_valid{false};

	float m_line_R{1.f}, m_line_G{0.f}, m_line_B{0.f}, m_line_A{0.5f};

	float m_points_R{1.0f}, m_points_G{0.0f}, m_points_B{0.0f},
		m_points_A{1.0f};

	float m_plane_R{0.01f}, m_plane_G{0.01f}, m_plane_B{0.6f}, m_plane_A{0.6f};

	bool m_enable_points{true};
	bool m_enable_line{true};
	bool m_enable_surface{true};
};

}  // namespace opengl
}  // namespace mrpt
