/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt::opengl
{
class COpenGLViewport;
class CCamera;

/** A camera: if added to a scene, the viewpoint defined by this camera will be
 * used instead of the camera parameters set in COpenGLViewport::m_camera.
 *  A camera can be defined to provide a projective or orthogonal view of the
 * world by setting the member CCamera::m_projectiveModel.
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CCamera : public CRenderizable
{
	friend class COpenGLViewport;

	DEFINE_SERIALIZABLE(CCamera)
   protected:
	float m_pointingX{0}, m_pointingY{0}, m_pointingZ{0};
	float m_distanceZoom{10};
	float m_azimuthDeg{45}, m_elevationDeg{45};

	/** If set to true (default), camera model is projective, otherwise, it's
	 * orthogonal. */
	bool m_projectiveModel{true};
	/** Field-of-View in degs, only when projectiveModel=true (default=30 deg).
	 */
	float m_projectiveFOVdeg{30};
	/** If set to true, camera pose is used when rendering the viewport */
	bool m_6DOFMode{false};

   public:
	void setPointingAt(float x, float y, float z)
	{
		m_pointingX = x;
		m_pointingY = y;
		m_pointingZ = z;
	}

	template <class POSEORPOINT>
	void setPointingAt(const POSEORPOINT& p)
	{
		m_pointingX = p.x();
		m_pointingY = p.y();
		m_pointingZ = p.is3DPoseOrPoint() ? p.m_coords[2] : 0;
	}
	inline void setPointingAt(const mrpt::math::TPoint3D& p)
	{
		setPointingAt(p.x, p.y, p.z);
	}

	float getPointingAtX() const { return m_pointingX; }
	float getPointingAtY() const { return m_pointingY; }
	float getPointingAtZ() const { return m_pointingZ; }
	void setZoomDistance(float z) { m_distanceZoom = z; }
	float getZoomDistance() const { return m_distanceZoom; }
	float getAzimuthDegrees() const { return m_azimuthDeg; }
	float getElevationDegrees() const { return m_elevationDeg; }
	void setAzimuthDegrees(float ang) { m_azimuthDeg = ang; }
	void setElevationDegrees(float ang) { m_elevationDeg = ang; }
	/** Enable/Disable projective mode (vs. orthogonal) */
	void setProjectiveModel(bool v = true) { m_projectiveModel = v; }
	/** Enable/Disable orthogonal mode (vs. projective) */
	void setOrthogonal(bool v = true) { m_projectiveModel = !v; }
	/** Set 6DOFMode, if enabled camera is set according to its pose
	 *(default=false).
	 *  Conventionally, eye is set looking towards the positive direction of Z
	 *axis.
	 *  Up is set as the Y axis.
	 *  In this mode azimuth/elevation are ignored.
	 **/
	void set6DOFMode(bool v) { m_6DOFMode = v; }
	bool isProjective() const { return m_projectiveModel; }
	bool isOrthogonal() const { return !m_projectiveModel; }
	bool is6DOFMode() const { return m_6DOFMode; }
	/** Field-of-View in degs, only when projectiveModel=true (default=30 deg).
	 */
	void setProjectiveFOVdeg(float ang) { m_projectiveFOVdeg = ang; }
	/** Field-of-View in degs, only when projectiveModel=true (default=30 deg).
	 */
	float getProjectiveFOVdeg() const { return m_projectiveFOVdeg; }
	/** Render does nothing here. */
	void render() const override {}
	/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor
	 */
	CCamera();

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CCamera() override = default;
};

}  // namespace mrpt::opengl
