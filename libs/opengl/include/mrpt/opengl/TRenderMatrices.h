/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

namespace mrpt::opengl
{
/** Rendering state related to the projection and model-view matrices.
 * Used to store matrices that will be sent to shaders.
 *
 * The homogeneous coordinates of a rendered point comes from:
 *
 *  p = p_matrix * mv_matrix * [x y z 1.0]'
 *
 * \ingroup mrpt_opengl_grp
 */
struct TRenderMatrices
{
	TRenderMatrices() = default;

	/** Is set to true by  COpenGLViewport::updateMatricesFromCamera() */
	bool initialized = false;

	/** The camera is here. */
	mrpt::math::TPoint3D eye = {0, 0, 0};

	/** The camera points to here */
	mrpt::math::TPoint3D pointing = {0, 0, 0};

	/** Up vector of the camera. */
	mrpt::math::TPoint3D up = {0, 0, 0};

	/** In pixels. This may be smaller than the total render window. */
	uint32_t viewport_width = 640, viewport_height = 480;

	/** Vertical FOV in degrees, used only if pinhole_model is not set. */
	double FOV = 30.0f;

	/** Use the intrinsics (cx,cy,fx,fy) from this model instead of FOV, if
	 * defined. */
	std::optional<mrpt::img::TCamera> pinhole_model;

	/** Camera elev & azimuth, in radians. */
	double azimuth = .0, elev = .0;
	double eyeDistance = 1.0f;

	/** true: projective, false: ortho */
	bool is_projective = true;

	/** Projection matrix, computed by renderNormalScene() from all the
	 * parameters above. Used in shaders. */
	mrpt::math::CMatrixFloat44 p_matrix;

	/** Model-view matrix. */
	mrpt::math::CMatrixFloat44 mv_matrix;

	/** Result of p_matrix * mv_matrix. Used in shaders.
	 * Updated by renderSetOfObjects()
	 */
	mrpt::math::CMatrixFloat44 pmv_matrix;

	/** Uses is_projective , vw,vh, etc. and computes p_matrix from either:
	 *  - pinhole_model if set, or
	 *  - FOV, otherwise.
	 * Replacement for obsolete: gluPerspective() and glOrtho() */
	void computeProjectionMatrix(float zmin, float zmax);

	/** Especial case for custom parameters of Orthographic projection.
	 * Replacement for obsolete: glOrtho()*/
	void computeOrthoProjectionMatrix(
		float left, float right, float bottom, float top, float znear,
		float zfar);

	/** Updates the current p_matrix such that it "looks at" pointing, with
	 * up vector "up". Replacement for deprecated OpenGL gluLookAt(). */
	void applyLookAt();

	/** Computes the normalized coordinates (range=[0,1]) on the current
	 * rendering viewport of a
	 * point with local coordinates (wrt to the current model matrix) of
	 * (x,y,z).
	 *  The output proj_z_depth is the real distance from the eye to the point.
	 */
	void projectPoint(
		float x, float y, float z, float& proj_u, float& proj_v,
		float& proj_z_depth) const;

	/** Projects a point from global world coordinates into (u,v) pixel
	 * coordinates. */
	void projectPointPixels(
		float x, float y, float z, float& proj_u_px, float& proj_v_px,
		float& proj_depth) const;

	float getLastClipZNear() const { return m_last_z_near; }
	float getLastClipZFar() const { return m_last_z_far; }

	void saveToYaml(mrpt::containers::yaml& c) const;
	void print(std::ostream& o) const;

   private:
	float m_last_z_near = 0, m_last_z_far = 0;
};

}  // namespace mrpt::opengl
