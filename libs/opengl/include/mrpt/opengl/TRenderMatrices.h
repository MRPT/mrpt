/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/opengl_frwds.h>

namespace mrpt::opengl
{
/** Rendering state related to the projection and model-view matrices.
 * Used to store matrices that will be sent to shaders.
 *
 * The homogeneous coordinates of a rendered point comes from the product
 * (from right to left) of MODEL, VIEW and PROJECTION matrices:
 *
 *  p = p_matrix * v_matrix * m_matrix * [x y z 1.0]'
 *
 * \ingroup mrpt_opengl_grp
 */
struct TRenderMatrices
{
  TRenderMatrices() = default;

  /** Projection matrix, computed by Viewport from camera parameters */
  mrpt::math::CMatrixFloat44 p_matrix;

  /** Model matrix. */
  mrpt::math::CMatrixFloat44 m_matrix;

  /** View matrix. */
  mrpt::math::CMatrixFloat44 v_matrix;

  /** View matrix with null translation */
  mrpt::math::CMatrixFloat44 v_matrix_no_translation;

  /** Result of p_matrix * mv_matrix (=P*V*M). Used in shaders.
   * Updated by Viewport::updateMatricesFromCamera()
   */
  mrpt::math::CMatrixFloat44 pmv_matrix;

  /** Result of v_matrix * m_matrix. Used in shaders.
   * Updated by Viewport::updateMatricesFromCamera()
   */
  mrpt::math::CMatrixFloat44 mv_matrix;

  /** Result of p_matrix * v_matrix (=P*V) for the directional light
   * point-of-view. Used in shadow-generation shaders.
   * Updated by Viewport::updateMatricesFromCamera()
   */
  mrpt::math::CMatrixFloat44 light_pv, light_p, light_v, light_pmv;

  bool is1stShadowMapPass = false;

  void matricesSetIdentity()
  {
    p_matrix.setIdentity();
    m_matrix.setIdentity();
    v_matrix.setIdentity();
    v_matrix_no_translation.setIdentity();
    mv_matrix.setIdentity();
    pmv_matrix.setIdentity();
    light_pv.setIdentity();
    light_p.setIdentity();
    light_v.setIdentity();
    light_pmv.setIdentity();
  }

  /** Use the intrinsics (cx,cy,fx,fy) from this model instead of FOV, if
   * defined. */
  std::optional<mrpt::img::TCamera> pinhole_model;

  /** Vertical FOV in degrees, used only if pinhole_model is not set. */
  double FOV = 30.0f;

  /** Camera elev & azimuth, in radians. */
  double azimuth = .0, elev = .0;
  double eyeDistance = 1.0f;

  /** In pixels. This may be smaller than the total render window. */
  uint32_t viewport_width = 640, viewport_height = 480;

 private:
  // (private fields reordered here to minimize padding)
  float m_last_z_near = 0, m_last_z_far = 0;
  float m_last_light_z_near = 0, m_last_light_z_far = 0;

 public:
  /** Is set to true by  Viewport::updateMatricesFromCamera() */
  bool initialized = false;

  /** true: projective, false: ortho */
  bool is_projective = true;

  /** The camera is here. */
  mrpt::math::TPoint3D eye = {0, 0, 0};

  /** The camera points to here */
  mrpt::math::TPoint3D pointing = {0, 0, 0};

  /** Up vector of the camera. */
  mrpt::math::TPoint3D up = {0, 0, 0};

  /** Uses is_projective , vw,vh, etc. and computes p_matrix from either:
   *  - pinhole_model if set, or
   *  - FOV, otherwise.
   * Replacement for obsolete: gluPerspective() and glOrtho() */
  void computeProjectionMatrix(float zmin, float zmax);

  /** Updates light_pv */
  void computeLightProjectionMatrix(float zmin, float zmax, const TLightParameters& lp);

  /** Especial case for custom parameters of Orthographic projection.
   *  Equivalent to `p_matrix = ortho(...);`.
   *
   * Replacement for obsolete: glOrtho()*/
  void computeOrthoProjectionMatrix(
      float left, float right, float bottom, float top, float znear, float zfar);

  /** Computes and returns an orthographic projection matrix.
   *  Equivalent to obsolete glOrtho() or glm::ortho().
   */
  [[nodiscard]] static mrpt::math::CMatrixFloat44 OrthoProjectionMatrix(
      float left, float right, float bottom, float top, float znear, float zfar);

  /** Especial case: no projection, opengl coordinates are pixels from (0,0)
   * bottom-left corner.*/
  void computeNoProjectionMatrix(float znear, float zfar);

  /** Updates v_matrix (and v_matrix_no_translation) using the current
   *  camera position and pointing-to coordinates.
   *  Replacement for deprecated OpenGL gluLookAt(). */
  void computeViewMatrix();

  /** Computes the view matrix from a "forward" and an "up" vector.
   *  Equivalent to obsolete gluLookAt() or glm::lookAt().
   */
  [[nodiscard]] static mrpt::math::CMatrixFloat44 LookAt(
      const mrpt::math::TVector3D& lookFrom,
      const mrpt::math::TVector3D& lookAt,
      const mrpt::math::TVector3D& up,
      mrpt::math::CMatrixFloat44* viewWithoutTranslation = nullptr);

  /** Computes the normalized coordinates (range=[0,1]) on the current
   * rendering viewport of a
   * point with local coordinates (wrt to the current model matrix) of
   * (x,y,z).
   *  The output proj_z_depth is the real distance from the eye to the point.
   */
  void projectPoint(
      float x, float y, float z, float& proj_u, float& proj_v, float& proj_z_depth) const;

  /** Projects a point from global world coordinates into (u,v) pixel
   * coordinates. */
  void projectPointPixels(
      float x, float y, float z, float& proj_u_px, float& proj_v_px, float& proj_depth) const;

  float getLastClipZNear() const { return m_last_z_near; }
  float getLastClipZFar() const { return m_last_z_far; }

  float getLastLightClipZNear() const { return m_last_light_z_near; }
  float getLastLightClipZFar() const { return m_last_light_z_far; }

  void saveToYaml(mrpt::containers::yaml& c) const;
  void print(std::ostream& o) const;
};

}  // namespace mrpt::opengl
