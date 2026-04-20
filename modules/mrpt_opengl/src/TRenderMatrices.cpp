/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/containers/yaml.h>
#include <mrpt/math/geometry.h>        // crossProduct3D()
#include <mrpt/math/ops_containers.h>  // dotProduct()
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/viz/TLightParameters.h>

#include <Eigen/Dense>
#include <cmath>  // atan2

using namespace mrpt::opengl;

mrpt::math::CMatrixFloat44 TRenderMatrices::OrthoProjectionMatrix(
    float left, float right, float bottom, float top, float znear, float zfar)
{
  ASSERT_GT_(zfar, znear);

  mrpt::math::CMatrixFloat44 p = mrpt::math::CMatrixFloat44::Identity();

  p(0, 0) = 2.0f / (right - left);
  p(1, 1) = 2.0f / (top - bottom);
  p(2, 2) = -2.0f / (zfar - znear);
  p(0, 3) = -(right + left) / (right - left);
  p(1, 3) = -(top + bottom) / (top - bottom);
  p(2, 3) = -(zfar + znear) / (zfar - znear);

  return p;
}

void TRenderMatrices::computeOrthoProjectionMatrix(
    float left, float right, float bottom, float top, float znear, float zfar)
{
  m_last_z_near = znear;
  m_last_z_far = zfar;

  p_matrix = OrthoProjectionMatrix(left, right, bottom, top, znear, zfar);
}

void TRenderMatrices::computeNoProjectionMatrix(float znear, float zfar)
{
  ASSERT_GT_(zfar, znear);
  m_last_z_near = znear;
  m_last_z_far = zfar;

  p_matrix.setIdentity();
  v_matrix.setIdentity();
}

// Replacement for obsolete: gluPerspective() and glOrtho()
void TRenderMatrices::computeProjectionMatrix(float znear, float zfar)
{
  ASSERT_GT_(FOV, .0f);
  ASSERT_GT_(zfar, znear);
  ASSERT_GT_(zfar, .0f);
  ASSERT_GE_(znear, .0f);

  m_last_z_near = znear;
  m_last_z_far = zfar;

  if (pinhole_model.has_value())
  {
    const auto& phm = pinhole_model.value();

    // Equivalent to gluPerspective(), from pinhole camera intrinsic
    // parameters (cx,cy,fx,fy):
    ASSERT_EQUAL_(viewport_width, pinhole_model->ncols);
    ASSERT_EQUAL_(viewport_height, pinhole_model->nrows);

    const float W = static_cast<float>(pinhole_model->ncols);
    const float H = static_cast<float>(pinhole_model->nrows);

    // See: e.g.
    // http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
    mrpt::math::CMatrixFloat44 persp;
    persp.setZero();

    persp(0, 0) = phm.fx();
    persp(1, 1) = phm.fy();

    persp(0, 2) = -phm.cx();
    persp(1, 2) = -H + phm.cy();
    persp(2, 2) = (zfar + znear);
    persp(3, 2) = -1.0f;
    persp(2, 3) = zfar * znear;

    // glOrtho(-W/2, W/2, -H/2, H/2, near, far);

    computeOrthoProjectionMatrix(0, W, 0 /*bottom*/, H /*top*/, znear, zfar);

    // glMultMatrix(persp);
    p_matrix.asEigen() *= persp.asEigen();
  }
  else if (is_projective)
  {
    // Was: gluPerspective()
    // Based on GLM's perspective (MIT license).

    const float aspect = viewport_width / (1.0f * viewport_height);
    ASSERT_GT_(std::abs(aspect - std::numeric_limits<float>::epsilon()), .0f);

    const float f = 1.0f / std::tan(mrpt::DEG2RAD(FOV) / 2.0f);
    p_matrix.setZero();

    p_matrix(0, 0) = f / aspect;
    p_matrix(1, 1) = f;
    p_matrix(2, 2) = -(zfar + znear) / (zfar - znear);
    p_matrix(3, 2) = -1.0f;
    p_matrix(2, 3) = -(2.0f * zfar * znear) / (zfar - znear);
  }
  else
  {
    // Was:
    // glOrtho(-Ax, Ax, -Ay, Ay, -0.5 * m_clip_max, 0.5 * m_clip_max);

    const float ratio = viewport_width / (1.0f * viewport_height);
    float Ax = eyeDistance * 0.5f;
    float Ay = eyeDistance * 0.5f;

    if (ratio > 1)
    {
      Ax *= ratio;
    }
    else
    {
      if (ratio != 0)
      {
        Ay /= ratio;
      }
    }

    const auto left = -.5f * Ax, right = .5f * Ax;
    const auto bottom = -.5f * Ay, top = .5f * Ay;
    computeOrthoProjectionMatrix(left, right, bottom, top, znear, zfar);
  }
}

namespace
{
void azimuthElevationFromDirection(
    const mrpt::math::TVector3Df& v, float& elevation, float& azimuth)
{
  // Compute the elevation angle
  elevation = std::atan2(v.z, sqrt(v.x * v.x + v.y * v.y));

  // Compute the azimuth angle
  if (v.x == 0 && v.y == 0)
  {
    azimuth = 0;
  }
  else
  {
    azimuth = std::atan2(v.y, v.x);
  }
}
}  // namespace

void TRenderMatrices::computeLightProjectionMatrix(
    float zmin, float zmax, const mrpt::viz::TLightParameters& lp)
{
  m_last_light_z_near = zmin;
  m_last_light_z_far = zmax;

  float dist = eyeDistance * lp.eyeDistance2lightShadowExtension;

  // Ensure dist is not too small:
  mrpt::keep_max(dist, zmax * lp.minimum_shadow_map_extension_ratio);

  light_p = OrthoProjectionMatrix(-dist, dist, -dist, dist, zmin, zmax);

  // "up" vector from elevation:

  const auto dir = lp.primaryDirectionalDirection();

  float azim = 0, elevation = 0;
  azimuthElevationFromDirection(dir, elevation, azim);

  const auto lightUp = mrpt::math::TVector3Df(
      -cos(azim) * sin(elevation),  // x
      -sin(azim) * sin(elevation),  // y
      cos(elevation)                // z
  );

  light_v = LookAt(pointing - dir * zmax * 0.5, pointing, lightUp);

  light_pv.asEigen() = light_p.asEigen() * light_v.asEigen();

  // light_pmv is updated in RenderQueue
}

void TRenderMatrices::computeCascadedLightProjectionMatrices(
    float zmin, float zmax, const mrpt::viz::TLightParameters& lp, unsigned int shadowMapSize)
{
  const int N = std::clamp<int>(lp.shadow_cascades, 1, 4);
  numShadowCascades = N;
  const float lambda = lp.shadow_cascade_lambda;

  // Ensure zmin > 0 for logarithmic splits
  const float nearClip = std::max(zmin, 0.01f);

  // Compute PSSM split distances (view-space)
  std::array<float, 5> splits;  // N+1 boundaries
  splits[0] = nearClip;
  for (int i = 1; i <= N; i++)
  {
    const float f = static_cast<float>(i) / N;
    const float cLog = nearClip * std::pow(zmax / nearClip, f);
    const float cUni = nearClip + (zmax - nearClip) * f;
    splits[i] = lambda * cLog + (1.0f - lambda) * cUni;
  }
  // Record far planes for each cascade (view-space distance)
  for (int i = 0; i < N; i++) cascade_far_planes[i] = splits[i + 1];

  // Light direction and up vector (shared by all cascades)
  const auto dir = lp.primaryDirectionalDirection();
  float azim = 0, elevation = 0;
  azimuthElevationFromDirection(dir, elevation, azim);
  const auto lightUp = mrpt::math::TVector3Df(
      -cos(azim) * sin(elevation), -sin(azim) * sin(elevation), cos(elevation));

  // For each cascade, compute a tight ortho frustum around the view sub-frustum
  for (int c = 0; c < N; c++)
  {
    const float cascNear = splits[c];
    const float cascFar = splits[c + 1];

    // Compute the 8 corners of the view sub-frustum in world space
    // We use the inverse of P*V to unproject NDC corners
    const auto invPV = (p_matrix.asEigen() * v_matrix.asEigen()).inverse();

    // NDC corners: near and far planes at z=-1 and z=+1
    const float zNDCNear =
        2.0f * (cascNear - m_last_z_near) / (m_last_z_far - m_last_z_near) - 1.0f;
    const float zNDCFar = 2.0f * (cascFar - m_last_z_near) / (m_last_z_far - m_last_z_near) - 1.0f;

    // Actually, it's easier to use linearized depth. For a perspective
    // projection, NDC z = (A*z + B) / z where A and B come from p_matrix.
    // Let's just compute corners using the actual projection matrix.
    float zNear_ndc, zFar_ndc;
    if (is_projective)
    {
      // p_matrix(2,2) = -(f+n)/(f-n), p_matrix(2,3) = -2fn/(f-n)
      const float A = p_matrix(2, 2);
      const float B = p_matrix(2, 3);
      // NDC z = (A*z_eye + B) / (-z_eye)  (note: eye-space z is negative)
      zNear_ndc = (A * (-cascNear) + B) / cascNear;
      zFar_ndc = (A * (-cascFar) + B) / cascFar;
    }
    else
    {
      // Ortho: linear mapping
      zNear_ndc = 2.0f * (cascNear - m_last_z_near) / (m_last_z_far - m_last_z_near) - 1.0f;
      zFar_ndc = 2.0f * (cascFar - m_last_z_near) / (m_last_z_far - m_last_z_near) - 1.0f;
    }

    Eigen::Vector4f corners[8];
    int idx = 0;
    for (float z : {zNear_ndc, zFar_ndc})
    {
      for (float x : {-1.0f, 1.0f})
      {
        for (float y : {-1.0f, 1.0f})
        {
          Eigen::Vector4f ndc(x, y, z, 1.0f);
          Eigen::Vector4f world = invPV * ndc;
          corners[idx++] = world / world.w();
        }
      }
    }

    // Compute light view matrix centered on the sub-frustum
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (int i = 0; i < 8; i++) center += corners[i].head<3>();
    center /= 8.0f;

    // Compute the radius of the sub-frustum bounding sphere to position
    // the light far enough back to capture all shadow casters.
    float maxRadius = 0;
    for (int i = 0; i < 8; i++)
      maxRadius = std::max(maxRadius, (corners[i].head<3>() - center).norm());

    const float lightBackOffset = maxRadius * 2.0f;

    const auto lightViewMat = LookAt(
        {center.x() - dir.x * lightBackOffset, center.y() - dir.y * lightBackOffset,
         center.z() - dir.z * lightBackOffset},
        {center.x(), center.y(), center.z()}, lightUp);

    // Transform corners to light view space and find AABB
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = minX, maxY = maxX, minZ = minX, maxZ = maxX;
    for (int i = 0; i < 8; i++)
    {
      Eigen::Vector4f lv = lightViewMat.asEigen() * corners[i];
      minX = std::min(minX, lv.x());
      maxX = std::max(maxX, lv.x());
      minY = std::min(minY, lv.y());
      maxY = std::max(maxY, lv.y());
      minZ = std::min(minZ, lv.z());
      maxZ = std::max(maxZ, lv.z());
    }

    // Use the bounding sphere radius for X/Y extents. This makes the
    // ortho frustum size rotationally stable (same regardless of camera
    // orientation), which is essential for texel snapping to prevent
    // shadow edge flickering on camera rotation. Z range stays tight (AABB).
    const Eigen::Vector4f lightCenter =
        lightViewMat.asEigen() * Eigen::Vector4f(center.x(), center.y(), center.z(), 1.0f);
    const float frustumSize = maxRadius * 2.0f;
    minX = lightCenter.x() - maxRadius;
    maxX = lightCenter.x() + maxRadius;
    minY = lightCenter.y() - maxRadius;
    maxY = lightCenter.y() + maxRadius;

    // Texel snapping: snap the ortho frustum origin to the nearest shadow
    // map texel boundary. This prevents shadow edges from "swimming" as the
    // camera translates, since the shadow map texel grid stays fixed in
    // world space. Combined with the stable sphere-based size above, this
    // eliminates flickering from both translation and rotation.
    if (shadowMapSize > 0)
    {
      const float worldTexelSize = frustumSize / static_cast<float>(shadowMapSize);
      minX = std::floor(minX / worldTexelSize) * worldTexelSize;
      maxX = minX + frustumSize;
      minY = std::floor(minY / worldTexelSize) * worldTexelSize;
      maxY = minY + frustumSize;
    }

    // Convert from view-space z (negative in front of camera) to positive
    // near/far distances for OrthoProjectionMatrix, and add padding to
    // capture shadow casters outside the view sub-frustum.
    const float extent = -minZ + maxZ;
    float nearZ = std::max(-maxZ - extent * 0.1f, 0.01f);
    float farZ = -minZ + extent * 0.5f;

    const auto lightProjMat = OrthoProjectionMatrix(minX, maxX, minY, maxY, nearZ, farZ);

    cascade_light_pv[c].asEigen() = lightProjMat.asEigen() * lightViewMat.asEigen();
  }

  // Keep legacy light_pv pointing to cascade 0 for compatibility
  light_pv = cascade_light_pv[0];
}

// Replacement for deprecated OpenGL gluLookAt():
mrpt::math::CMatrixFloat44 TRenderMatrices::LookAt(
    const mrpt::math::TVector3D& lookFrom,
    const mrpt::math::TVector3D& lookAt,
    const mrpt::math::TVector3D& up,
    mrpt::math::CMatrixFloat44* viewWithoutTranslation)
{
  using mrpt::math::TVector3D;

  // Note: Use double instead of float to avoid numerical innacuracies that
  // are really noticeable with the naked eye when elevation is close to 90
  // deg (!)
  TVector3D forward = TVector3D(lookAt - lookFrom);

  const double fn = forward.norm();
  ASSERT_(fn != 0);
  forward *= 1.0 / fn;

  // Side = forward x up
  TVector3D side = mrpt::math::crossProduct3D(forward, up);
  const double sn = side.norm();
  ASSERT_(sn != 0);
  side *= 1.0 / sn;

  // Recompute up as: up = side x forward
  const TVector3D up2 = mrpt::math::crossProduct3D(side, forward);

  //  s.x   s.y   s.z  -dot(s, eye)
  //  u.x   u.y   u.z  -dot(u, eye)
  // -f.x  -f.y  -f.z  dot(up, eye)
  //   0     0     0      1

  mrpt::math::CMatrixFloat44 m(mrpt::math::UNINITIALIZED_MATRIX);
  // Axis X:
  m(0, 0) = d2f(side[0]);
  m(0, 1) = d2f(side[1]);
  m(0, 2) = d2f(side[2]);
  // Axis Y:
  m(1, 0) = d2f(up2[0]);
  m(1, 1) = d2f(up2[1]);
  m(1, 2) = d2f(up2[2]);
  // Axis Z:
  m(2, 0) = d2f(-forward[0]);
  m(2, 1) = d2f(-forward[1]);
  m(2, 2) = d2f(-forward[2]);
  // Last row:
  m(3, 0) = .0f;
  m(3, 1) = .0f;
  m(3, 2) = .0f;
  m(3, 3) = 1.f;

  if (viewWithoutTranslation) *viewWithoutTranslation = m;

  // Translation:
  m(0, 3) = d2f(-mrpt::math::dotProduct<3, double>(side, lookFrom));
  m(1, 3) = d2f(-mrpt::math::dotProduct<3, double>(up2, lookFrom));
  m(2, 3) = d2f(mrpt::math::dotProduct<3, double>(forward, lookFrom));

  return m;
}

// Replacement for deprecated OpenGL gluLookAt():
void TRenderMatrices::computeViewMatrix()
{
  v_matrix = LookAt(eye, pointing, up, &v_matrix_no_translation);
}

void TRenderMatrices::projectPoint(
    float x, float y, float z, float& proj_u, float& proj_v, float& proj_z_depth) const
{
  const Eigen::Matrix<float, 4, 1, Eigen::ColMajor> proj =
      pmv_matrix.asEigen() * Eigen::Matrix<float, 4, 1, Eigen::ColMajor>(x, y, z, 1);
  proj_u = proj[3] ? proj[0] / proj[3] : 0;
  proj_v = proj[3] ? proj[1] / proj[3] : 0;
  proj_z_depth = proj[2];
}

void TRenderMatrices::projectPointPixels(
    float x, float y, float z, float& proj_u_px, float& proj_v_px, float& proj_depth) const
{
  projectPoint(x, y, z, proj_u_px, proj_v_px, proj_depth);
  proj_u_px = (proj_u_px + 1.0f) * (viewport_width * 0.5f);
  proj_v_px = (proj_v_px + 1.0f) * (viewport_height * 0.5f);
}

void TRenderMatrices::saveToYaml(mrpt::containers::yaml& c) const
{
  c = mrpt::containers::yaml::Map();

  MCP_SAVE(c, initialized);
  MCP_SAVE(c, viewport_width);
  MCP_SAVE(c, viewport_height);
  MCP_SAVE(c, FOV);
  MCP_SAVE_DEG(c, azimuth);
  MCP_SAVE_DEG(c, elev);
  MCP_SAVE(c, eyeDistance);
  MCP_SAVE(c, is_projective);

  c["eye"] = eye.asString();
  c["pointing"] = pointing.asString();
  c["up"] = up.asString();

  c["p_matrix"] = mrpt::containers::yaml::FromMatrix(p_matrix);
  c["v_matrix"] = mrpt::containers::yaml::FromMatrix(v_matrix);
  c["m_matrix"] = mrpt::containers::yaml::FromMatrix(m_matrix);
  c["light_p_matrix"] = mrpt::containers::yaml::FromMatrix(light_p);
  c["light_v_matrix"] = mrpt::containers::yaml::FromMatrix(light_v);
}

void TRenderMatrices::print(std::ostream& o) const
{
  mrpt::containers::yaml c;
  saveToYaml(c);
  o << c;
}
