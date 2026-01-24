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

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/RenderableProxy.h>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::poses;

/** Helper: Transform a point from local to eye space */
static TPoint3Df toEyeSpace(
    const TPoint3Df& localPt, const CPose3D& objPose, const TRenderMatrices& state)
{
  // Transform to world space
  TPoint3D worldPt;
  objPose.composePoint(localPt.x, localPt.y, localPt.z, worldPt.x, worldPt.y, worldPt.z);

  // Transform to eye space using view matrix
  // The view matrix transforms world coordinates to eye coordinates
  // Eye space: camera at origin, looking down -Z axis
  const auto& V = state.v_matrix;

  // Apply view matrix (V is row-major in MRPT)
  const float ex = V(0, 0) * worldPt.x + V(0, 1) * worldPt.y + V(0, 2) * worldPt.z + V(0, 3);
  const float ey = V(1, 0) * worldPt.x + V(1, 1) * worldPt.y + V(1, 2) * worldPt.z + V(1, 3);
  const float ez = V(2, 0) * worldPt.x + V(2, 1) * worldPt.y + V(2, 2) * worldPt.z + V(2, 3);

  return TPoint3Df(ex, ey, ez);
}

/** Helper: Check if a point is inside the view frustum (in clip space) */
static bool isPointInFrustum(const TPoint3Df& eyePt, const TRenderMatrices& state)
{
  // Apply projection matrix to get clip coordinates
  const auto& P = state.p_matrix;

  const float cx = P(0, 0) * eyePt.x + P(0, 1) * eyePt.y + P(0, 2) * eyePt.z + P(0, 3);
  const float cy = P(1, 0) * eyePt.x + P(1, 1) * eyePt.y + P(1, 2) * eyePt.z + P(1, 3);
  const float cz = P(2, 0) * eyePt.x + P(2, 1) * eyePt.y + P(2, 2) * eyePt.z + P(2, 3);
  const float cw = P(3, 0) * eyePt.x + P(3, 1) * eyePt.y + P(3, 2) * eyePt.z + P(3, 3);

  // Point is inside frustum if: -w <= x,y,z <= w
  // (after perspective division this becomes [-1,1] in NDC)
  if (cw <= 0) return false;  // Behind camera

  return (cx >= -cw && cx <= cw && cy >= -cw && cy <= cw && cz >= -cw && cz <= cw);
}

/** Helper: Check if a bounding box intersects the view frustum */
static bool bboxIntersectsFrustum(
    const TBoundingBoxf& bbox, const CPose3D& objPose, const TRenderMatrices& state)
{
  // Get the 8 corners of the bounding box
  const auto& minPt = bbox.min;
  const auto& maxPt = bbox.max;

  // Transform each corner to eye space and check if any is inside frustum
  // This is a conservative test - if any corner is inside, the bbox might be visible
  TPoint3Df corners[8] = {
      {minPt.x, minPt.y, minPt.z},
      {maxPt.x, minPt.y, minPt.z},
      {minPt.x, maxPt.y, minPt.z},
      {maxPt.x, maxPt.y, minPt.z},
      {minPt.x, minPt.y, maxPt.z},
      {maxPt.x, minPt.y, maxPt.z},
      {minPt.x, maxPt.y, maxPt.z},
      {maxPt.x, maxPt.y, maxPt.z}
  };

  bool anyInside = false;
  bool allInside = true;

  for (const auto& corner : corners)
  {
    TPoint3Df eyeCorner = toEyeSpace(corner, objPose, state);
    bool inside = isPointInFrustum(eyeCorner, state);
    anyInside = anyInside || inside;
    allInside = allInside && inside;
  }

  // If any corner is inside, the bbox is at least partially visible
  // For a more accurate test, we'd need to check if the frustum planes
  // intersect the bbox, but this conservative test is usually sufficient
  return anyInside;
}

std::tuple<double, bool, bool> mrpt::opengl::depthAndVisibleInView(
    const RenderableProxy* proxy,
    const TRenderMatrices& objState,
    const CPose3D& objPose,
    bool skipCullChecks)
{
  // Default result: visible, not fully visible, depth 0
  if (!proxy) return {0.0, false, false};

  // Get bounding box in local coordinates
  const TBoundingBoxf localBBox = proxy->getBoundingBoxLocal();

  // Check if object should be culled
  if (!proxy->cullEligible())
  {
    // Object doesn't participate in culling (e.g., skybox)
    skipCullChecks = true;
  }

  // Compute representative point for depth sorting
  // Use center of bounding box, or origin if bbox is empty
  TPoint3Df representativePoint;
  if (localBBox.volume() > 0)
  {
    representativePoint = TPoint3Df(
        (localBBox.min.x + localBBox.max.x) * 0.5f, (localBBox.min.y + localBBox.max.y) * 0.5f,
        (localBBox.min.z + localBBox.max.z) * 0.5f);
  }
  else
  {
    representativePoint = TPoint3Df(0, 0, 0);
  }

  // Transform representative point to eye space
  TPoint3Df eyePoint = toEyeSpace(representativePoint, objPose, objState);

  // Depth is the negative Z in eye space (camera looks down -Z)
  // More negative Z = further from camera = larger depth value
  const double depth = -eyePoint.z;

  // Check visibility
  bool visible = true;
  bool fullyVisible = true;

  if (!skipCullChecks && localBBox.volume() > 0)
  {
    // Perform frustum culling
    visible = bboxIntersectsFrustum(localBBox, objPose, objState);

    // Check if fully visible (all corners inside frustum)
    if (visible)
    {
      const auto& minPt = localBBox.min;
      const auto& maxPt = localBBox.max;

      TPoint3Df corners[8] = {
          {minPt.x, minPt.y, minPt.z},
          {maxPt.x, minPt.y, minPt.z},
          {minPt.x, maxPt.y, minPt.z},
          {maxPt.x, maxPt.y, minPt.z},
          {minPt.x, minPt.y, maxPt.z},
          {maxPt.x, minPt.y, maxPt.z},
          {minPt.x, maxPt.y, maxPt.z},
          {maxPt.x, maxPt.y, maxPt.z}
      };

      fullyVisible = true;
      for (const auto& corner : corners)
      {
        TPoint3Df eyeCorner = toEyeSpace(corner, objPose, objState);
        if (!isPointInFrustum(eyeCorner, objState))
        {
          fullyVisible = false;
          break;
        }
      }
    }
    else
    {
      fullyVisible = false;
    }
  }

  return {depth, visible, fullyVisible};
}
