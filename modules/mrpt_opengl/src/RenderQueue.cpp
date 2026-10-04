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

#include <mrpt/opengl/RenderQueue.h>

#include <algorithm>

using namespace mrpt::opengl;

void RenderQueue::sort()
{
  std::stable_sort(
      opaque.begin(), opaque.end(),
      [](const RenderQueueElement& a, const RenderQueueElement& b)
      { return a.shader != b.shader ? a.shader < b.shader : a.depth < b.depth; });
  std::stable_sort(
      background.begin(), background.end(),
      [](const RenderQueueElement& a, const RenderQueueElement& b) { return a.shader < b.shader; });
  std::stable_sort(
      transparent.begin(), transparent.end(),
      [](const RenderQueueElement& a, const RenderQueueElement& b) { return a.depth > b.depth; });
}

bool mrpt::opengl::boxIntersectsClipVolume(
    const mrpt::math::TBoundingBoxf& bb, const mrpt::math::CMatrixFloat44& M, unsigned int planes)
{
  // Clip-space coordinates (x,y,z,w) of the 8 box corners. Each plane test is
  // linear in homogeneous coordinates, so corners behind the camera (w<0) need
  // no special handling.
  unsigned int outsideAll = planes;
  for (int i = 0; i < 8; i++)
  {
    const float px = (i & 1) ? bb.max.x : bb.min.x;
    const float py = (i & 2) ? bb.max.y : bb.min.y;
    const float pz = (i & 4) ? bb.max.z : bb.min.z;
    const float x = M(0, 0) * px + M(0, 1) * py + M(0, 2) * pz + M(0, 3);
    const float y = M(1, 0) * px + M(1, 1) * py + M(1, 2) * pz + M(1, 3);
    const float z = M(2, 0) * px + M(2, 1) * py + M(2, 2) * pz + M(2, 3);
    const float w = M(3, 0) * px + M(3, 1) * py + M(3, 2) * pz + M(3, 3);
    unsigned int out = 0;
    if (x < -w)
    {
      out |= 1;
    }
    if (x > w)
    {
      out |= 2;
    }
    if (y < -w)
    {
      out |= 4;
    }
    if (y > w)
    {
      out |= 8;
    }
    if (z < -w)
    {
      out |= 16;
    }
    if (z > w)
    {
      out |= 32;
    }
    outsideAll &= out;
    if (outsideAll == 0)
    {
      return true;
    }
  }
  return outsideAll == 0;
}
