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
#pragma once

#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TLightParameters.h>

namespace mrpt::viz
{
/** A light source (directional, point or spot) placed in the scene graph.
 *
 * Unlike the lights in Viewport::lightParameters(), which are given in world
 * coordinates, the position and direction of this light are relative to the
 * pose of this object, composed with the poses of its parent containers. For
 * example, a headlight inserted into the CSetOfObjects of a vehicle moves
 * with it.
 *
 * The light is switched on and off with setVisibility(), and it is also off
 * while any of its parent containers is hidden. Scaling the object (or its
 * parents) does not change TLight::range.
 *
 * Lights of all visible CLight objects in a viewport are appended to that
 * viewport's own lights. If there are more than MAX_LIGHTS in total, the
 * renderer keeps all directional lights, then the point/spot lights closest
 * to the camera.
 *
 * The object itself is not drawn: insert any 3D model (e.g. with an emissive
 * material) as a sibling to show the lamp.
 *
 * \sa TLight, Viewport::lightParameters()
 * \ingroup mrpt_viz_grp
 */
class CLight : public CVisualObject
{
  DEFINE_SERIALIZABLE(CLight, mrpt::viz)

 public:
  CLight() = default;
  explicit CLight(const TLight& l) : m_light(l) {}
  ~CLight() override = default;

  /** Returns the light parameters, in the local frame of this object. */
  [[nodiscard]] TLight light() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_light;
  }

  /** Changes the light parameters, in the local frame of this object. */
  void light(const TLight& l)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_light = l;
    lckWrite.unlock();
    notifyChange();
  }

  /** The light position (a zero-size box), or an empty box for directional
   * lights. */
  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

 private:
  TLight m_light;
};

}  // namespace mrpt::viz
