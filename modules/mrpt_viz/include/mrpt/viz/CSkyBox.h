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

#include <mrpt/img/CImage.h>
#include <mrpt/viz/CUBE_TEXTURE_FACE.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TTriangle.h>

namespace mrpt::viz
{
/** A Sky Box: 6 textures that are always rendered at "infinity" to give the
 *  impression of the scene to be much larger.
 *
 * Refer to example \ref opengl_skybox_example
 *
 *  <img src="mrpt-skybox-demo.gif" />
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CSkyBox : public CVisualObject
{
  DEFINE_SERIALIZABLE(CSkyBox, mrpt::viz)

 public:
  CSkyBox() = default;
  virtual ~CSkyBox() override;

  /** Assigns a texture. It is mandatory to assign all 6 faces before
   * initializing/rendering the texture.
   *
   * \note Images are copied, the original ones can be deleted.
   */
  void assignImage(const CUBE_TEXTURE_FACE face, const mrpt::img::CImage& img);

  /// \overload with move semantics for the image.
  void assignImage(const CUBE_TEXTURE_FACE face, img::CImage&& img);

  auto internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf override;

  bool cullElegible() const override { return false; }

 private:
  /// The cube texture for the 6 faces
  std::array<mrpt::img::CImage, 6> m_textureImages;
};

}  // namespace mrpt::viz
