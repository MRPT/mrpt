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

#include <mrpt/opengl/SkyBoxProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CSkyBox.h>

using namespace mrpt::opengl;
using namespace mrpt::viz;

// Unit cube vertices used for skybox rendering.
// 36 vertices × 3 floats = 108 floats.  Each position is also the cube-map
// lookup direction (swizzled in the vertex shader as vec3(x, z, y)).
// Taken from the mrpt v2 CSkyBox implementation.
static constexpr float s_skyboxVertices[108] = {
    // Back face
    -1.0f, 1.0f, -1.0f,   //
    -1.0f, -1.0f, -1.0f,  //
    1.0f, -1.0f, -1.0f,   //
    1.0f, -1.0f, -1.0f,   //
    1.0f, 1.0f, -1.0f,    //
    -1.0f, 1.0f, -1.0f,   //

    // Left face
    -1.0f, -1.0f, 1.0f,   //
    -1.0f, -1.0f, -1.0f,  //
    -1.0f, 1.0f, -1.0f,   //
    -1.0f, 1.0f, -1.0f,   //
    -1.0f, 1.0f, 1.0f,    //
    -1.0f, -1.0f, 1.0f,   //

    // Right face
    1.0f, -1.0f, -1.0f,  //
    1.0f, -1.0f, 1.0f,   //
    1.0f, 1.0f, 1.0f,    //
    1.0f, 1.0f, 1.0f,    //
    1.0f, 1.0f, -1.0f,   //
    1.0f, -1.0f, -1.0f,  //

    // Front face
    -1.0f, -1.0f, 1.0f,  //
    -1.0f, 1.0f, 1.0f,   //
    1.0f, 1.0f, 1.0f,    //
    1.0f, 1.0f, 1.0f,    //
    1.0f, -1.0f, 1.0f,   //
    -1.0f, -1.0f, 1.0f,  //

    // Top face
    -1.0f, 1.0f, -1.0f,  //
    1.0f, 1.0f, -1.0f,   //
    1.0f, 1.0f, 1.0f,    //
    1.0f, 1.0f, 1.0f,    //
    -1.0f, 1.0f, 1.0f,   //
    -1.0f, 1.0f, -1.0f,  //

    // Bottom face
    -1.0f, -1.0f, -1.0f,  //
    -1.0f, -1.0f, 1.0f,   //
    1.0f, -1.0f, -1.0f,   //
    1.0f, -1.0f, -1.0f,   //
    -1.0f, -1.0f, 1.0f,   //
    1.0f, -1.0f, 1.0f     //
};

void SkyBoxProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  uploadCubeVertices();
  updateCubeTexture(sourceObj);

  MRPT_END
#endif
}

void SkyBoxProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  updateCubeTexture(sourceObj);

  MRPT_END
#endif
}

void SkyBoxProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!m_textureLoaded || !m_cubeTexture) return;
  if (!rc.shader || !rc.state) return;

  // Upload v_matrix_no_translation (view rotation only, no translation,
  // so the skybox appears stationary as the camera orbits).
  if (rc.shader->hasUniform("v_matrix_no_translation"))
  {
    uploadMatrix(rc, "v_matrix_no_translation", rc.state->v_matrix_no_translation);
  }

  // Bind cube map texture and tell the shader which unit to sample from
  const int texUnit = m_cubeTexture->textureUnit();
  glActiveTexture(GL_TEXTURE0 + texUnit);
  m_cubeTexture->bindAsCubeTexture();
  if (rc.shader->hasUniform("skybox"))
  {
    uploadInt(rc, "skybox", texUnit);
  }

  // Skybox depth trick: render at depth = 1.0 (the far plane) so it
  // appears behind every other object.
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);

  m_vao.bind();
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0);

  // Restore state
  glDepthFunc(GL_LESS);
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void SkyBoxProxy::uploadCubeVertices()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  m_vao.createOnce();
  m_vao.bind();

  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(s_skyboxVertices, sizeof(s_skyboxVertices));

  // layout(location = 0) in vec3 position;
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}

void SkyBoxProxy::updateCubeTexture(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!sourceObj) return;

  const auto* skybox = dynamic_cast<const mrpt::viz::CSkyBox*>(sourceObj);
  if (!skybox) return;

  const auto& imgs = skybox->getTextureImages();

  // All 6 faces must be assigned before uploading
  for (const auto& img : imgs)
  {
    if (img.isEmpty()) return;
  }

  if (!m_cubeTexture)
  {
    m_cubeTexture = std::make_unique<Texture>();
  }
  else if (m_cubeTexture->initialized())
  {
    m_cubeTexture->unloadTexture();
  }

  m_cubeTexture->assignCubeImages(imgs, 0 /*textureUnit*/);
  m_textureLoaded = true;

  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}
