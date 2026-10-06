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

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <mrpt/imgui/CImGuiSceneView.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace mrpt::imgui;

// -----------------------------------------------------------------------
// Construction / destruction
// -----------------------------------------------------------------------
CImGuiSceneView::CImGuiSceneView()
{
  // Set up a reasonable default orbit camera:
  m_camera.setProjectiveModel(true);
  m_camera.setProjectiveFOVdeg(45.0f);
  m_camera.setZoomDistance(15.0f);
  m_camera.setAzimuthDegrees(-135.0f);
  m_camera.setElevationDegrees(25.0f);
  m_camera.setPointingAt(0.0f, 0.0f, 0.0f);
}

CImGuiSceneView::~CImGuiSceneView() { destroyFBO(); }

// -----------------------------------------------------------------------
// FBO management — simple color+depth, no MSAA for now
// -----------------------------------------------------------------------
void CImGuiSceneView::ensureFBO(int w, int h)
{
  using namespace std::string_literals;

  if (w <= 0 || h <= 0)
  {
    return;
  }

  if (m_fbo != 0 && m_fboWidth == w && m_fboHeight == h)
  {
    return;  // Already the right size
  }

  // Tear down old resources
  destroyFBO();

  m_fboWidth = w;
  m_fboHeight = h;

#if MRPT_HAS_OPENGL || MRPT_HAS_EGL

  // --- Color texture (this is what ImGui::Image will display) ---
  glGenTextures(1, &m_texColor);
  glBindTexture(GL_TEXTURE_2D, m_texColor);
  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_RGBA8, static_cast<GLsizei>(w), static_cast<GLsizei>(h), 0, GL_RGBA,
      GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  // --- Depth renderbuffer ---
  glGenRenderbuffers(1, &m_rboDepth);
  glBindRenderbuffer(GL_RENDERBUFFER, m_rboDepth);
  glRenderbufferStorage(
      GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // --- Framebuffer object ---
  glGenFramebuffers(1, &m_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texColor, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_rboDepth);

  const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE)
  {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    destroyFBO();
    throw std::runtime_error(
        "[CImGuiSceneView] Failed to create FBO (status="s +
        std::to_string(static_cast<int>(status)) + ")"s);
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

#endif
}

void CImGuiSceneView::destroyFBO()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL

  if (m_fbo != 0)
  {
    glDeleteFramebuffers(1, &m_fbo);
    m_fbo = 0;
  }
  if (m_rboDepth != 0)
  {
    glDeleteRenderbuffers(1, &m_rboDepth);
    m_rboDepth = 0;
  }
  if (m_texColor != 0)
  {
    glDeleteTextures(1, &m_texColor);
    m_texColor = 0;
  }

#endif

  m_fboWidth = 0;
  m_fboHeight = 0;
}

// -----------------------------------------------------------------------
// Scene preparation, shared by both rendering modes
// -----------------------------------------------------------------------
bool CImGuiSceneView::prepareCompiledScene()
{
  // ---- Sync controller → camera snapshot before rendering ----
  cameraController.applyTo(m_camera);

  if (!m_scene)
  {
    return false;
  }

  // Push camera into the scene's main viewport
  if (auto vp = m_scene->getViewport("main"); vp)
  {
    vp->getCamera() = m_camera;
    if (m_hasCustomBg)
    {
      vp->setCustomBackgroundColor({m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]});
    }
  }

  // Compile / incrementally update the scene
  auto lastPtr = m_lastCompiledScenePtr.lock();
  if (!m_compiledScene || lastPtr.get() != m_scene.get())
  {
    m_compiledScene = std::make_unique<mrpt::opengl::CompiledScene>();
    m_compiledScene->compile(*m_scene);
    m_lastCompiledScenePtr = m_scene;
  }
  else
  {
    m_compiledScene->updateIfNeeded();
  }
  return true;
}

// -----------------------------------------------------------------------
// Scene rendering into the FBO
// -----------------------------------------------------------------------
void CImGuiSceneView::renderSceneToFBO(int w, int h)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!prepareCompiledScene())
  {
    return;
  }

  // Save and bind our FBO
  GLint prevViewport[4];
  glGetIntegerv(GL_VIEWPORT, prevViewport);
  GLint prevFBO = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);

  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  glViewport(0, 0, static_cast<GLsizei>(w), static_cast<GLsizei>(h));

  if (m_hasCustomBg)
  {
    glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]);
  }
  else
  {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  m_compiledScene->render(w, h, 0, 0);

  // Restore previous GL state
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prevFBO));
  glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
#else
  (void)w;
  (void)h;
#endif
}

// -----------------------------------------------------------------------
// Scene rendering into the current framebuffer (background mode)
// -----------------------------------------------------------------------
void CImGuiSceneView::renderSceneDirect()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  try
  {
    if (m_directRect[2] <= 0 || m_directRect[3] <= 0 || !prepareCompiledScene())
    {
      return;
    }
    // The viewport clears its own area (with scissor), and saves/restores
    // the GL viewport. ImGui restores the rest of its state afterwards.
    glDisable(GL_SCISSOR_TEST);
    // ImGui may leave a sampler object bound to texture unit 0, which would
    // override the wrapping and mipmap settings of the scene textures:
    glBindSampler(0, 0);
    m_compiledScene->render(m_directRect[2], m_directRect[3], m_directRect[0], m_directRect[1]);
  }
  catch (const std::exception& e)
  {
    // Called from within the ImGui renderer: do not propagate.
    std::cerr << "[CImGuiSceneView] Exception rendering the scene:\n" << e.what() << "\n";
  }
#endif
}

// -----------------------------------------------------------------------
// Mouse picking
// -----------------------------------------------------------------------
std::optional<mrpt::math::TLine3D> CImGuiSceneView::mouseRay() const
{
  if (!m_hovered || !m_scene)
  {
    return std::nullopt;
  }
  auto vp = m_scene->getViewport("main");
  if (!vp)
  {
    return std::nullopt;
  }
  return vp->get3DRayForPixelCoord(
      {static_cast<int>(m_mouseX * m_pixelScale), static_cast<int>(m_mouseY * m_pixelScale)});
}
