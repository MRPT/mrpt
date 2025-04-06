/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/PerThreadDataHolder.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/Buffer.h>

namespace mrpt::opengl
{
/** IDs of FrameBuffers, as used in FrameBuffer
 *
 * \ingroup mrpt_opengl_grp
 */
struct FrameBufferBinding
{
  unsigned int drawFbId = 0;
  unsigned int readFbId = 0;
};

/** An OpenGL FrameBuffer resource (FBO) with either RGBA+depth or depth only
 * render buffers.
 *
 * Refer to docs for glGenFramebuffers() and glGenRenderbuffers().
 *
 * \sa Buffer, DepthMapFBO
 * \ingroup mrpt_opengl_grp
 */
class FrameBuffer
{
 public:
  FrameBuffer() = default;
  ~FrameBuffer() = default;

  /** @name Main API
   *  @{ */

  /** Creates a new FB object and the two (RGBA+depth) render buffers.
   */
  void create(unsigned int width, unsigned int height, int nSamples = 1)
  {
    m_impl.create(width, height, nSamples);
  }

  /** Creates a new depth-only FBO.
   */
  void createDepthMap(unsigned int width, unsigned int height)
  {
    m_impl.createDepthMap(width, height);
  }

  /** Release resources */
  void destroy() { m_impl.destroy(); }

  /** Bind this framebuffer object to the current context.
   *  \return The former binding
   *  \sa Bind(), CurrentBinding()
   */
  FrameBufferBinding bind() { return m_impl.bind(); }

  /** Unbind the framebuffer object from the context */
  void unbind() { return m_impl.unbind(); }

  /// Blit the framebuffer object onto the screen
  void blit();

  bool initialized() const { return m_impl.m_state.get().m_created; }

  unsigned int width() const { return m_impl.m_state.get().m_width; }
  unsigned int height() const { return m_impl.m_state.get().m_height; }
  int numSamples() const { return m_impl.m_state.get().m_Samples; }

  unsigned int depthMapTextureId() const { return m_impl.m_state.get().m_DepthMapTexture; }

  /** @} */

  /** @name Static methods
   *  @{ */

  static void Bind(const FrameBufferBinding& ids);
  static void Unbind();  //!< Calls glBindFramebuffer(GL_FRAMEBUFFER, 0);
  static FrameBufferBinding CurrentBinding();

  /** @} */

 protected:
  struct RAII_Impl
  {
    RAII_Impl() = default;
    ~RAII_Impl() { destroy(); }

    Buffer::Type type;
    Buffer::Usage usage = Buffer::Usage::StaticDraw;

    void create(unsigned int width, unsigned int height, int nSamples);
    void createDepthMap(unsigned int width, unsigned int height);
    void destroy();
    FrameBufferBinding bind();
    void unbind();

    struct State
    {
      bool m_created = false;
      bool m_isDepthMap = false;

      // Regular FBO:
      unsigned int m_Framebuffer = 0, m_Depth = 0, m_Color = 0;
      // DepthMap rendering to texture:
      unsigned int m_DepthMapTexture = 0;

      unsigned int m_width = 0, m_height = 0;  /// In pixels
      int m_Samples = 0;
    };
    mrpt::containers::PerThreadDataHolder<State> m_state;
  };
  RAII_Impl m_impl;
};

}  // namespace mrpt::opengl
