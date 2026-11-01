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

// clang-format off
#include <mrpt/opengl/opengl_api.h>
// clang-format on

#include <imgui.h>
#include <mrpt/imgui/CImGuiImageView.h>
#include <mrpt/imgui/icons.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace mrpt::imgui
{
void ImageViewTransform::fit(float imgW, float imgH, float widgetW, float widgetH)
{
  if (imgW <= 0 || imgH <= 0 || widgetW <= 0 || widgetH <= 0)
  {
    return;
  }
  zoom = std::clamp(std::min(widgetW / imgW, widgetH / imgH), minZoom, maxZoom);
  // Center the image:
  offsetX = -(widgetW / zoom - imgW) * 0.5f;
  offsetY = -(widgetH / zoom - imgH) * 0.5f;
}

void ImageViewTransform::zoomAt(float x, float y, float factor)
{
  const auto [u, v] = widgetToImage(x, y);
  zoom = std::clamp(zoom * factor, minZoom, maxZoom);
  offsetX = u - x / zoom;
  offsetY = v - y / zoom;
}

CImGuiImageView::~CImGuiImageView()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (m_tex != 0)
  {
    glDeleteTextures(1, &m_tex);
  }
#endif
}

void CImGuiImageView::setImage(const mrpt::img::CImage& img)
{
  m_image = img.makeShallowCopy();
  m_dirty = true;
}

void CImGuiImageView::clear()
{
  m_image = mrpt::img::CImage();
  m_dirty = true;
}

std::vector<uint8_t> CImGuiImageView::toRGBA8(const mrpt::img::CImage& img)
{
  std::vector<uint8_t> out;
  if (img.isEmpty())
  {
    return out;
  }
  const int w = static_cast<int>(img.getWidth());
  const int h = static_cast<int>(img.getHeight());
  const int nch = static_cast<int>(img.channels());
  out.resize(static_cast<size_t>(w) * static_cast<size_t>(h) * 4);

  if (img.getPixelDepth() == mrpt::img::PixelDepth::D16U)
  {
    // Scale the value range to 8 bits (first channel only):
    uint16_t lo = 0xffff;
    uint16_t hi = 0;
    for (int y = 0; y < h; y++)
    {
      const auto* row = img.ptrLine<uint16_t>(y);
      for (int x = 0; x < w; x++)
      {
        lo = std::min(lo, row[x * nch]);
        hi = std::max(hi, row[x * nch]);
      }
    }
    const float k = hi > lo ? 255.0f / static_cast<float>(hi - lo) : 0.0f;
    for (int y = 0; y < h; y++)
    {
      const auto* row = img.ptrLine<uint16_t>(y);
      uint8_t* dst = &out[static_cast<size_t>(y) * static_cast<size_t>(w) * 4];
      for (int x = 0; x < w; x++)
      {
        const auto v = static_cast<uint8_t>(std::lround((row[x * nch] - lo) * k));
        dst[4 * x + 0] = v;
        dst[4 * x + 1] = v;
        dst[4 * x + 2] = v;
        dst[4 * x + 3] = 0xff;
      }
    }
    return out;
  }

  const bool bgr = img.getChannelsOrder().rfind("BGR", 0) == 0;
  for (int y = 0; y < h; y++)
  {
    const auto* row = img.ptrLine<uint8_t>(y);
    uint8_t* dst = &out[static_cast<size_t>(y) * static_cast<size_t>(w) * 4];
    for (int x = 0; x < w; x++)
    {
      const uint8_t* px = &row[x * nch];
      if (nch == 1)
      {
        dst[4 * x + 0] = px[0];
        dst[4 * x + 1] = px[0];
        dst[4 * x + 2] = px[0];
        dst[4 * x + 3] = 0xff;
      }
      else
      {
        dst[4 * x + 0] = px[bgr ? 2 : 0];
        dst[4 * x + 1] = px[1];
        dst[4 * x + 2] = px[bgr ? 0 : 2];
        dst[4 * x + 3] = nch >= 4 ? px[3] : 0xff;
      }
    }
  }
  return out;
}

std::string CImGuiImageView::pixelInfo(const mrpt::img::CImage& img, int u, int v)
{
  if (img.isEmpty() || u < 0 || v < 0 || u >= static_cast<int>(img.getWidth()) ||
      v >= static_cast<int>(img.getHeight()))
  {
    return {};
  }
  const int nch = static_cast<int>(img.channels());
  std::string s = "(" + std::to_string(u) + ", " + std::to_string(v) + "): ";
  if (img.getPixelDepth() == mrpt::img::PixelDepth::D16U)
  {
    s += std::to_string(img.ptrLine<uint16_t>(v)[u * nch]);
    return s;
  }
  const uint8_t* px = &img.ptrLine<uint8_t>(v)[u * nch];
  if (nch == 1)
  {
    return s + std::to_string(px[0]);
  }
  const bool bgr = img.getChannelsOrder().rfind("BGR", 0) == 0;
  s += "R=" + std::to_string(px[bgr ? 2 : 0]) + " G=" + std::to_string(px[1]) +
       " B=" + std::to_string(px[bgr ? 0 : 2]);
  if (nch >= 4)
  {
    s += " A=" + std::to_string(px[3]);
  }
  return s;
}

void CImGuiImageView::uploadIfNeeded()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!m_dirty)
  {
    return;
  }
  m_dirty = false;
  if (m_image.isEmpty())
  {
    m_texW = 0;
    m_texH = 0;
    return;
  }
  const auto rgba = toRGBA8(m_image);
  const int w = static_cast<int>(m_image.getWidth());
  const int h = static_cast<int>(m_image.getHeight());
  if (m_tex == 0)
  {
    glGenTextures(1, &m_tex);
  }
  glBindTexture(GL_TEXTURE_2D, m_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  if (w != m_texW || h != m_texH)
  {
    m_userMoved = false;  // new image size: fit again
  }
  m_texW = w;
  m_texH = h;
#endif
}

void CImGuiImageView::render(const char* id, float width, float height)
{
  uploadIfNeeded();

  ImGui::PushID(id);
  ImGui::BeginChild(
      "##img", ImVec2(width, height), ImGuiChildFlags_None,
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float ww = std::max(1.0f, avail.x);
  const float wh = std::max(1.0f, avail.y);

  if (m_texW == 0 || m_tex == 0)
  {
    ImGui::TextDisabled("(no image)");
    ImGui::EndChild();
    ImGui::PopID();
    return;
  }

  if (!m_userMoved && autoFit)
  {
    m_tf.fit(static_cast<float>(m_texW), static_cast<float>(m_texH), ww, wh);
  }

  ImGui::InvisibleButton("##canvas", ImVec2(ww, wh));
  const bool hovered = ImGui::IsItemHovered();
  const ImGuiIO& io = ImGui::GetIO();
  const float mx = io.MousePos.x - p0.x;
  const float my = io.MousePos.y - p0.y;

  if (hovered && io.MouseWheel != 0.0f)
  {
    m_tf.zoomAt(mx, my, std::pow(1.2f, io.MouseWheel));
    m_userMoved = true;
  }
  if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f))
  {
    m_tf.pan(io.MouseDelta.x, io.MouseDelta.y);
    m_userMoved = true;
  }
  if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
  {
    m_userMoved = false;
    m_tf.fit(static_cast<float>(m_texW), static_cast<float>(m_texH), ww, wh);
  }

  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->PushClipRect(p0, ImVec2(p0.x + ww, p0.y + wh), true);
  const auto [x0, y0] = m_tf.imageToWidget(0, 0);
  const auto [x1, y1] = m_tf.imageToWidget(static_cast<float>(m_texW), static_cast<float>(m_texH));
  dl->AddImage(
      static_cast<ImTextureID>(m_tex), ImVec2(p0.x + x0, p0.y + y0), ImVec2(p0.x + x1, p0.y + y1));
  dl->PopClipRect();

  if (hovered && showPixelInfo)
  {
    const auto [u, v] = m_tf.widgetToImage(mx, my);
    const auto info =
        pixelInfo(m_image, static_cast<int>(std::floor(u)), static_cast<int>(std::floor(v)));
    if (!info.empty())
    {
      ImGui::SetTooltip("%s", info.c_str());
    }
  }

  if (ImGui::BeginPopupContextItem("##ctx"))
  {
    if (ImGui::MenuItem(ICON_MS_FIT_SCREEN " Fit"))
    {
      m_userMoved = false;
    }
    if (ImGui::MenuItem("1:1"))
    {
      m_tf.zoomAt(ww * 0.5f, wh * 0.5f, 1.0f / m_tf.zoom);
      m_userMoved = true;
    }
    if (ImGui::MenuItem(ICON_MS_SAVE " Save image as..."))
    {
      m_saveDialog.open(
          CImGuiFileDialog::Mode::Save, "Save image",
          {
              { "PNG images",         {"png"}},
              {"JPEG images", {"jpg", "jpeg"}}
      });
    }
    if (onContextMenu)
    {
      ImGui::Separator();
      onContextMenu();
    }
    ImGui::EndPopup();
  }

  if (auto r = m_saveDialog.poll(); r && !r->empty())
  {
    if (!m_image.saveToFile(r->front()))
    {
      std::cerr << "[CImGuiImageView] Error saving image to: " << r->front() << "\n";
    }
  }

  ImGui::EndChild();
  ImGui::PopID();
}

}  // namespace mrpt::imgui
