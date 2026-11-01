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
#include <mrpt/imgui/CImGuiFileDialog.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::imgui
{
/** Mapping between image pixel coordinates and widget coordinates of a
 * zoomable, pannable image view: `widget = (image - offset) * zoom`.
 * \ingroup mrpt_imgui_grp
 */
struct ImageViewTransform
{
  /** Widget pixels per image pixel. */
  float zoom = 1.0f;
  /** Image coordinates at the widget top-left corner. */
  float offsetX = 0.0f;
  float offsetY = 0.0f;

  float minZoom = 1.0f / 64;
  float maxZoom = 64.0f;

  [[nodiscard]] std::pair<float, float> imageToWidget(float u, float v) const
  {
    return {(u - offsetX) * zoom, (v - offsetY) * zoom};
  }
  [[nodiscard]] std::pair<float, float> widgetToImage(float x, float y) const
  {
    return {x / zoom + offsetX, y / zoom + offsetY};
  }

  /** Scales the image to fit the widget, keeping its aspect ratio, centered. */
  void fit(float imgW, float imgH, float widgetW, float widgetH);

  /** Zooms by `factor` keeping the image point under widget point (x,y) still. */
  void zoomAt(float x, float y, float factor);

  /** Pans by a displacement in widget pixels. */
  void pan(float dx, float dy)
  {
    offsetX -= dx / zoom;
    offsetY -= dy / zoom;
  }
};

/** Shows an mrpt::img::CImage in an ImGui panel, as an OpenGL texture.
 *
 * Mouse wheel zooms at the cursor, dragging pans, double click fits the
 * image to the panel; hovering shows the pixel coordinates and value. The
 * right-click menu offers "Fit", "1:1" and "Save image as...".
 *
 * Gray and color images of 8 bits per channel are shown as they are;
 * 16-bit images are scaled to their own value range for display.
 *
 * Must be used from the thread owning the GL context.
 * \ingroup mrpt_imgui_grp
 */
class CImGuiImageView
{
 public:
  CImGuiImageView() = default;
  ~CImGuiImageView();

  CImGuiImageView(const CImGuiImageView&) = delete;
  CImGuiImageView& operator=(const CImGuiImageView&) = delete;

  /** Sets the image to show (uploaded to the GPU on the next render). */
  void setImage(const mrpt::img::CImage& img);
  void clear();
  [[nodiscard]] bool hasImage() const { return !m_image.isEmpty(); }
  [[nodiscard]] const mrpt::img::CImage& image() const { return m_image; }

  /** Draws the view as a child region of the given size (0: all available
   * space) in the current window. */
  void render(const char* id, float width = 0, float height = 0);

  /** Keep the image fitted to the panel when the panel is resized, until the
   * user zooms or pans. */
  bool autoFit = true;
  /** Show the pixel coordinates and value under the mouse. */
  bool showPixelInfo = true;

  /** Extra items for the right-click menu. */
  std::function<void()> onContextMenu;

  [[nodiscard]] ImageViewTransform& transform() { return m_tf; }

  /** Converts an image into the 8-bit RGBA buffer uploaded as texture
   * (rows top to bottom). Exposed for testing. */
  [[nodiscard]] static std::vector<uint8_t> toRGBA8(const mrpt::img::CImage& img);

  /** Text describing the pixel (u,v) of the image (value of each channel),
   * or empty if out of the image. */
  [[nodiscard]] static std::string pixelInfo(const mrpt::img::CImage& img, int u, int v);

 private:
  mrpt::img::CImage m_image;
  bool m_dirty = false;
  unsigned int m_tex = 0;
  int m_texW = 0;
  int m_texH = 0;
  ImageViewTransform m_tf;
  bool m_userMoved = false;
  CImGuiFileDialog m_saveDialog;

  void uploadIfNeeded();
};

}  // namespace mrpt::imgui
