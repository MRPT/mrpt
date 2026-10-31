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

#include <imgui.h>
#include <mrpt/img/TColor.h>

namespace mrpt::imgui
{
/** Applies the MRPT dark ImGui style (colors, rounding and spacing) to the
 * current ImGui context, scaling sizes by `dpiScale`.
 * \ingroup mrpt_imgui_grp
 */
void applyStyle(float dpiScale = 1.0f);

/** Converts an MRPT color to an ImGui one. */
[[nodiscard]] inline ImVec4 toImVec4(const mrpt::img::TColor& c)
{
  return {c.R / 255.0f, c.G / 255.0f, c.B / 255.0f, c.A / 255.0f};
}
[[nodiscard]] inline ImVec4 toImVec4(const mrpt::img::TColorf& c) { return {c.R, c.G, c.B, c.A}; }

/** Converts an ImGui color to an MRPT one (components clamped to [0,1]). */
[[nodiscard]] mrpt::img::TColor toTColor(const ImVec4& c);

/** Packed 32-bit ImGui color (as used by ImDrawList) of an MRPT color. */
[[nodiscard]] inline ImU32 toImU32(const mrpt::img::TColor& c)
{
  return IM_COL32(c.R, c.G, c.B, c.A);
}

/** Size, in framebuffer pixels, of an ImGui region of `size` (ImGui units)
 * in the main viewport, accounting for HiDPI framebuffer scaling. Use it to
 * size off-screen renders of an mrpt::viz::Scene shown in a panel. */
[[nodiscard]] ImVec2 regionSizeInPixels(const ImVec2& size);

/** Shows `text` as a tooltip when the previous item is hovered. */
void tooltip(const char* text);

/** A "(?)" marker that shows `text` as a tooltip when hovered. */
void helpMarker(const char* text);

}  // namespace mrpt::imgui
