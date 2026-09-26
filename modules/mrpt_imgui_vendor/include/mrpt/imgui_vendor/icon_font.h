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

/** \file Access to the embedded Material Symbols icon font.
 *
 *  The font is compiled into this library as a byte array, so there is no data
 *  file to install and no runtime font lookup. Use together with the
 *  ICON_MS_* macros from <IconsMaterialSymbols.h>:
 *
 *  \code
 *    #include <IconsMaterialSymbols.h>
 *    #include <imgui.h>
 *    #include <mrpt/imgui_vendor/icon_font.h>
 *
 *    namespace miv = mrpt::imgui_vendor;
 *
 *    ImFontConfig cfg;
 *    cfg.MergeMode = true;              // merge into the current text font
 *    cfg.FontDataOwnedByAtlas = false;  // the array is static, do not free it
 *    static const ImWchar ranges[] = {ICON_MIN_MS, ICON_MAX_16_MS, 0};
 *
 *    ImGui::GetIO().Fonts->AddFontFromMemoryTTF(
 *        const_cast<void*>(miv::iconFontData()),
 *        static_cast<int>(miv::iconFontDataSize()), 16.0f, &cfg, ranges);
 *
 *    // ... later, in the frame loop:
 *    ImGui::Button(ICON_MS_FOLDER_OPEN " Open");
 *  \endcode
 *
 *  Only a curated subset of the ~3600 upstream icons is embedded, to keep the
 *  library small. See scripts/regenerate_icon_font.py to add more.
 */

namespace mrpt::imgui_vendor
{
/** Pointer to the embedded TTF data. Statically allocated; never freed. */
const void* iconFontData();

/** Size in bytes of the buffer returned by iconFontData(). */
unsigned int iconFontDataSize();
}  // namespace mrpt::imgui_vendor
