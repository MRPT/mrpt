/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/about_box.h>

#if MRPT_HAS_WXWIDGETS
#include "CAboutBox_wx.h"
#endif

void mrpt::gui::show_mrpt_about_box_wxWidgets(
    void* parent_wx_window,
    const std::string& appName,
    const std::string& additionalInfo,
    const bool showStandardInfo)
{
#if MRPT_HAS_WXWIDGETS
  auto* parent = reinterpret_cast<wxWindow*>(parent_wx_window);
  CAboutBox dlg(parent, appName, additionalInfo, showStandardInfo);
  dlg.ShowModal();
#else
  THROW_EXCEPTION("MRPT compiled without wxWidgets support");
#endif
}
