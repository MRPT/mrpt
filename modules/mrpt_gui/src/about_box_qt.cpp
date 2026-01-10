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

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/about_box.h>

#if MRPT_HAS_Qt5
#include "CAboutBoxQt.h"
#endif

void mrpt::gui::show_mrpt_about_box_Qt(
    const std::string& appName, const std::string& additionalInfo, const bool showStandardInfo)
{
#if MRPT_HAS_Qt5
  CAboutBoxQt dlg(appName, additionalInfo, showStandardInfo);
  dlg.exec();
#else
  THROW_EXCEPTION("MRPT compiled without Qt support");
#endif
}
