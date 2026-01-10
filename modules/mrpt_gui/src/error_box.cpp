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

#include <mrpt/gui/error_box.h>
#include <mrpt/system/string_utils.h>  // firstNLines()

#if MRPT_HAS_Qt5
#include <QErrorMessage>
#include <QString>
#else
#include <iostream>
#endif  // MRPT_HAS_Qt5
#if MRPT_HAS_WXWIDGETS
#include <mrpt/gui/WxUtils.h>
#endif

void mrpt::gui::tryCatch(const std::function<void()>& tryPart, const std::string& catchMessage)
{
  try
  {
    tryPart();
  }
  catch (const std::exception& e)
  {
    showErrorMessage(catchMessage + e.what());
  }
  catch (...)
  {
    showErrorMessage("Untyped exception!");
  }
}

void mrpt::gui::showErrorMessage(const std::string& str)
{
  const size_t maxLines = 7;
  const std::string sErr = mrpt::system::firstNLines(str, maxLines);

#if MRPT_HAS_Qt5
  QErrorMessage msg;
  msg.showMessage(QString::fromStdString(sErr));
  msg.exec();
#elif MRPT_HAS_WXWIDGETS
  wxMessageBox(sErr.c_str(), _("Exception"));
#else
  std::cerr << str << std::endl;
#endif  // MRPT_HAS_Qt5
}
