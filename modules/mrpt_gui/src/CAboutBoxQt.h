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
#include <QDialog>

#include "CAboutBoxBase.h"

class QTextEdit;
class QTabWidget;

class CAboutBoxQt : public QDialog, public CAboutBoxBase
{
 public:
  CAboutBoxQt(
      const std::string& appName, const std::string& additionalInfo, const bool showStandardInfo);
  ~CAboutBoxQt() override = default;

 private:
  QTextEdit* widgetForTabs(const std::string& str, QTabWidget* parent) const;
};
