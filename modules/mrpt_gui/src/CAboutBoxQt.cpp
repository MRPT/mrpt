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

#include <mrpt/gui/config.h>

#if MRPT_HAS_Qt5

#include <QLabel>
#include <QTabWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include "CAboutBoxQt.h"

CAboutBoxQt::CAboutBoxQt(
    const std::string& appName, const std::string& additionalInfo, const bool showStandardInfo) :
    CAboutBoxBase(appName, additionalInfo, showStandardInfo)
{
  auto lay = new QVBoxLayout();

  auto buildText = QString::fromStdString(MRPTBuildVersion());
  auto label = new QLabel(buildText, this);
  lay->addWidget(label);

  auto tabs = new QTabWidget(this);

  auto infoText = information("Qt", QT_VERSION_STR);
  tabs->addTab(widgetForTabs(infoText, tabs), tr("Information"));
  tabs->addTab(widgetForTabs(license(), tabs), tr("License"));
  tabs->addTab(widgetForTabs(tutorial(), tabs), tr("Tutorial"));

  lay->addWidget(tabs);
  setLayout(lay);
}

QTextEdit* CAboutBoxQt::widgetForTabs(const std::string& str, QTabWidget* parent) const
{
  auto text = QString::fromStdString(str);
  auto action = new QTextEdit(parent);
  action->setText(text);
  return action;
}

#endif  // MRPT_HAS_Qt5
