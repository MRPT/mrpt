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

/*---------------------------------------------------------------
  APPLICATION: robot-map-gui
  FILE: robot-map-gui_main.cpp
  AUTHOR: LisGein <alred402@gmail.com>

  See README.txt for instructions.
  ---------------------------------------------------------------*/
#include <QApplication>
#include <QCommandLineParser>

#include "gui/CMainWindow.h"

int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC, "C");
  QLocale::setDefault(QLocale::C);

  QApplication app(argc, argv);
  QApplication::setOrganizationName("MRPT");
  QApplication::setOrganizationDomain("mrpt.org");
  QApplication::setApplicationName("robot-map-gui");

  QCommandLineParser parser;
  parser.parse(QApplication::arguments());
  const QStringList args = parser.positionalArguments();

  CMainWindow mainWindow;

  if (!args.isEmpty()) mainWindow.loadMap(args.first());

  mainWindow.show();
  return app.exec();
}
