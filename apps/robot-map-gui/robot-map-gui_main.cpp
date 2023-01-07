/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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
