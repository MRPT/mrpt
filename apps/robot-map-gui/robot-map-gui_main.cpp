/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


/*---------------------------------------------------------------
    APPLICATION: robot-map-gui
    FILE: robot-map-gui_main.cpp
    AUTHOR: LisGein <alred402@gmail.com>

	See README.txt for instructions.
  ---------------------------------------------------------------*/
#include <QApplication>

#include "gui/cmainwindow.h"


int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    CMainWindow mainWindow;
    mainWindow.show();
    return app.exec();
}

