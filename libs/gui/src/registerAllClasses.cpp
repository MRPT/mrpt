/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"
//
#include <mrpt/core/initializer.h>
#include <mrpt/gui.h>
#include <mrpt/gui/registerAllClasses.h>
#include <mrpt/serialization/CSerializable.h>
// deps:
#include <mrpt/opengl/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_gui) {}

void mrpt::gui::registerAllClasses_mrpt_gui()
{
	::registerAllClasses_mrpt_gui();
	mrpt::opengl::registerAllClasses_mrpt_opengl();
}
