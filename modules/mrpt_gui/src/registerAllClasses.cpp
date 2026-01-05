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

#include <mrpt/core/initializer.h>
#include <mrpt/gui.h>
#include <mrpt/gui/registerAllClasses.h>
#include <mrpt/serialization/CSerializable.h>
// deps:
#include <mrpt/viz/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_gui) {}

void mrpt::gui::registerAllClasses_mrpt_gui()
{
  ::registerAllClasses_mrpt_gui();
  mrpt::viz::registerAllClasses_mrpt_opengl();
}
