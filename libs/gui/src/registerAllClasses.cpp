/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt::gui;
using namespace mrpt::utils;

void registerAllClasses_mrpt_gui();

CStartUpClassesRegister  mrpt_gui_class_reg(&registerAllClasses_mrpt_gui);
//const int dumm = mrpt_gui_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


/*---------------------------------------------------------------
					registerAllClasses_mrpt_gui
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_gui()
{
	registerClass( CLASS_ID( CDisplayWindow ) );
	registerClass( CLASS_ID( CDisplayWindow3D ) );
	registerClass( CLASS_ID( CDisplayWindowPlots ) );
}

