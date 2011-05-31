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
//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop
USERES("PTG_Designer.res");
USEFORM("mainUnit.cpp", Ventana);
USEUNIT("..\ReactiveNavigatorLibrary\CPTG3.cpp");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG1.cpp");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG2.cpp");
USEUNIT("..\ReactiveNavigatorLibrary\CParameterizedTrajectoryGenerator.cpp");
USELIB("..\..\lib\MRPT_BC.lib");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG4.cpp");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG5.cpp");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG6.cpp");
USELIB("..\..\lib\highgui_BC.lib");
USELIB("..\..\lib\cv_BC.lib");
USELIB("..\..\lib\cvaux_BC.lib");
USELIB("..\..\lib\cxcore_BC.lib");
USEUNIT("..\ReactiveNavigatorLibrary\CPTG7.cpp");
//---------------------------------------------------------------------------
WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
        try
        {
                 Application->Initialize();
                 Application->Title = "PTG Designer - ISA 2005";
                 Application->CreateForm(__classid(TVentana), &Ventana);
		Application->Run();
        }
        catch (Exception &exception)
        {
                 Application->ShowException(&exception);
        }
        return 0;
}
//---------------------------------------------------------------------------
