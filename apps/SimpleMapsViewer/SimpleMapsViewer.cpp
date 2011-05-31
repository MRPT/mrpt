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
USEFORM("mainUnit.cpp", Ventana);
USEFORM("formChangeCamerasPose.cpp", formCameraPose);
USEFORM("formAlignerConfig.cpp", formAligner);
USEFORM("formPointsList.cpp", formList);
USEFORM("unFormLuMilios.cpp", formLuMilios);
USEFORM("unEditorConfig.cpp", formMapsConfig);
//---------------------------------------------------------------------------
WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
        try
        {
                 Application->Initialize();
                 Application->CreateForm(__classid(TVentana), &Ventana);
                 Application->CreateForm(__classid(TformCameraPose), &formCameraPose);
                 Application->CreateForm(__classid(TformAligner), &formAligner);
                 Application->CreateForm(__classid(TformList), &formList);
                 Application->CreateForm(__classid(TformLuMilios), &formLuMilios);
                 Application->CreateForm(__classid(TformMapsConfig), &formMapsConfig);
                 Application->Run();
        }
        catch (Exception &exception)
        {
                 Application->ShowException(&exception);
        }
        return 0;
}
//---------------------------------------------------------------------------
