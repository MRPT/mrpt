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

#include "formPointsList.h"
#include "mainUnit.h"
#include <stdio.h>

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TformList *formList;
//---------------------------------------------------------------------------
__fastcall TformList::TformList(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------

void __fastcall TformList::Button2Click(TObject *Sender)
{
	mem->Clear();
}
//---------------------------------------------------------------------------

void __fastcall TformList::Button1Click(TObject *Sender)
{
	if (!SD->Execute()) return;
    mem->Lines->SaveToFile( SD->FileName );	
}
//---------------------------------------------------------------------------

void __fastcall TformList::Button3Click(TObject *Sender)
{
	Ventana->pListPoints->LockInvalidate = true;
    Ventana->pListPoints->Clear();

    for (int i=0;i<mem->Lines->Count;i++)
    {
    	AnsiString	str = mem->Lines->Strings[i];
        float	x,y;
        sscanf(str.c_str(),"%f %f", &x,&y);
        Ventana->pListPoints->AddXY(x,y);
    }

	Ventana->pListPoints->LockInvalidate = false;
}
//---------------------------------------------------------------------------
void __fastcall TformList::Button4Click(TObject *Sender)
{
	if (!OD->Execute()) return;

    mem->Lines->LoadFromFile(OD->FileName);	
}
//---------------------------------------------------------------------------
