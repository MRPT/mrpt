/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include "formChangeCamerasPose.h"

// Include MRML library
#include <MRPT/MRML/mrml.h>
using namespace MRML;

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TformCameraPose *formCameraPose;


extern CSimpleMap          posesAndSFs;

//---------------------------------------------------------------------------
__fastcall TformCameraPose::TformCameraPose(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TformCameraPose::Button1Click(TObject *Sender)
{
        float           x = StrToFloat( Edit1->Text );
        float           y = StrToFloat( Edit2->Text );
        float           z = StrToFloat( Edit3->Text );
        float           yaw = DEG2RAD( (float)StrToFloat( Edit4->Text ) );
        float           pitch = DEG2RAD( (float)StrToFloat( Edit5->Text ) );
        float           roll = DEG2RAD( (float)StrToFloat( Edit6->Text ) );

        CPose3D         cameraPose(x,y,z,yaw,pitch,roll);


        int             i, n = posesAndSFs.size();

        int             changes = 0;
        int             indx2Change = rgIndex->ItemIndex;

        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;

                posesAndSFs.get(i, posePDF, SF);

                for (int j=0;j<SF->size();j++)
                {
                        CObservation    *obs = SF->getObservationByIndex(j);

                        if ( j==indx2Change)
                        {
                                if (obs->GetRuntimeClass()==CLASS_ID(CObservationImage) )
                                {
                                        // Change:
                                        CObservationImage       *o = (CObservationImage*) obs;
                                        o->cameraPose = cameraPose;

                                        changes ++;
                                }
                                else
                                if (obs->GetRuntimeClass()==CLASS_ID(CObservationStereoImages) )
                                {
                                        // Change:
                                        CObservationStereoImages       *o = (CObservationStereoImages*) obs;
                                        o->cameraPose = cameraPose;

                                        changes ++;
                                }

                        }

                }

        }

        ShowMessage(IntToStr(changes)+" observations changed!");
}
//---------------------------------------------------------------------------

void __fastcall TformCameraPose::Button2Click(TObject *Sender)
{
        CMatrix         intrinsicParams(3,3);

        intrinsicParams(0,0) = StrToFloat(sgMatrix->Cells[0][0]);
        intrinsicParams(0,1) = StrToFloat(sgMatrix->Cells[1][0]);
        intrinsicParams(0,2) = StrToFloat(sgMatrix->Cells[2][0]);
        intrinsicParams(1,0) = StrToFloat(sgMatrix->Cells[0][1]);
        intrinsicParams(1,1) = StrToFloat(sgMatrix->Cells[1][1]);
        intrinsicParams(1,2) = StrToFloat(sgMatrix->Cells[2][1]);
        intrinsicParams(2,0) = StrToFloat(sgMatrix->Cells[0][2]);
        intrinsicParams(2,1) = StrToFloat(sgMatrix->Cells[1][2]);
        intrinsicParams(2,2) = StrToFloat(sgMatrix->Cells[2][2]);

        int             i, n = posesAndSFs.size();

        int             changes = 0;
        int             indx2Change = rgIndex->ItemIndex;

        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;

                posesAndSFs.get(i, posePDF, SF);

                for (int j=0;j<SF->size();j++)
                {
                        CObservation    *obs = SF->getObservationByIndex(j);

                        if ( j==indx2Change &&
                             obs->GetRuntimeClass()==CLASS_ID(CObservationImage) )
                        {
                                // Change:
                                CObservationImage       *o = (CObservationImage*) obs;
                                o->intrinsicParams = intrinsicParams;

                                changes ++;
                        }

                }

        }

        ShowMessage(IntToStr(changes)+" observations changed!");
}
//---------------------------------------------------------------------------

void __fastcall TformCameraPose::Button3Click(TObject *Sender)
{
        CMatrix         distParams(4,1);

        distParams(0,0) = StrToFloat(sgDistortion->Cells[0][0]);
        distParams(1,0) = StrToFloat(sgDistortion->Cells[0][1]);
        distParams(2,0) = StrToFloat(sgDistortion->Cells[0][2]);
        distParams(3,0) = StrToFloat(sgDistortion->Cells[0][3]);

        int             i, n = posesAndSFs.size();

        int             changes = 0;
        int             indx2Change = rgIndex->ItemIndex;

        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;

                posesAndSFs.get(i, posePDF, SF);

                for (int j=0;j<SF->size();j++)
                {
                        CObservation    *obs = SF->getObservationByIndex(j);

                        if ( j==indx2Change &&
                             obs->GetRuntimeClass()==CLASS_ID(CObservationImage) )
                        {
                                // Change:
                                CObservationImage       *o = (CObservationImage*) obs;
                                o->distorsionParams = distParams;

                                changes ++;
                        }

                }

        }

        ShowMessage(IntToStr(changes)+" observations changed!");
}
//---------------------------------------------------------------------------

