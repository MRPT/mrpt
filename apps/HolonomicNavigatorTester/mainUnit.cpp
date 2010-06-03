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

#include <mrml/MRML.h>
#include "../ReactiveNavigatorLibrary/CHolonomicSBG.h"
#include "../ReactiveNavigatorLibrary/CHolonomicVFF.h"

#include <vcl.h>
#pragma hdrstop

#include "mainUnit.h"

int _matherr(struct _exception *e)
{
        ShowMessage(e->name);

        e->retval = 0;
}

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "sgr_data"
#pragma link "sgr_def"
#pragma resource "*.dfm"
TVentana *Ventana;

using namespace MRPTAPPS::RNAV;

#define ROBOT_MAX_SPEED         2.0f

CAbstractHolonomicReactiveMethod        *holonomicMethod = NULL;
COccupancyGridMap2D                      gridMap;

// Posicion del robot y target:
CPoint2D                                targetPose;
CPose2D                                 robotPose;



float           RESOLUTION = 0.05f;
FILE            *fil;


//---------------------------------------------------------------------------
__fastcall TVentana::TVentana(TComponent* Owner)
        : TForm(Owner)
{
        fil = fopen("histogram_dif_dirs.txt","at");
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton1Click(TObject *Sender)
{
        if (!OD->Execute()) return;

        img->Picture->LoadFromFile( OD->FileName );


}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton7Click(TObject *Sender)
{
        Close();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton2Click(TObject *Sender)
{
        // MAPA
        img->Picture->SaveToFile("tmpFile.bmp");
        gridMap.loadFromBitmapFile("tmpFile.bmp",RESOLUTION,0, img->Height );
        DeleteFile("tmpFile.bmp");

        float p;
        p=gridMap.getXMin();
        p=gridMap.getXMax();
        p=gridMap.getYMin();
        p=gridMap.getYMax();


        // Navegador:
        if (holonomicMethod) delete holonomicMethod;

        switch ( rgMethod->ItemIndex )
        {
                default:
                case 0:
                        holonomicMethod = new CHolonomicVFF();
                        break;
                case 1:
                        holonomicMethod = new CHolonomicSBG();
                        break;
        };

        // Load poses from graphical ones:
        robotPose.x = (shRobot->Left+1) * RESOLUTION;
        robotPose.y = - (shRobot->Top+1) * RESOLUTION;
        robotPose.phi = 0;

        targetPose.x = (shTarget->Left+8) * RESOLUTION;
        targetPose.y = -(shTarget->Top+8) * RESOLUTION;

        ToolButton2->Enabled = false;
        ToolButton5->Enabled = true;

        timRun->Enabled = true;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton5Click(TObject *Sender)
{
        //
        ToolButton5->Enabled = false;
        ToolButton2->Enabled = true;

        timRun->Enabled = false;
}
//---------------------------------------------------------------------------
void __fastcall TVentana::timRunTimer(TObject *Sender)
{
        CTicTac                         tictac;
        AnsiString                      s;
        int                             N = StrToInt(edN->Text);

        // Simular scan 360º:

        CObservation2DRangeScan      simulatedScan;

        simulatedScan.aperture = M_2PI;
        simulatedScan.rightToLeft = true;
        simulatedScan.maxRange = 5.0f;
        simulatedScan.sensorPose = CPose2D(0,0,0);

        tictac.Tic();

        gridMap.laserScanSimulator( simulatedScan, robotPose,0.5,N );

        // Normalize:
        for (int j=0;j<simulatedScan.scan.size();j++)
                simulatedScan.scan[j] /= simulatedScan.maxRange;

        s.sprintf("Time to scan:%.01fms", 1000 * tictac.Tac());
        lbTimeScan->Caption = s;

        // Plot scan ---------------------------------
        puntosScan->LockInvalidate = true;
        puntosScan->Clear();
        for (int i=0;i<simulatedScan.scan.size();i++)
        {
                float ang = M_PI *( -1 + 2*i/((float)simulatedScan.scan.size()) );
                float d = simulatedScan.scan[i];
                puntosScan->AddXY( - d*sin(ang), d*cos(ang));
        }
        puntosScan->LockInvalidate = false;

        // Navigate:
        float        desiredDirection,desiredSpeed,evaluation;
        static float        lastDesiredDirection=0;
        CPoint2D     relTargetPose = targetPose - robotPose;
        relTargetPose*= 1.0/simulatedScan.maxRange;     // Normalizado
        float        maxSpeed = ROBOT_MAX_SPEED;
        CHolonomicLogFileRecord         *log=NULL;

        tictac.Tic();
        holonomicMethod->navigate(
                        relTargetPose,
                        simulatedScan.scan,
                        maxSpeed,
                        desiredDirection,
                        desiredSpeed,
                        &log );

        s.sprintf("Time to nav.:%.01fms", 1000 * tictac.Tac());
        lbTimeNav->Caption = s;
//        s.sprintf("Evaluation:%.02f",log-> evaluation);
//        lbEvaluation->Caption = s;

        robotPose.x += cos(desiredDirection) * desiredSpeed * timRun->Interval/1000.0f;
        robotPose.y += sin(desiredDirection) * desiredSpeed * timRun->Interval/1000.0f;

        // Estadistica de diferencia de direccion entre iteraciones --------------------
        float difDir = lastDesiredDirection - desiredDirection;
        fprintf( fil, "%f ", difDir );

        lastDesiredDirection = desiredDirection;

        // Plots ---------------------------------
        if (rgMethod->ItemIndex == 1)
        {
             CLogFileRecord_SBG      *l = (CLogFileRecord_SBG*)log;
             if (l)
             {
                lbSituation->Caption = IntToStr(l->situation);


                puntosSectors->LockInvalidate = true;
                puntosSectors->Clear();
                int nGaps = l->gaps_ini.size();
                for (int i=0;i<nGaps;i++)
                {
                        float ang,d;

                        puntosSectors->AddXY(0,0);

                        ang = M_PI *( -1 + 2*l->gaps_ini[i]/((float)simulatedScan.scan.size()) );
                        d = simulatedScan.scan[l->gaps_ini[i]]-0.1;
                        puntosSectors->AddXY( - d*sin(ang), d*cos(ang));

                        for (int j=0;j<10;j++)
                        {
                                float sec = l->gaps_ini[i] + j*(l->gaps_end[i]-l->gaps_ini[i])/10.0;
                                ang = M_PI *( -1 + 2*sec/((float)simulatedScan.scan.size()) );
                                d = simulatedScan.scan[sec]-0.1;
                                puntosSectors->AddXY( - d*sin(ang), d*cos(ang));
                        }

                        ang = M_PI *( -1 + 2*l->gaps_end[i]/((float)simulatedScan.scan.size()) );
                        d = simulatedScan.scan[l->gaps_end[i]]-0.1;
                        puntosSectors->AddXY( - d*sin(ang), d*cos(ang));

                        puntosSectors->AddXY(0,0);

                }

                puntosSectors->LockInvalidate = false;
             }
        }

        // Movement direction:
        flechaDireccion->LockInvalidate = true;
        flechaDireccion->Clear();
        flechaDireccion->AddXY(0,0);
        flechaDireccion->AddXY( -sin(desiredDirection) * desiredSpeed * 2, cos(desiredDirection) * desiredSpeed * 2 );

        flechaDireccion->LockInvalidate = false;

        // Update graphics:
        shRobot->Left = robotPose.x / RESOLUTION - 1;
        shRobot->Top  = -robotPose.y/RESOLUTION - 1;

        img->Picture->Bitmap->Canvas->Pixels[shRobot->Left+1][shRobot->Top+1]=RGB(30,30,30);

        delete log;
        Application->ProcessMessages();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::imgMouseDown(TObject *Sender,
      TMouseButton Button, TShiftState Shift, int X, int Y)
{
        float x,y;
        x = X * RESOLUTION;
        y = - Y* RESOLUTION;

        edX->Text = FloatToStr( x );
        edY->Text = FloatToStr( y );

}
//---------------------------------------------------------------------------

void __fastcall TVentana::btnSetRobotClick(TObject *Sender)
{
        float x,y;
        x = atof(edX->Text.c_str());
        y = atof(edY->Text.c_str());

        shRobot->Left = x / RESOLUTION - 1;
        shRobot->Top  = -y / RESOLUTION - 1;

        robotPose.x = (shRobot->Left+1) * RESOLUTION;
        robotPose.y = -(shRobot->Top+1) * RESOLUTION;
        robotPose.phi = 0;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::btnSetTargetClick(TObject *Sender)
{
        float x,y;
        x = atof(edX->Text.c_str());
        y = atof(edY->Text.c_str());

        shTarget->Left = x / RESOLUTION - 8;
        shTarget->Top  = - y / RESOLUTION - 8;

        targetPose.x = (shTarget->Left+8) * RESOLUTION;
        targetPose.y = - (shTarget->Top+8) * RESOLUTION;

}
//---------------------------------------------------------------------------

void __fastcall TVentana::FormCreate(TObject *Sender)
{
        DecimalSeparator = '.';
}
//---------------------------------------------------------------------------

void __fastcall TVentana::FormDestroy(TObject *Sender)
{
        fclose(fil);
}
//---------------------------------------------------------------------------

