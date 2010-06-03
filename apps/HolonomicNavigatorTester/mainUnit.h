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

#ifndef mainUnitH
#define mainUnitH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include <Dialogs.hpp>
#include <ExtDlgs.hpp>
#include <Graphics.hpp>
#include <ComCtrls.hpp>
#include <ImgList.hpp>
#include <ToolWin.hpp>
#include "sgr_data.hpp"
#include "sgr_def.hpp"
//---------------------------------------------------------------------------
class TVentana : public TForm
{
__published:	// IDE-managed Components
        TPanel *Panel1;
        TOpenPictureDialog *OD;
        TPanel *Panel2;
        TScrollBox *ScrollBox1;
        TImage *img;
        TShape *shRobot;
        TShape *shTarget;
        TRadioGroup *rgMethod;
        TLabel *Label1;
        TToolBar *ToolBar1;
        TToolButton *ToolButton1;
        TToolButton *ToolButton2;
        TToolButton *ToolButton3;
        TToolButton *ToolButton4;
        TToolButton *ToolButton5;
        TToolButton *ToolButton6;
        TToolButton *ToolButton7;
        TTimer *timRun;
        TEdit *edX;
        TEdit *edY;
        TButton *btnSetRobot;
        TButton *btnSetTarget;
        Tsp_XYLine *puntosScan;
        Tsp_XYPlot *plotScan;
        TLabel *lbTimeNav;
        TLabel *lbTimeScan;
        Tsp_XYLine *puntosSectors;
        Tsp_XYLine *flechaDireccion;
        TEdit *edN;
        TLabel *Label2;
        TLabel *lbEvaluation;
        TLabel *lbSituation;
        TImageList *ImageList1;
        void __fastcall ToolButton1Click(TObject *Sender);
        void __fastcall ToolButton7Click(TObject *Sender);
        void __fastcall ToolButton2Click(TObject *Sender);
        void __fastcall ToolButton5Click(TObject *Sender);
        void __fastcall timRunTimer(TObject *Sender);
        void __fastcall imgMouseDown(TObject *Sender, TMouseButton Button,
          TShiftState Shift, int X, int Y);
        void __fastcall btnSetRobotClick(TObject *Sender);
        void __fastcall btnSetTargetClick(TObject *Sender);
        void __fastcall FormCreate(TObject *Sender);
        void __fastcall FormDestroy(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TVentana(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TVentana *Ventana;
//---------------------------------------------------------------------------
#endif
