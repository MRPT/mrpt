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
#include <ComCtrls.hpp>
#include <ExtCtrls.hpp>
#include "sgr_data.hpp"
#include "sgr_def.hpp"
#include <Dialogs.hpp>
//---------------------------------------------------------------------------
class TVentana : public TForm
{
__published:	// IDE-managed Components
        TPanel *Panel1;
        TPageControl *PageControl1;
        TTabSheet *TabSheet1;
        TSplitter *Splitter1;
        Tsp_XYPlot *plot;
        Tsp_XYLine *pTrajs;
        TMemo *memOut;
        TTabSheet *TabSheet4;
        TScrollBar *sbH;
        TScrollBar *sbV;
        TPageControl *PageControl2;
        TTabSheet *TabSheet5;
        TTabSheet *TabSheet6;
        TPanel *Panel2;
        TLabel *Label1;
        TLabel *Label2;
        TLabel *Label4;
        TLabel *Label5;
        TLabel *Label6;
        TLabel *Label7;
        TLabel *Label8;
        TLabel *Label9;
        TLabel *Label10;
        TLabel *Label11;
        TButton *btnGenerate;
        TEdit *edRefDist;
        TEdit *edRes;
        TEdit *edAlfas;
        TEdit *edMaxTime;
        TEdit *edMaxDist;
        TEdit *edMaxN;
        TEdit *edAT;
        TEdit *edMinDist;
        TEdit *edVM;
        TEdit *edWM;
        TLabel *Label15;
        TEdit *edSelAlfa;
        TButton *Button1;
        Tsp_XYLine *pSelTraj;
        TLabel *Label16;
        TLabel *lbCursor1;
        TLabel *lbCursor2;
        TLabel *Label17;
        TEdit *edTAU;
        TLabel *Label18;
        TEdit *edDELAY;
        TButton *Button2;
        TSaveDialog *SD;
        TButton *Button3;
        TTabSheet *TabSheet7;
        Tsp_XYPlot *plotW;
        Tsp_XYPlot *plotV;
        Tsp_XYLine *pV;
        Tsp_XYLine *pW;
	TPanel *Panel3;
	TLabel *Label13;
	TLabel *Label14;
	TLabel *Label3;
	TEdit *edA0V;
	TEdit *edA0W;
	TEdit *edK;
	TListBox *rgPTG;
	TTabSheet *TabSheet2;
	Tsp_XYPlot *plotTarget;
	Tsp_XYLine *trajsTarget;
        void __fastcall btnGenerateClick(TObject *Sender);
        void __fastcall Button1Click(TObject *Sender);
        void __fastcall plotMouseMove(TObject *Sender, TShiftState Shift,
          int X, int Y);
        void __fastcall Button2Click(TObject *Sender);
        void __fastcall Button3Click(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TVentana(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TVentana *Ventana;
//---------------------------------------------------------------------------
#endif
