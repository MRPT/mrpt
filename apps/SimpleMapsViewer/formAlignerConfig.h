/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#ifndef formAlignerConfigH
#define formAlignerConfigH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <Buttons.hpp>
#include <ExtCtrls.hpp>
#include <ComCtrls.hpp>
#include "sgr_data.hpp"
#include "sgr_def.hpp"
//---------------------------------------------------------------------------
class TformAligner : public TForm
{
__published:	// IDE-managed Components
        TBitBtn *btnCompute;
        TLabel *Label1;
        TEdit *edRefIndex;
        TLabel *Label2;
        TRadioGroup *rgMethod;
        TPageControl *PageControl1;
        TTabSheet *TabSheet1;
        TTabSheet *TabSheet2;
        TLabel *Label3;
        TEdit *edMaxDist;
        TLabel *Label4;
        TEdit *edMaxIters;
        TLabel *Label5;
        TEdit *edMaxDistCorr;
        TLabel *Label6;
        TEdit *edRangeXY;
        TLabel *Label7;
        TEdit *edRangePHI;
        TLabel *Label8;
        TEdit *edStepXY;
        TLabel *Label9;
        TEdit *edStepPHI;
        TLabel *Label10;
        TLabel *Label11;
        TGroupBox *GroupBox1;
        TLabel *Label12;
        TEdit *edX;
        TEdit *edY;
        TLabel *Label13;
        TEdit *edPhi;
        TLabel *Label14;
        TLabel *Label15;
        TLabel *Label16;
        TEdit *edIndex;
        Tsp_XYPlot *plot;
        Tsp_XYLine *p1;
        Tsp_XYLine *p2;
        TMemo *log;
        TLabel *Label17;
        TEdit *edSmallestDist;
        TButton *Button1;
        TLabel *Label18;
        TEdit *edAlfa;
        void __fastcall btnComputeClick(TObject *Sender);
        void __fastcall Button1Click(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TformAligner(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TformAligner *formAligner;
//---------------------------------------------------------------------------
#endif
