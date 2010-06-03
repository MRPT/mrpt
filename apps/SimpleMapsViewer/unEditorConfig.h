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

#ifndef unEditorConfigH
#define unEditorConfigH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <SynEdit.hpp>
#include <SynMemo.hpp>
#include <ExtCtrls.hpp>
#include "SynEditHighlighter.hpp"
#include "SynHighlighterIni.hpp"
//---------------------------------------------------------------------------
class TformMapsConfig : public TForm
{
__published:	// IDE-managed Components
	TSynMemo *memo;
	TPanel *Panel1;
	TButton *Button1;
	TSynIniSyn *SynIniSyn1;
private:	// User declarations
public:		// User declarations
	__fastcall TformMapsConfig(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TformMapsConfig *formMapsConfig;
//---------------------------------------------------------------------------
#endif
