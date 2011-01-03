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

#include <vcl.h>
#pragma hdrstop

#include "unFormLuMilios.h"


#include <MRPT/MRML/mrml.h>
#include <MRPT/UTILS/utils.h>
using namespace MRML;
using namespace UTILS;

extern CSimpleMap          posesAndSFs;

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"

TformLuMilios *formLuMilios;


class CDebugOutput : public CStdOutStream
{
	protected:
		/** Method responsible for writing to the stream.
		 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		 */
		size_t Write(const void *Buffer,size_t Count)
		{
			char *s = new char[Count+1];
			memcpy(s,Buffer,Count);
			s[Count]='\0';
			formLuMilios->logMemo->Lines->Add(s);
            Application->ProcessMessages();
			delete[] s;
            return Count;
		}

} debugOut;

//---------------------------------------------------------------------------
__fastcall TformLuMilios::TformLuMilios(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------

void __fastcall TformLuMilios::btnExecuteClick(TObject *Sender)
{
	int		nIters = StrToInt( edIters->Text );

    logMemo->Clear();

	CConsistentObservationAlignment		consistent;
    CSensFrameProbSequence				in_map, out_map;
    AnsiString							s;
    CTicTac								tictac;

    consistent.debugOut	= &debugOut;

	//  -------------- METHOD OPTIONS --------------
    consistent.options.matchAgainstGridmap = true;

	consistent.options.pointsMapOptions.minDistBetweenLaserPoints	= 0.03f;
	consistent.options.pointsMapOptions.also_interpolate				= false;

    consistent.options.gridMapsResolution						= 0.03f;

	consistent.options.icpOptions.thresholdAng					= (float) DEG2RAD(1);
	consistent.options.icpOptions.thresholdDist					= 0.25f;
	consistent.options.icpOptions.smallestThresholdDist			= 0.10f;
	consistent.options.icpOptions.onlyClosestCorrespondences	= true;
	//  --------------------------------------------

    in_map = posesAndSFs;

    Screen->Cursor = crHourGlass;

    try
    {
		// Execute the method:
		for (int iter=0;iter<nIters;iter++)
		{
    	    s.sprintf("\nITER#%u -------------------------",iter);
			logMemo->Lines->Add(s);
			s.sprintf("Executing the method...");
			logMemo->Lines->Add(s);

			tictac.Tic();
			consistent.execute(in_map, out_map);

	        s.sprintf("Done! in %.03fms", (float)(1000*tictac.Tac()) );
			logMemo->Lines->Add(s);

            Application->ProcessMessages();
            Sleep(1);

			in_map = out_map;
		}

    	posesAndSFs = out_map;
    }
    catch(std::exception &e)
    {
    	ShowMessage(AnsiString("Exception while reading file: ")+AnsiString(e.what()));
    }
    catch(...)
    {
    	ShowMessage("Unknown exception while reading file!");
    }

    Screen->Cursor = crDefault;

}
//---------------------------------------------------------------------------
 
