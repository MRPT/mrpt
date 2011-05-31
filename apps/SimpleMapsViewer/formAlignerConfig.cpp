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

#include "formAlignerConfig.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "sgr_data"
#pragma link "sgr_def"
#pragma resource "*.dfm"

TformAligner *formAligner;

// Include MRML library
#include <MRPT/MRML/mrml.h>
using namespace MRML;

extern CSensFrameProbSequence          posesAndSFs;


//---------------------------------------------------------------------------
__fastcall TformAligner::TformAligner(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TformAligner::btnComputeClick(TObject *Sender)
{
  try
  {
        CSensoryFrame                 *sf;
        CPose3DPDF                        *dummyPDF;
        CMetricMap                      *globalMap, *localMap;
        CMetricMapsAlignmentAlgorithm   *aligner;

        // Get params:
        int             refIndx = StrToInt( formAligner->edRefIndex->Text );
        int             obsIndx = StrToInt( formAligner->edIndex->Text );

        int             method = formAligner->rgMethod->ItemIndex;

        CPose2D         initialEst( StrToFloat( formAligner->edX->Text),
                                    StrToFloat( formAligner->edY->Text),
                                    DEG2RAD((float)StrToFloat( formAligner->edPhi->Text)) );

        // -------------------------------
        //    ICP
        // -------------------------------
        if(method==0 || method==1)
        {
                // Algoritm:
                aligner = new CICP();

                ((CICP*)aligner)->options.maxIterations = StrToInt( formAligner->edMaxIters->Text );
                ((CICP*)aligner)->options.thresholdDist =  StrToFloat( formAligner->edMaxDist->Text );
                ((CICP*)aligner)->options.smallestThresholdDist = StrToFloat( formAligner->edSmallestDist->Text );
                ((CICP*)aligner)->options.ALFA = StrToFloat( formAligner->edAlfa->Text );

                if (method==0)
                {
                        // Global map is points:
                        posesAndSFs.get(refIndx, dummyPDF, sf);
                        globalMap = new CSimplePointsMap();
                        ((CSimplePointsMap*)globalMap)->insertionOptions.minDistBetweenLaserPoints = 0.1;
                        ((CSimplePointsMap*)globalMap)->insertionOptions.also_interpolate = true;
                }
                else
                {
                        // Global map is grid:
                        posesAndSFs.get(refIndx, dummyPDF, sf);
                        globalMap = new COccupancyGridMap2D(-10,10,-10,10,0.10f);
                }

                sf->insertObservationsInto( globalMap );

                // Local map is a points map:
                posesAndSFs.get(obsIndx, dummyPDF, sf);
                localMap = new CSimplePointsMap();
                ((CSimplePointsMap*)localMap)->insertionOptions.minDistBetweenLaserPoints = 0.1;
                ((CSimplePointsMap*)localMap)->insertionOptions.also_interpolate = true;
                
                sf->insertObservationsInto( localMap );
        }

        // Align:
        float           runTime;
        CPosePDF        *poseEst = aligner->Align( globalMap,
                        localMap,
                        initialEst,
                        &runTime );

        // Show result:
        AnsiString      res;
        CPose2D         estMean = poseEst->getEstimatedPose();
        CMatrix         estCov  = poseEst->getEstimatedCovariance();
        delete poseEst; poseEst=NULL;

        res.sprintf("Time:%fms\n EstPose=(%f,%f,%fdeg)\n COV:\n %3.3e %3.3e %3.3e\n %3.3e %3.3e %3.3e\n %3.3e %3.3e %3.3e\n",
                runTime*1e3,
                estMean.x,
                estMean.y,
                RAD2DEG(estMean.phi),
                estCov(0,0),estCov(0,1),estCov(0,2),
                estCov(1,0),estCov(1,1),estCov(1,2),
                estCov(2,0),estCov(2,1),estCov(2,2)
                );
        //log->Lines->Clear();
        //log->Lines->Add(res);


        // Plot:
        int     i, n;
        float   x,y;

        // Global:
        if ( globalMap->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap)) )
        {
                CPointsMap      *map = (CPointsMap*) globalMap;

                p1->LockInvalidate = true;
                p1->Clear();

                n = map->getPointsCount();
                for (i=0;i<n;i++)
                {
                        map->getPoint(i,x,y);
                        p1->AddXY(x,y);
                }
                p1->LockInvalidate = false;
        }

        // Local:
        if ( localMap->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap)) )
        {
                CPointsMap      *map = (CPointsMap*) localMap;

                p2->LockInvalidate = true;
                p2->Clear();

                n = map->getPointsCount();
                for (i=0;i<n;i++)
                {
                        map->getPoint(i,x,y);
                        float nx = estMean.x + cos(estMean.phi) * x - sin(estMean.phi)*y;
                        float ny = estMean.y + sin(estMean.phi) * x + cos(estMean.phi)*y;
                        p2->AddXY(nx,ny);
                }
                p2->LockInvalidate = false;
        }


        // Delete:
        delete aligner;
        delete globalMap;
        delete localMap;
  }
  catch (std::exception &e)
  {
        ShowMessage(e.what());
  }
}
//---------------------------------------------------------------------------


void __fastcall TformAligner::Button1Click(TObject *Sender)
{
        int     nMaxIters = StrToInt( edMaxIters->Text );

        for (int i=1;i<=nMaxIters;i++)
        {
                edMaxIters->Text = IntToStr(i);

                btnComputeClick(this);                



                Application->ProcessMessages();
                Sleep(500);
        }

}
//---------------------------------------------------------------------------

