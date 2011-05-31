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

#include "mainUnit.h"

#include "../ReactiveNavigatorLibrary/CPTG1.h"
#include "../ReactiveNavigatorLibrary/CPTG2.h"
#include "../ReactiveNavigatorLibrary/CPTG3.h"
#include "../ReactiveNavigatorLibrary/CPTG4.h"
#include "../ReactiveNavigatorLibrary/CPTG5.h"
#include "../ReactiveNavigatorLibrary/CPTG6.h"
#include "../ReactiveNavigatorLibrary/CPTG7.h"

#include <MRML/CPose2D.h>
#include <MRML/CPoint2D.h>

using namespace MRML;


//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "sgr_data"
#pragma link "sgr_def"
#pragma resource "*.dfm"


#include <UTILS/utils.h>

using namespace MRPTAPPS::RNAV;

TVentana *Ventana;

CParameterizedTrajectoryGenerator       *PTG = NULL;
UTILS::CTicTac                           tictac;


//---------------------------------------------------------------------------
__fastcall TVentana::TVentana(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TVentana::btnGenerateClick(TObject *Sender)
{
        //
        if (PTG)
        {
                delete PTG;
                PTG = NULL;
        }

        DecimalSeparator='.';

        // Recoger parametros generales:
        float   refDist, resolutionX, resolutionY,V_MAX,W_MAX;
        refDist         = StrToFloat(edRefDist->Text);
        resolutionX     = StrToFloat(edRes->Text);
        resolutionY     = StrToFloat(edRes->Text);
        V_MAX           = StrToFloat(edVM->Text);
        W_MAX           = DEG2RAD( (float)StrToFloat(edWM->Text) );
        float TAU       = StrToFloat(edTAU->Text);
        float DELAY     = StrToFloat(edDELAY->Text);


		float 	K      	= StrToFloat(edK->Text);
        float   cte_a0v = DEG2RAD((float)StrToFloat(edA0V->Text));
        float   cte_a0w = DEG2RAD((float)StrToFloat(edA0W->Text));

        std::vector<float>	securityDistances;
        securityDistances.push_back(0);


        switch( rgPTG->ItemIndex )
        {
        default:
        case 0:
                {
                        PTG = new  CPTG1(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        K);
                }
                break;
        case 1:
                {
                        PTG = new CPTG2(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        cte_a0v,
                                        cte_a0w);
                }
                break;
        case 2:
                {
                        PTG = new  CPTG3(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        K
                                        );
                }
                break;
        case 3:
                {
                        PTG = new  CPTG4(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        K
                                        );
                }
                break;
        case 4:
                {
                        PTG = new  CPTG5(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        K
                                        );
                }
                break;
        case 5:
                {
                        PTG = new  CPTG6(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        cte_a0v,
                                        cte_a0w);
                }
                break;
        case 6:
                {
                        PTG = new  CPTG7(
                                        refDist,
                                        resolutionX,
                                        resolutionY,
                                        V_MAX,
                                        W_MAX,
                                        TAU,
                                        DELAY,
                                        securityDistances,
                                        cte_a0v,
                                        cte_a0w);
                }
                break;
        }


        // Generar trajectorias:
        // ---------------------------
        int     alfaValuesCount = StrToInt( edAlfas->Text );
        float   max_time        = StrToFloat( edMaxTime->Text );
        float   max_dist        = StrToFloat( edMaxDist->Text );
        float   diferencial_t   = StrToFloat( edAT->Text );
        float   min_dist        = StrToFloat( edMinDist->Text );
        int     max_n           = StrToInt( edMaxN->Text );

        float   max_acc_v,max_acc_w;

        Screen->Cursor=crHourGlass;
        PTG->simulateTrajectories( alfaValuesCount,
                                   max_time,
                                   max_dist,
                                   max_n,
                                   diferencial_t,
                                   min_dist,
                                   &max_acc_v,
                                   &max_acc_w );
        Screen->Cursor=crDefault;

        // Out:
        // ---------
        memOut->Lines->Clear();
        memOut->Lines->Add( "Results from simulations:" );
        memOut->Lines->Add( "Max.lin.acc.: " +FloatToStr(max_acc_v)+"m/s2" );
        memOut->Lines->Add( "Max.ang.acc.: " +FloatToStr(RAD2DEG(max_acc_w))+"º/s2" );

        // Draw:
        //----------------
        pTrajs->LockInvalidate = true;
        pTrajs->Clear();
        pSelTraj->Clear();

        for (int k=0;k<alfaValuesCount;k++)
        {
                int n;
                for (n=0;n<PTG->getPointsCountInCPath_k(k);n++)
                {
                        pTrajs->QuickAddXY( PTG->GetCPathPoint_x(k,n),PTG->GetCPathPoint_y(k,n) );
                } // for n
                for (n=PTG->getPointsCountInCPath_k(k)-1;n>=0;n--)
                {
                        pTrajs->QuickAddXY( PTG->GetCPathPoint_x(k,n),PTG->GetCPathPoint_y(k,n) );
                } // for n
        } // for k

        pTrajs->LockInvalidate = false;


        // Ajustar escala:
                plot->LeftAxis->AutoMin = true;
                plot->LeftAxis->AutoMax = true;
                plot->BottomAxis->AutoMin = true;
                plot->BottomAxis->AutoMax = true;

                plot->LeftAxis->AutoMin = false;
                plot->LeftAxis->AutoMax = false;
                plot->BottomAxis->AutoMin = false;
                plot->BottomAxis->AutoMax = false;

                // Adjust aspect ratio:
                double   Ax = plot->BottomAxis->Max - plot->BottomAxis->Min;
                double   Ay = plot->LeftAxis->Max - plot->LeftAxis->Min;
                double   cx = 0.5*(plot->BottomAxis->Max + plot->BottomAxis->Min);
                double   cy = 0.5*(plot->LeftAxis->Max + plot->LeftAxis->Min);
                if (Ay>Ax)
                {
                        plot->BottomAxis->Max = cx + 0.5*Ay;
                        plot->BottomAxis->Min = cx - 0.5*Ay;
                }
                else
                {
                        plot->LeftAxis->Max = cy + 0.5*Ax;
                        plot->LeftAxis->Min = cy - 0.5*Ax;
                }


        // Draw:
        //----------------
        trajsTarget->LockInvalidate = true;
        trajsTarget->Clear();

        for (k=0;k<alfaValuesCount;k++)
        {
                int n = PTG->getPointsCountInCPath_k(k);
                CPoint2D	targ( PTG->GetCPathPoint_x(k,n-1),PTG->GetCPathPoint_y(k,n-1) );
                for (n=PTG->getPointsCountInCPath_k(k)-1;n>=0;n--)
                {
                	CPose2D	curRobotPose( PTG->GetCPathPoint_x(k,n),PTG->GetCPathPoint_y(k,n), PTG->GetCPathPoint_phi(k,n) );
                    CPoint2D	relTarg( targ - curRobotPose );
                    trajsTarget->QuickAddXY( relTarg.x, relTarg.y );
                } // for n
                for (n=0;n<PTG->getPointsCountInCPath_k(k);n++)
                {
                	CPose2D	curRobotPose( PTG->GetCPathPoint_x(k,n),PTG->GetCPathPoint_y(k,n), PTG->GetCPathPoint_phi(k,n) );
                    CPoint2D	relTarg( targ - curRobotPose );
                    trajsTarget->QuickAddXY( relTarg.x, relTarg.y );
                } // for n

        } // for k

        trajsTarget->LockInvalidate = false;




        // Dibujar funciones de diseño:
        pV->LockInvalidate = true;
        pW->LockInvalidate = true;
        pV->Clear();
        pW->Clear();

        for (float a= -3.14;a<3.14;a+=DEG2RAD(1))
        {
                float   v,w;
                PTG->PTG_Generator( a,0,0,0,0,v,w );

                pV->AddXY(a,v);
                pW->AddXY(a,RAD2DEG(w));
        }

        pV->LockInvalidate = false;
        pW->LockInvalidate = false;
}
//---------------------------------------------------------------------------
void __fastcall TVentana::Button1Click(TObject *Sender)
{
        if (!PTG) return;


        // Draw:
        //----------------
        pSelTraj->LockInvalidate = true;
        pSelTraj->Clear();

        int k= PTG->alfa2index( StrToFloat(edSelAlfa->Text) );

        for (int n=0;n<PTG->getPointsCountInCPath_k(k);n++)
                pSelTraj->AddXY( PTG->GetCPathPoint_x(k,n),PTG->GetCPathPoint_y(k,n) );

        pSelTraj->LockInvalidate = false;

}
//---------------------------------------------------------------------------
void __fastcall TVentana::plotMouseMove(TObject *Sender, TShiftState Shift,
      int X, int Y)
{
        float y = plot->LeftAxis->P2V( Y );
        float x = plot->BottomAxis->P2V( X );

        lbCursor1->Caption=AnsiString().sprintf("WS:(%.02f,%.02f)",x,y);

        if (!PTG)
        {
                lbCursor2->Caption="No PTG!!";
                return;
        }

        float a,d;
        int k;
        tictac.Tic();

        if (! PTG->PTG_IsIntoDomain(x,y) )
        {
	        lbCursor2->Caption= AnsiString().sprintf("PT-Space:NOT INTO THE PTG DOMAIN!");
        }
        else
        {
	        PTG->lambdaFunction(x,y,k,d);
    	    a=PTG->index2alfa(k);
        	float T = tictac.Tac();
	        lbCursor2->Caption= AnsiString().sprintf("PT-Space:a=%.02f,d=%.02f (T=%.02fus)",a,d,T*1e6);
        }


}
//---------------------------------------------------------------------------
void __fastcall TVentana::Button2Click(TObject *Sender)
{
        SD->Title="File for 3D surface:";
        SD->FileName = "ptg_data.txt";//(PTG->getDescription()+std::string(".txt")).c_str();
        if (!SD->Execute()) return;

        FILE    *f=fopen(SD->FileName.c_str(),"wt");


        int alfaValuesCount = PTG->getAlfaValuesCount();

        for (int k=0;k<alfaValuesCount;k++)
        {
                for (int n=0;n<PTG->getPointsCountInCPath_k(k);n++)
                {
                        float phi = PTG->GetCPathPoint_phi(k,n);
//                        if (phi>M_PI)  phi-=(float)M_2PI;
//                        if (phi<-M_PI) phi+=(float)M_2PI;

                        fprintf(f,"%f %f %f\n",
                                PTG->GetCPathPoint_x(k,n),
                                PTG->GetCPathPoint_y(k,n),
                                phi );
                }
        } // for k

        fclose(f);
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Button3Click(TObject *Sender)
{
        if (!PTG) return;

        ShowMessage("A text file will be saved with 3 columns: alfa values, v and w actuations for that alfa value");

        //
        SD->Title="File for V(a)";
        SD->FileName = (std::string("V_")+PTG->getDescription()+std::string(".txt")).c_str();
        if (!SD->Execute()) return;
        FILE    *fv = fopen( SD->FileName.c_str(),"wt" );

        for (float a= -M_PI;a<M_PI;a+=DEG2RAD(0.5))
        {
                float   v,w;

                PTG->PTG_Generator( a,0,0,0,0,v,w );
                fprintf(fv,"%f %f %f\n",a,v,w);
        }

        fclose(fv);
}
//---------------------------------------------------------------------------


