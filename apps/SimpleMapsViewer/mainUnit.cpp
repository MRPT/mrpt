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

#include <MRPT/MRML/mrml.h>
#include <MRPT/UTILS/utils.h>
using namespace UTILS;
using namespace MRML;


#include <registry.hpp>

#include "mainUnit.h"
#include "formChangeCamerasPose.h"
#include "formAlignerConfig.h"

// Include MRML library
#include "unFormLuMilios.h"

#include "formPointsList.h"
#include "unEditorConfig.h"



//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "sgr_data"
#pragma link "sgr_def"
#pragma resource "*.dfm"
TVentana *Ventana;

CTicTac         				tictac;
CSensFrameProbSequence          posesAndSFs;
CMultiMetricMap                *simpleMap=NULL;
AnsiString                      simpleMapName = "Unnamed.simplemap";

CPoint2D					leftClickOnMap,
							rightClickOnMap;

//---------------------------------------------------------------------------
__fastcall TVentana::TVentana(TComponent* Owner)
        : TForm(Owner)
{
	MRML::registerAllClasses();
	DecimalSeparator = '.';

	// Create the maps:
	TSetOfMetricMapInitializers			mapInitializer;
	TMetricMapInitializer				mapElement;

	mapElement.metricMapClassType = CLASS_ID( CSimplePointsMap );
	mapInitializer.push_back( mapElement );

	mapElement.metricMapClassType = CLASS_ID( CLandmarksMap );
	mapInitializer.push_back( mapElement );

	mapElement.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
	mapElement.occupancyGridMap2D_options.resolution = 0.05;
	mapInitializer.push_back( mapElement );
	simpleMap = new CMultiMetricMap( &mapInitializer );
}
//---------------------------------------------------------------------------
void __fastcall TVentana::ToolButton3Click(TObject *Sender)
{
        Close();
}
/* -----------------------------------------------------------------------
                        RebuildTreeView
   ----------------------------------------------------------------------- */
void __fastcall TVentana::RebuildTreeView()
{
        int     i,j,n,m;
        tv->Items->Clear();

        TStringList             *sl = new TStringList();
        std::vector<int>        slCount;

        // Raiz:
        TTreeNode *root = tv->Items->Add(NULL,"Map: '"+simpleMapName+"'");
        root->ImageIndex = 0;
        root->SelectedIndex = 0;
        root->Data			= NULL;


        // Añadir una entrada por SF:
        n = posesAndSFs.size();
        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;
                AnsiString              s;

                // get:
                posesAndSFs.get(i, posePDF, SF);

                // Observations count:
                m = SF->size();

                // Añadir nodo para el SF:
                TTreeNode *nodoSF = tv->Items->AddChild(root,"SensorialFrame #"+IntToStr(i)+"");
                nodoSF->ImageIndex = 1;
                nodoSF->SelectedIndex = 1;
                nodoSF->Data 		  = NULL;

                // Nodo para la pose del SF:
                s.sprintf("Robot pose ('%s'),mean=(%.02f,%.02f,%.01fdeg)",
                        posePDF->GetRuntimeClass()->className,
                        posePDF->getEstimatedPose().x,
                        posePDF->getEstimatedPose().y,
                        RAD2DEG( posePDF->getEstimatedPose().yaw ) );
                TTreeNode *nodoSF_Pose = tv->Items->AddChild( nodoSF, s);
                nodoSF_Pose->ImageIndex         = 2;
                nodoSF_Pose->SelectedIndex      = 2;
                nodoSF_Pose->Data               = (void*) posePDF;

                // Nodo para el ID del SF:
                s.sprintf("ID: %u",SF->ID);
                TTreeNode *nodoSF_ID = tv->Items->AddChild( nodoSF, s);
                nodoSF_ID->ImageIndex         = 2;
                nodoSF_ID->SelectedIndex      = 2;
                nodoSF_ID->Data               = NULL;

                // Nodo para las obs del SF:
                TTreeNode *nodoSF_Obss = tv->Items->AddChild( nodoSF, "Observations");
                nodoSF_Obss->ImageIndex = 1;
                nodoSF_Obss->SelectedIndex      = 1;
                nodoSF_Obss->Data				= NULL;

                for (j=0;j<m;j++)
                {
                        CObservation    *o = SF->getObservationByIndex(j);
                        int             idx;

                        // Add a node per observation:
                        const char    *className = o->GetRuntimeClass()->className;
                        TTreeNode *nodoSF_obs = tv->Items->AddChild( nodoSF_Obss, className);
                        nodoSF_obs->ImageIndex = 3;
                        nodoSF_obs->SelectedIndex = 3;
                        nodoSF_obs->Data		  = (void*)o;

                        // Count observations of each type:
                        if (-1!=(idx=sl->IndexOf(className)))
                        {
                                slCount[idx]++;
                        }
                        else
                        {
                                sl->Add(className);
                                slCount.push_back(1);
                        }
                }
        }

        // Load summary:
        // ---------------------------
        memLoad->Lines->Clear();

        for (j=0;j<slCount.size();j++)
        {
                AnsiString      str;
                str.sprintf("%04u    %s",slCount[j],sl->Strings[j].c_str());
                memLoad->Lines->Add(str);
        }


        delete sl;
}

void __fastcall TVentana::FormShow(TObject *Sender)
{
        RebuildTreeView();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton1Click(TObject *Sender)
{
        if (Sender)
        {
                TRegistryIniFile                *iniFile;
                iniFile = new TRegistryIniFile( "JLBC_SimpleMapsViewer" );

                // Load directory:
                OD->InitialDir = iniFile->ReadString("Config","LastDirectory",".");

                if (!OD->Execute()) return;

                // Save directory:
                iniFile->WriteString("Config","LastDirectory", ExtractFilePath( OD->FileName ) );

                delete  iniFile;
        }
        else
        {       // Autoload
        }


        CFileStream     f( OD->FileName.c_str(), fomRead);

        // File extension:
        if (!AnsiCompareText( ExtractFileExt(OD->FileName),".hybridmetricmap"))
        {
                // Es un mapa metrico:

                try
                {
                        f >> simpleMap;

                        RebuildPointsMapView();
                        RebuildGridMapView();
                        RebuildLandmarksMapView();
                }
                catch(...)
                {
                        ShowMessage("Unknown exception while reading file!");
                }

                return;
        }
        else
        if (!AnsiCompareText( ExtractFileExt(OD->FileName),".gridmap"))
        {
                // Es un mapa de grid:

                try
                {
                		ASSERT_( simpleMap->m_gridMaps.size()>0 );
                        f >> (*simpleMap->m_gridMaps[0]);
                        RebuildGridMapView();
                }
                catch(...)
                {
                        ShowMessage("Unknown exception while reading file!");
                }

                return;
        }

        // Es un ".simplemap"

        tictac.Tic();

        try
        {
                SB->Panels->Items[0]->Text = "LOADING...";
                Screen->Cursor = crHourGlass;

                Application->ProcessMessages();

        	// Load from file:
        	f >> posesAndSFs;

        }
        catch(std::exception &e)
        {
                        ShowMessage(AnsiString("Exception while reading file: ")+AnsiString(e.what()));
        }
        catch(...)
        {
                ShowMessage("Unknown exception while reading file!");
        }

        simpleMapName = ExtractFileName( OD->FileName );


        SB->Panels->Items[0]->Text = AnsiString().sprintf("Map loaded in %.03fm: %i Sensorial frames / ...", 1000*tictac.Tac(), posesAndSFs.size() );

        Screen->Cursor = crAppStart;

        // Update views:
        // --------------------
        RebuildSimpleMapAndViews();


        Screen->Cursor = crDefault;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::RebuildSimpleMap()
{
	try
    {
		// Parse to hybrid map:
		TSetOfMetricMapInitializers			mapInitializer;
	    CStringList							configStrs;

	    // Load the strings from the memo:
	    configStrs.setText( std::string( formMapsConfig->memo->Lines->Text.c_str() ) );

	    // Create a "memory config file":
	    CConfigFileMemory					memIniFile(configStrs);

	    mapInitializer.loadFromConfigFile( memIniFile,"MetricMap" );
		simpleMap->setListOfMaps( &mapInitializer );

	     tictac.Tic();

	     SB->Panels->Items[3]->Text = AnsiString().sprintf(" *** REBUILDING METRIC MAPS... ***");
	     Application->ProcessMessages();

//	     simpleMap->loadFromProbabilisticPosesAndObservations( posesAndSFs );
		CPose3DPDF			*posePDF;
		CSensoryFrame		*sf;
		int					i,n = posesAndSFs.size();

		// Erase previous contents:
		simpleMap->clear();


		// Insert new content:
		for (i=0;i<n;i++)
		{
	        CPose3D		robotPose;	// For insertion:

			posesAndSFs.get(i,posePDF, sf);

            if (posePDF->GetRuntimeClass() == CLASS_ID(CPosePDFParticles))
            {
            	robotPose = CPose3D( *((CPosePDFParticles*)posePDF)->m_particles[0].d );
            }
            else
            {
            	robotPose = posePDF->getEstimatedPose();
            }


			sf->insertObservationsInto(
					simpleMap,		// Insert into THIS map.
					&robotPose	// At this pose.
					);
		}



	     SB->Panels->Items[3]->Text = AnsiString().sprintf("CMultiMetricMap built in %.03fm", 1000*tictac.Tac());

    }
    catch(std::exception &e)
    {
    	ShowMessage(e.what());
    }

}

void __fastcall TVentana::timAutoLoadTimer(TObject *Sender)
{
        timAutoLoad->Enabled = false;

        // Load file?
        AnsiString      cmdLine = ParamStr(1);
        if ( !FileExists(cmdLine)) return;

        OD->FileName = cmdLine;

        ToolButton1Click(NULL);

}
//---------------------------------------------------------------------------
void __fastcall TVentana::RebuildPointsMapView()
{
        pPointsMap->LockInvalidate = true;
        pPointsMap->Clear();
        pPlannedPath->Clear();
        pSimScan->Clear();

  if( simpleMap->m_pointsMap )
  {
        int i,n = simpleMap->m_pointsMap->getPointsCount();

        for (i=0;i<n;i++)
        {
                float x,y;
                simpleMap->m_pointsMap->getPoint(i,x,y);
                pPointsMap->QuickAddXY(x,y);
        }

        // Ajustar escala:
        adjustAspectRatio(plot_PointsMap);
  }
}


void __fastcall TVentana::plot_PointsMapAxisZoom(Tsp_Axis *Sender,
      double &min, double &max, bool &CanZoom)
{
        if ( Sender == plot_PointsMap->LeftAxis)
        {
                // Ya se ha cambiado el eje X, ahora va a cambiar el Y:
                // -----------------------------------------------------
                double   Ax = plot_PointsMap->BottomAxis->Max - plot_PointsMap->BottomAxis->Min;
                double   Ay = max - min;
                double   cx = 0.5*(plot_PointsMap->BottomAxis->Max + plot_PointsMap->BottomAxis->Min);
                double   cy = 0.5*(max+min);
                if (Ay>Ax)
                {
                        plot_PointsMap->BottomAxis->Max = cx + 0.5*Ay;
                        plot_PointsMap->BottomAxis->Min = cx - 0.5*Ay;
                }
                else
                {
                        max = cy + 0.5*Ax;
                        min = cy - 0.5*Ax;
                }



        }
}
//---------------------------------------------------------------------------
bool    clickQuitarZoom = false;


void __fastcall TVentana::plot_PointsMapMouseUp(TObject *Sender,
      TMouseButton Button, TShiftState Shift, int X, int Y)
{
        if ( Shift.Contains(Classes::ssShift) && clickQuitarZoom)
        {
        // Ajustar escala:
                plot_PointsMap->LeftAxis->AutoMin = true;
                plot_PointsMap->LeftAxis->AutoMax = true;
                plot_PointsMap->BottomAxis->AutoMin = true;
                plot_PointsMap->BottomAxis->AutoMax = true;

                plot_PointsMap->LeftAxis->AutoMin = false;
                plot_PointsMap->LeftAxis->AutoMax = false;
                plot_PointsMap->BottomAxis->AutoMin = false;
                plot_PointsMap->BottomAxis->AutoMax = false;

                // Adjust aspect ratio:
                double   Ax = plot_PointsMap->BottomAxis->Max - plot_PointsMap->BottomAxis->Min;
                double   Ay = plot_PointsMap->LeftAxis->Max - plot_PointsMap->LeftAxis->Min;
                double   cx = 0.5*(plot_PointsMap->BottomAxis->Max + plot_PointsMap->BottomAxis->Min);
                double   cy = 0.5*(plot_PointsMap->LeftAxis->Max + plot_PointsMap->LeftAxis->Min);
                if (Ay>Ax)
                {
                        plot_PointsMap->BottomAxis->Max = cx + 0.5*Ay;
                        plot_PointsMap->BottomAxis->Min = cx - 0.5*Ay;
                }
                else
                {
                        plot_PointsMap->LeftAxis->Max = cy + 0.5*Ax;
                        plot_PointsMap->LeftAxis->Min = cy - 0.5*Ax;
                }

        }

}
//---------------------------------------------------------------------------

void __fastcall TVentana::plot_PointsMapMouseDown(TObject *Sender,
      TMouseButton Button, TShiftState Shift, int X, int Y)
{
        if ( Shift.Contains(Classes::ssShift) )
                clickQuitarZoom = true;

	float x = plot_PointsMap->BottomAxis->P2V( X );
    float y = plot_PointsMap->LeftAxis->P2V( Y );

    if (formList->Visible)
    {
    	AnsiString str;
    	str.sprintf("%.03f %.03f",x,y);
    	formList->mem->Lines->Add( str);
    }

        // Guardar coordenadas de clic izq y der:
        if ( Button == mbLeft )
        {
        	leftClickOnMap.x = x;
        	leftClickOnMap.y = y;
        }
        else if ( Button == mbRight )
        {
        	rightClickOnMap.x = x;
        	rightClickOnMap.y = y;
        }
}
//---------------------------------------------------------------------------

void __fastcall TVentana::plot_PointsMapMouseMove(TObject *Sender,
      TShiftState Shift, int X, int Y)
{
        if ( Shift.Contains(Classes::ssShift) )
                clickQuitarZoom = false;

        SB->Panels->Items[1]->Text =
          AnsiString().sprintf("(%.03f,%.03f)",
        	plot_PointsMap->BottomAxis->P2V( X ),
        	plot_PointsMap->LeftAxis->P2V( Y ) );

}
//---------------------------------------------------------------------------
void __fastcall TVentana::tvClick(TObject *Sender)
{
        // Se ha seleccionado un elemento:
        TTreeNode       *node = tv->Selected;
        if (!node) return;

	// Habilitar tabs segun el tipo de nodo marcado:
    try
    {
        pScan->Clear();
        tsCObservationImage->TabVisible = false;
    	tsRobotPose->TabVisible = false;
        tsCObservation2DRangeScan->TabVisible = false;
        tsCObservationStereoImages->TabVisible = false;

    	CSerializable	*obj = (CSerializable	*)node->Data;
    	if (obj)
    	{
            if (obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CPosePDF)) )
            {
                tsRobotPose->TabVisible = true;

                CPosePDF        *pdf = (CPosePDF*) obj;

                CPose2D		mean = pdf->getEstimatedPose();
                CMatrix		cov = pdf->getEstimatedCovariance();
                AnsiString	s;

                memPose->Clear();

                s.sprintf("Pose: mean=(%.03f,%.03f,%.03fdeg)", mean.x,mean.y,RAD2DEG(mean.phi));
                memPose->Lines->Add(s);

                s.sprintf("   Covariance matrix="); memPose->Lines->Add(s);
                s.sprintf("        %12.3e %12.3e %12.3e", cov(0,0),cov(0,1),cov(0,2)); memPose->Lines->Add(s);
                s.sprintf("        %12.3e %12.3e %12.3e", cov(1,0),cov(1,1),cov(1,2)); memPose->Lines->Add(s);
                s.sprintf("        %12.3e %12.3e %12.3e", cov(2,0),cov(2,1),cov(2,2)); memPose->Lines->Add(s);

                if (obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CPosePDFParticles)))
                {
	                memPose->Lines->Add("");
                    memPose->Lines->Add("List of individual particles:");

                    CPosePDFParticles	*parts = (CPosePDFParticles*) obj;

                    for (unsigned int i=0;i<parts->m_particles.size();i++)
                    {
		                s.sprintf("[%03u]=(%.03f,%.03f,%.03fdeg)", i, parts->m_particles[i].d->x,parts->m_particles[i].d->y,RAD2DEG(parts->m_particles[i].d->phi));
        		        memPose->Lines->Add(s);
                    }
                }
            }

            if (obj->GetRuntimeClass()==CLASS_ID(CObservation2DRangeScan) )
            {
                CObservation2DRangeScan         *o = (CObservation2DRangeScan*) obj;
                CSimplePointsMap                temp;

                CPosePDF                       *pdf = (CPosePDF*)node->Parent->Parent->Item[0]->Data;

                if (pdf)
                {
                        CPose3D         robotPose( pdf->getEstimatedPose() );
                        o->insertObservationInto( &temp, &robotPose );

                        pScan->LockInvalidate = true;
                        pScan->Clear();

                        pScan->AddXY( pdf->getEstimatedPose().x, pdf->getEstimatedPose().y );

                        for (int i=0;i<temp.getPointsCount();i++)
                        {
                                float   x,y;
                                temp.getPoint(i,x,y);
                                pScan->AddXY(x,y);
                        }

                        pScan->AddXY( pdf->getEstimatedPose().x, pdf->getEstimatedPose().y );

                        pScan->LockInvalidate = false;

                        //pc->ActivePage = tsPointsMap;

                        tsCObservation2DRangeScan->TabVisible = true;

                        pc->ActivePage = tsCObservation2DRangeScan;

                        if ( tsCObservation2DRangeScan->OnShow )
                                tsCObservation2DRangeScan->OnShow( this );


                }
            }
            else if (obj->GetRuntimeClass()==CLASS_ID(CObservationImage) )
            {
                tsCObservationImage->TabVisible = true;
                pc->ActivePage = tsCObservationImage;

                if ( tsCObservationImage->OnShow )
                        tsCObservationImage->OnShow( this );
            }
            else if (obj->GetRuntimeClass()==CLASS_ID(CObservationStereoImages) )
            {
                tsCObservationStereoImages->TabVisible = true;
                pc->ActivePage = tsCObservationStereoImages;

                if ( tsCObservationStereoImages->OnShow )
                        tsCObservationStereoImages->OnShow( this );
            }

        }

    }
    catch (...) { }


      // Mas...
      try
      {

        switch (node->ImageIndex)
        {
        case 0: // Mapa global
                {

                } break;
        case 2: // Robot Pose
                {
                try
                {
                        CPosePDF        *robotPose = (CPosePDF*)node->Data;
                        CPose2D         estPose = robotPose->getEstimatedPose();
                        CPoint2D        p;

                        pRobotPose->LockInvalidate = true;
                        pRobotPose->Clear();

                        p=CPoint2D(0.00f, 0.00f); p=estPose+p; pRobotPose->AddXY( p.x, p.y  );
                        p=CPoint2D(0.50f, 0.00f); p=estPose+p; pRobotPose->AddXY( p.x, p.y  );
                        p=CPoint2D(0.40f, 0.08f); p=estPose+p; pRobotPose->AddXY( p.x, p.y  );
                        p=CPoint2D(0.50f, 0.00f); p=estPose+p; pRobotPose->AddXY( p.x, p.y  );
                        p=CPoint2D(0.40f,-0.08f); p=estPose+p; pRobotPose->AddXY( p.x, p.y  );

                        pRobotPose->LockInvalidate = false;
                }  catch(...)
                  {
                  }
                } break;


        };
      }
      catch(...)
      {
      }


}
//---------------------------------------------------------------------------
void __fastcall TVentana::RebuildGridMapView()
{
	if (simpleMap->m_gridMaps.size()==0) return;

//        imgGridMap->Stretch = false;
//        imgGridMap->AutoSize = true;

        //
        int sizeX = simpleMap->m_gridMaps[0]->getSizeX();
        int sizeY = simpleMap->m_gridMaps[0]->getSizeY();
        Graphics::TBitmap *bmp = new Graphics::TBitmap();

        // Create palette:
        LOGPALETTE      *defPal = (LOGPALETTE*) calloc( 1, sizeof(LOGPALETTE)+256*sizeof(PALETTEENTRY) );
        defPal->palVersion = 0x0300;
        defPal->palNumEntries = 256;

        for (int i=0;i<256;i++)
        {
                defPal->palPalEntry[i].peRed = i;
                defPal->palPalEntry[i].peGreen = i;
                defPal->palPalEntry[i].peBlue = i;
                defPal->palPalEntry[i].peFlags = 0;
        }

        // Create bitmap:
        bmp->HandleType = bmDIB;
        bmp->PixelFormat = pf8bit;
        bmp->Width = sizeX;
        bmp->Height = sizeY;
        bmp->Palette = CreatePalette( defPal );

        free( defPal );

        for (int y=0;y<sizeY;y++)
        {
                ::BYTE    *bmpLine  = (::BYTE*) bmp->ScanLine[sizeY-1-y];

#ifdef	CELL_SIZE_8BITS
                BYTE    *gridLine = (BYTE*) simpleMap->m_gridMaps[0]->getRow(y);
                if (gridLine)
                        memcpy(bmpLine,gridLine,sizeX);
#else
                ::WORD    *gridLine = (::WORD*) simpleMap->m_gridMaps[0]->getRow(y);
                if (gridLine)
                {
                        for (int x=0;x<sizeX;x++)
                                bmpLine[x] = (gridLine[x] & 0xFF00) >> 8;
                }
#endif
        }



        imgGridMap->Picture->Bitmap = bmp;

//        tbZoomGrid->Position = 10;
        tbZoomGridChange(this);
}

void __fastcall TVentana::RebuildLandmarksMapView()
{
	if (simpleMap->m_landmarksMap)
		lbLandmarksCount->Caption = IntToStr( simpleMap->m_landmarksMap->size() );
}


void __fastcall TVentana::tbZoomGridChange(TObject *Sender)
{
        float     Zoom = tbZoomGrid->Position / 10.0f;

        imgGridMap->AutoSize = false;
        imgGridMap->Stretch = true;
        int sizeX = simpleMap->m_gridMaps[0]->getSizeX();
        int sizeY = simpleMap->m_gridMaps[0]->getSizeY();

        imgGridMap->Width = sizeX * Zoom;
        imgGridMap->Height = sizeY * Zoom;

        pbGrid->Width = imgGridMap->Width;
        pbGrid->Height = imgGridMap->Height;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton5Click(TObject *Sender)
{
	int	 i1 = StrToInt( InputBox("Clip of map","First index to maintain:","0")	);
	int	 i2 = StrToInt( InputBox("Clip of map","Last index to maintain:",IntToStr(posesAndSFs.size()-1))	);

	CSensFrameProbSequence 	newMap;
    for (int i=0;i<posesAndSFs.size();i++)
    {
    	if (i1<=i && i<=i2)
        {
    		CPose3DPDF		*p;
        	CSensoryFrame *sf;

    		posesAndSFs.get(i,p,sf);
    		newMap.insert(p,sf);
        }
    }
    posesAndSFs.clear();

    posesAndSFs = newMap;

    // Update views:
    // --------------------
	RebuildSimpleMapAndViews();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton2Click(TObject *Sender)
{
    if (!SD->Execute()) return;

    Screen->Cursor = crHourGlass;

    CFileStream     f( SD->FileName.c_str(), fomWrite);
	// Save to file:
    f << posesAndSFs;

    Screen->Cursor = crDefault;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton7Click(TObject *Sender)
{
	CTicTac							tictac;
    CPathPlanningCircularRobot		planer;
    CPoint2D						org = leftClickOnMap,
									target = rightClickOnMap;
    std::deque<CPoint2D>			path;
    bool							notFound;

    planer.robotRadius 				= 0.25f;
    planer.minStepInReturnedPath 	= 1.0f;


    tictac.Tic();
    planer.computePath( simpleMap->m_gridMaps[0],
    				    &org,
                        &target,
                        path,
                        notFound);
    SB->Panels->Items[2]->Text =
    	AnsiString().sprintf(
          "Path found in %.03f ms", (float)(1000.0f*tictac.Tac()));



    if (notFound)
    {
    	ShowMessage("Planner reported no feasible path!!");
    	return;
    }

    pPlannedPath->LockInvalidate = true;
    pPlannedPath->Clear();

    // Tb en el grid:
    RebuildGridMapView();

    imgGridMap->Picture->Bitmap->Canvas->Pen->Color = clRed;
    imgGridMap->Picture->Bitmap->Canvas->Pen->Width = 2;

    imgGridMap->Picture->Bitmap->Canvas->MoveTo(
    	simpleMap->m_gridMaps[0]->x2idx(path[0].x),
        simpleMap->m_gridMaps[0]->getSizeY()-1-simpleMap->m_gridMaps[0]->y2idx(path[0].y) );

    for (int i=0;i<path.size();i++)
    {
	    pPlannedPath->AddXY( path[i].x, path[i].y);

	    imgGridMap->Picture->Bitmap->Canvas->LineTo(
    		simpleMap->m_gridMaps[0]->x2idx(path[i].x),
        	simpleMap->m_gridMaps[0]->getSizeY()-1-simpleMap->m_gridMaps[0]->y2idx(path[i].y) );
    }

    pPlannedPath->LockInvalidate = false;




}
//---------------------------------------------------------------------------

void __fastcall TVentana::imgGridMapMouseMove(TObject *Sender,
      TShiftState Shift, int X, int Y)
{
	if (simpleMap->m_gridMaps.size()==0) return;

	float     Zoom = tbZoomGrid->Position / 10.0f;
    float 	  x = simpleMap->m_gridMaps[0]->getXMin() + simpleMap->m_gridMaps[0]->getResolution() * X /Zoom;
    float	  y = simpleMap->m_gridMaps[0]->getYMin() + simpleMap->m_gridMaps[0]->getResolution()*(imgGridMap->Height-1-Y)/Zoom ;

        SB->Panels->Items[1]->Text =
          AnsiString().sprintf("(%.03f,%.03f)",x,y);

}
//---------------------------------------------------------------------------

void __fastcall TVentana::imgGridMapMouseDown(TObject *Sender,
      TMouseButton Button, TShiftState Shift, int X, int Y)
{
	if (simpleMap->m_gridMaps.size()==0) return;

	float     Zoom = tbZoomGrid->Position / 10.0f;
    float 	  x = simpleMap->m_gridMaps[0]->getXMin() + simpleMap->m_gridMaps[0]->getResolution() * X /Zoom;
    float	  y = simpleMap->m_gridMaps[0]->getYMin() + simpleMap->m_gridMaps[0]->getResolution()*(imgGridMap->Height-1-Y)/Zoom ;

    if (formList->Visible)
    {
    	AnsiString str;
    	str.sprintf("%.03f %.03f",x,y);
    	formList->mem->Lines->Add( str);

    }

        // Guardar coordenadas de clic izq y der:
        if ( Button == mbLeft )
        {
        	leftClickOnMap.x = x;
        	leftClickOnMap.y =  y;
        }
        else if ( Button == mbRight )
        {
        	rightClickOnMap.x = x;
        	rightClickOnMap.y =  y;
        }

}
//---------------------------------------------------------------------------

void __fastcall TVentana::SaveinafileasaMRMLCSimplePointsMap1Click(
      TObject *Sender)
{
	if (!simpleMap->m_pointsMap)
    {
    	ShowMessage("There is no point map!");
    	return;
    }
	//
    SD->FileName="map_name.pointsmap";
    if (!SD->Execute()) return;

    CFileStream fil(SD->FileName.c_str(),fomWrite);

    fil << *simpleMap->m_pointsMap;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::SavetofileasMRMLCOccupancyGridMap2D1Click(
      TObject *Sender)
{
	//
    SD->FileName="map_name.gridmap";
    if (!SD->Execute()) return;

    CFileStream fil(SD->FileName.c_str(),fomWrite);

    fil << *simpleMap->m_gridMaps[0];
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Button1Click(TObject *Sender)
{
        //
    CPose2D		robotPose(leftClickOnMap);
    float	phi = DEG2RAD((float)StrToFloat( InputBox("Enter sensor orientation","PHI (in degs.)=","0") ));
    robotPose.phi = phi;

    CObservation2DRangeScan		*obs= new CObservation2DRangeScan();
    CSimplePointsMap			ptns;

    obs->aperture = M_PI;
    obs->maxRange = 20;
    obs->rightToLeft = true;

	simpleMap->m_gridMaps[0]->laserScanSimulator(
    		*obs,
            robotPose,
            0.3,
            361);

    CPose3D		pos(robotPose);

    ptns.insertObservation( obs, &pos);
    pSimScan->LockInvalidate = true;
    pSimScan->Clear();
    pSimScan->AddXY(robotPose.x,robotPose.y);
    for (int i=0;i<ptns.getPointsCount();i++)
    {
    	float x,y;
        ptns.getPoint(i,x,y);
        pSimScan->AddXY(x,y);
    }
    pSimScan->AddXY(robotPose.x,robotPose.y);

    pSimScan->LockInvalidate = false;

    delete obs;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Savetotxtfileinmatlabformat1Click(
      TObject *Sender)
{
	if (!simpleMap->m_pointsMap)
    {
    	ShowMessage("There is no point map!");
    	return;
    }

	//
    SD->FileName="map_name.txt";
    if (!SD->Execute()) return;

    FILE	*f = fopen(SD->FileName.c_str(),"wt");
    for (int i=0;i<simpleMap->m_pointsMap->getPointsCount();i++)
    {
    	float	x,y,z;
        simpleMap->m_pointsMap->getPoint(i,x,y,z);
    	fprintf(f,"%f %f %f\n",x,y,z);
    }

    fclose(f);
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Loadgridfrombitmap1Click(TObject *Sender)
{
	OD->FileName = "*.bmp";
	if (!OD->Execute()) return;

        float   res = StrToFloat( InputBox("Resolution of the map","Resolution (m):","0.05") );

    simpleMap->m_gridMaps[0]->loadFromBitmapFile(  OD->FileName.c_str(),res);

    RebuildGridMapView();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Loadgridmapfromgridmapfile1Click(TObject *Sender)
{
	OD->FileName = "*.gridmap";
	if (!OD->Execute()) return;

        CFileStream     fil(OD->FileName.c_str(),fomRead);

        fil >> (*simpleMap->m_gridMaps[0]);

        RebuildGridMapView();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Savetofileasbitmap1Click(TObject *Sender)
{
	//
    SD->FileName="map_name.bmp";
    if (!SD->Execute()) return;

//    simpleMap->m_gridMaps[0]->saveAsBitmapFile( SD->FileName.c_str() );
	imgGridMap->Picture->Bitmap->SaveToFile( SD->FileName );

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Performsubsample1Click(TObject *Sender)
{
        int     ratio = StrToInt(InputBox("Subsample","Downsampling ratio (integer):","1"));

        simpleMap->m_gridMaps[0]->subSample( ratio );

        RebuildGridMapView();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Computeentropyinformation1Click(TObject *Sender)
{
        //
        COccupancyGridMap2D::TEntropyInfo       H;
        AnsiString                              s;

        simpleMap->m_gridMaps[0]->computeEntropy( H );

        s.sprintf("Entropy=%lf\nInformation=%lf\n\nMean Entr.=%lf\nMean Info.=%lf\n\nMapped Area=%lf m2\nMapped cells=%lu",
                        H.H, H.I, H.mean_H, H.mean_I, H.effectiveMappedArea, H.effectiveMappedCells);

        ShowMessage(s);
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ToolButton8Click(TObject *Sender)
{
		RebuildSimpleMapAndViews();
}
//---------------------------------------------------------------------------
void __fastcall TVentana::changeCoordinatesReference1Click(TObject *Sender)
{
        ShowMessage("This will change the pose of the first node, and all other nodes relative to it");

        float   x = StrToFloat( InputBox("Enter new reference system:","Node 0: x","0") );
        float   y = StrToFloat( InputBox("Enter new reference system:","Node 0: y","0") );
        float   phi = DEG2RAD((float)StrToFloat( InputBox("Enter new reference system:","Node 0: phi (degrees)","0") ));

        CPose2D                 oldRef, newRef(x,y,phi);
        CPose3DPDF                *posePDF;
        CSensoryFrame         *SF;
        float                   rotation;
        CMatrixD                 rotMatrix(3,3);
        CPosePDFGaussian        *pose;

        // get first node pose:
        posesAndSFs.get(0, posePDF, SF);
        oldRef = posePDF->getEstimatedPose();

        rotation = newRef.phi - oldRef.phi;

        rotMatrix.zeros();
        rotMatrix(0,0) = cos(rotation);
        rotMatrix(0,1) = -sin(rotation);
        rotMatrix(1,0) = sin(rotation);
        rotMatrix(1,1) = cos(rotation);
        rotMatrix(2,2) = 1;

        for (int i=1;i<posesAndSFs.size();i++)
        {
                // get:
                posesAndSFs.get(i, posePDF, SF);

                if (posePDF->GetRuntimeClass()==CLASS_ID(CPosePDFGaussian))
                {
                        pose = (CPosePDFGaussian*) posePDF;

                        pose->mean = newRef + (pose->mean - oldRef);
                        pose->cov = rotMatrix * pose->cov * (~rotMatrix);
                        pose->cov(0,1) = pose->cov(1,0);
                        pose->cov(0,2) = pose->cov(2,0);
                        pose->cov(1,2) = pose->cov(2,1);
                }
                else
                {
                        ShowMessage("Currently this is implemented only for Gaussian pose PDFs!!");
                        return;
                }


        }

        // Set first node to the given reference:
        posesAndSFs.get(0, posePDF, SF);
        pose = (CPosePDFGaussian*) posePDF;
        pose->mean = newRef;
        pose->cov = rotMatrix * pose->cov * (~rotMatrix);


		RebuildSimpleMapAndViews();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::tsCObservationImageShow(TObject *Sender)
{
        AnsiString      str;

        // Segun el item seleccionado:
        TTreeNode       *node = tv->Selected;
        if (!node) return;      // No selected node.

        CSerializable *obj = ((CSerializable*)node->Data);
        if (!obj) return; // No data associated to sel. node.

        // Visualize object:
        CObservationImage         *obs = (CObservationImage*) obj;

        memImg->Lines->Clear();

        str.sprintf(" Timestamp: %s", SystemUtils::dateTimeToString( obs->timestamp ).c_str() );
         mem2DScan->Lines->Add(str);

        memImg->Lines->Add("Homogeneous matrix for the camera's 3D pose, relative to robot base:");
        CMatrix         mat = obs->cameraPose.getHomogeneousMatrix();

        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(0,0), mat(0,1), mat(0,2), mat(0,3) );
         memImg->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(1,0), mat(1,1), mat(1,2), mat(1,3) );
         memImg->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(2,0), mat(2,1), mat(2,2), mat(2,3) );
         memImg->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(3,0), mat(3,1), mat(3,2), mat(3,3) );
         memImg->Lines->Add(str);

        memImg->Lines->Add("");
        memImg->Lines->Add("Intrinsic parameters matrix for the camera:");
         mat = obs->intrinsicParams;
        memImg->Lines->Add(  mat.inMatlabFormat().c_str() );
/*        str.sprintf("      %4.03f %4.03f %4.03f", mat(0,0), mat(0,1), mat(0,3) );
         memImg->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f", mat(1,0), mat(1,1), mat(1,3) );
         memImg->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f", mat(2,0), mat(2,1), mat(2,3) );
         memImg->Lines->Add(str);
*/
        memImg->Lines->Add("");
        memImg->Lines->Add("Distorsion parameters for the camera:");
         mat = obs->distorsionParams;
        memImg->Lines->Add(  mat.inMatlabFormat().c_str() );

        memImg->Lines->Add("");
        str.sprintf(" Image size: %ux%u pixels", obs->image.getWidth(), obs->image.getHeight() );
         memImg->Lines->Add(str);

        str.sprintf(" Channels order: %s", obs->image.getChannelsOrder() );
         memImg->Lines->Add(str);

        // Get bitmap:
        Graphics::TBitmap         *bmp = new Graphics::TBitmap();

        bmp->Width = obs->image.getWidth();
        bmp->Height = obs->image.getHeight();
        bmp->PixelFormat = pf24bit;

        for (int y=0;y<bmp->Height;y++)
        {
                char *ptr = (char*)bmp->ScanLine[bmp->Height-1-y];
                memcpy( ptr, obs->image(0,y), bmp->Width * 3  );

        }

        obsImg->Picture->Bitmap = bmp;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Expandall1Click(TObject *Sender)
{
        tv->FullExpand();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Collapseall1Click(TObject *Sender)
{
        tv->FullCollapse();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Loadfrompointsmapfile1Click(TObject *Sender)
{
	if (!simpleMap->m_pointsMap)
    {
    	ShowMessage("There is no point map!");
    	return;
    }

	//
    OD->FileName="*.pointsmap";
    if (!OD->Execute()) return;

    CFileStream fil(OD->FileName.c_str(),fomRead);
    CPointsMap  *map = simpleMap->m_pointsMap;

    fil.ReadObject( map );


    RebuildPointsMapView();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Changecamerasobservations1Click(TObject *Sender)
{
        //
        formCameraPose->ShowModal();

}
//---------------------------------------------------------------------------



void __fastcall TVentana::ExportasCHybridMetricMap1Click(TObject *Sender)
{
	//
    SD->FileName="map_name.hybridmetricmap";
    if (!SD->Execute()) return;

    CFileStream fil(SD->FileName.c_str(),fomWrite);

    fil << *simpleMap;

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Eraseaseqofobservations1Click(TObject *Sender)
{
	int	 i1 = StrToInt( InputBox("Erase a segment of map","First index to erase:","1")	);
	int	 i2 = StrToInt( InputBox("Erase a segment of map","Last index to erase:",IntToStr(posesAndSFs.size()-2))	);

	CSensFrameProbSequence 	newMap;
    for (int i=0;i<posesAndSFs.size();i++)
    {
    	if (i1>i || i>i2)
        {
    		CPose3DPDF		*p;
        	CSensoryFrame *sf;

    		posesAndSFs.get(i,p,sf);
    		newMap.insert(p,sf);
        }
    }
    posesAndSFs.clear();

    posesAndSFs = newMap;

    // Update views:
    // --------------------
	RebuildSimpleMapAndViews();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Button2Click(TObject *Sender)
{
        TTreeNode       *node = tv->Selected;
        if (!node) return;

    	CSerializable	*obj = (CSerializable	*)node->Data;
    	if (!obj)  return;

        if (!obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CPosePDFGaussian)) )
                ShowMessage("WARNING: The pose is not of class 'CPosePDFGaussian', information may be lost!");

        CPosePDFGaussian        posePDF;
        posePDF.mean = ((CPosePDF*) obj)->getEstimatedPose();
        posePDF.cov = ((CPosePDF*) obj)->getEstimatedCovariance();


	posePDF.mean.x = StrToFloat( InputBox("Modify robot pose","x:",FloatToStr(posePDF.mean.x)));
	posePDF.mean.y = StrToFloat( InputBox("Modify robot pose","y:",FloatToStr(posePDF.mean.y)));
	posePDF.mean.phi = DEG2RAD((float)StrToFloat( InputBox("Modify robot pose","phi: (deg)",FloatToStr(RAD2DEG(posePDF.mean.phi)))));

        ((CPosePDF*)obj)->copyFrom( posePDF );

        memPose->Lines->Add("WARNING: Pose has been manually edited: Rebuild the map to see changes!");

}
//---------------------------------------------------------------------------


void __fastcall TVentana::Alignobservationagainsthemap1Click(
      TObject *Sender)
{
        //
}
//---------------------------------------------------------------------------


void __fastcall TVentana::Alignanobservationagainsthemap1Click(
      TObject *Sender)
{
        formAligner->ShowModal();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::tsCObservation2DRangeScanShow(TObject *Sender)
{
        AnsiString      str;

        // Segun el item seleccionado:
        TTreeNode       *node = tv->Selected;
        if (!node) return;      // No selected node.

        CSerializable *obj = ((CSerializable*)node->Data);
        if (!obj) return; // No data associated to sel. node.

        // Visualize object:
        CObservation2DRangeScan         *obs = (CObservation2DRangeScan*) obj;

        mem2DScan->Lines->Clear();


        str.sprintf(" Timestamp: %s", SystemUtils::dateTimeToString( obs->timestamp ).c_str() );
         mem2DScan->Lines->Add(str);

        mem2DScan->Lines->Add("Homogeneous matrix for the sensor's 3D pose, relative to robot base:");
        CMatrix         mat = obs->sensorPose.getHomogeneousMatrix();

        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(0,0), mat(0,1), mat(0,2), mat(0,3) );
         mem2DScan->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(1,0), mat(1,1), mat(1,2), mat(1,3) );
         mem2DScan->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(2,0), mat(2,1), mat(2,2), mat(2,3) );
         mem2DScan->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(3,0), mat(3,1), mat(3,2), mat(3,3) );
         mem2DScan->Lines->Add(str);

        mem2DScan->Lines->Add("");

        str.sprintf(" Samples direction: %s", obs->rightToLeft ? "Right->Left" : "Left->Right" );
         mem2DScan->Lines->Add(str);

        str.sprintf("Points in the scan: %u", obs->scan.size());
        mem2DScan->Lines->Add(str);

        int i,inval = 0;
        for (i=0;i<obs->scan.size();i++) if (!obs->validRange[i]) inval++;
        str.sprintf("Invalid points in the scan: %u", inval);
        mem2DScan->Lines->Add(str);

        str.sprintf("Sensor maximum range: %.02f m", obs->maxRange );
        mem2DScan->Lines->Add(str);

        str.sprintf("Sensor aperture: %.01f deg", RAD2DEG(obs->aperture) );
        mem2DScan->Lines->Add(str);

        AnsiString      str2;
        str="Raw scan values: [";
        for (i=0;i<obs->scan.size();i++)
        {
                str2.sprintf("%.03f ", obs->scan[i] );
                str+= str2;
        }
        str+="]";
        mem2DScan->Lines->Add(str);

        str="Raw valid-scan values: [";
        for (i=0;i<obs->scan.size();i++)
        {
                str2.sprintf("%u ", obs->validRange[i] ? 1:0 );
                str+= str2;
        }
        str+="]";
        mem2DScan->Lines->Add(str);


        // Plot:
        p2DScan->LockInvalidate = true;
        p2DScan->Clear();

        CSimplePointsMap        pointsMap;

        pointsMap.insertionOptions.also_interpolate = cbInterp->Checked;
        pointsMap.insertionOptions.minDistBetweenLaserPoints = StrToFloat( edMinDist->Text );

        // Insert into the map:
        obs->insertObservationInto( &pointsMap );

        p2DScan->AddXY( 0,0 );
        for (i=0;i<pointsMap.getPointsCount();i++)
        {
                float   x,y;
                pointsMap.getPoint(i,x,y);

                p2DScan->AddXY( x,y );
        }
        p2DScan->AddXY( 0,0 );
        p2DScan->LockInvalidate = false;

        // Adjust graph scale:
        adjustAspectRatio( plot2DScan );
}

//---------------------------------------------------------------------------
//                              adjustAspectRatio
//---------------------------------------------------------------------------
void __fastcall TVentana::adjustAspectRatio( Tsp_XYPlot *graf )
{
        // Ajustar escala de la grafica de mapa actual:
        graf->LeftAxis->AutoMax=true;
        graf->LeftAxis->AutoMin=true;
        graf->BottomAxis->AutoMax=true;
        graf->BottomAxis->AutoMin=true;

        graf->LeftAxis->AutoMax=false;
        graf->LeftAxis->AutoMin=false;
        graf->BottomAxis->AutoMax=false;
        graf->BottomAxis->AutoMin=false;

        float Ay = (graf->LeftAxis->Max - graf->LeftAxis->Min)*1.3;
        float Ax = (graf->BottomAxis->Max - graf->BottomAxis->Min)*1.3;

        float My = (graf->LeftAxis->Max + graf->LeftAxis->Min)*0.5;
        float Mx = (graf->BottomAxis->Max + graf->BottomAxis->Min)*0.5;

        float Py = graf->ClientHeight;
        float Px = graf->ClientWidth;

        if ( Ax>Ay )
        {
                // Mas ancho que alto
                Ay = (Py*4.0f/3) * Ax / Px;
                graf->LeftAxis->Min = My- Ay / 2;
                graf->LeftAxis->Max = My+ Ay / 2;
        }
        else
        {
                // Mas alto que ancho:
                Ax = Px * Ay / (Py*4.0f/3);
                graf->BottomAxis->Min = Mx-Ax / 2;
                graf->BottomAxis->Max = Mx+ Ax / 2;
        }

}

void __fastcall TVentana::SaveImagetofile1Click(TObject *Sender)
{
        // Segun el item seleccionado:
        TTreeNode       *node = tv->Selected;
        if (!node) return;      // No selected node.

        CSerializable *obj = ((CSerializable*)node->Data);
        if (!obj) return; // No data associated to sel. node.

        // Visualize object:
        if (obj->GetRuntimeClass() != CLASS_ID(CObservationImage))
        	return;

        CObservationImage         *obs = (CObservationImage*) obj;


        SD->FileName = "image.bmp";
        if (!SD->Execute()) return;

        obs->image.saveToBMP( SD->FileName.c_str() );

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Decimatelistofobservations1Click(TObject *Sender)
{
        int     K = StrToInt( InputBox("Decimate the observations:","Decimation K:","1") );

        CSensFrameProbSequence          newSeq;
        CPose3DPDF                        *posePDF;
        CSensoryFrame                 *SF;

        for (int i=0;i<posesAndSFs.size();i+=K)
        {
                posesAndSFs.get(i, posePDF,SF);
                newSeq.insert( posePDF, SF );
        }

        posesAndSFs = newSeq;

        RebuildSimpleMapAndViews();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::TransformposesintoGaussian1Click(TObject *Sender)
{
	ShowMessage("Poses PDF will be transformed into Gaussian distributions (i.e. particles -> aprox. gaussian)");

        CSensFrameProbSequence          newSeq;
        CPose3DPDF                        *posePDF;
        CPose3DPDFGaussian				poseGauss;
        CSensoryFrame                 *SF;

        for (int i=0;i<posesAndSFs.size();i++)
        {
                posesAndSFs.get(i, posePDF,SF);

                poseGauss.copyFrom( *posePDF );

                newSeq.insert( &poseGauss, SF );
        }

        posesAndSFs = newSeq;

	RebuildSimpleMapAndViews();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::selectedLocationsMenuClick(TObject *Sender)
{
	selectedLocationsMenu->Checked = !selectedLocationsMenu->Checked;
    formList->Visible = selectedLocationsMenu->Checked;
}
//---------------------------------------------------------------------------


void __fastcall TVentana::Drawalltherobotposes1Click(TObject *Sender)
{
	// Draw the robot poses in the grid map:

    unsigned int 	i,n = posesAndSFs.size();

    for (i=0;i<n;i++)
    {
		CPose3DPDF                *posePDF;
        CSensoryFrame         *SF;

        // get:
        posesAndSFs.get(i, posePDF, SF);

        CPose2D		p = posePDF->getEstimatedPose();

        int			cx = simpleMap->m_gridMaps[0]->x2idx( p.x );
        int			cy = simpleMap->m_gridMaps[0]->y2idx( p.y );
        int			ly = imgGridMap->Picture->Bitmap->Height;

        // Draw a circle for the "robot"
        float	R = 5;

        imgGridMap->Picture->Bitmap->PixelFormat = pf24bit;

        for (float a=-3.14159;a<3.14159;a+=0.05)
        {
        	int		x = cx + cos(a)*R;
        	int		y = cy + sin(a)*R;
            imgGridMap->Picture->Bitmap->Canvas->Pixels[x][ly-1-y] = clRed;
        }

        // And an arrow in the heading direction:
        for (float d=0;d<R;d+=0.9f)
        {
        	int		x = cx + cos(p.phi)*d;
        	int		y = cy + sin(p.phi)*d;
            imgGridMap->Picture->Bitmap->Canvas->Pixels[x][ly-1-y] = clRed;
        }


    }

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Button3Click(TObject *Sender)
{
	if (!simpleMap->m_landmarksMap)
    	return;

    SD->FileName="viewLandmarksMap.m";
    if (!SD->Execute()) return;

    Screen->Cursor = crHourGlass;
    Application->ProcessMessages();

    simpleMap->m_landmarksMap->saveToMATLABScript2D(std::string(SD->FileName.c_str()));

    Screen->Cursor = crDefault;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::RunsLuMiliosconsistentalignment1Click(
      TObject *Sender)
{
		formLuMilios->ShowModal();

    // Update:
        Screen->Cursor = crHourGlass;

        RebuildSimpleMapAndViews();

        Screen->Cursor = crDefault;

}
//---------------------------------------------------------------------------

void __fastcall TVentana::tsCObservationStereoImagesShow(TObject *Sender)
{
        AnsiString      str;

        // Segun el item seleccionado:
        TTreeNode       *node = tv->Selected;
        if (!node) return;      // No selected node.

        CSerializable *obj = ((CSerializable*)node->Data);
        if (!obj) return; // No data associated to sel. node.

        // Visualize object:
        CObservationStereoImages         *obs = (CObservationStereoImages*) obj;

        memImg2->Lines->Clear();

        str.sprintf("Timestamp: %s", SystemUtils::dateTimeToString(obs->timestamp).c_str());
        memImg2->Lines->Add(str);

        memImg2->Lines->Add("Homogeneous matrix for the LEFT camera's 3D pose, relative to robot base:");
        CMatrix         mat = obs->cameraPose.getHomogeneousMatrix();

        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(0,0), mat(0,1), mat(0,2), mat(0,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(1,0), mat(1,1), mat(1,2), mat(1,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(2,0), mat(2,1), mat(2,2), mat(2,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(3,0), mat(3,1), mat(3,2), mat(3,3) );
         memImg2->Lines->Add(str);

        memImg2->Lines->Add("");
        memImg2->Lines->Add("Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:");
        mat = obs->rightCameraPose.getHomogeneousMatrix();

        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(0,0), mat(0,1), mat(0,2), mat(0,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(1,0), mat(1,1), mat(1,2), mat(1,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(2,0), mat(2,1), mat(2,2), mat(2,3) );
         memImg2->Lines->Add(str);
        str.sprintf("      %4.03f %4.03f %4.03f %4.03f", mat(3,0), mat(3,1), mat(3,2), mat(3,3) );
         memImg2->Lines->Add(str);
        memImg2->Lines->Add("");

        memImg2->Lines->Add("Intrinsic parameters matrix for the cameras:");
         mat = obs->intrinsicParams;
        memImg2->Lines->Add(  mat.inMatlabFormat().c_str() );

        memImg2->Lines->Add("");
        str.sprintf(" Image size: %ux%u pixels", obs->imageLeft.getWidth(), obs->imageLeft.getHeight() );
         memImg2->Lines->Add(str);

        str.sprintf(" Channels order: %s", obs->imageLeft.getChannelsOrder() );
         memImg2->Lines->Add(str);

        str.sprintf(" Rows are stored in top-bottom order: %s", obs->imageLeft.isOriginTopLeft() ? "YES":"NO" );
         memImg2->Lines->Add(str);

        // Get bitmap: LEFT
        Graphics::TBitmap         *bmp = new Graphics::TBitmap();

        bmp->Width = obs->imageLeft.getWidth();
        bmp->Height = obs->imageLeft.getHeight();
        bmp->PixelFormat = pf24bit;

        for (int y=0;y<bmp->Height;y++)
        {
                char *ptr;
                if (obs->imageLeft.isOriginTopLeft())
                        ptr = (char*)bmp->ScanLine[y];
                else    ptr = (char*)bmp->ScanLine[bmp->Height-1-y];

                memcpy( ptr, obs->imageLeft(0,y), bmp->Width * 3  );
        }
        imgLeft->Picture->Bitmap = bmp;

        // Get bitmap: RIGHT
        bmp = new Graphics::TBitmap();

        bmp->Width = obs->imageRight.getWidth();
        bmp->Height = obs->imageRight.getHeight();
        bmp->PixelFormat = pf24bit;

        for (int y=0;y<bmp->Height;y++)
        {
                char *ptr;
                if (obs->imageRight.isOriginTopLeft())
                        ptr = (char*)bmp->ScanLine[y];
                else    ptr = (char*)bmp->ScanLine[bmp->Height-1-y];

                memcpy( ptr, obs->imageRight(0,y), bmp->Width * 3  );
        }
        imgRight->Picture->Bitmap = bmp;
}
//---------------------------------------------------------------------------
void __fastcall TVentana::RebuildSimpleMapAndViews()
{
	RebuildSimpleMap();

    RebuildTreeView();

	RebuildPointsMapView();
	RebuildGridMapView();
	RebuildLandmarksMapView();
}

//---------------------------------------------------------------------------

void __fastcall TVentana::Showgridinformation1Click(TObject *Sender)
{
        AnsiString                              s;

        s.sprintf("Grid resolution=%f\nx_min=%f  x_max=%f\ny_min=%f  y_max=%f\n",
        	simpleMap->m_gridMaps[0]->getResolution(),
			simpleMap->m_gridMaps[0]->getXMin(),
			simpleMap->m_gridMaps[0]->getXMax(),
			simpleMap->m_gridMaps[0]->getYMin(),
			simpleMap->m_gridMaps[0]->getYMax() );

        ShowMessage(s);

}
//---------------------------------------------------------------------------


void __fastcall TVentana::Markalltheobservationsongridmap1Click(
      TObject *Sender)
{
		imgGridMap->Picture->Bitmap->PixelFormat = pf24bit;

    	imgGridMap->Picture->Bitmap->Canvas->Pen->Color = clRed;
	    imgGridMap->Picture->Bitmap->Canvas->Pen->Width = 2;
        imgGridMap->Picture->Bitmap->Canvas->Brush->Color = clRed;

        // Añadir una entrada por SF:
        int i, n = posesAndSFs.size();
        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;
                AnsiString              s;

                // get:
                posesAndSFs.get(i, posePDF, SF);

                CPose2D			obsPose = posePDF->getEstimatedPose();

                imgGridMap->Picture->Bitmap->Canvas->Ellipse(
                	simpleMap->m_gridMaps[0]->x2idx(obsPose.x)-3,
	                simpleMap->m_gridMaps[0]->getSizeY()-1-simpleMap->m_gridMaps[0]->y2idx(obsPose.y)-3,
                	simpleMap->m_gridMaps[0]->x2idx(obsPose.x)+3,
	                simpleMap->m_gridMaps[0]->getSizeY()-1-simpleMap->m_gridMaps[0]->y2idx(obsPose.y)+3
                     );
        }


}
//---------------------------------------------------------------------------

void __fastcall TVentana::Resizegridmap1Click(TObject *Sender)
{
	float	x_min = StrToFloat( InputBox("Resize grid:","x_min:","-10.0") );
	float	x_max = StrToFloat( InputBox("Resize grid:","x_max:","10.0") );
	float	y_min = StrToFloat( InputBox("Resize grid:","y_min:","-10.0") );
	float	y_max = StrToFloat( InputBox("Resize grid:","y_max:","10.0") );


	simpleMap->m_gridMaps[0]->resizeGrid(x_min,x_max,y_min,y_max,0.5f,false);
	RebuildGridMapView();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::FormDestroy(TObject *Sender)
{
	delete simpleMap;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Saveto3Dscenefile1Click(TObject *Sender)
{
	//
    SD->FileName="gridmap_name.3Dscene";
    if (!SD->Execute()) return;

    CFileStream fil(SD->FileName.c_str(),fomWrite);

    COpenGLScene		scene;
    UTILS::OPENGL::CSetOfObjects	*objs = new UTILS::OPENGL::CSetOfObjects();

    simpleMap->m_gridMaps[0]->getAs3DObject( *objs );

    scene.insert(objs );

    fil << scene;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Saveto3Dscenefile2Click(TObject *Sender)
{
	if (!simpleMap->m_pointsMap)
    {
    	ShowMessage("There is no point map!");
    	return;
    }

	//
    SD->FileName="pointcloud_name.3Dscene";
    if (!SD->Execute()) return;

    CFileStream fil(SD->FileName.c_str(),fomWrite);

    COpenGLScene		scene;
    {
            UTILS::OPENGL::CPointCloud *obj = new UTILS::OPENGL::CPointCloud();
            obj->loadFromPointsMap( simpleMap->m_pointsMap );
            obj->m_colorFromZ = true;
            obj->m_color_R = 0;
            obj->m_color_G = 0;
            obj->m_color_B = 1;

            scene.insert( obj );
    }

    {
            UTILS::OPENGL::CGridPlaneXY  *obj = new UTILS::OPENGL::CGridPlaneXY(-200,200,-200,200,0,5);
            obj->m_color_R = 0.7;
            obj->m_color_G = 0.7;
            obj->m_color_B = 0.7;
            scene.insert( obj );
    }

    fil << scene;
}
//---------------------------------------------------------------------------

void __fastcall TVentana::ComputeVoronoidiagram1Click(TObject *Sender)
{
	float thres = StrToFloat( InputBox("Voronoi Diagram","Cells threshold:","0.51")	);
	float robotSize = StrToFloat( InputBox("Voronoi Diagram","Aprox. robot size:","0.05")	);

//	simpleMap->m_gridMaps[0]->buildVoronoiDiagram( thres, robotSize );

	float x1 = min( leftClickOnMap.x, rightClickOnMap.x );
	float x2 = max( leftClickOnMap.x, rightClickOnMap.x );
	float y1 = min( leftClickOnMap.y, rightClickOnMap.y );
	float y2 = max( leftClickOnMap.y, rightClickOnMap.y );

	simpleMap->m_gridMaps[0]->buildVoronoiDiagram(
    	thres,
        robotSize,
		simpleMap->m_gridMaps[0]->x2idx(x1),
    	simpleMap->m_gridMaps[0]->x2idx(x2),
		simpleMap->m_gridMaps[0]->y2idx(y1),
    	simpleMap->m_gridMaps[0]->y2idx(y2) );


	float filterDist = StrToFloat( InputBox("Voronoi Diagram","Min. dist. between critical points:","0.20")	);
    simpleMap->m_gridMaps[0]->findCriticalPoints( filterDist );


    size_t	i,n = simpleMap->m_gridMaps[0]->CriticalPointsList.x.size();

    imgGridMap->Picture->Bitmap->PixelFormat = pf24bit;

    imgGridMap->Picture->Bitmap->Canvas->Pen->Color = clRed;
    imgGridMap->Picture->Bitmap->Canvas->Pen->Width = 2;
    imgGridMap->Picture->Bitmap->Canvas->Brush->Color = clRed;

    int ly = simpleMap->m_gridMaps[0]->getSizeY();

    for (i=0;i<n;i++)
    {
    	int x = simpleMap->m_gridMaps[0]->CriticalPointsList.x[i];
    	int	y = simpleMap->m_gridMaps[0]->CriticalPointsList.y[i];

        imgGridMap->Picture->Bitmap->Canvas->Ellipse(
			x-3,
	        ly-1-y-3,
			x+3,
	        ly-1-y+3 );
	}

	SB->Panels->Items[0]->Text = AnsiString(n)+AnsiString(" critical points found in the Voronoi diagram!");
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Binarizecells1Click(TObject *Sender)
{
	float thres = StrToFloat( InputBox("Binarize cells","Cells threshold:","0.51")	);

    for (int cx=0;cx<simpleMap->m_gridMaps[0]->getSizeX();cx++)
    {
    	for (int cy=0;cy<simpleMap->m_gridMaps[0]->getSizeY();cy++)
        {
        	float c = simpleMap->m_gridMaps[0]->getCell(cx,cy);
            c = (c<=thres) ? 0:1;
        	simpleMap->m_gridMaps[0]->setCell(cx,cy, c);
        }
    }

    RebuildGridMapView();
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Clipobservationsoutofanarea1Click(
      TObject *Sender)
{
	float x_min = StrToFloat( InputBox("Clip of map by coordinates","x_min:","-10")	);
	float x_max = StrToFloat( InputBox("Clip of map by coordinates","x_max:","10")	);
	float y_min = StrToFloat( InputBox("Clip of map by coordinates","y_min:","-10")	);
	float y_max = StrToFloat( InputBox("Clip of map by coordinates","y_max:","10")	);

	CSensFrameProbSequence 	newMap;
    for (int i=0;i<posesAndSFs.size();i++)
    {
    	CPose3DPDF		*p;
        CSensoryFrame *sf;
        posesAndSFs.get(i,p,sf);
        CPose2D			meanPose( p->getEstimatedPose() );

    	if ( x_min <= meanPose.x && meanPose.x <= x_max &&
             y_min <= meanPose.y && meanPose.y <= y_max  )
        {
    		newMap.insert(p,sf);
        }
    }
    posesAndSFs.clear();

    posesAndSFs = newMap;

    // Update views:
    // --------------------
	RebuildSimpleMapAndViews();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::btnMapsConfigClick(TObject *Sender)
{
	formMapsConfig->ShowModal();
    if (mrYes == MessageDlg(
    	"Do you want to rebuild all the metric maps?",
    	mtConfirmation,
        TMsgDlgButtons() << mbYes << mbNo,
         0) )
	{
        RebuildSimpleMapAndViews();
    }
}
//---------------------------------------------------------------------------

void __fastcall TVentana::Deleteobservationsbyindex1Click(TObject *Sender)
{
        unsigned int i, n = posesAndSFs.size();
        unsigned int m;

        unsigned int idx = StrToInt( InputBox("Delete Observations by Index within the SF:","Index to be deleted (first one=0)","0") );

        for (i=0;i<n;i++)
        {
                CPose3DPDF                *posePDF;
                CSensoryFrame         *SF;

                // get:
                posesAndSFs.get(i, posePDF, SF);

                // Observations count:
                m = SF->size();

                if (m>idx)
                {
                        SF->eraseByIndex(idx);
                }
        }

        RebuildTreeView();

}
//---------------------------------------------------------------------------

void __fastcall TVentana::Button4Click(TObject *Sender)
{
	if (!simpleMap->m_landmarksMap) return;

        UTILS::CDisplayWindow3D		win("LandmarkMap 3D View - Press any key to close");

        COpenGLScene	**scene = win.get3DSceneAndLock();

        UTILS::OPENGL::CGridPlaneXY	*gr = new UTILS::OPENGL::CGridPlaneXY();
        gr->m_xMin = -50;
        gr->m_xMax =  50;
        gr->m_yMin = -50;
        gr->m_yMax =  50;
        gr->m_frequency = 5;
        gr->m_color_R = 0.4f;
        gr->m_color_G = 0.4f;
        gr->m_color_B = 0.4f;
        gr->m_color_A = 0.5f;
        (*scene)->insert( gr );


        UTILS::OPENGL::CSetOfObjects	*objs = new OPENGL::CSetOfObjects();
        simpleMap->m_landmarksMap->getAs3DObject( *objs );
        (*scene)->insert( objs );

        win.unlockAccess3DScene();

        win.clearKeyHitFlag();

        while ( !win.keyHit() )
        {
        	Application->ProcessMessages();
                Sleep(50);
        }

}
//---------------------------------------------------------------------------

void __fastcall TVentana::DeleteSFswithoutobservations1Click(
      TObject *Sender)
{
        int i, n = posesAndSFs.size();

        for (i=n-1;i>=0;i--)
        {
        	CPose3DPDF                *posePDF;
            CSensoryFrame         *SF;

            // get:
            posesAndSFs.get(i, posePDF, SF);

            // Observations count:
            if (! SF->size() )
            {
                posesAndSFs.remove( i );
            }
        }

        RebuildTreeView();


}
//---------------------------------------------------------------------------

