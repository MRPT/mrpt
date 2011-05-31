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

#ifndef mainUnitH
#define mainUnitH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ComCtrls.hpp>
#include <ImgList.hpp>
#include <ToolWin.hpp>
#include <ExtCtrls.hpp>
#include <Dialogs.hpp>
#include "sgr_data.hpp"
#include "sgr_def.hpp"
#include <Menus.hpp>
//---------------------------------------------------------------------------
class TVentana : public TForm
{
__published:	// IDE-managed Components
        TToolBar *ToolBar1;
        TToolButton *ToolButton1;
        TToolButton *ToolButton2;
        TImageList *ImageList1;
        TToolButton *ToolButton3;
        TPanel *Panel1;
        TPageControl *pc;
        TTabSheet *tsPointsMap;
        TTabSheet *TabSheet2;
        TScrollBox *ScrollBox1;
        TScrollBox *ScrollBox2;
        TTreeView *tv;
        TSplitter *Splitter1;
        TImageList *ImageList2;
        TOpenDialog *OD;
        TTimer *timAutoLoad;
        Tsp_XYPlot *plot_PointsMap;
        Tsp_XYLine *pPointsMap;
        Tsp_XYLine *pRobotPose;
        TImage *imgGridMap;
        TStatusBar *SB;
	TTabSheet *tsRobotPose;
	TMemo *memPose;
	TToolButton *ToolButton6;
	TSaveDialog *SD;
	Tsp_XYLine *pPlannedPath;
	TPopupMenu *mnuPointsMap;
	TMenuItem *SaveinafileasaMRMLCSimplePointsMap1;
	TPopupMenu *mnuGridMap;
	TMenuItem *SavetofileasMRMLCOccupancyGridMap2D1;
	TPanel *Panel2;
	TTrackBar *tbZoomGrid;
	Tsp_XYLine *pSimScan;
	TPanel *Panel4;
	TPanel *Panel5;
	TStaticText *StaticText3;
	TStaticText *StaticText2;
	TStaticText *StaticText1;
	TPanel *Panel6;
	TButton *Button1;
	TMenuItem *Savetotxtfileinmatlabformat1;
	TMenuItem *Loadgridfrombitmap1;
        TMenuItem *Loadgridmapfromgridmapfile1;
        TMenuItem *N1;
        TMenuItem *Savetofileasbitmap1;
        TMenuItem *N2;
        TMenuItem *Performsubsample1;
        TMenuItem *Computeentropyinformation1;
        Tsp_XYLine *pScan;
        TMainMenu *MainMenu1;
        TMenuItem *File1;
        TMenuItem *Help1;
        TMenuItem *About1;
        TMenuItem *Operations1;
        TMenuItem *Load1;
        TMenuItem *Saveas1;
        TMenuItem *Exit1;
        TMenuItem *N3;
        TMenuItem *Clipobservationslist1;
        TMenuItem *Computeplannedpath1;
        TMenuItem *Rebuildmap1;
        TTabSheet *tsCObservationImage;
        TScrollBox *ScrollBox3;
        TImage *obsImg;
        TMemo *memImg;
        TPopupMenu *mnuTree;
        TMenuItem *Expandall1;
        TMenuItem *Collapseall1;
        TMemo *memLoad;
        TMenuItem *N4;
        TMenuItem *Loadfrompointsmapfile1;
        TMenuItem *Changecamerasobservations1;
	TMenuItem *N5;
	TMenuItem *ExportasCHybridMetricMap1;
        TMenuItem *N6;
        TMenuItem *Eraseaseqofobservations1;
        TMenuItem *N7;
        TMenuItem *Alignanobservationagainsthemap1;
        TTabSheet *tsCObservation2DRangeScan;
        TPanel *Panel7;
        TLabel *Label2;
        TCheckBox *cbInterp;
        TEdit *edMinDist;
        Tsp_XYPlot *plot2DScan;
        TMemo *mem2DScan;
        Tsp_XYLine *p2DScan;
	TPanel *Panel8;
	TButton *Button2;
	TPopupMenu *popupImages;
	TMenuItem *SaveImagetofile1;
	TMenuItem *Loadimagefromfile1;
        TMenuItem *Decimatelistofobservations1;
	TMenuItem *TransformposesintoGaussian1;
	TMenuItem *View1;
	TMenuItem *selectedLocationsMenu;
	Tsp_XYLine *pListPoints;
        TMenuItem *Deleteobservationsbyindex1;
	TPaintBox *pbGrid;
	TMenuItem *N8;
	TMenuItem *Drawalltherobotposes1;
	TTabSheet *tsPoints3D;
	TTabSheet *tsLandmarks;
	TPanel *Panel9;
	TButton *Button3;
	TLabel *Label3;
	TStaticText *lbLandmarksCount;
	TMenuItem *RunsLuMiliosconsistentalignment1;
        TTabSheet *tsCObservationStereoImages;
        TMemo *memImg2;
        TPageControl *pcLR;
        TTabSheet *Left;
        TImage *imgLeft;
        TTabSheet *Right;
        TImage *imgRight;
	TSplitter *Splitter2;
	TMenuItem *Showgridinformation1;
	TMenuItem *Markalltheobservationsongridmap1;
	TMenuItem *Resizegridmap1;
	TMenuItem *Saveto3Dscenefile1;
	TMenuItem *Saveto3Dscenefile2;
	TMenuItem *ComputeVoronoidiagram1;
	TMenuItem *Binarizecells1;
	TMenuItem *Clipobservationsoutofanarea1;
	TToolButton *ToolButton4;
	TToolButton *btnMapsConfig;
	TButton *Button4;
	TMenuItem *DeleteSFswithoutobservations1;
        void __fastcall ToolButton3Click(TObject *Sender);
        void __fastcall FormShow(TObject *Sender);
        void __fastcall ToolButton1Click(TObject *Sender);
        void __fastcall timAutoLoadTimer(TObject *Sender);
        void __fastcall plot_PointsMapAxisZoom(Tsp_Axis *Sender,
          double &min, double &max, bool &CanZoom);
        void __fastcall plot_PointsMapMouseUp(TObject *Sender,
          TMouseButton Button, TShiftState Shift, int X, int Y);
        void __fastcall plot_PointsMapMouseDown(TObject *Sender,
          TMouseButton Button, TShiftState Shift, int X, int Y);
        void __fastcall plot_PointsMapMouseMove(TObject *Sender,
          TShiftState Shift, int X, int Y);
        void __fastcall tvClick(TObject *Sender);
        void __fastcall tbZoomGridChange(TObject *Sender);
	void __fastcall ToolButton5Click(TObject *Sender);
	void __fastcall ToolButton2Click(TObject *Sender);
	void __fastcall ToolButton7Click(TObject *Sender);
	void __fastcall imgGridMapMouseMove(TObject *Sender, TShiftState Shift,
          int X, int Y);
	void __fastcall imgGridMapMouseDown(TObject *Sender, TMouseButton Button,
          TShiftState Shift, int X, int Y);
	void __fastcall SaveinafileasaMRMLCSimplePointsMap1Click(TObject *Sender);
	void __fastcall SavetofileasMRMLCOccupancyGridMap2D1Click(
          TObject *Sender);
	void __fastcall Button1Click(TObject *Sender);
	void __fastcall Savetotxtfileinmatlabformat1Click(TObject *Sender);
	void __fastcall Loadgridfrombitmap1Click(TObject *Sender);
        void __fastcall Loadgridmapfromgridmapfile1Click(TObject *Sender);
        void __fastcall Savetofileasbitmap1Click(TObject *Sender);
        void __fastcall Performsubsample1Click(TObject *Sender);
        void __fastcall Computeentropyinformation1Click(TObject *Sender);
        void __fastcall ToolButton8Click(TObject *Sender);
        void __fastcall changeCoordinatesReference1Click(TObject *Sender);
        void __fastcall tsCObservationImageShow(TObject *Sender);
        void __fastcall Expandall1Click(TObject *Sender);
        void __fastcall Collapseall1Click(TObject *Sender);
        void __fastcall Loadfrompointsmapfile1Click(TObject *Sender);
        void __fastcall Changecamerasobservations1Click(TObject *Sender);
	void __fastcall ExportasCHybridMetricMap1Click(TObject *Sender);
        void __fastcall Eraseaseqofobservations1Click(TObject *Sender);
        void __fastcall Button2Click(TObject *Sender);
        void __fastcall Alignobservationagainsthemap1Click(
          TObject *Sender);
        void __fastcall Alignanobservationagainsthemap1Click(
          TObject *Sender);
        void __fastcall tsCObservation2DRangeScanShow(TObject *Sender);
	void __fastcall SaveImagetofile1Click(TObject *Sender);
        void __fastcall Decimatelistofobservations1Click(TObject *Sender);
	void __fastcall TransformposesintoGaussian1Click(TObject *Sender);
	void __fastcall selectedLocationsMenuClick(TObject *Sender);
	void __fastcall Drawalltherobotposes1Click(TObject *Sender);
	void __fastcall Button3Click(TObject *Sender);
	void __fastcall RunsLuMiliosconsistentalignment1Click(TObject *Sender);
        void __fastcall tsCObservationStereoImagesShow(TObject *Sender);
	void __fastcall Showgridinformation1Click(TObject *Sender);
	void __fastcall Markalltheobservationsongridmap1Click(TObject *Sender);
	void __fastcall Resizegridmap1Click(TObject *Sender);
	void __fastcall FormDestroy(TObject *Sender);
	void __fastcall Saveto3Dscenefile1Click(TObject *Sender);
	void __fastcall Saveto3Dscenefile2Click(TObject *Sender);
	void __fastcall ComputeVoronoidiagram1Click(TObject *Sender);
	void __fastcall Binarizecells1Click(TObject *Sender);
	void __fastcall Clipobservationsoutofanarea1Click(TObject *Sender);
	void __fastcall btnMapsConfigClick(TObject *Sender);
	void __fastcall Deleteobservationsbyindex1Click(TObject *Sender);
	void __fastcall Button4Click(TObject *Sender);
	void __fastcall DeleteSFswithoutobservations1Click(TObject *Sender);
private:	// User declarations

        void __fastcall RebuildTreeView();
        void __fastcall RebuildSimpleMap();
        void __fastcall RebuildSimpleMapAndViews();

        void __fastcall RebuildPointsMapView();
        void __fastcall RebuildGridMapView();
        void __fastcall RebuildLandmarksMapView();

        void __fastcall adjustAspectRatio( Tsp_XYPlot *graf );
public:		// User declarations
        __fastcall TVentana(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TVentana *Ventana;
//---------------------------------------------------------------------------
#endif
