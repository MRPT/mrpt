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

#ifndef XRAWLOGVIEWERMAIN_H
#define XRAWLOGVIEWERMAIN_H


//(*Headers(xRawLogViewerFrame)
#include <wx/toolbar.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/slider.h>
#include <wx/statusbr.h>
#include "CRawlogTreeView.h"
#include <wx/frame.h>
#include "MyGLCanvas.h"
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/statbmp.h>
//*)

#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
#include <wx/combobox.h>


#include <map>
#include <string>
#include <wx/docview.h>

// General global variables:
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

// A list of sensor labels (and the times they appear) in the currently loaded rawlog.
struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() : occurences(0), first(INVALID_TIMESTAMP) ,last(INVALID_TIMESTAMP)
	{}
	size_t					occurences;
	mrpt::system::TTimeStamp	first,last;
};


class wxStaticBitmapPopup : public wxStaticBitmap
{
public:
    wxStaticBitmapPopup () {}
    wxStaticBitmapPopup ( wxWindow *parent, wxWindowID id,
                          const wxBitmap&img,
                          const wxPoint &pos = wxDefaultPosition,
                          const wxSize &size = wxDefaultSize,
                          int flags = 0, const wxString &name=wxT(""));
    ~wxStaticBitmapPopup ();

    void OnShowPopupMenu(wxMouseEvent &event);

protected:
    wxMenu mnuImages;

    void OnPopupSaveImage(wxCommandEvent& event);
    void OnPopupLoadImage(wxCommandEvent& event);

    static const long ID_MENUITEM_IMG_LOAD;
    static const long ID_MENUITEM_IMG_SAVE;

    DECLARE_DYNAMIC_CLASS(wxStaticBitmapPopup )
    DECLARE_EVENT_TABLE()
};


// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
	virtual wxBitmap CreateBitmap(const wxArtID& id,
								  const wxArtClient& client,
								  const wxSize& size);
};


// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

// Auxiliary data types used to import ALOG files:
struct TAlogRecord
{
	std::string		label;
	char   			type;  // 0: odo, 1: 2d laser, 2:3d laser, 3: image
	std::vector<float>   data;
	std::string		imgFile;
	double          startElev, endElev;
};


class xRawLogViewerFrame: public wxFrame
{
	friend class wxStaticBitmapPopup;

public:

    xRawLogViewerFrame(wxWindow* parent,wxWindowID id = -1);
    virtual ~xRawLogViewerFrame();


    bool AskForOpenRawlog( std::string &fil );
    bool AskForSaveRawlog( std::string &fil );

    /** Return an empty string if cancel, otherwise "bmp","jpg",... */
    std::string AskForImageFileFormat();

    /** Return the label of the selected sensor, or "" if cancel or no labels */
    std::string AskForObservationByLabel(const std::string & title);

    /** Return one or more labels of the selected sensor, or "" if cancel or no labels */
    std::vector<std::string> AskForObservationByLabelMultiple(const std::string & title);

	void saveImportedLogToRawlog(
		const std::string &target_filename,
		const std::map<double,TAlogRecord>	&theAlog,
		const std::string &dir_for_images
		);

private:

    /** Loads the given file in memory, in the varibale "rawlog"
      */
    void loadRawlogFile(
        const std::string &str,
        int		first = 0,
        int		last  = -1 );

    /** Rebuilds the tree view with the data in "rawlog".
      */
    void rebuildTreeView();

    // Open most recent file
    void OnMRUFile(wxCommandEvent& event);

    //(*Handlers(xRawLogViewerFrame)
    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    void OnFileOpen(wxCommandEvent& event);
    void OnSaveFile(wxCommandEvent& event);
    void OnEditRawlog(wxCommandEvent& event);
    void OnMenuItem37Selected(wxCommandEvent& event);
    void OntimAutoLoadTrigger(wxTimerEvent& event);
    void OnNotebook1PageChanging(wxNotebookEvent& event);
    void OnChangeMotionModel(wxCommandEvent& event);
    void OnShowImagesAsVideo(wxCommandEvent& event);
    void OnRawMapOdo(wxCommandEvent& event);
    void OnMenuGenerateBeaconList(wxCommandEvent& event);
    void OnImportCARMEN(wxCommandEvent& event);
    void OnGenerateSeqImgs(wxCommandEvent& event);
    void OnGenGasTxt(wxCommandEvent& event);
    void OnGenWifiTxt(wxCommandEvent& event);
    void OnGenGPSTxt(wxCommandEvent& event);
    void OnGenOdoLaser(wxCommandEvent& event);
    void OnSummaryGPS(wxCommandEvent& event);
    void OnShowICP(wxCommandEvent& event);
    void OnLoadAPartOnly(wxCommandEvent& event);
    void OnFileCountEntries(wxCommandEvent& event);
    void OnFileSaveImages(wxCommandEvent& event);
    void OnFileGenVisualLMFromStereoImages(wxCommandEvent& event);
    void OnChangeSensorPositions(wxCommandEvent& event);
    void OnDecimateRecords(wxCommandEvent& event);
    void OnCountBadScans(wxCommandEvent& event);
    void OnRecalculateActionsICP(wxCommandEvent& event);
    void OnRecomputeOdometry(wxCommandEvent& event);
    void OnFilterSpureousGas(wxCommandEvent& event);
    void OnFilterErroneousScans(wxCommandEvent& event);
    void OnRemoveSpecificRangeMeas(wxCommandEvent& event);
    void OnGenerateTextFileRangeBearing(wxCommandEvent& event);
    void OnForceEncodersFalse(wxCommandEvent& event);
    void OnGenerateIMUTextFile(wxCommandEvent& event);
    void OnRangeFinder1DGenTextFile(wxCommandEvent& event);
    void OnImportSequenceOfImages(wxCommandEvent& event);
    void OnMenuItem47Selected(wxCommandEvent& event);
    void OnMenuItem46Selected(wxCommandEvent& event);
    void OnMenuExpandAll(wxCommandEvent& event);
    void OnMenuCollapseAll(wxCommandEvent& event);
    void OnMenuModifyICPActionsUncertainty(wxCommandEvent& event);
    void OnShowAnimateScans(wxCommandEvent& event);
    void OnMenuExportALOG(wxCommandEvent& event);
    void OnMenuImportALOG(wxCommandEvent& event);
    void OnMenuCompactRawlog(wxCommandEvent& event);
    void OnMenuLossLessDecimate(wxCommandEvent& event);
    void OnMenCompactFILE(wxCommandEvent& event);
    void OnMenuLossLessDecFILE(wxCommandEvent& event);
    void OnMenuConvertExternallyStored(wxCommandEvent& event);
    void OnImportRTL(wxCommandEvent& event);
    void OnMenuConvertObservationOnly(wxCommandEvent& event);
    void OnMenuDistanceBtwGPSs(wxCommandEvent& event);
    void OnMenuResortByTimestamp(wxCommandEvent& event);
    void OnMenuRegenerateGPSTimestamps(wxCommandEvent& event);
    void OnMenuShiftTimestampsByLabel(wxCommandEvent& event);
    void OnMenuShowTips(wxCommandEvent& event);
    void OnMenuVisualOdometry(wxCommandEvent& event);
    void OnbtnEditCommentsClick(wxCommandEvent& event);
    void OnMenuInsertComment(wxCommandEvent& event);
    void OnMenuDrawGPSPath(wxCommandEvent& event);
    void OnMenuRenameSensor(wxCommandEvent& event);
    void OnMenuChangePosesBatch(wxCommandEvent& event);
    void OnMenuMono2Stereo(wxCommandEvent& event);
    void OnMenuRectifyImages(wxCommandEvent& event);
    void OnMenuMarkLaserScanInvalid(wxCommandEvent& event);
    void OnMenuChangeMaxRangeLaser(wxCommandEvent& event);
    void OnbtnEditCommentsClick1(wxCommandEvent& event);
    void OnMenuConvertSF(wxCommandEvent& event);
    void OnMenuRevert(wxCommandEvent& event);
    void OnMenuBatchLaserExclusionZones(wxCommandEvent& event);
    void OnMenuRenameImageFiles(wxCommandEvent& event);
    void OnLaserFilterAngles(wxCommandEvent& event);
    void OnMenuGPSDeleteNaN(wxCommandEvent& event);
    void OnMenuRangeBearFilterIDs(wxCommandEvent& event);
    void OnMenuRegenerateTimestampBySF(wxCommandEvent& event);
    void OnmnuCreateAVISelected(wxCommandEvent& event);
    void OnMenuRegenerateOdometryTimes(wxCommandEvent& event);
    void OnMenuItem3DObsRecoverParams(wxCommandEvent& event);
    void Onslid3DcamConfCmdScrollChanged(wxScrollEvent& event);
    void OnMenuItemImportBremenDLRLog(wxCommandEvent& event);
    //*)

    //(*Identifiers(xRawLogViewerFrame)
    static const long ID_CUSTOM5;
    static const long ID_PANEL1;
    static const long ID_TEXTCTRL1;
    static const long ID_PANEL3;
    static const long ID_BUTTON1;
    static const long ID_PANEL18;
    static const long ID_TEXTCTRL2;
    static const long ID_TEXTCTRL3;
    static const long ID_NOTEBOOK3;
    static const long ID_PANEL6;
    static const long ID_CUSTOM2;
    static const long ID_CUSTOM3;
    static const long ID_PANEL7;
    static const long ID_CUSTOM1;
    static const long ID_PANEL4;
    static const long ID_PANEL8;
    static const long ID_STATICTEXT2;
    static const long ID_STATICBITMAP1;
    static const long ID_PANEL9;
    static const long ID_STATICTEXT1;
    static const long ID_STATICBITMAP2;
    static const long ID_PANEL13;
    static const long ID_STATICBITMAP3;
    static const long ID_PANEL14;
    static const long ID_STATICBITMAP7;
    static const long ID_PANEL24;
    static const long ID_NOTEBOOK2;
    static const long ID_PANEL10;
    static const long ID_PANEL11;
    static const long ID_PANEL12;
    static const long ID_PANEL15;
    static const long ID_CUSTOM4;
    static const long ID_PANEL17;
    static const long ID_PANEL16;
    static const long ID_XY_GLCANVAS;
    static const long ID_STATICTEXT3;
    static const long ID_SLIDER1;
    static const long ID_PANEL20;
    static const long ID_STATICBITMAP4;
    static const long ID_PANEL21;
    static const long ID_STATICBITMAP5;
    static const long ID_PANEL22;
    static const long ID_STATICBITMAP6;
    static const long ID_PANEL23;
    static const long ID_NOTEBOOK4;
    static const long ID_PANEL19;
    static const long ID_NOTEBOOK1;
    static const long ID_PANEL5;
    static const long ID_SPLITTERWINDOW3;
    static const long ID_PANEL2;
    static const long ID_SPLITTERWINDOW1;
    static const long ID_MENUITEM1;
    static const long ID_MENUITEM2;
    static const long ID_MENUITEM11;
    static const long ID_MENUITEM4;
    static const long ID_MENUITEM76;
    static const long ID_MENUITEM7;
    static const long ID_MENUITEM8;
    static const long ID_MENUITEM10;
    static const long ID_MENUITEM62;
    static const long ID_MENUITEM64;
    static const long ID_MENUITEM13;
    static const long ID_MENUITEM60;
    static const long ID_MENUITEM61;
    static const long ID_MENUITEM6;
    static const long ID_MENUITEM5;
    static const long ID_MENUITEM47;
    static const long ID_MENUITEM56;
    static const long ID_MENUITEM63;
    static const long ID_MENUITEM87;
    static const long ID_MENUITEM3;
    static const long ID_MENUITEM58;
    static const long ID_MENUITEM55;
    static const long ID_MENUITEM54;
    static const long idMenuQuit;
    static const long ID_MENUITEM14;
    static const long ID_MENUITEM51;
    static const long ID_MENUITEM69;
    static const long ID_MENUITEM15;
    static const long ID_MENUITEM70;
    static const long ID_MENUITEM16;
    static const long ID_MENUITEM59;
    static const long ID_MENUITEM57;
    static const long ID_MENUITEM75;
    static const long ID_MENUITEM67;
    static const long ID_MENUITEM68;
    static const long ID_MENUITEM82;
    static const long ID_MENUITEM20;
    static const long ID_MENUITEM22;
    static const long ID_MENUITEM53;
    static const long ID_MENUITEM23;
    static const long ID_MENUITEM41;
    static const long ID_MENUITEM84;
    static const long ID_MENUITEM12;
    static const long ID_MENUITEM17;
    static const long ID_MENUITEM44;
    static const long ID_MENUITEM19;
    static const long ID_MENUITEM25;
    static const long ID_MENUITEM73;
    static const long ID_MENUITEM74;
    static const long ID_MENUITEM77;
    static const long ID_MENUITEM79;
    static const long ID_MENUITEM18;
    static const long ID_MENUITEM86;
    static const long ID_MENUITEM90;
    static const long ID_MENUITEM85;
    static const long ID_MENUITEM29;
    static const long ID_MENUITEM9;
    static const long ID_MENUITEM28;
    static const long ID_MENUITEM71;
    static const long ID_MENUITEM72;
    static const long ID_MENUITEM78;
    static const long ID_MENUITEM83;
    static const long ID_MENUITEM21;
    static const long ID_MENUITEM30;
    static const long ID_MENUITEM24;
    static const long ID_MENUITEM35;
    static const long ID_MENUITEM31;
    static const long ID_MENUITEM34;
    static const long ID_MENUITEM65;
    static const long ID_MENUITEM66;
    static const long ID_MENUITEM52;
    static const long ID_MENUITEM80;
    static const long ID_MENUITEM36;
    static const long ID_MENUITEM33;
    static const long ID_MENUITEM38;
    static const long ID_MENUITEM37;
    static const long ID_MENUITEM40;
    static const long ID_MENUITEM81;
    static const long ID_MENUITEM39;
    static const long ID_MENUITEM46;
    static const long ID_MENUITEM45;
    static const long ID_MENUITEM43;
    static const long ID_MENUITEM42;
    static const long ID_MENUITEM89;
    static const long ID_MENUITEM88;
    static const long ID_MENUITEM26;
    static const long ID_MENUITEM32;
    static const long ID_MENUITEM27;
    static const long idMenuAbout;
    static const long ID_STATUSBAR1;
    static const long ID_TOOLBARITEM1;
    static const long ID_TOOLBARITEM2;
    static const long ID_TOOLBARITEM3;
    static const long ID_TOOLBARITEM4;
    static const long ID_TOOLBARITEM5;
    static const long ID_TOOLBARITEM9;
    static const long ID_TOOLBARITEM10;
    static const long ID_TOOLBARITEM6;
    static const long ID_TOOLBARITEM7;
    static const long ID_TOOLBARITEM8;
    static const long ID_TOOLBAR1;
    static const long MNU_1;
    static const long ID_MENUITEM49;
    static const long ID_MENUITEM50;
    static const long ID_MENUITEM48;
    static const long ID_TIMER1;
    //*)

    //(*Declarations(xRawLogViewerFrame)
    wxMenuItem* MenuItem29;
    wxPanel* pn_CObservationStereoImage;
    wxMenuItem* MenuItem23;
    wxMenu* Menu14;
    wxPanel* pn_Action;
    wxMenuItem* MenuItem40;
    wxMenuItem* MenuItem59;
    wxMenuItem* MenuItem31;
    wxMenuItem* MenuItem49;
    wxToolBarToolBase* ToolBarItem5;
    wxPanel* Panel6;
    wxPanel* Panel1;
    wxMenuItem* MenuItem30;
    mpWindow* plotAct2D_PHI;
    wxPanel* pn_CSensorialFrame;
    wxPanel* Panel7;
    wxBoxSizer* BoxSizer3;
    wxPanel* pn3Dobs_Int;
    wxMenuItem* MenuItem68;
    wxStatusBar* StatusBar1;
    wxStaticBitmapPopup* bmpObsStereoRight;
    wxMenuItem* MenuItem16;
    wxMenuItem* MenuItem73;
    wxMenuItem* MenuItem36;
    wxPanel* pn3Dobs_Depth;
    mpWindow* plotScan2D;
    wxTextCtrl* txtException;
    wxMenuItem* MenuItem80;
    wxMenuItem* MenuItem50;
    wxMenu* MenuItem51;
    wxStaticBitmapPopup* bmp3Dobs_conf;
    wxToolBarToolBase* ToolBarItem6;
    wxPanel* pn_CObservation3DRangeScan;
    wxMenu* Menu3;
    wxStaticBitmapPopup* bmp3Dobs_int;
    wxMenu* MenuItem39;
    wxMenuItem* MenuItem32;
    wxMenu* MenuItem45;
    wxMenuItem* MenuItem52;
    wxMenu* Menu40;
    wxPanel* pn_CObservationBearingRange;
    wxTimer timAutoLoad;
    wxMenu* Menu6;
    wxMenuItem* MenuItem66;
    wxMenuItem* MenuItem48;
    wxMenuItem* MenuItem76;
    wxPanel* Panel8;
    wxPanel* Panel9;
    wxMenuItem* MenuItem64;
    wxBoxSizer* BoxSizer2;
    wxMenuItem* MenuItem15;
    wxMenuItem* MenuItem72;
    wxStaticText* StaticText1;
    wxPanel* Panel10;
    wxToolBarToolBase* ToolBarItem7;
    wxPanel* pn3Dobs_3D;
    wxPanel* Panel2;
    wxToolBarToolBase* ToolBarItem2;
    wxMenuItem* MenuItem21;
    wxSplitterWindow* SplitterWindow1;
    wxMenuItem* MenuItem57;
    wxToolBarToolBase* ToolBarItem9;
    wxMenuItem* mnuCreateAVI;
    wxMenuItem* MenuItem54;
    wxPanel* pn3Dobs_Conf;
    wxStaticText* StaticText3;
    wxMenuItem* MenuItem74;
    wxMenu* MenuItem6;
    wxPanel* Panel4;
    wxButton* btnEditComments;
    wxPanel* pn_CObservationBeaconRanges;
    wxPanel* pn_CObservationGPS;
    wxTextCtrl* memStats;
    wxMenuItem* MenuItem3;
    mpWindow* plotRangeBearing;
    wxStaticBitmapPopup* bmpObsStereoDisp;
    wxMenu mnuTree;
    wxMenu* Menu20;
    wxPanel* Panel5;
    wxMenuItem* MenuItem78;
    wxPanel* pn_CObservationImage;
    wxMenuItem* MenuItem9;
    wxToolBarToolBase* ToolBarItem10;
    wxMenu* Menu4;
    wxBoxSizer* BoxSizer4;
    wxSplitterWindow* SplitterWindow3;
    wxMenu* MenuItem5;
    wxMenuItem* MenuItem85;
    wxMenuItem* MenuItem35;
    wxToolBar* ToolBar1;
    CRawlogTreeView* tree_view;
    wxMenuItem* MenuItem11;
    wxPanel* Panel3;
    wxMenuItem* MenuItem63;
    wxStaticBitmapPopup* bmpObsImage;
    wxBoxSizer* BoxSizer8;
    mpWindow* plotAct2D_XY;
    wxNotebook* Notebook3;
    wxToolBarToolBase* ToolBarItem4;
    wxMenuItem* MenuItem77;
    wxNotebook* Notebook2;
    wxMenuItem* MenuItem79;
    wxMenuItem* MenuItem44;
    wxStaticBitmapPopup* bmpObsStereoLeft;
    wxToolBarToolBase* ToolBarItem1;
    wxTextCtrl* memo;
    wxBoxSizer* BoxSizer1;
    wxMenuItem* MenuItem38;
    wxStaticBitmapPopup* bmp3Dobs_depth;
    wxMenuItem* MenuItem10;
    CMyGLCanvas* m_gl3DRangeScan;
    wxStaticText* StaticText2;
    wxSlider* slid3DcamConf;
    wxNotebook* Notebook1;
    wxPanel* pn_CObservation2DRangeScan;
    wxMenu* MenuItem84;
    wxToolBarToolBase* ToolBarItem3;
    wxMenuItem* MenuItem53;
    wxMenuItem* MenuItem7;
    wxMenuItem* MenuItem4;
    wxMenu* MenuItem42;
    wxMenuItem* MenuItem62;
    wxMenuItem* MenuItem13;
    wxNotebook* nb_3DObsChannels;
    wxMenu* Menu23;
    wxMenuItem* MenuItem28;
    wxMenuItem* MenuItem37;
    wxMenuItem* mnuItemEnable3DCamAutoGenPoints;
    wxMenuItem* MenuItem61;
    wxToolBarToolBase* ToolBarItem8;
    wxMenu* MenuItem8;
    wxPanel* pn_CObservationGasSensors;
    wxMenuItem* MenuItem34;
    wxMenuItem* MenuItem47;
    wxMenuItem* MenuItem83;
    wxMenuItem* MenuItem14;
    wxMenu* Menu38;
    wxMenu* MenuItem20;
    wxMenuItem* MenuItem58;
    wxBoxSizer* BoxSizer5;
    wxMenuItem* MenuItem46;
    wxMenuItem* MenuItem71;
    //*)


    wxComboBox* toolbarcomboImages;
	static const long ID_COMBO_IMG_DIRS;
    void OnComboImageDirsChange(wxCommandEvent& event);

	//void OntreeViewItemRightClick(wxTreeEvent& event);

	static void OntreeViewSelectionChanged(
		wxWindow*				me,
		CRawlogTreeView*		the_tree,
		TRawlogTreeViewEvent	ev,
		int					item_index,
		const mrpt::utils::CSerializablePtr &item_data);

	void SelectObjectInTreeView( const mrpt::utils::CSerializablePtr & sel_obj );

	void showNextTip(bool forceShow = false);

    // Layers for the 2D graphs:
    mpFXYVector     *lyScan2D, *lyRangeBearingLandmarks;
    mpFXYVector     *lyAction2D_XY,*lyAction2D_PHI;

    wxImageList     *imgList;     // Image list for the tree view:


    wxFileHistory	m_fileHistory;



	std::map<std::string,TInfoPerSensorLabel>	listOfSensorLabels;

	// ALWAYS access this inside a "try" block, for the case...
	mrpt::slam::CObservationPtr			curSelectedObservation;
	mrpt::utils::CSerializablePtr		curSelectedObject;
	mrpt::gui::CDisplayWindow3DPtr		winGPSPath;
	mrpt::gui::CDisplayWindowPlotsPtr  	winGPSPath2D_xy, winGPSPath2D_xz;


    DECLARE_EVENT_TABLE()
};

#ifdef wxUSE_UNICODE
#define _U(x) wxString((x),wxConvUTF8)
#define _UU(x,y) wxString((x),y)
#else
#define _U(x) (x)
#define _UU(x,y) (x)
#endif


#define WX_START_TRY \
    try \
    {


#define WX_END_TRY \
    } \
	catch(std::exception &e) \
    { \
		std::cerr << e.what() << std::endl; \
        wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this); \
    } \
    catch(...) \
    { \
		std::cerr << "Untyped exception!" << std::endl; \
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, this); \
    }


extern std::string             	iniFileSect;
extern mrpt::utils::CConfigFile *iniFile;
extern std::string              loadedFileName;
extern mrpt::slam::CRawlog		rawlog;


#endif // XRAWLOGVIEWERMAIN_H
