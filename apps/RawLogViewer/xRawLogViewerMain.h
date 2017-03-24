/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef XRAWLOGVIEWERMAIN_H
#define XRAWLOGVIEWERMAIN_H


//(*Headers(xRawLogViewerFrame)
#include "CRawlogTreeView.h"
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include <wx/splitter.h>
#include <wx/statline.h>
#include "MyGLCanvas.h"
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/statbmp.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/combobox.h>
#include <wx/statusbr.h>
//*)

#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
#include <wx/combobox.h>

#include <map>
#include <string>
#include <wx/docview.h>

// General global variables:
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#	undef Button1
#	undef Button2
#	undef Button3
#	undef Button4
#	undef Button5
#	undef Button6
#	undef Button7
#endif
// To avoid conflicts between Eigen & X11 headers
#ifdef Success 
#	undef Success 
#endif


// A list of sensor labels (and the times they appear) in the currently loaded rawlog.
struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() : max_ellapsed_tim_between_obs(.0), first(INVALID_TIMESTAMP) ,last(INVALID_TIMESTAMP)
	{}
	std::vector<double> timOccurs;
	double max_ellapsed_tim_between_obs;
	mrpt::system::TTimeStamp first,last;

	size_t getOccurences() const;
	void addOcurrence(mrpt::system::TTimeStamp obs_tim, mrpt::system::TTimeStamp first_dataset_tim);

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
    void OnGenRFIDTxt(wxCommandEvent& event);
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
    void OnMenuRenameSingleObs(wxCommandEvent& event);
    //*)

    //(*Identifiers(xRawLogViewerFrame)
    static const long ID_BUTTON2;
    static const long ID_BUTTON3;
    static const long ID_STATICLINE2;
    static const long ID_BUTTON4;
    static const long ID_BUTTON5;
    static const long ID_BUTTON6;
    static const long ID_BUTTON7;
    static const long ID_STATICLINE3;
    static const long ID_BUTTON8;
    static const long ID_BUTTON9;
    static const long ID_STATICLINE4;
    static const long ID_BUTTON10;
    static const long ID_BUTTON11;
    static const long ID_STATICLINE1;
    static const long ID_STATICTEXT4;
    static const long ID_COMBO_IMG_DIRS;
    static const long ID_CUSTOM5;
    static const long ID_PANEL1;
    static const long ID_TEXTCTRL1;
    static const long ID_PANEL3;
    static const long ID_BUTTON1;
    static const long ID_PANEL18;
    static const long ID_TEXTCTRL2;
    static const long ID_CUSTOM6;
    static const long ID_PANEL25;
    static const long ID_SPLITTERWINDOW2;
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
    static const long ID_MENUITEM91;
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
    static const long MNU_1;
    static const long ID_MENUITEM49;
    static const long ID_MENUITEM50;
    static const long ID_MENUITEM48;
    static const long ID_TIMER1;
    //*)

    //(*Declarations(xRawLogViewerFrame)
    wxMenu* MenuItem51;
    wxBoxSizer* BoxSizer4;
    wxMenu* MenuItem42;
    mpWindow* plotRangeBearing;
    wxMenuItem* MenuItem61;
    wxMenuItem* MenuItem31;
    wxMenuItem* MenuItem57;
    wxStaticBitmapPopup* bmpObsImage;
    wxPanel* pn3Dobs_Conf;
    wxMenuItem* MenuItem59;
    wxPanel* pn_CSensorialFrame;
    wxPanel* Panel5;
    wxCustomButton* Button4;
    wxPanel* pn_CObservation2DRangeScan;
    wxMenuItem* MenuItem7;
    wxMenuItem* MenuItem74;
    wxBoxSizer* BoxSizer5;
    wxMenuItem* MenuItem40;
    wxNotebook* Notebook1;
    wxPanel* pn_CObservationGasSensors;
    wxMenuItem* MenuItem80;
    wxBoxSizer* BoxSizer8;
    wxCustomButton* Button2;
    wxMenuItem* MenuItem54;
    wxMenu* MenuItem84;
    wxPanel* pn_CObservationBeaconRanges;
    wxStaticText* StaticText2;
    wxPanel* Panel4;
    wxMenuItem* MenuItem49;
    wxMenuItem* MenuItem50;
    mpWindow* plotScan2D;
    wxMenuItem* MenuItem68;
    wxMenu* Menu14;
    wxStaticBitmapPopup* bmp3Dobs_depth;
    wxMenu* Menu3;
    wxCustomButton* Button6;
    wxMenu* Menu20;
    wxMenuItem* MenuItem71;
    mpWindow* plotAct2D_PHI;
    wxMenuItem* MenuItem86;
    wxMenu* MenuItem20;
    wxSplitterWindow* SplitterWindow2;
    wxMenuItem* MenuItem46;
    wxStaticBitmapPopup* bmpObsStereoLeft;
    wxMenuItem* MenuItem4;
    wxMenuItem* MenuItem76;
    wxPanel* pn_Action;
    wxMenuItem* MenuItem14;
    wxPanel* Panel11;
    wxMenuItem* MenuItem36;
    wxCustomButton* btnToolbarOpen;
    wxMenuItem* mnuItemEnable3DCamAutoGenPoints;
    wxMenuItem* MenuItem11;
    wxPanel* pn3Dobs_3D;
    wxMenu mnuTree;
    wxPanel* Panel9;
    wxMenuItem* MenuItem29;
    wxCustomButton* Button1;
    wxMenu* Menu40;
    wxCustomButton* Button7;
    wxPanel* Panel8;
    wxPanel* pn_CObservationImage;
    wxMenuItem* MenuItem15;
    wxMenu* MenuItem39;
    wxMenuItem* MenuItem73;
    wxMenu* MenuItem6;
    wxPanel* pn_CObservationGPS;
    wxPanel* Panel1;
    wxMenuItem* MenuItem37;
    wxMenu* Menu38;
    wxStaticText* StaticText1;
    wxMenuItem* MenuItem32;
    wxBoxSizer* BoxSizer2;
    wxTimer timAutoLoad;
    wxStaticText* StaticText3;
    wxMenuItem* MenuItem13;
    wxMenu* MenuItem8;
    mpWindow* plotAct2D_XY;
    wxMenuItem* MenuItem10;
    wxNotebook* nb_3DObsChannels;
    wxMenuItem* MenuItem62;
    wxPanel* Panel6;
    wxPanel* Panel3;
    wxStaticLine* StaticLine4;
    wxStaticLine* StaticLine2;
    mpWindow* plotRawlogSensorTimes;
    wxMenuItem* MenuItem72;
    wxMenuItem* MenuItem44;
    wxPanel* pn_CObservationBearingRange;
    wxCustomButton* Button8;
    wxMenuItem* MenuItem79;
    wxComboBox* toolbarcomboImages;
    wxMenu* Menu23;
    wxMenuItem* MenuItem38;
    wxMenuItem* MenuItem3;
    wxCustomButton* Button3;
    CRawlogTreeView* tree_view;
    wxMenuItem* MenuItem64;
    wxStaticBitmapPopup* bmp3Dobs_int;
    wxTextCtrl* memo;
    wxMenuItem* MenuItem28;
    wxMenuItem* MenuItem63;
    CMyGLCanvas* m_gl3DRangeScan;
    wxPanel* Panel7;
    wxMenuItem* MenuItem78;
    wxPanel* pn3Dobs_Depth;
    wxMenuItem* mnuCreateAVI;
    wxMenuItem* MenuItem83;
    wxMenu* MenuItem45;
    wxStatusBar* StatusBar1;
    wxMenuItem* MenuItem52;
    wxStaticBitmapPopup* bmpObsStereoDisp;
    wxCustomButton* Button5;
    wxStaticLine* StaticLine3;
    wxMenuItem* MenuItem35;
    wxStaticLine* StaticLine1;
    wxSplitterWindow* SplitterWindow3;
    wxMenuItem* MenuItem23;
    wxBoxSizer* BoxSizer1;
    wxStaticBitmapPopup* bmp3Dobs_conf;
    wxMenuItem* MenuItem58;
    wxPanel* pn3Dobs_Int;
    wxTextCtrl* memStats;
    wxPanel* pn_CObservationStereoImage;
    wxSlider* slid3DcamConf;
    wxPanel* Panel2;
    wxMenu* MenuItem5;
    wxMenuItem* MenuItem21;
    wxPanel* Panel10;
    wxNotebook* Notebook2;
    wxMenuItem* MenuItem34;
    wxMenuItem* MenuItem16;
    wxSplitterWindow* SplitterWindow1;
    wxBoxSizer* BoxSizer3;
    wxNotebook* Notebook3;
    wxMenu* Menu6;
    wxMenuItem* MenuItem9;
    wxStaticText* StaticText4;
    wxPanel* pn_CObservation3DRangeScan;
    wxMenuItem* MenuItem47;
    wxMenuItem* MenuItem30;
    wxStaticBitmapPopup* bmpObsStereoRight;
    wxMenuItem* MenuItem77;
    wxMenuItem* MenuItem66;
    wxMenuItem* MenuItem53;
    wxMenuItem* MenuItem48;
    wxCustomButton* Button9;
    wxMenu* Menu4;
    wxTextCtrl* txtException;
    wxButton* btnEditComments;
    wxMenuItem* MenuItem85;
    //*)


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
	mrpt::obs::CObservationPtr			curSelectedObservation;
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
        wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, NULL); \
    } \
    catch(...) \
    { \
		std::cerr << "Untyped exception!" << std::endl; \
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, NULL); \
    }


extern std::string             	iniFileSect;
extern mrpt::utils::CConfigFile *iniFile;
extern std::string              loadedFileName;
extern mrpt::obs::CRawlog		rawlog;


#endif // XRAWLOGVIEWERMAIN_H
