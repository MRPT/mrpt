/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef XRAWLOGVIEWERMAIN_H
#define XRAWLOGVIEWERMAIN_H

//(*Headers(xRawLogViewerFrame)
#include <wx/button.h>
#include <wx/checklst.h>
#include <wx/combobox.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/splitter.h>
#include <wx/statbmp.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/statusbr.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include <wx/timer.h>

#include "CRawlogTreeView.h"
#include "MyGLCanvas.h"
//*)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/combobox.h>
#include <wx/docview.h>
#include <wx/image.h>
#include <wx/scrolbar.h>
#include <wx/simplebook.h>

#include <map>
#include <string>

#include "ViewOptions3DPoints.h"

// General global variables:
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/bimap.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/serialization/CSerializable.h>

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#undef Button1
#undef Button2
#undef Button3
#undef Button4
#undef Button5
#undef Button6
#undef Button7
#endif
// To avoid conflicts between Eigen & X11 headers
#ifdef Success
#undef Success
#endif

// A list of sensor labels (and the times they appear) in the currently loaded
// rawlog.
struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() = default;
	std::vector<mrpt::Clock::time_point> timOccurs;
	double max_ellapsed_tim_between_obs{.0};
	mrpt::Clock::time_point first, last;

	size_t getOccurences() const;
	void addOcurrence(mrpt::Clock::time_point obsTim);
};

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
   protected:
	wxBitmap CreateBitmap(
		const wxArtID& id, const wxArtClient& client,
		const wxSize& size) override;
};

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/3rdparty/mathplot/mathplot.h>

// Auxiliary data types used to import ALOG files:
struct TAlogRecord
{
	std::string label;
	int8_t type;  // 0: odo, 1: 2d laser, 2:3d laser, 3: image
	std::vector<float> data;
	std::string imgFile;
	double startElev, endElev;
};

class xRawLogViewerFrame : public wxFrame
{
   public:
	xRawLogViewerFrame(wxWindow* parent, wxWindowID id = -1);
	~xRawLogViewerFrame() override;

	bool AskForOpenRawlog(std::string& fil);
	bool AskForSaveRawlog(std::string& fil);

	/** Return an empty string if cancel, otherwise "bmp","jpg",... */
	std::string AskForImageFileFormat();

	/** Return the label of the selected sensor, or "" if cancel or no labels */
	std::string AskForObservationByLabel(const std::string& title);

	/** Return one or more labels of the selected sensor, or "" if cancel or no
	 * labels */
	std::vector<std::string> AskForObservationByLabelMultiple(
		const std::string& title);

	void saveImportedLogToRawlog(
		const std::string& target_filename,
		const std::map<double, TAlogRecord>& theAlog,
		const std::string& dir_for_images);

	ViewOptions3DPoints* getViewOptions() { return pnViewOptions; }

	void bottomTimeLineUpdateCursorFromTreeScrollPos();

   private:
	bool m_needsToRefreshTimeline = true;
	bool m_needsToRefresh3DRangeScanView = true;

	/** Loads the given file in memory, in the varibale "rawlog"
	 */
	void loadRawlogFile(const std::string& str, int first = 0, int last = -1);

	/** Rebuilds the tree view with the data in "rawlog".
	 */
	void rebuildTreeView();

	/// This needs to be called when thedataset changes
	/// (automatically called from rebuildTreeView()).
	void rebuildBottomTimeLine();

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
	void OnMenuItemImportBremenDLRLog(wxCommandEvent& event);
	void OnMenuRenameSingleObs(wxCommandEvent& event);
	//*)
	void OnMenuRenameBySFIndex(wxCommandEvent& event);

	void OnTimeLineDoScrollToMouseX(
		const std::optional<std::pair<double, size_t>>& selPt);

	void OnTimeLineMouseMove(wxMouseEvent& e);
	void OnTimeLineMouseLeftDown(
		const std::optional<std::pair<double, size_t>>& selPt);
	void OnTimeLineMouseLeftUp(wxMouseEvent& e);
	void OnTimeLineMouseRightDown(wxMouseEvent& e);
	void OnTimeLineMouseRightUp(wxMouseEvent& e);
	void OnTimeLineMouseWheel(wxMouseEvent& e);
	void OnTimelineZoomScroolBar(const wxScrollEvent& e);

	//(*Declarations(xRawLogViewerFrame)
	wxMenu* MenuItem51;
	wxBoxSizer* BoxSizer4;
	wxMenu* MenuItem42;
	mpWindow* plotRangeBearing;
	wxMenuItem* MenuItem61;
	wxMenuItem* MenuItem31;
	wxMenuItem* MenuItem57;
	CMyGLCanvas* bmpObsImage;
	wxPanel* pn3Dobs_Conf;
	ViewOptions3DPoints* pnViewOptions;
	wxMenuItem* MenuItem59;
	wxPanel* pn_CSensorialFrame;
	wxPanel* Panel5;
	wxCustomButton* Button4;
	wxMenuItem* MenuItem7;
	wxMenuItem* MenuItem74;
	wxMenuItem* MenuItem40;
	wxSimplebook* Notebook1;
	wxPanel* pn_CObservationGasSensors;
	wxMenuItem* MenuItem80;
	wxBoxSizer* BoxSizer8;
	wxCustomButton* Button2;
	wxMenuItem* MenuItem54;
	wxMenu* MenuItem84;
	wxPanel* pn_CObservationBeaconRanges;
	wxStaticText* StaticText2;
	wxMenuItem* MenuItem49;
	wxMenuItem* MenuItem50;
	wxMenuItem* MenuItem68;
	wxMenu* Menu14;
	CMyGLCanvas* bmp3Dobs_depth;
	wxMenu* Menu3;
	wxCustomButton* Button6;
	wxMenu* Menu20;
	wxMenuItem* MenuItem71;
	mpWindow* plotAct2D_PHI;
	wxMenuItem* MenuItem86;
	wxMenu* MenuItem20;
	wxMenuItem* MenuItem46;
	CMyGLCanvas* bmpObsStereoLeft;
	wxMenuItem* MenuItem4;
	wxMenuItem* MenuItem76;
	wxPanel* pn_Action;
	wxMenuItem* MenuItem14;
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
	CRawlogTreeView* m_treeView;
	wxMenuItem* MenuItem64;
	CMyGLCanvas* bmp3Dobs_int;
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
	CMyGLCanvas* bmpObsStereoDisp;
	wxStaticLine* StaticLine3;
	wxMenuItem* MenuItem35;
	wxStaticLine* StaticLine1;
	wxSplitterWindow* SplitterWindow3;
	wxMenuItem* MenuItem23;
	wxBoxSizer* BoxSizer1;
	CMyGLCanvas* bmp3Dobs_conf;
	wxMenuItem* MenuItem58;
	wxPanel* pn3Dobs_Int;
	wxTextCtrl* memStats;
	wxPanel* pn_CObservationStereoImage;
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
	CMyGLCanvas* bmpObsStereoRight;
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
	wxFlexGridSizer* FlexGridSizerImg = nullptr;
	wxScrolledWindow* ScrolledWindow2 = nullptr;
	wxTextCtrl* edSelectedTimeInfo = nullptr;
	CMyGLCanvas* m_glTimeLine = nullptr;

	void OnComboImageDirsChange(wxCommandEvent& event);
	void On3DObsPagesChange(wxBookCtrlEvent& event);

	void OnSize(wxSizeEvent& event);
	void OnMaximize(wxMaximizeEvent& event);
	void OnSizeOrMaximize();

	void createTimeLineObjects(wxFlexGridSizer* fgzMain);

	// void OntreeViewItemRightClick(wxTreeEvent& event);

	static void OntreeViewSelectionChanged(
		wxWindow* me, CRawlogTreeView* the_tree, TRawlogTreeViewEvent ev,
		int item_index,
		const mrpt::serialization::CSerializable::Ptr& item_data);

	void SelectObjectInTreeView(
		const mrpt::serialization::CSerializable::Ptr& sel_obj);

	void showNextTip(bool forceShow = false);

	// Layers for the 2D graphs:
	mpFXYVector* lyRangeBearingLandmarks;
	mpFXYVector *lyAction2D_XY, *lyAction2D_PHI;

	wxFileHistory m_fileHistory;

	std::map<std::string, TInfoPerSensorLabel> listOfSensorLabels;

	struct TimeLineData
	{
		TimeLineData() = default;

		mrpt::Clock::time_point min_t = INVALID_TIMESTAMP;
		mrpt::Clock::time_point max_t = INVALID_TIMESTAMP;
		// correspondence between xs (-1,1 coordinates) and tree element index.

		/// opengl [-1,1] xCoords => indices
		std::multimap<double, size_t> xs2treeIndices;
		std::map<size_t, double> treeIndices2xs;
		size_t actualLeftBorderPixels = 10;
		std::optional<std::string> currentTrackedSensorLabel;

		void clearStats()
		{
			min_t = INVALID_TIMESTAMP;
			max_t = INVALID_TIMESTAMP;
			xs2treeIndices.clear();
			treeIndices2xs.clear();
		}

		void resetAfterDatasetChanged()
		{
			scrollBarZoomVisiblePercent = 1.0;
			scrollBarStartPercent = .0;
		}

		wxScrollBar* sbTimeLineRange = nullptr;
		double scrollBarZoomVisiblePercent = 1.0;  // [0-1]: visible range
		double scrollBarStartPercent = 0.0;	 // [0-1]: left-most part

		mrpt::opengl::CBox::Ptr borderBox;
		mrpt::opengl::CSetOfObjects::Ptr xTicks;
		mrpt::opengl::CPointCloud::Ptr allSensorDots;
		mrpt::opengl::CBox::Ptr cursor;
		mrpt::opengl::CBox::Ptr visiblePage;
		mrpt::opengl::CSetOfObjects::Ptr ySensorLabels;
		mrpt::opengl::CBox::Ptr horizontalCursor;

		mrpt::containers::bimap<double, std::string> yCoordToSensorLabel;
	};

	TimeLineData m_timeline;

	/// Return (xsTime, TreeIndex) in xs2treeIndices, none if no match:
	std::optional<std::pair<double, size_t>> timeLineMouseXYToTreeIndex(
		const wxMouseEvent& e, bool refineBySensorLabelVerticalMatch = true,
		const std::optional<std::string>& forceThisSensorLabel = std::nullopt,
		const mrpt::optional_ref<std::string>& outSelectedSensorLabel =
			std::nullopt) const;

	// ALWAYS access this inside a "try" block, just in case...
	mrpt::obs::CObservation::Ptr m_selectedObj;
	mrpt::serialization::CSerializable::Ptr curSelectedObject;
	mrpt::gui::CDisplayWindow3D::Ptr winGPSPath;
	mrpt::gui::CDisplayWindowPlots::Ptr winGPSPath2D_xy, winGPSPath2D_xz;

	DECLARE_EVENT_TABLE()
};

extern std::string iniFileSect;
extern std::unique_ptr<mrpt::config::CConfigFile> iniFile;
extern std::string loadedFileName;
extern mrpt::obs::CRawlog rawlog;

#endif	// XRAWLOGVIEWERMAIN_H
