/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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

#include <string>

#include "ViewOptions3DPoints.h"

// General global variables:
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/bimap.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSimpleLine.h>

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
  wxBitmap CreateBitmap(const wxArtID& id, const wxArtClient& client, const wxSize& size) override;
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
  std::vector<std::string> AskForObservationByLabelMultiple(const std::string& title);

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
  void OnMRUFile([[maybe_unused]] wxCommandEvent& event);

  //(*Handlers(xRawLogViewerFrame)
  void OnQuit([[maybe_unused]] wxCommandEvent& event);
  void OnAbout([[maybe_unused]] wxCommandEvent& event);
  void OnFileOpen([[maybe_unused]] wxCommandEvent& event);
  void OnSaveFile([[maybe_unused]] wxCommandEvent& event);
  void OnEditRawlog([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItem37Selected([[maybe_unused]] wxCommandEvent& event);
  void OntimAutoLoadTrigger(wxTimerEvent& event);
  void OnNotebook1PageChanging(wxNotebookEvent& event);
  void OnChangeMotionModel([[maybe_unused]] wxCommandEvent& event);
  void OnShowImagesAsVideo([[maybe_unused]] wxCommandEvent& event);
  void OnRawMapOdo([[maybe_unused]] wxCommandEvent& event);
  void OnMenuGenerateBeaconList([[maybe_unused]] wxCommandEvent& event);
  void OnImportCARMEN([[maybe_unused]] wxCommandEvent& event);
  void OnGenerateSeqImgs([[maybe_unused]] wxCommandEvent& event);
  void OnGenGasTxt([[maybe_unused]] wxCommandEvent& event);
  void OnGenWifiTxt([[maybe_unused]] wxCommandEvent& event);
  void OnGenRFIDTxt([[maybe_unused]] wxCommandEvent& event);
  void OnGenGPSTxt([[maybe_unused]] wxCommandEvent& event);
  void OnGenOdoLaser([[maybe_unused]] wxCommandEvent& event);
  void OnSummaryGPS([[maybe_unused]] wxCommandEvent& event);
  void OnShowICP([[maybe_unused]] wxCommandEvent& event);
  void OnLoadAPartOnly([[maybe_unused]] wxCommandEvent& event);
  void OnFileCountEntries([[maybe_unused]] wxCommandEvent& event);
  void OnFileSaveImages([[maybe_unused]] wxCommandEvent& event);
  void OnFileGenVisualLMFromStereoImages([[maybe_unused]] wxCommandEvent& event);
  void OnChangeSensorPositions([[maybe_unused]] wxCommandEvent& event);
  void OnDecimateRecords([[maybe_unused]] wxCommandEvent& event);
  void OnCountBadScans([[maybe_unused]] wxCommandEvent& event);
  void OnRecalculateActionsICP([[maybe_unused]] wxCommandEvent& event);
  void OnRecomputeOdometry([[maybe_unused]] wxCommandEvent& event);
  void OnFilterSpureousGas([[maybe_unused]] wxCommandEvent& event);
  void OnFilterErroneousScans([[maybe_unused]] wxCommandEvent& event);
  void OnRemoveSpecificRangeMeas([[maybe_unused]] wxCommandEvent& event);
  void OnGenerateTextFileRangeBearing([[maybe_unused]] wxCommandEvent& event);
  void OnForceEncodersFalse([[maybe_unused]] wxCommandEvent& event);
  void OnGenerateIMUTextFile([[maybe_unused]] wxCommandEvent& event);
  void OnRangeFinder1DGenTextFile([[maybe_unused]] wxCommandEvent& event);
  void OnImportSequenceOfImages([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItem47Selected([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItem46Selected([[maybe_unused]] wxCommandEvent& event);
  void OnMenuExpandAll([[maybe_unused]] wxCommandEvent& event);
  void OnMenuCollapseAll([[maybe_unused]] wxCommandEvent& event);
  void OnMenuModifyICPActionsUncertainty([[maybe_unused]] wxCommandEvent& event);
  void OnShowAnimateScans([[maybe_unused]] wxCommandEvent& event);
  void OnMenuExportALOG([[maybe_unused]] wxCommandEvent& event);
  void OnMenuImportALOG([[maybe_unused]] wxCommandEvent& event);
  void OnMenuCompactRawlog([[maybe_unused]] wxCommandEvent& event);
  void OnMenuLossLessDecimate([[maybe_unused]] wxCommandEvent& event);
  void OnMenCompactFILE([[maybe_unused]] wxCommandEvent& event);
  void OnMenuLossLessDecFILE([[maybe_unused]] wxCommandEvent& event);
  void OnMenuConvertExternallyStored([[maybe_unused]] wxCommandEvent& event);
  void OnImportRTL([[maybe_unused]] wxCommandEvent& event);
  void OnMenuConvertObservationOnly([[maybe_unused]] wxCommandEvent& event);
  void OnMenuDistanceBtwGPSs([[maybe_unused]] wxCommandEvent& event);
  void OnMenuResortByTimestamp([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRegenerateGPSTimestamps([[maybe_unused]] wxCommandEvent& event);
  void OnMenuShiftTimestampsByLabel([[maybe_unused]] wxCommandEvent& event);
  void OnMenuShowTips([[maybe_unused]] wxCommandEvent& event);
  void OnbtnEditCommentsClick([[maybe_unused]] wxCommandEvent& event);
  void OnMenuInsertComment([[maybe_unused]] wxCommandEvent& event);
  void OnMenuDrawGPSPath([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRenameSensor([[maybe_unused]] wxCommandEvent& event);
  void OnMenuChangePosesBatch([[maybe_unused]] wxCommandEvent& event);
  void OnMenuMono2Stereo([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRectifyImages([[maybe_unused]] wxCommandEvent& event);
  void OnMenuMarkLaserScanInvalid([[maybe_unused]] wxCommandEvent& event);
  void OnMenuChangeMaxRangeLaser([[maybe_unused]] wxCommandEvent& event);
  void OnbtnEditCommentsClick1([[maybe_unused]] wxCommandEvent& event);
  void OnMenuConvertSF([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRevert([[maybe_unused]] wxCommandEvent& event);
  void OnMenuBatchLaserExclusionZones([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRenameImageFiles([[maybe_unused]] wxCommandEvent& event);
  void OnLaserFilterAngles([[maybe_unused]] wxCommandEvent& event);
  void OnMenuGPSDeleteNaN([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRangeBearFilterIDs([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRegenerateTimestampBySF([[maybe_unused]] wxCommandEvent& event);
  void OnmnuCreateAVISelected([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRegenerateOdometryTimes([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItem3DObsRecoverParams([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItemImportBremenDLRLog([[maybe_unused]] wxCommandEvent& event);
  void OnMenuRenameSingleObs([[maybe_unused]] wxCommandEvent& event);
  //*)
  void OnMenuRenameBySFIndex([[maybe_unused]] wxCommandEvent& event);

  void OnTimeLineDoScrollToMouseX(const std::optional<std::pair<double, size_t>>& selPt);

  void OnTimeLineMouseMove(wxMouseEvent& e);
  void OnTimeLineMouseLeftDown(const std::optional<std::pair<double, size_t>>& selPt);
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
  wxTextCtrl* memo;
  wxMenuItem* MenuItem28;
  wxMenuItem* MenuItem63;
  CMyGLCanvas* m_gl3DRangeScan;
  wxPanel* Panel7;
  wxMenuItem* MenuItem78;
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

  mrpt::viz::Viewport::Ptr bmp3Dobs_depth;
  mrpt::viz::Viewport::Ptr bmp3Dobs_int;
  mrpt::viz::Viewport::Ptr bmp3Dobs_3dcloud;

  void OnComboImageDirsChange([[maybe_unused]] wxCommandEvent& event);
  void On3DObsPagesChange(wxBookCtrlEvent& event);

  void OnSize(wxSizeEvent& event);
  void OnMaximize(wxMaximizeEvent& event);
  void OnSizeOrMaximize();

  void createTimeLineObjects(wxFlexGridSizer* fgzMain);

  // void OntreeViewItemRightClick(wxTreeEvent& event);

  static void OntreeViewSelectionChanged(
      wxWindow* me,
      CRawlogTreeView* the_tree,
      TRawlogTreeViewEvent ev,
      int item_index,
      const mrpt::serialization::CSerializable::Ptr& item_data);

  void SelectObjectInTreeView(const mrpt::serialization::CSerializable::Ptr& sel_obj);

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
    double scrollBarStartPercent = 0.0;        // [0-1]: left-most part

    mrpt::viz::CBox::Ptr borderBox;
    mrpt::viz::CSetOfObjects::Ptr xTicks;
    /// one pointcloud per sensorLabel
    mrpt::viz::CSetOfObjects::Ptr allSensorDots;
    mrpt::viz::CBox::Ptr cursor;
    mrpt::viz::CBox::Ptr visiblePage;
    mrpt::viz::CSetOfObjects::Ptr ySensorLabels;
    mrpt::viz::CBox::Ptr horizontalCursor;

    mrpt::containers::bimap<double, std::string> yCoordToSensorLabel;
  };

  TimeLineData m_timeline;

  /// Return (xsTime, TreeIndex) in xs2treeIndices, none if no match:
  std::optional<std::pair<double, size_t>> timeLineMouseXYToTreeIndex(
      const wxMouseEvent& e,
      bool refineBySensorLabelVerticalMatch = true,
      const std::optional<std::string>& forceThisSensorLabel = std::nullopt,
      const mrpt::optional_ref<std::string>& outSelectedSensorLabel = std::nullopt) const;

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

#endif  // XRAWLOGVIEWERMAIN_H
