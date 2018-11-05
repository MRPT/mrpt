/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef _DSCENEVIEWERMAIN_H
#define _DSCENEVIEWERMAIN_H

#include <wx/menu.h>
#include <wx/toolbar.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
#include <wx/msgdlg.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
#include <wx/statline.h>

#include <wx/things/toggle.h>

#include <mrpt/system/datetime.h>
#include <mrpt/gui/CWxGLCanvasBase.h>
#include <mrpt/gui/WxUtils.h>

class CDlgCamTracking;
class CMyGLCanvas : public mrpt::gui::CWxGLCanvasBase
{
   public:
	CMyGLCanvas(
		wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize, long style = 0,
		const wxString& name = _T("CWxGLCanvasBase"))
		: CWxGLCanvasBase(parent, id, pos, size, style, name)
	{
	}

	void OnCharCustom(wxKeyEvent& event) override;

	void OnPreRender() override;
	void OnPostRender() override;
	void OnPostRenderSwapBuffers(double At, wxPaintDC& dc) override;
	void OnRenderError(const wxString& str) override;
};

class _DSceneViewerFrame : public wxFrame
{
	friend class CMyGLCanvas;
	friend class CDlgCamTracking;

   public:
	_DSceneViewerFrame(wxWindow* parent, wxWindowID id = -1);
	~_DSceneViewerFrame() override;

   private:
	//(*Handlers(_DSceneViewerFrame)
	void OnQuit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	void OnNewScene(wxCommandEvent& event);
	void OnOpenFile(wxCommandEvent& event);
	void OntimLoadFileCmdLineTrigger(wxTimerEvent& event);
	void OnbtnAutoplayClicked(wxCommandEvent& event);
	void OnMenuBackColor(wxCommandEvent& event);
	void OnMenuOptions(wxCommandEvent& event);
	void OnPrevious(wxCommandEvent& event);
	void OnNext(wxCommandEvent& event);
	void OnClose(wxCloseEvent& event);
	void OnBtnRecordClicked(wxCommandEvent& event);
	void OnbtnOrthoClicked(wxCommandEvent& event);
	void OnReload(wxCommandEvent& event);
	void OnInsert3DS(wxCommandEvent& event);
	void OnMenuSave(wxCommandEvent& event);
	void OnChangeCameraPose(wxCommandEvent& event);
	void OnTravellingTrigger(wxTimerEvent& event);
	void OnStartCameraTravelling(wxCommandEvent& event);
	void OnClose1(wxCloseEvent& event);
	void OnMenuAddSICK(wxCommandEvent& event);
	void OnMenuDeleteAll(wxCommandEvent& event);
	void OnMenuItem14Selected(wxCommandEvent& event);
	void OnMenuCameraTrackingArbitrary(wxCommandEvent& event);
	void OnmnuItemShowCloudOctreesSelected(wxCommandEvent& event);
	void OnmnuItemChangeMaxPointsPerOctreeNodeSelected(wxCommandEvent& event);
	void OnmnuSceneStatsSelected(wxCommandEvent& event);
	void OnMenuItemImportPLYPointCloud(wxCommandEvent& event);
	void OnMenuItemExportPointsPLY(wxCommandEvent& event);
	void OnMenuItemHighResRender(wxCommandEvent& event);
	void OnmnuSelectionDeleteSelected(wxCommandEvent& event);
	void OnmnuSelectionScaleSelected(wxCommandEvent& event);
	void OnmnuSelectByClassSelected(wxCommandEvent& event);
	void OnmnuSelectNoneSelected(wxCommandEvent& event);
	void OnmnuImportLASSelected(wxCommandEvent& event);
	void OnmnuImportLASSelected1(wxCommandEvent& event);
	//*)

	void OntimAutoplay(wxTimerEvent& event);

	//(*Identifiers(_DSceneViewerFrame)
	static const long ID_BUTTON1;
	static const long ID_BUTTON2;
	static const long ID_STATICLINE1;
	static const long ID_BUTTON3;
	static const long ID_BUTTON4;
	static const long ID_BUTTON5;
	static const long ID_STATICLINE2;
	static const long ID_BUTTON6;
	static const long ID_BUTTON7;
	static const long ID_BUTTON8;
	static const long ID_BUTTON9;
	static const long ID_STATICLINE3;
	static const long ID_BUTTON10;
	static const long ID_BUTTON11;
	static const long ID_MENUITEM1;
	static const long ID_MENUITEM2;
	static const long ID_MENUITEM5;
	static const long ID_MENUITEM7;
	static const long ID_MENUITEM6;
	static const long ID_MENUITEM20;
	static const long ID_MENUITEM25;
	static const long ID_MENUITEM19;
	static const long ID_MENUITEM22;
	static const long ID_MENUITEM21;
	static const long ID_MENUITEM29;
	static const long ID_MENUITEM30;
	static const long ID_MENUITEM12;
	static const long ID_MENUITEM23;
	static const long ID_MENUITEM18;
	static const long idMenuQuit;
	static const long ID_MENUITEM24;
	static const long ID_MENUITEM26;
	static const long ID_MENUITEM27;
	static const long ID_MENUITEM28;
	static const long ID_MENUITEM4;
	static const long ID_MENUITEM3;
	static const long ID_MENUITEM15;
	static const long ID_MENUITEM17;
	static const long ID_MENUITEM16;
	static const long ID_MENUITEM11;
	static const long ID_MENUITEM9;
	static const long ID_MENUITEM8;
	static const long ID_MENUITEM10;
	static const long ID_MENUITEM14;
	static const long ID_MENUITEM13;
	static const long idMenuAbout;
	static const long ID_STATUSBAR1;
	static const long ID_TIMER1;
	//*)

	static const long ID_TRAVELLING_TIMER;
	static const long ID_TIMER_AUTOPLAY;

	//(*Declarations(_DSceneViewerFrame)
	wxMenuItem* MenuItem8;
	wxMenuItem* mnuSelectionDelete;
	wxMenuItem* MenuItem7;
	wxCustomButton* btnCapture;
	wxCustomButton* btnNew;
	wxCustomButton* btnNext;
	wxMenuItem* MenuItem5;
	wxMenu* Menu3;
	wxMenu* MenuItem20;
	wxMenuItem* MenuItem14;
	wxCustomButton* btnToolbarOpen;
	wxMenuItem* mnuItemChangeMaxPointsPerOctreeNode;
	wxCustomButton* btnQuit;
	wxCustomButton* btnAutoplay;
	wxMenuItem* MenuItem22;
	wxMenuItem* MenuItem10;
	wxStaticLine* StaticLine2;
	wxMenuItem* MenuItem24;
	wxCustomButton* btnOptions;
	wxMenuItem* mnuSelectNone;
	wxMenuItem* mnuSceneStats;
	wxMenuItem* mnuSelectionScale;
	wxMenuItem* mnuImportLAS;
	wxCustomButton* btnOrtho;
	wxStatusBar* StatusBar1;
	wxMenuItem* MenuItem6;
	wxStaticLine* StaticLine3;
	wxStaticLine* StaticLine1;
	wxMenuItem* MenuItem23;
	wxTimer timLoadFileCmdLine;
	wxCustomButton* btnAbout;
	wxMenuItem* MenuItem21;
	wxMenuItem* mnuSelectByClass;
	wxMenuItem* MenuItem16;
	wxMenuItem* MenuItem9;
	wxMenu* MenuItem18;
	wxMenuItem* mnuItemShowCloudOctrees;
	wxCustomButton* btnReload;
	wxCustomButton* btnPrev;
	wxMenu* Menu4;
	wxMenuItem* MenuItem19;
	wxMenu* MenuItem11;
	wxMenu* MenuItem17;
	//*)

	CMyGLCanvas* m_canvas;
	wxTimer* m_autoplayTimer;

	/** The list of currently selected objects */
	std::vector<mrpt::opengl::CRenderizable::Ptr> m_selected_gl_objects;

	wxTimer m_tTravelling;
	bool m_travelling_is_arbitrary;
	std::optional<mrpt::Clock::time_point> m_travelling_start_time;

	int m_nTicksNumber;
	double m_nCurrentAzimuth;
	int maxv;
	void loadFromFile(const std::string& fil, bool isInASequence = false);
	void updateTitle();
	void clear_all_octrees_in_scene();

	CDlgCamTracking* m_dlg_tracking;

	DECLARE_EVENT_TABLE()
};

#endif  // _DSCENEVIEWERMAIN_H
