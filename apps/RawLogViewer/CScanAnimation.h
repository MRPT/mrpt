/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef CSCANANIMATION_H
#define CSCANANIMATION_H

//(*Headers(CScanAnimation)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/checklst.h>
#include <wx/dialog.h>
#include <wx/radiobut.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "MyGLCanvas.h"
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/3rdparty/mathplot/mathplot.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CPointCloudColoured.h>

class CScanAnimation : public wxDialog
{
   public:
	CScanAnimation(
		wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize);
	~CScanAnimation() override;

	//(*Declarations(CScanAnimation)
	wxButton* btnStop;
	wxSlider* slPos;
	wxSpinCtrl* edDelay;
	wxStaticText* StaticText1;
	wxStaticText* StaticText2;
	wxStaticText* StaticText3;
	wxButton* btnClose;
	wxButton* btnJump;
	wxSpinCtrl* edIndex;
	wxCheckBox* cbViewOrtho;
	wxButton* btnPlay;
	CMyGLCanvas* m_plot3D;
	wxTextCtrl* edTimestamp;
	wxCheckListBox* lstObsLabels;
	wxButton* btnVizOptions;
	//*)

   protected:
	//(*Identifiers(CScanAnimation)
	static const long ID_LIST_OBS_LABELS;
	static const long ID_RADIOBUTTON2;
	static const long ID_STATICTEXT4;
	static const long ID_STATICTEXT22;
	static const long ID_TEXTCTRL11;
	static const long ID_BUTTON5;
	static const long ID_BUTTON1;
	static const long ID_BUTTON2;
	static const long ID_SPINCTRL2;
	static const long ID_CHECKBOX1;
	static const long ID_BUTTON3;
	static const long ID_XY_GLCANVAS;
	static const long ID_SLIDER1;
	static const long ID_STATICTEXT1;
	static const long ID_SPINCTRL1;
	static const long ID_BUTTON4;
	static const long ID_STATICTEXT2;
	static const long ID_CHECKBOX2;
	static const long ID_STATICTEXT3;
	static const long ID_BUTTON6;
	static const long ID_BUTTON7;
	//*)
	static const long ID_BUTTON_SAVE_SCENE;

   private:
	//(*Handlers(CScanAnimation)
	void OnbtnPlayClick(wxCommandEvent& event);
	void OnbtnStopClick(wxCommandEvent& event);
	void OnbtnCloseClick(wxCommandEvent& event);
	void OnslPosCmdScrollChanged(wxCommandEvent& event);
	void OnbtnJumpClick(wxCommandEvent& event);
	void OnslPosCmdScroll(wxScrollEvent& event);
	void OnbtnPickInputClick(wxCommandEvent& event);
	void OnInit(wxInitDialogEvent& event);
	void OncbViewOrthoClick(wxCommandEvent& event);
	//*)

	void OnbtnVizOptions(wxCommandEvent& event);
	void OnbtnSave3DScene(wxCommandEvent& event);

	DECLARE_EVENT_TABLE()

	bool m_stop;

	using sensor_label_t = std::string;
	std::map<sensor_label_t, bool> m_visibleSensors;

	struct TRenderObject
	{
		mrpt::system::TTimeStamp timestamp;
		mrpt::opengl::CRenderizable::Ptr obj;
	};
	using TListGlObjects = std::map<std::string, TRenderObject>;
	/** All the observations added to the map. */
	TListGlObjects m_gl_objects;

	/// Get the rawlog entry (from cur. loaded rawlog), build and displays its
	/// map:
	/// \return true if the viz has been refreshed.
	bool rebuild_view(bool forceRefreshView = false);

	/// This method is called in any case for displaying a laser scan.
	///  We keep an internal list of recent scans so they don't vanish
	///  instantaneously.
	/// \return true if the viz should be refreshed.
	bool update_opengl_viz(const mrpt::obs::CSensoryFrame& sf);
};

#endif
