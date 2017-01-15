/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef CDLGCAMTRACKING_H
#define CDLGCAMTRACKING_H

//(*Headers(CDlgCamTracking)
#include <wx/grid.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
//*)

#include <mrpt/poses/CPose3DInterpolator.h>


class _DSceneViewerFrame;

class CDlgCamTracking: public wxDialog
{
	public:

		CDlgCamTracking(_DSceneViewerFrame* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDlgCamTracking();

		//(*Declarations(CDlgCamTracking)
		wxMenuItem* MenuItem1;
		wxGrid* gridPoses;
		wxButton* btnClose;
		wxButton* btnStop;
		wxButton* btnLoad;
		wxMenu menuGrid;
		wxButton* btnGrab;
		wxTextCtrl* edVel;
		wxButton* btnStart;
		wxButton* btnSave;
		wxCheckBox* cbConstVel;
		//*)


		// The camera path:
		mrpt::poses::CPose3DInterpolator  m_poses;
		void UpdateTableFromPoses();



	protected:

		_DSceneViewerFrame *m_main_win;

		//(*Identifiers(CDlgCamTracking)
		static const long ID_BUTTON2;
		static const long ID_BUTTON3;
		static const long ID_BUTTON4;
		static const long ID_CHECKBOX1;
		static const long ID_TEXTCTRL1;
		static const long ID_BUTTON6;
		static const long ID_BUTTON5;
		static const long ID_GRID1;
		static const long ID_BUTTON1;
		static const long ID_MENUITEM1;
		//*)

	private:

		//(*Handlers(CDlgCamTracking)
		void OnbtnCloseClick(wxCommandEvent& event);
		void OnMenuItemDelete(wxCommandEvent& event);
		void OnbtnSaveClick(wxCommandEvent& event);
		void OnbtnLoadClick(wxCommandEvent& event);
		void OnbtnGrabClick(wxCommandEvent& event);
		void OnbtnStartClick(wxCommandEvent& event);
		void OnbtnStopClick(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
