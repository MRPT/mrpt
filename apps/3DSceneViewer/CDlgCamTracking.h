/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
