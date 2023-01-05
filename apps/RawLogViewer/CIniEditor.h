/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef CINIEDITOR_H
#define CINIEDITOR_H

//(*Headers(CIniEditor)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
//*)

class CIniEditor : public wxDialog
{
   public:
	CIniEditor(
		wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize);
	~CIniEditor() override;

	//(*Declarations(CIniEditor)
	wxButton* btnCancel;
	wxTextCtrl* edText;
	wxButton* btnOK;
	//*)

   protected:
	//(*Identifiers(CIniEditor)
	static const long ID_BUTTON1;
	static const long ID_BUTTON2;
	static const long ID_TEXTCTRL1;
	//*)

   private:
	//(*Handlers(CIniEditor)
	void OnbtnCancelClick(wxCommandEvent& event);
	void OnbtnOKClick(wxCommandEvent& event);
	//*)

	DECLARE_EVENT_TABLE()
};

#endif
