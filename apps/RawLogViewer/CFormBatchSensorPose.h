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
#ifndef CFORMBATCHSENSORPOSE_H
#define CFORMBATCHSENSORPOSE_H

//(*Headers(CFormBatchSensorPose)
#include <wx/bmpbuttn.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

class CFormBatchSensorPose: public wxDialog
{
	public:

		CFormBatchSensorPose(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~CFormBatchSensorPose();

		//(*Declarations(CFormBatchSensorPose)
		wxStaticText* StaticText1;
		wxButton* btnApply;
		wxBitmapButton* btnOpen;
		wxButton* btnCancel;
		wxTextCtrl* edText;
		//*)

	protected:

		//(*Identifiers(CFormBatchSensorPose)
		static const long ID_STATICTEXT1;
		static const long ID_TEXTCTRL1;
		static const long ID_BITMAPBUTTON1;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		//*)

	private:

		//(*Handlers(CFormBatchSensorPose)
		void OnbtnOpenClick(wxCommandEvent& event);
		void OnbtnApplyClick(wxCommandEvent& event);
		void OnbtnCancelClick(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
