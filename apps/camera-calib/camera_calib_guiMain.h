/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef CAMERA_CALIB_GUIMAIN_H
#define CAMERA_CALIB_GUIMAIN_H

//(*Headers(camera_calib_guiDialog)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/scrolwin.h>
#include "MyGLCanvas.h"
#include <mrpt/gui/WxUtils.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/listbox.h>
//*)

#include <mrpt/gui/CDisplayWindow3D.h>

#define CAMERA_CALIB_GUI_VERSION  "2.0"

class camera_calib_guiDialog: public wxDialog
{
    public:

        camera_calib_guiDialog(wxWindow* parent,wxWindowID id = -1);
        virtual ~camera_calib_guiDialog();

    private:

        //(*Handlers(camera_calib_guiDialog)
        void OnAddImage(wxCommandEvent& event);
        void OnListClear(wxCommandEvent& event);
        void OnbtnRunCalibClick(wxCommandEvent& event);
        void OnbtnCloseClick(wxCommandEvent& event);
        void OnbtnAboutClick(wxCommandEvent& event);
        void OnbtnSaveClick(wxCommandEvent& event);
        void OnlbFilesSelect(wxCommandEvent& event);
        void OnbtnManualRectClick(wxCommandEvent& event);
        void OnbtnCaptureNowClick(wxCommandEvent& event);
        void OnbtnSaveImagesClick(wxCommandEvent& event);
        void OncbZoomSelect(wxCommandEvent& event);
        //*)

        //(*Identifiers(camera_calib_guiDialog)
        static const long ID_BUTTON8;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_BUTTON9;
        static const long ID_LISTBOX1;
        static const long ID_STATICTEXT5;
        static const long ID_CHOICE1;
        static const long ID_STATICTEXT1;
        static const long ID_SPINCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_SPINCTRL2;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL3;
        static const long ID_CHECKBOX1;
        static const long ID_TEXTCTRL2;
        static const long ID_BUTTON3;
        static const long ID_BUTTON6;
        static const long ID_BUTTON7;
        static const long ID_BUTTON5;
        static const long ID_BUTTON4;
        static const long ID_CUSTOM2;
        static const long ID_SCROLLEDWINDOW2;
        static const long ID_PANEL2;
        static const long ID_CUSTOM1;
        static const long ID_SCROLLEDWINDOW3;
        static const long ID_PANEL3;
        static const long ID_XY_GLCANVAS;
        static const long ID_PANEL1;
        static const long ID_NOTEBOOK1;
        //*)

        //(*Declarations(camera_calib_guiDialog)
        wxTextCtrl* edLengthY;
        wxPanel* Panel1;
        wxButton* btnManualRect;
        wxButton* btnClose;
        wxCheckBox* cbNormalize;
        wxFlexGridSizer* FlexGridSizer11;
        wxButton* btnAbout;
        CMyGLCanvas* m_3Dview;
        wxButton* btnRunCalib;
        mrpt::gui::wxMRPTImageControl* bmpOriginal;
        wxSpinCtrl* edSizeY;
        wxStaticText* StaticText1;
        wxButton* btnSaveImages;
        wxFlexGridSizer* FlexGridSizer14;
        mrpt::gui::wxMRPTImageControl* bmpRectified;
        wxPanel* Panel2;
        wxStaticText* StaticText3;
        wxSpinCtrl* edSizeX;
        wxButton* btnCaptureNow;
        wxPanel* Panel3;
        wxListBox* lbFiles;
        wxTextCtrl* txtLog;
        wxScrolledWindow* ScrolledWindow3;
        wxScrolledWindow* ScrolledWindow2;
        wxStaticText* StaticText4;
        wxChoice* cbZoom;
        wxStaticText* StaticText5;
        wxStaticText* StaticText2;
        wxNotebook* Notebook1;
        wxTextCtrl* edLengthX;
        wxButton* Button22;
        wxButton* Button11;
        wxButton* btnSave;
        //*)

        DECLARE_EVENT_TABLE()

		void updateListOfImages();

		// Shows the image selected in the listbox:
		void refreshDisplayedImage();

		// Shows a 3D view of the cams.
		void show3Dview();

		void clearListImages();

};


#endif // CAMERA_CALIB_GUIMAIN_H
