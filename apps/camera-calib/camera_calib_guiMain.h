/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#ifndef CAMERA_CALIB_GUIMAIN_H
#define CAMERA_CALIB_GUIMAIN_H

//(*Headers(camera_calib_guiDialog)
#include <wx/scrolwin.h>
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/listbox.h>
#include <wx/spinctrl.h>
#include "MyGLCanvas.h"
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <mrpt/gui/WxUtils.h>
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
        static const long ID_RADIOBOX1;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT6;
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
        mrpt::gui::wxMRPTImageControl* bmpOriginal;
        wxCheckBox* cbNormalize;
        wxTextCtrl* edLengthX;
        CMyGLCanvas* m_3Dview;
        wxNotebook* Notebook1;
        wxRadioBox* rbMethod;
        wxButton* btnSave;
        wxButton* btnAbout;
        wxStaticText* StaticText2;
        wxButton* btnClose;
        wxButton* btnRunCalib;
        wxScrolledWindow* ScrolledWindow3;
        wxStaticText* StaticText6;
        wxTextCtrl* edLengthY;
        wxSpinCtrl* edSizeY;
        wxChoice* cbZoom;
        wxPanel* Panel1;
        wxButton* btnCaptureNow;
        wxStaticText* StaticText1;
        wxTextCtrl* txtLog;
        wxStaticText* StaticText3;
        wxButton* Button22;
        wxButton* btnSaveImages;
        wxPanel* Panel3;
        wxButton* Button11;
        wxStaticText* StaticText5;
        wxScrolledWindow* ScrolledWindow2;
        wxFlexGridSizer* FlexGridSizer14;
        wxSpinCtrl* edSizeX;
        mrpt::gui::wxMRPTImageControl* bmpRectified;
        wxPanel* Panel2;
        wxListBox* lbFiles;
        wxFlexGridSizer* FlexGridSizer11;
        wxButton* btnManualRect;
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
