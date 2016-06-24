/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

//(*Headers(ptgConfiguratorframe)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include "MyGLCanvas.h"
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/combobox.h>
#include <wx/statusbr.h>
//*)

#include "../wx-common/CMyRedirector.h"

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#	undef Button1
#	undef Button2
#	undef Button3
#	undef Button4
#	undef Button5
#	undef Button6
#	undef Button7
#endif
// To avoid conflicts between Eigen & X11 headers
#ifdef Success 
#	undef Success 
#endif

class ptgConfiguratorframe: public wxFrame
{
    public:

        ptgConfiguratorframe(wxWindow* parent,wxWindowID id = -1);
        virtual ~ptgConfiguratorframe();

    private:

        //(*Handlers(ptgConfiguratorframe)
        void OnAbout(wxCommandEvent& event);
        void OnQuit(wxCommandEvent& event);
        void OnbtnReloadParamsClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(ptgConfiguratorframe)
        static const long ID_STATICTEXT1;
        static const long ID_COMBOBOX1;
        static const long ID_STATICTEXT2;
        static const long ID_SPINCTRL1;
        static const long ID_TEXTCTRL1;
        static const long ID_BUTTON1;
        static const long ID_PANEL1;
        static const long ID_XY_GLCANVAS;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(ptgConfiguratorframe)
        wxStaticText* StaticText2;
        wxSpinCtrl* edPTGIndex;
        wxPanel* Panel1;
        wxStaticText* StaticText1;
        CMyGLCanvas* m_plot;
        wxStatusBar* StatusBar1;
        wxButton* btnReloadParams;
        wxTextCtrl* edCfg;
        wxComboBox* cbPTGClass;
        //*)

        DECLARE_EVENT_TABLE()

		/* Methods: */
		CMyRedirector *m_myRedirector;

		// ========= Opengl View =======
		mrpt::opengl::CSetOfObjectsPtr		gl_grid;
		mrpt::opengl::CSetOfObjectsPtr		gl_robot,gl_robot_local, gl_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_nav_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_robot;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_path;
		mrpt::opengl::CPointCloudPtr 		gl_path;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_ptg_prediction;

};

