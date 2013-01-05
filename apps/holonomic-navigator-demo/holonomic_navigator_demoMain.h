/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef HOLONOMIC_NAVIGATOR_DEMOMAIN_H
#define HOLONOMIC_NAVIGATOR_DEMOMAIN_H

//(*Headers(holonomic_navigator_demoFrame)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include "MyGLCanvas.h"
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
//*)

#include <mrpt/opengl.h>
#include <mrpt/reactivenav.h>
#include <mrpt/opengl/CPlanarLaserScan.h>			// It's in the lib mrpt-maps

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


class holonomic_navigator_demoFrame: public wxFrame
{
    public:

        holonomic_navigator_demoFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~holonomic_navigator_demoFrame();

    private:

        //(*Handlers(holonomic_navigator_demoFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnbtnPlaceRobotClick(wxCommandEvent& event);
        void OnbtnPlaceTargetClick(wxCommandEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnStopClick(wxCommandEvent& event);
        void OntimRunSimulTrigger(wxTimerEvent& event);
        void OnMenuItemChangeVisibleStuff(wxCommandEvent& event);
        void OnMenuItemClearRobotPath(wxCommandEvent& event);
        void OnbtnLoadMapClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(holonomic_navigator_demoFrame)
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_BUTTON6;
        static const long ID_BUTTON7;
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_RADIOBOX1;
        static const long ID_TEXTCTRL1;
        static const long ID_PANEL1;
        static const long ID_PANEL2;
        static const long ID_NOTEBOOK1;
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT1;
        static const long ID_XY_GLCANVAS;
        static const long ID_CUSTOM1;
        static const long ID_TEXTCTRL2;
        static const long ID_MENUITEM4;
        static const long idMenuQuit;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM3;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(holonomic_navigator_demoFrame)
        wxTimer timRunSimul;
        wxTextCtrl* edInfoLocalView;
        wxCustomButton* btnStop;
        wxNotebook* Notebook1;
        wxRadioBox* rbHoloMethod;
        wxMenuItem* MenuItem5;
        wxStaticText* StaticText2;
        wxCustomButton* btnStart;
        wxMenu* Menu3;
        wxTextCtrl* edHoloParams;
        wxCustomButton* btnLoadMap;
        wxCustomButton* btnQuit;
        wxPanel* Panel1;
        wxStaticText* StaticText1;
        wxMenuItem* MenuItem3;
        wxMenuItem* mnuViewRobotPath;
        wxStatusBar* StatusBar1;
        wxCustomButton* btnHelp;
        wxCustomButton* btnPlaceRobot;
        wxPanel* Panel2;
        wxMenuItem* mnuViewMaxRange;
        wxCustomButton* btnPlaceTarget;
        CMyGLCanvas* m_plotScan;
        CMyGLCanvas* m_plot3D;
        //*)

        DECLARE_EVENT_TABLE()

		/* Methods: */
		void updateMap3DView();
		void reinitSimulator();  // Create navigator object & load params from GUI
		void simulateOneStep(double time_step);
		void updateViewsDynamicObjects(); // Update 3D object positions and refresh views.

		void Onplot3DMouseClick(wxMouseEvent& event);
		void Onplot3DMouseMove(wxMouseEvent& event);

		/* Vars: */
		struct TOptions : public mrpt::utils::CLoadableOptions
		{
			double ROBOT_MAX_SPEED;
			double MAX_SENSOR_RADIUS;
			unsigned int SENSOR_NUM_RANGES;
			double SENSOR_RANGE_NOISE_STD;

			TOptions();
			virtual void saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const;
			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section);
		};

		TOptions m_simul_options;

		/**  The state of the cursor onto the 3D view:
          */
        enum TCursorPickState
        {
        	cpsNone = 0,
        	cpsPickTarget,
			cpsPlaceRobot
        };


		mrpt::reactivenav::CAbstractHolonomicReactiveMethod *m_holonomicMethod;
		mrpt::slam::COccupancyGridMap2D  m_gridMap;
		mrpt::math::TPoint2D             m_targetPoint;
		mrpt::math::TPose2D              m_robotPose;

		mrpt::utils::CTicTac             m_runtime;
		mrpt::math::TPoint2D             m_curCursorPos; //!< Of the cursor on the 3D view (in world coordinates at Z=0)
        TCursorPickState                 m_cursorPickState;   //!< The state of the cursor onto the 3D view:

		// ========= Opengl View: Map & robot  =======
		mrpt::opengl::CSetOfObjectsPtr		gl_grid;
		mrpt::opengl::CSetOfObjectsPtr		gl_robot, gl_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_nav_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_robot;
		mrpt::opengl::CDiskPtr		        gl_robot_sensor_range;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_path;
		mrpt::opengl::CPlanarLaserScanPtr   gl_scan3D, gl_scan2D;
		mrpt::opengl::CPointCloudPtr 		gl_path;

		// ========= Opengl View: Local view (holonomic)  =======
		mrpt::opengl::CSimpleLinePtr        gl_line_direction;
		mrpt::opengl::CPointCloudPtr        gl_rel_target;
		mrpt::opengl::CSetOfLinesPtr        gl_nd_gaps;

};

#endif // HOLONOMIC_NAVIGATOR_DEMOMAIN_H
