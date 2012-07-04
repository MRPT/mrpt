/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#ifndef HOLONOMIC_NAVIGATOR_DEMOMAIN_H
#define HOLONOMIC_NAVIGATOR_DEMOMAIN_H

//(*Headers(holonomic_navigator_demoFrame)
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include "MyGLCanvas.h"
#include <wx/textctrl.h>
#include <wx/radiobox.h>
#include <wx/timer.h>
#include <wx/things/toggle.h>
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
        static const long ID_XY_GLCANVAS;
        static const long ID_CUSTOM1;
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
        wxMenuItem* mnuViewMaxRange;
        wxPanel* Panel1;
        wxStatusBar* StatusBar1;
        wxTextCtrl* edHoloParams;
        wxMenu* Menu3;
        wxTimer timRunSimul;
        wxMenuItem* mnuViewRobotPath;
        wxPanel* Panel2;
        CMyGLCanvas* m_plot3D;
        wxCustomButton* btnLoadMap;
        wxCustomButton* btnPlaceRobot;
        wxMenuItem* MenuItem3;
        wxCustomButton* btnStart;
        wxRadioBox* rbHoloMethod;
        wxMenuItem* MenuItem5;
        wxCustomButton* btnStop;
        wxCustomButton* btnPlaceTarget;
        wxNotebook* Notebook1;
        CMyGLCanvas* m_plotScan;
        wxCustomButton* btnHelp;
        wxCustomButton* btnQuit;
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

};

#endif // HOLONOMIC_NAVIGATOR_DEMOMAIN_H
