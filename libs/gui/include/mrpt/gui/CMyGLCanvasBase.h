/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CMyGLCanvas_H
#define CMyGLCanvas_H

#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/opengl/CTextMessageCapable.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/link_pragmas.h>

namespace mrpt { namespace gui { } }  // At least declare the existence of the namespace mrpt::gui even if we don't have wxWidgets libs


#if MRPT_HAS_WXWIDGETS

#include <wx/string.h>
#include <wx/intl.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>

#if wxUSE_GLCANVAS && MRPT_HAS_OPENGL_GLUT

#include <wx/glcanvas.h>
#include <wx/dcclient.h>

// To avoid conflicts between Eigen & X11 headers
#ifdef Success 
#	undef Success 
#endif


namespace mrpt
{
	namespace gui
	{
		/** This class implements a OpenGL canvas, and it's used in gui::CDisplayWindow3D and a number of standalone applications in the MRPT project.
		  *  There is a filter to control the user capability of moving the camera with the mouse. See OnUserManuallyMovesCamera
		  * \ingroup mrpt_gui_grp
		  */
		class GUI_IMPEXP CMyGLCanvasBase :
			public wxGLCanvas, 
			public mrpt::opengl::CTextMessageCapable
		{
		public:
			CMyGLCanvasBase( wxWindow *parent, wxWindowID id = wxID_ANY,
						 const wxPoint& pos = wxDefaultPosition,
						 const wxSize& size = wxDefaultSize,
						 long style = 0, const wxString& name = _T("CMyGLCanvasBase") );

			virtual ~CMyGLCanvasBase();

			void OnPaint(wxPaintEvent& event);
			void OnSize(wxSizeEvent& event);
			void OnEraseBackground(wxEraseEvent& event);
			void OnEnterWindow(wxMouseEvent& event);

			void OnChar(wxKeyEvent& event);

			void OnMouseDown(wxMouseEvent& event);
			void OnMouseMove(wxMouseEvent& event);
			void OnMouseUp(wxMouseEvent& event);
			void OnMouseWheel(wxMouseEvent& event);

			void Render();
			void InitGL();

			// Visualization params:
			float	cameraPointingX,cameraPointingY,cameraPointingZ;
			float	cameraZoomDistance;
			float	cameraElevationDeg,cameraAzimuthDeg;
			bool	cameraIsProjective;
			float   cameraFOV;

			/** If set to true (default=false), the cameraPointingX,... parameters are ignored and the camera stored in the 3D scene is used instead.
			  */
			bool    useCameraFromScene;

			/** Set the camera from a CPose3D, which defines the +X,+Y axis as image place RIGHT and UP dirctions, and -Z as towards the pointing direction.
			  */
			void setCameraPose(const mrpt::poses::CPose3D &camPose);


			float	clearColorR,clearColorG,clearColorB;

			static float  SENSIBILITY_DEG_PER_PIXEL;		// Default = 0.1

			/**  Methods that can be implemented in custom derived classes  */
			virtual void OnCharCustom( wxKeyEvent& event ) {
            MRPT_UNUSED_PARAM(event);
         }

			virtual void OnPreRender() { }
			virtual void OnPostRender()  { }
			virtual void OnPostRenderSwapBuffers(double At, wxPaintDC &dc) {
            MRPT_UNUSED_PARAM(At); MRPT_UNUSED_PARAM(dc);
         }

			virtual void OnRenderError( const wxString &str ) {
            MRPT_UNUSED_PARAM(str);
         }

			/** Overload this method to limit the capabilities of the user to move the camera using the mouse.
			  *  For all these variables:
			  *  - cameraPointingX
			  *  - cameraPointingY
			  *  - cameraPointingZ
			  *  - cameraZoomDistance
			  *  - cameraElevationDeg
			  *  - cameraAzimuthDeg
			  *
			  *  A "new_NAME" variable will be passed with the temptative new value after the user action.
			  *   The default behavior should be to copy all the new variables to the variables listed above
			  *   but in the middle any find of user-defined filter can be implemented.
			  */
			virtual void OnUserManuallyMovesCamera(
				float	new_cameraPointingX,
				float 	new_cameraPointingY,
				float 	new_cameraPointingZ,
				float	new_cameraZoomDistance,
				float	new_cameraElevationDeg,
				float	new_cameraAzimuthDeg )
			{
				cameraPointingX 	= new_cameraPointingX;
				cameraPointingY 	= new_cameraPointingY;
				cameraPointingZ 	= new_cameraPointingZ;
				cameraZoomDistance 	= new_cameraZoomDistance;
				cameraElevationDeg 	= new_cameraElevationDeg ;
				cameraAzimuthDeg 	= new_cameraAzimuthDeg;
			}

			inline void getLastMousePosition(int &x,int& y) const {
				x =m_mouseLastX;
				y =m_mouseLastY;
			}

			/**  At constructor an empty scene is created. The object is freed at GL canvas destructor.
			  */
			mrpt::opengl::COpenGLScenePtr		m_openGLScene;

		protected:
			wxGLContext *m_gl_context;
			bool   m_init;

			int 	m_mouseLastX,m_mouseLastY;

			int 	mouseClickX,mouseClickY;
			bool 	mouseClicked;

			long           m_Key;
			unsigned long  m_StartTime;
			unsigned long  m_LastTime;
			unsigned long  m_LastRedraw;

			// Used to create the gl context at startup.
			void OnWindowCreation(wxWindowCreateEvent &ev);

			DECLARE_EVENT_TABLE()

		};  // end of class

	}	// end namespace
}	// end namespace

#endif		// wxUSE_GLCANVAS
#endif 		// MRPT_HAS_WXWIDGETS
#endif 		// CMyGLCanvas_H

