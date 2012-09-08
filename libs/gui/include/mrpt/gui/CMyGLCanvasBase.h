/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef CMyGLCanvas_H
#define CMyGLCanvas_H

#include <mrpt/opengl.h>
#include <mrpt/opengl/opengl_fonts.h>

#include <mrpt/gui/link_pragmas.h>

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

namespace mrpt
{
	namespace gui
	{
		/** This class implements a OpenGL canvas, and it's used in gui::CDisplayWindow3D and a number of standalone applications in the MRPT project.
		  *  There is a filter to control the user capability of moving the camera with the mouse. See OnUserManuallyMovesCamera
		  * \ingroup mrpt_gui_grp
		  */
		class GUI_IMPEXP CMyGLCanvasBase: public wxGLCanvas
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

			void OnLeftDown(wxMouseEvent& event);
			void OnMouseMove(wxMouseEvent& event);
			void OnRightDown(wxMouseEvent& event);
			void OnRightUp(wxMouseEvent& event);
			void OnLeftUp(wxMouseEvent& event);
			void OnMouseWheel(wxMouseEvent& event);

			void Render();
			void InitGL();

			// Visualization params:
			float	cameraPointingX,cameraPointingY,cameraPointingZ;
			float	cameraZoomDistance;
			float	cameraElevationDeg,cameraAzimuthDeg;
			bool	cameraIsProjective;

			/** If set to true (default=false), the cameraPointingX,... parameters are ignored and the camera stored in the 3D scene is used instead.
			  */
			bool    useCameraFromScene;

			/** Set the camera from a CPose3D, which defines the +X,+Y axis as image place RIGHT and UP dirctions, and -Z as towards the pointing direction.
			  */
			void setCameraPose(const mrpt::poses::CPose3D &camPose);


			float	clearColorR,clearColorG,clearColorB;

			static float  SENSIBILITY_DEG_PER_PIXEL;		// Default = 0.1

			/**  Methods that can be implemented in custom derived classes  */
			virtual void OnCharCustom( wxKeyEvent& event ) { }

			virtual void OnPreRender() { }
			virtual void OnPostRender()  { }
			virtual void OnPostRenderSwapBuffers(double At, wxPaintDC &dc) { }
			virtual void OnRenderError( const wxString &str ) { }

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
			opengl::COpenGLScenePtr		m_openGLScene;

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

			/** DEPRECATED: Use CRenderizable static method instead */
			static void renderTextBitmap(
				int screen_x,
				int screen_y,
				const std::string &str,
				float  color_r=1,
				float  color_g=1,
				float  color_b=1,
				mrpt::opengl::TOpenGLFont    font = mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24
				)
			{
				mrpt::opengl::CRenderizable::renderTextBitmap(screen_x,screen_y,str,color_r,color_g,color_b, font);
			}

			/** DEPRECATED: Use CRenderizable static method instead */
			static int textBitmapWidth(
				const std::string &str,
				mrpt::opengl::TOpenGLFont    font = mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24 )
			{
				return mrpt::opengl::CRenderizable::textBitmapWidth(str,font);
			}

			// Used to create the gl context at startup.
			void OnWindowCreation(wxWindowCreateEvent &ev);

			DECLARE_EVENT_TABLE()

		};  // end of class

	}	// end namespace
}	// end namespace

#endif		// wxUSE_GLCANVAS
#endif 		// MRPT_HAS_WXWIDGETS
#endif 		// CMyGLCanvas_H

