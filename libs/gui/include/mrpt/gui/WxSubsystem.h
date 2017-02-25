/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_WX_SUBSYSTEM_H
#define  MRPT_WX_SUBSYSTEM_H

#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/threads.h>
#include <mrpt/config.h>
#include <mrpt/synch/CSemaphore.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/gui/gui_frwds.h>

#include <mrpt/gui/link_pragmas.h>

#include <queue>
#include <map>

#if MRPT_HAS_WXWIDGETS

#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/menu.h>
#include <wx/toolbar.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
#include <wx/msgdlg.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
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
#include <wx/app.h>
#include <wx/pen.h>

// The wxMathPlot library
#include <mrpt/otherlibs/mathplot/mathplot.h>

#if 0
// The wxFreeChart library
#include <wx/chartpanel.h>
#include <wx/bars/barplot.h>

#include <wx/axis/numberaxis.h>
#include <wx/axis/categoryaxis.h>
#include <wx/axis/dateaxis.h>

#include <wx/xy/xyhistorenderer.h>
#include <wx/xy/xydataset.h>
#include <wx/xy/xylinerenderer.h>
#include <wx/xy/xyplot.h>
#include <wx/xy/xysimpledataset.h>

#include <wx/xyz/xyzdataset.h>
#include <wx/xyz/bubbleplot.h>

#include <wx/category/categorydataset.h>
#include <wx/category/categorysimpledataset.h>
#endif

#endif
#include <mrpt/gui/gui_frwds.h>

namespace mrpt
{
	namespace gui
	{
		/** This class implements the GUI thread required for the wxWidgets-based GUI.
		  *  This system is employed internally by gui::CDisplayWindow and gui::CDisplayWindow3D, and must be not used in any way directly by the MRPT user.
		  *
		  *  The system works by creating a invisible wxFrame that process timer events where it checks a queue of requests sent from the main MRPT thread. The
		  *   requests include the creation, deletion,... of windows (2D/3D). In that way, just one thread is required for all the GUI windows, and the wxWidgets
		  *   is initialized and clean-up correctly.
		  *
		  *  This header should be included just from the implementation files of CDisplayWindow and CDisplayWindow3D, since it uses wxWidgets classes.
		  *
		  *  \sa gui::CDisplayWindow, gui::CDisplayWindow3D
		  * \ingroup mrpt_gui_grp
		  */
		class GUI_IMPEXP WxSubsystem
		{
	#if MRPT_HAS_WXWIDGETS

		public:

			/** This method must be called in the destructor of the user class FROM THE MAIN THREAD, in order to wait for the shutdown of the wx thread if this was the last open window.
			  */
			static void waitWxShutdownsIfNoWindows();

			/** Will be set to true at runtime if it's not detected a running wxApp instance.
			  *  For console apps, we'll create a new thread and run wxEntry from there.
			  *  For GUI apps (MRPT-based Windows are a part of a user wxWidget apps), we must leave the control of
			  *   message dispatching to the current main loop, so we cannot create a different threads, making things a little different (hence this variable).
			  */
			static volatile bool isConsoleApp;

			/** An auxiliary global object used just to launch a final request to the wxSubsystem for shutdown:
			  */
			class CAuxWxSubsystemShutdowner
			{
			public:
				CAuxWxSubsystemShutdowner();
				~CAuxWxSubsystemShutdowner();
			};

			static CAuxWxSubsystemShutdowner  global_wxsubsystem_shutdown;


			/** The main frame of the wxWidgets application
			  */
			class CWXMainFrame: public wxFrame
			{
        		friend  void WxSubsystem::waitWxShutdownsIfNoWindows();

				public:
					CWXMainFrame(wxWindow* parent,wxWindowID id = -1);
					virtual ~CWXMainFrame();

					/** Atomically increments the number of windows created with the main frame as parent.
					  * \return The updated number of windows.
					  */
					static int notifyWindowCreation();

					/** Atomically decrements the number of windows created with the main frame as parent.
					  * \return The updated number of windows (0 if the calling was the last one).
					  */
					static int notifyWindowDestruction();

					static volatile CWXMainFrame* oneInstance;


				private:

					static synch::CCriticalSection     cs_windowCount;
					static int                         m_windowCount;

					wxTimer                         *m_theTimer;

					void OnTimerProcessRequests(wxTimerEvent& event);

					DECLARE_EVENT_TABLE()

			}; // end class CWXMainFrame

			struct TWxMainThreadData
			{
				TWxMainThreadData();
				mrpt::system::TThreadHandle  m_wxMainThreadId; //!< The thread ID of wxMainThread, or 0 if it is not running.
				mrpt::synch::CSemaphore m_semWxMainThreadReady; //!< This is signaled when wxMainThread is ready.
				mrpt::synch::CCriticalSection m_csWxMainThreadId; //!< The critical section for accessing "m_wxMainThreadId"
			};

			static TWxMainThreadData& GetWxMainThreadInstance();


			/**  This will be the "MAIN" of wxWidgets: It starts an application object and does not end until all the windows are closed.
			  *   Only one instance of this thread can be running at a given instant, no matter how many windows are open.
			  */
			static void	wxMainThread();

			/** The data structure for each inter-thread request:
			  */
			struct GUI_IMPEXP TRequestToWxMainThread
			{
				TRequestToWxMainThread() :
					source2D		( NULL ),
					source3D		( NULL ),
					sourcePlots		( NULL ),
					sourceCameraSelectDialog(false),
					voidPtr			(NULL),
					voidPtr2		(NULL),
					x				(400),
					y				(400),
					boolVal			(false)
				{ }

				/** Only one of source* can be non-NULL, indicating the class that generated the request. */
				mrpt::gui::CDisplayWindow    *source2D;

				/** Only one of source* can be non-NULL, indicating the class that generated the request. */
				mrpt::gui::CDisplayWindow3D  *source3D;

				/** Only one of source* can be non-NULL, indicating the class that generated the request. */
				mrpt::gui::CDisplayWindowPlots *sourcePlots;

				/** Only one of source* can be non-NULL, indicating the class that generated the request. */
				bool sourceCameraSelectDialog;

				/** Parameters, depending on OPCODE.
				  */
				std::string  str;

				/** Parameters, depending on OPCODE.
				  */
				void         *voidPtr, *voidPtr2;
				int          x,y;
				bool         boolVal;
				mrpt::math::CVectorFloat vector_x,vector_y;
				std::string  plotName;

				/** Valid codes are:
				  *  For CDisplayWindow:
				  *     - 200: Create a new 2D window, with caption "str" and initial size "x" & "y", and save the "wxFrame*" in the "void**" passed in voidPtr.
				  *     - 201: Updates the image shown in the window, from a "wxImage*" passed in voidPtr2. The wxImage object will be freed with delete after that. voidPtr must be a "wxFrame*", a "CWindowDialog*" actually.
				  *     - 202: Set position to x,y
				  *     - 203: Change size to x,y
				  *     - 204: Change title to "str"
				  *     - 299: Delete the window associated with this source object.
				  *
				  *  For CDisplayWindow3D:
				  *     - 300: Create a new 3D window, with caption "str" and initial size "x" & "y", and save the "wxFrame*" in the "void**" passed in voidPtr.
				  *     - 302: Set position to x,y
				  *     - 303: Change size to x,y
				  *     - 304: Change title to "str"
				  *		- 350: Force refresh
				  *		- 360: Add a 2D text message: vector_x: [0]:x, [1]:y, [2,3,4]:R G B, "x": enum of desired font. "y": unique index, "str": String.
				  *		- 361: Clear all 2D text messages.
				  *		- 362: Add a 2D text message (vectorized fonts)
				  *		- 370: Change min/max range: min=vector_x[0], max=vector_x[1]
				  *     - 399: Delete the window associated with this source object.
				  *
				  *  For CDisplayWindowPlots:
				  *     - 400: Create a new Plots window, with caption "str" and initial size "x" & "y",and save the "wxFrame*" in the "void**" passed in voidPtr.
				  *     - 402: Set position to x,y
				  *     - 403: Change size to x,y
				  *     - 404: Change title to "str"
				  *     - 499: Delete the window associated with this source object.
				  *		- 410: Depending on "boolVal", enable/disable the mouse-zoom & pan
				  *		- 411: Depending on "boolVal", enable/disable the aspect ratio fix
				  *		- 412: Zoom over a rectangle vectorx[0-1] & vectory[0-1]
				  *		- 413: Axis fit, with aspect ratio fix to boolVal.
				  *		- 414: Clear all plot objects.
				  *		- 420: Add/update a 2D line/points plot: x/y data= vector_x/vector_y, format string=str, plot name =plotName.
				  *		- 421: Add/update a 2D ellipse: format string=str, plot name =plotName, vector_x[0,1]:X/Y center, vector_x[2]:quantiles, vector_y[0,1,2]: Covariance matrix entries 00,11,01,  boolVal=showName?
				  *		- 422: Add/update a bitmap: plot name =plotName, vector_x[0,1]:X/Y corner, vector_x[2,3]: X/Y widths, voidPtr2: pointer to a newly created wxImage with the bitmap.
				  *		- 440: Insert submenu in the popup menu. plotName=menu label, x=user-defined ID.
				  *		- 700: Shows a camera-pick-dialog and wait for user selection. "voidPtr" must point to a CSemaphore, which will be signaled twice (1st upon construction, 2nd upon dialog close); voidPtr2 must point to a "mrpt::gui::CPanelCameraSelection*" which will be filled with the selection (the panel must be deleted by the caller)
				  *
				  */
				int  OPCODE;

			};

			/** Thread-safe method to return the next pending request, or NULL if there is none (After usage, FREE the memory!)
			  */
			static TRequestToWxMainThread  * popPendingWxRequest();

			/** Thread-safe method to insert a new pending request (The memory must be dinamically allocated with "new T[1]", will be freed by receiver.)
			  */
			static void pushPendingWxRequest( TRequestToWxMainThread *data );

			/** Thread-safe method to create one single instance of the main wxWidgets thread: it will create the thread only if it is not running yet.
			  */
			static bool createOneInstanceMainThread();


			static wxBitmap getMRPTDefaultIcon();
		private:
			/** Do not access directly to this, use the thread-safe functions
			  */
			static std::queue<TRequestToWxMainThread*>  *listPendingWxRequests;
			static synch::CCriticalSection              *cs_listPendingWxRequests;
	#endif
		}; // End of class def.


	#if MRPT_HAS_WXWIDGETS

		/** The wx dialog for gui::CDisplayWindow
		  */
		class CWindowDialog: public wxFrame
		{
		public:
			/** A custom control to display the bitmap and avoid flicker
			  */
			class wxMRPTImageControl : public wxPanel
			{
			protected:
				wxBitmap *m_img;
				mrpt::synch::CCriticalSection	m_img_cs;
				CDisplayWindow *m_win2D;

			public:
				wxMRPTImageControl(	wxWindow *parent,wxWindowID winID,int x, int y, int width, int height);
				virtual ~wxMRPTImageControl();

				wxPoint m_last_mouse_point, m_last_mouse_click;
				//mrpt::synch::CCriticalSection	m_mouse_cs;

				void AssignImage(wxBitmap *img); //!< Assigns this image. This object has the ownship of the image and will delete it when appropriate.
				void GetBitmap(wxBitmap &bmp);

				void OnPaint(wxPaintEvent &ev);
				void OnMouseMove(wxMouseEvent& ev);
				void OnMouseClick(wxMouseEvent& ev);
				void OnChar(wxKeyEvent& ev);

				void OnEraseBackground(wxEraseEvent &ev) { /* Do nothing */ }
			};



			public:
				CWindowDialog( CDisplayWindow *win2D, WxSubsystem::CWXMainFrame* parent,wxWindowID id = -1, const std::string &caption = std::string("[MRPT-CDisplayWindow]"), wxSize initialSize = wxDefaultSize );
				virtual ~CWindowDialog();

				CDisplayWindow *m_win2D;
				WxSubsystem::CWXMainFrame   *m_mainFrame;

				//wxStaticBitmap      *m_image;
				wxMRPTImageControl    *m_image;

				static const long         ID_IMAGE_BITMAP;

			private:

				void OnClose (wxCloseEvent& event);
				void OnMenuClose(wxCommandEvent& event);
				void OnMenuAbout(wxCommandEvent& event);
				void OnMenuSave(wxCommandEvent& event);
				void OnChar(wxKeyEvent& event);
				void OnKeyDown(wxKeyEvent& event);
				void OnResize(wxSizeEvent& event);
				void OnMouseDown(wxMouseEvent& event);

				DECLARE_EVENT_TABLE()
		}; // end class CWindowDialog

		class C3DWindowDialog: public wxFrame
		{
			friend class gui::CMyGLCanvas_DisplayWindow3D;

			public:

				C3DWindowDialog(CDisplayWindow3D *win3D, WxSubsystem::CWXMainFrame* parent,wxWindowID id = -1, const std::string &caption = std::string("[MRPT-CDisplayWindow3D]"), wxSize initialSize = wxDefaultSize );
				virtual ~C3DWindowDialog();

				CDisplayWindow3D *m_win3D;
				WxSubsystem::CWXMainFrame   *m_mainFrame;

				CMyGLCanvas_DisplayWindow3D	*m_canvas;

				void clearTextMessages();
				void addTextMessage(
					const double x_frac,
					const double y_frac,
					const std::string &text,
					const mrpt::utils::TColorf &color,
					const size_t unique_index,
					const mrpt::opengl::TOpenGLFont font
					);
				void addTextMessage(
					const double x_frac,
					const double y_frac,
					const std::string &text,
					const mrpt::utils::TColorf &color,
					const std::string  &font_name,
					const double  font_size,
					const mrpt::opengl::TOpenGLFontStyle font_style,
					const size_t  unique_index,
					const double  font_spacing,
					const double  font_kerning,
					const bool has_shadow,
					const mrpt::utils::TColorf &shadow_color
					);

			private:

				void OnClose (wxCloseEvent& event);
				void OnMenuClose(wxCommandEvent& event);
				void OnMenuAbout(wxCommandEvent& event);
				void OnChar(wxKeyEvent& event);
				void OnResize(wxSizeEvent& event);

				static const long ID_MENUITEM1;
				static const long ID_MENUITEM2;

				DECLARE_EVENT_TABLE()
		};

		/** The wx dialog for gui::CDisplayWindowPlots
		  */
		class CWindowDialogPlots: public wxFrame
		{
			public:
				CWindowDialogPlots( CDisplayWindowPlots *winPlots, WxSubsystem::CWXMainFrame* parent,wxWindowID id = -1, const std::string &caption = std::string("[MRPT-CDisplayWindowPlots]"), wxSize initialSize = wxDefaultSize );
				virtual ~CWindowDialogPlots();

				CDisplayWindowPlots 		*m_winPlots;
				WxSubsystem::CWXMainFrame   *m_mainFrame;

				mpWindow					*m_plot;
				// wxChartPanel 			*m_chartPanel;
				static const long           ID_PLOT;
				static const long           ID_MENU_PRINT;
				bool                        m_firstSubmenu; //!< to know whether to insert a separator the first time.
				std::map<long,long>			m_ID2ID; //!< wxIDs to user IDs for submenus.
				mrpt::math::TPoint2D  		m_curCursorPos;  //!< In graph coords
				wxPoint 					m_last_mouse_point; //!< In pixels

				void OnMenuSelected(wxCommandEvent& ev);
				void OnMouseMove(wxMouseEvent& event);


				/** Redirected from CDisplayWindowPlots::plot
				  */
				void plot(
					const mrpt::math::CVectorFloat &x,
					const mrpt::math::CVectorFloat &y,
					const std::string  &lineFormat,
					const std::string  &plotName);

				/** Redirected from CDisplayWindowPlots::plotEllipse
				  */
				void plotEllipse(
					const mrpt::math::CVectorFloat &x,
					const mrpt::math::CVectorFloat &y,
					const std::string  &lineFormat,
					const std::string  &plotName,
					bool showName = false);

				/** Redirected from CDisplayWindowPlots::image
				  */
				void image(
					void *theWxImage,
					const float &x0,
					const float &y0,
					const float &w,
					const float &h,
					const std::string &plotName);

			private:

				void OnClose (wxCloseEvent& event);
				void OnMenuPrint(wxCommandEvent& event);
				void OnMenuClose(wxCommandEvent& event);
				void OnMenuAbout(wxCommandEvent& event);
				void OnChar(wxKeyEvent& event);
				void OnResize(wxSizeEvent& event);
				void OnMouseDown(wxMouseEvent& event);

				DECLARE_EVENT_TABLE()
		}; // end class CWindowDialog

		#ifndef _U
			#ifdef wxUSE_UNICODE
			#define _U(x) wxString((x),wxConvUTF8)
			#define _UU(x,y) wxString((x),y)
			#else
			#define _U(x) (x)
			#define _UU(x,y) (x)
			#endif
		#endif

	#endif

	} // End of namespace
} // End of namespace

#endif
