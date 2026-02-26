/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/gui/config.h>
#include <mrpt/gui/gui_frwds.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/viz/opengl_fonts.h>

#include <future>
#include <map>
#include <mutex>
#include <thread>

#if MRPT_HAS_WXWIDGETS

#include <wx/app.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/busyinfo.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/frame.h>
#include <wx/image.h>
#include <wx/imaglist.h>
#include <wx/intl.h>
#include <wx/log.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/pen.h>
#include <wx/progdlg.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/statusbr.h>
#include <wx/string.h>
#include <wx/textdlg.h>
#include <wx/timer.h>
#include <wx/toolbar.h>

// The wxMathPlot library
#include <mathplot/mathplot.h>

#endif

namespace mrpt::gui
{
/** This class implements the GUI thread required for the wxWidgets-based GUI.
 *  This system is employed internally by gui::CDisplayWindow and
 * gui::CDisplayWindow3D, and must be not used in any way directly by the MRPT
 * user.
 *
 *  The system works by creating a invisible wxFrame that process timer events
 * where it checks a queue of requests sent from the main MRPT thread. The
 *   requests include the creation, deletion,... of windows (2D/3D). In that
 * way, just one thread is required for all the GUI windows, and the wxWidgets
 *   is initialized and clean-up correctly.
 *
 *  This header should be included just from the implementation files of
 * CDisplayWindow and CDisplayWindow3D, since it uses wxWidgets classes.
 *
 *  \sa gui::CDisplayWindow, gui::CDisplayWindow3D
 * \ingroup mrpt_gui_grp
 */
class WxSubsystem
{
#if MRPT_HAS_WXWIDGETS

 public:
  /** Named opcodes for inter-thread requests */
  enum class OpCode : std::uint16_t
  {
    // ---- CDisplayWindow (2D) ----
    /** Create a new 2D window. caption=str, size=(x,y).
     *  voidPtr must point to a void* that will receive the new wxFrame*. */
    WIN2D_CREATE = 200,
    /** Update the displayed image. voidPtr=CWindowDialog*, voidPtr2=wxImage* (ownership
       transferred). */
    WIN2D_UPDATE_IMAGE = 201,
    /** Set window position to (x,y). */
    WIN2D_SET_POS = 202,
    /** Resize window to (x,y). */
    WIN2D_SET_SIZE = 203,
    /** Change window title to str. */
    WIN2D_SET_TITLE = 204,
    /** Destroy 2D window. */
    WIN2D_DESTROY = 299,

    // ---- CDisplayWindow3D ----
    /** Create a new 3D window. caption=str, size=(x,y).
     *  voidPtr must point to a void* that will receive the new wxFrame*. */
    WIN3D_CREATE = 300,
    /** Set window position to (x,y). */
    WIN3D_SET_POS = 302,
    /** Resize window to (x,y). */
    WIN3D_SET_SIZE = 303,
    /** Change window title to str. */
    WIN3D_SET_TITLE = 304,
    /** Force a repaint of the 3D window. */
    WIN3D_FORCE_REPAINT = 350,
    /** Destroy 3D window. */
    WIN3D_DESTROY = 399,

    // ---- CDisplayWindowPlots ----
    /** Create a new Plots window. caption=str, size=(x,y).
     *  voidPtr must point to a void* that will receive the new wxFrame*. */
    PLOTS_CREATE = 400,
    /** Set window position to (x,y). */
    PLOTS_SET_POS = 402,
    /** Resize window to (x,y). */
    PLOTS_SET_SIZE = 403,
    /** Change window title to str. */
    PLOTS_SET_TITLE = 404,
    /** Destroy Plots window. */
    PLOTS_DESTROY = 499,
    /** Enable/disable mouse pan+zoom according to boolVal. */
    PLOTS_SET_MOUSE_PANZOOM = 410,
    /** Enable/disable aspect ratio lock according to boolVal. */
    PLOTS_SET_ASPECT_RATIO = 411,
    /** Zoom to rect: vector_x[0..1] = x range, vector_y[0..1] = y range. */
    PLOTS_ZOOM_RECT = 412,
    /** Fit axes to data; boolVal controls aspect-ratio lock. */
    PLOTS_AXIS_FIT = 413,
    /** Clear all plot layer objects. */
    PLOTS_CLEAR = 414,
    /** Add/update a 2D line/points plot.
     *  x/y data = vector_x/vector_y, format string = str, name = plotName. */
    PLOTS_ADD_LINE = 420,
    /** Add/update a 2D ellipse.
     *  vector_x[0,1]=X/Y centre, vector_x[2]=quantiles,
     *  vector_y[0,1,2]=cov entries 00,11,01, boolVal=showName. */
    PLOTS_ADD_ELLIPSE = 421,
    /** Add/update a bitmap image layer.
     *  plotName=layer name, vector_x[0,1]=X/Y corner, vector_x[2,3]=W/H,
     *  voidPtr2=newly-created wxImage* (ownership transferred). */
    PLOTS_ADD_BITMAP = 422,
    /** Insert a submenu entry into the popup menu.
     *  plotName=label, x=user-defined integer ID. */
    PLOTS_INSERT_SUBMENU = 440,

    // ---- Misc ----
    /** Show a camera-selection dialog.
     *  voidPtr  = std::promise<void>*   (signalled when dialog is ready).
     *  voidPtr2 = std::promise<TReturnAskUserOpenCamera>* (filled on close). */
    CAMERA_SELECT_DIALOG = 700,
    /** Execute userFunction in the GUI thread. */
    RUN_USER_FUNCTION = 800,
    /** Shut down the wxWidgets subsystem. */
    SHUTDOWN = 999,
  };

  /** This method must be called in the destructor of the user class FROM THE
   * MAIN THREAD, in order to wait for the shutdown of the wx thread if this
   * was the last open window.
   */
  static void waitWxShutdownsIfNoWindows();

  /** Will be set to true at runtime if it's not detected a running wxApp
   * instance.
   *  For console apps, we'll create a new thread and run wxEntry from there.
   *  For GUI apps (MRPT-based Windows are a part of a user wxWidget apps),
   * we must leave the control of
   *   message dispatching to the current main loop, so we cannot create a
   * different threads, making things a little different (hence this
   * variable).
   */
  static bool isConsoleApp();

  /** An auxiliary global object used just to launch a final request to the
   * wxSubsystem for shutdown:
   */
  class CAuxWxSubsystemShutdowner
  {
   public:
    CAuxWxSubsystemShutdowner();
    ~CAuxWxSubsystemShutdowner();

    CAuxWxSubsystemShutdowner(const CAuxWxSubsystemShutdowner&) = delete;
    CAuxWxSubsystemShutdowner& operator=(const CAuxWxSubsystemShutdowner&) = delete;
    CAuxWxSubsystemShutdowner(CAuxWxSubsystemShutdowner&&) = delete;
    CAuxWxSubsystemShutdowner& operator=(CAuxWxSubsystemShutdowner&&) = delete;
  };

  static CAuxWxSubsystemShutdowner global_wxsubsystem_shutdown;

  /** The main frame of the wxWidgets application
   */
  class CWXMainFrame : public wxFrame
  {
    friend void WxSubsystem::waitWxShutdownsIfNoWindows();

   public:
    CWXMainFrame(wxWindow* parent, wxWindowID id = -1);
    ~CWXMainFrame() override;

    CWXMainFrame(const CWXMainFrame&) = delete;
    CWXMainFrame& operator=(const CWXMainFrame&) = delete;
    CWXMainFrame(CWXMainFrame&&) = delete;
    CWXMainFrame& operator=(CWXMainFrame&&) = delete;

    /** Atomically increments the number of windows created with the main
     * frame as parent.
     * \return The updated number of windows.
     */
    static int notifyWindowCreation();

    /** Atomically decrements the number of windows created with the main
     * frame as parent.
     * \return The updated number of windows (0 if the calling was the last
     * one).
     */
    static int notifyWindowDestruction();

    static volatile CWXMainFrame* oneInstance;

   private:
    wxTimer* m_theTimer{nullptr};

    void OnTimerProcessRequests(wxTimerEvent& event);

    DECLARE_EVENT_TABLE()

  };  // end class CWXMainFrame

  struct TWxMainThreadData
  {
    /** The thread ID of wxMainThread, or a default-constructed id if it is not running. */
    std::thread m_wxMainThreadId;
    /** This is signaled when wxMainThread is ready. */
    std::promise<void> m_semWxMainThreadReady;
    std::promise<void> m_done;
    /** The critical section for accessing "m_wxMainThreadId" */
    std::mutex m_csWxMainThreadId;
  };

  static TWxMainThreadData& GetWxMainThreadInstance();

  /**  This will be the "MAIN" of wxWidgets: It starts an application object
   * and does not end until all the windows are closed.
   *   Only one instance of this thread can be running at a given instant, no
   * matter how many windows are open.
   */
  static void wxMainThread();

  /** The data structure for each inter-thread request.
   *
   *  The OPCODE field selects the operation; use the WxSubsystem::OpCode enum
   *  for all assignments and comparisons — never use raw integer literals.
   */
  struct TRequestToWxMainThread
  {
    TRequestToWxMainThread() = default;

    /** Only one of source* can be non-nullptr, indicating the class that
     * generated the request. */
    mrpt::gui::CDisplayWindow* source2D{nullptr};
    mrpt::gui::CDisplayWindow3D* source3D{nullptr};
    mrpt::gui::CDisplayWindowPlots* sourcePlots{nullptr};

    /** Set to true when the request originates from a camera-select dialog. */
    bool sourceCameraSelectDialog{false};

    /** String parameter; meaning depends on OPCODE. */
    std::string str;

    /** Generic pointer parameters; meaning depends on OPCODE. */
    void* voidPtr{nullptr};
    void* voidPtr2{nullptr};

    int x{400}, y{400};
    bool boolVal{false};
    mrpt::math::CVectorFloat vector_x, vector_y;
    std::string plotName;

    /** Operation to perform. Use WxSubsystem::OpCode values. */
    OpCode OPCODE{OpCode::SHUTDOWN};

    /** Callable invoked in the GUI thread when OPCODE == RUN_USER_FUNCTION. */
    std::function<void(void)> userFunction;
  };

  /** Thread-safe method to return the next pending request, or nullptr if
   * there is none.  The caller takes ownership and must delete the object.
   */
  static std::unique_ptr<TRequestToWxMainThread> popPendingWxRequest();

  /** Thread-safe method to insert a new pending request.
   *  Ownership of \a data is transferred; it will be deleted by the receiver.
   *  \a data must have been allocated with \c new (single object).
   */
  static void pushPendingWxRequest(std::unique_ptr<TRequestToWxMainThread>&& data);

  /** Thread-safe method to create one single instance of the main wxWidgets
   * thread: it will create the thread only if it is not running yet.
   */
  static bool createOneInstanceMainThread();

  static wxBitmap getMRPTDefaultIcon();
#endif
};  // End of class def.

#if MRPT_HAS_WXWIDGETS

/** The wx dialog for gui::CDisplayWindow
 */
class CWindowDialog : public wxFrame
{
 public:
  CWindowDialog(const CWindowDialog&) = delete;
  CWindowDialog& operator=(const CWindowDialog&) = delete;
  CWindowDialog(CWindowDialog&&) = delete;
  CWindowDialog& operator=(CWindowDialog&&) = delete;

  /** A custom control to display the bitmap and avoid flicker
   */
  class wxMRPTImageControl : public wxPanel
  {
   protected:
    std::unique_ptr<wxBitmap> m_img;
    std::mutex m_img_cs;
    CDisplayWindow* m_win2D{nullptr};

   public:
    wxMRPTImageControl(wxWindow* parent, wxWindowID winID, int x, int y, int width, int height);
    ~wxMRPTImageControl() override;

    wxMRPTImageControl(const wxMRPTImageControl&) = delete;
    wxMRPTImageControl& operator=(const wxMRPTImageControl&) = delete;
    wxMRPTImageControl(wxMRPTImageControl&&) = delete;
    wxMRPTImageControl& operator=(wxMRPTImageControl&&) = delete;

    wxPoint m_last_mouse_point;
    wxPoint m_last_mouse_click;

    /** Assigns this image. This object takes ownership and will delete it
     * when appropriate. */
    void AssignImage(wxBitmap* img);
    void GetBitmap(wxBitmap& bmp);

    void OnPaint(wxPaintEvent& ev);
    void OnMouseMove(wxMouseEvent& ev);
    void OnMouseClick(wxMouseEvent& ev);
    void OnChar(wxKeyEvent& ev);

    void OnEraseBackground(wxEraseEvent& /*ev*/)
    { /* Do nothing */
    }
  };

  CWindowDialog(
      CDisplayWindow* win2D,
      WxSubsystem::CWXMainFrame* parent,
      wxWindowID id = -1,
      const std::string& caption = std::string("[MRPT-CDisplayWindow]"),
      wxSize initialSize = wxDefaultSize);
  ~CWindowDialog() override;

  CDisplayWindow* m_win2D{nullptr};
  WxSubsystem::CWXMainFrame* m_mainFrame{nullptr};

  wxMRPTImageControl* m_image{nullptr};

  static const wxWindowID ID_IMAGE_BITMAP;

 private:
  void OnClose(wxCloseEvent& event);
  void OnMenuClose(wxCommandEvent& event);
  void OnMenuAbout(wxCommandEvent& event);
  void OnMenuSave(wxCommandEvent& event);
  void OnChar(wxKeyEvent& event);
  void OnKeyDown(wxKeyEvent& event);
  void OnResize(wxSizeEvent& event);
  void OnMouseDown(wxMouseEvent& event);
  void OnMouseMove(wxMouseEvent& event);

  DECLARE_EVENT_TABLE()
};  // end class CWindowDialog

class C3DWindowDialog : public wxFrame
{
  friend class gui::CMyGLCanvas_DisplayWindow3D;

 public:
  C3DWindowDialog(
      CDisplayWindow3D* win3D,
      WxSubsystem::CWXMainFrame* parent,
      wxWindowID id = -1,
      const std::string& caption = std::string("[MRPT-CDisplayWindow3D]"),
      wxSize initialSize = wxDefaultSize);

  CDisplayWindow3D* m_win3D{nullptr};
  WxSubsystem::CWXMainFrame* m_mainFrame{nullptr};

  CMyGLCanvas_DisplayWindow3D* m_canvas{nullptr};

 private:
  void OnClose(wxCloseEvent& event);
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
class CWindowDialogPlots : public wxFrame
{
 public:
  CWindowDialogPlots(
      CDisplayWindowPlots* winPlots,
      WxSubsystem::CWXMainFrame* parent,
      wxWindowID id = -1,
      const std::string& caption = std::string("[MRPT-CDisplayWindowPlots]"),
      wxSize initialSize = wxDefaultSize);

  CDisplayWindowPlots* m_winPlots{nullptr};
  WxSubsystem::CWXMainFrame* m_mainFrame{nullptr};

  mpWindow* m_plot{nullptr};
  static const long ID_PLOT;
  static const long ID_MENU_PRINT;
  /** True until the first user submenu is inserted (separator not yet added). */
  bool m_firstSubmenu{true};
  /** Maps internal wxIDs to user-defined IDs for submenus. */
  std::map<long, long> m_ID2ID;
  /** Current cursor position in graph coordinates. */
  mrpt::math::TPoint2D m_curCursorPos;
  /** Current cursor position in pixels. */
  wxPoint m_last_mouse_point;

  void OnMenuSelected(wxCommandEvent& ev);
  void OnMouseMove(wxMouseEvent& event);

  /** Redirected from CDisplayWindowPlots::plot */
  void plot(
      const mrpt::math::CVectorFloat& x,
      const mrpt::math::CVectorFloat& y,
      const std::string& lineFormat,
      const std::string& plotName);

  /** Redirected from CDisplayWindowPlots::plotEllipse */
  void plotEllipse(
      const mrpt::math::CVectorFloat& x,
      const mrpt::math::CVectorFloat& y,
      const std::string& lineFormat,
      const std::string& plotName,
      bool showName = false);

  /** Redirected from CDisplayWindowPlots::image */
  void image(void* theWxImage, float x0, float y0, float w, float h, const std::string& plotName);

 private:
  void OnClose(wxCloseEvent& event);
  void OnMenuPrint(wxCommandEvent& event);
  void OnMenuClose(wxCommandEvent& event);
  void OnMenuAbout(wxCommandEvent& event);
  void OnChar(wxKeyEvent& event);
  void OnResize(wxSizeEvent& event);
  void OnMouseDown(wxMouseEvent& event);

  DECLARE_EVENT_TABLE()
};  // end class CWindowDialogPlots

#endif

}  // namespace mrpt::gui