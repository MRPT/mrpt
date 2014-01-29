/////////////////////////////////////////////////////////////////////////////
// Name:        DropDownBase
// Purpose:     base class for a control like a combobox
// Author:      John Labenski
// Modified by:
// Created:     11/05/2002
// Copyright:   (c) John Labenski
// Licence:     wxWidgets licence
/////////////////////////////////////////////////////////////////////////////

/*

YOU MUST USE A wxWIDGETS RELEASE NEWER THAN 1/29/05 - AFTER POPUPWINDOW FIX

DropDownBase is a class that has a DropDownPopup as a child. It works like a
wxComboBox in that there is a button to the right that you use to drop down
a window. You can put whatever you want in the DropDownPopup by calling
GetPopupWindow()->SetChild(win) in your DoShowPopup function.
Additionally, you'll want to put a window to the left of the dropdown button.

You need to subclass this to make a new control.

You need to override these function in DropDownBase
DoGetBestSize() let wxWidgets know how big this control wants to be
DoGetBestDropHeight(max) - max is the # pixels to bottom of screen, you probably want
    it smaller so return that height, return -1 if you don't want the popup shown.
DoShowPopup() - this is called from ShowPopup after creating the m_popupWin.
    Create your child window with the popup as the parent and call
    GetPopupWindow()->SetChild(win). Make sure you call
    DropDownBase::DoShowPopup() to have the popup window shown.
*/

#ifndef _WX_DROPDOWNBASE_H_
#define _WX_DROPDOWNBASE_H_

#include "wx/popupwin.h"
#include "wx/things/thingdef.h"

#if wxUSE_POPUPWIN

class WXDLLEXPORT wxTimer;
class WXDLLEXPORT wxTimerEvent;
class WXDLLIMPEXP_THINGS wxCustomButton;
class WXDLLIMPEXP_THINGS DropDownPopup;

#define DROPDOWN_DROP_WIDTH  14  // these are the default sizes
#define DROPDOWN_DROP_HEIGHT 22

//-----------------------------------------------------------------------------
// DropDownBase generic combobox type widget that drops down a DropDownPopup
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS DropDownBase : public wxControl
{
public:

    DropDownBase() : wxControl() { Init(); }

    DropDownBase( wxWindow *parent, wxWindowID id = wxID_ANY,
                  const wxPoint& pos = wxDefaultPosition,
                  const wxSize& size = wxDefaultSize,
                  long style = 0,
                  const wxValidator& val = wxDefaultValidator,
                  const wxString& name = wxT("DropDownBase"))
                  : wxControl()
    {
        Init();
        Create(parent, id, pos, size, style, val, name);
    }

    virtual ~DropDownBase();

    bool Create(wxWindow* parent,
                wxWindowID id = wxID_ANY,
                const wxPoint& pos = wxDefaultPosition,
                const wxSize& size = wxDefaultSize,
                long style = 0,
                const wxValidator& val = wxDefaultValidator,
                const wxString& name = wxT("DropDownBase"));

    virtual bool ShowPopup();
    virtual void HidePopup();
    bool IsPopupShown();

    // implementation
    void OnDropButton( wxCommandEvent &event );
    wxCustomButton* GetDropDownButton() { return m_dropdownButton; }
    // Get the popup window, NULL when not shown
    DropDownPopup* GetPopupWindow() { return m_popupWin; }

protected:
    virtual void DoSetSize(int x, int y, int width, int height,
                           int sizeFlags = wxSIZE_AUTO);

    void OnSize( wxSizeEvent& event );
    virtual wxSize DoGetBestSize() const;

    virtual bool DoShowPopup();

    // override to set the height of the dropdown box
    //   input max_height is height from bottom of ctrl to bottom of screen
    //   return < 1 to not have the popup displayed
    virtual int DoGetBestDropHeight(int max_height) { return max_height; }

    wxCustomButton *m_dropdownButton;
    DropDownPopup *m_popupWin;

private:
    void Init();
    DECLARE_DYNAMIC_CLASS(DropDownBase)
    DECLARE_EVENT_TABLE()
};

//-----------------------------------------------------------------------------
// DropDownPopup generic popup window, call SetChild
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS DropDownPopup : public wxPopupTransientWindow
{
public:
    DropDownPopup() : wxPopupTransientWindow() { Init(); }
    DropDownPopup(DropDownBase *parent, int style = wxBORDER_NONE) : wxPopupTransientWindow()
    {
        Init();
        Create(parent, style);
    }
    virtual ~DropDownPopup();

    bool Create(DropDownBase *parent, int style = wxBORDER_NONE);

    virtual void Popup(wxWindow *focus = NULL);
    virtual void Dismiss();
    virtual bool ProcessLeftDown(wxMouseEvent& event);

    virtual void SetChild(wxWindow *child);
    virtual wxWindow *GetChild() const { return m_childWin; }
    bool m_ignore_popup;

protected:

    // start/stop timer shat pushes and pops handler when the mouse goes over
    //  the scrollbars (if any) of the child window
    void StartTimer();
    void StopTimer();
    void PushPopupHandler(wxWindow* child);
    void PopPopupHandler(wxWindow* child);

    void OnMouse( wxMouseEvent& event );
    void OnKeyDown( wxKeyEvent &event );
    void OnTimer( wxTimerEvent& event );
    void OnIdle( wxIdleEvent& event );

    wxPoint       m_mouse;           // last/current mouse position
    wxWindow     *m_childWin;        // store our own child pointer
    DropDownBase *m_owner;
    wxTimer      *m_timer;           // timer for tracking mouse position
    bool          m_popped_handler;  // state of the event handler

private:
    void Init();
    DECLARE_DYNAMIC_CLASS(DropDownPopup)
    DECLARE_EVENT_TABLE()
};

#endif // wxUSE_POPUPWIN

#endif  // _WX_DROPDOWNBASE_H_
