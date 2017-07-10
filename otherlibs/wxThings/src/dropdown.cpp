/////////////////////////////////////////////////////////////////////////////
// Name:        DropDownBase
// Purpose:     base class for a control like a combobox
// Author:      John Labenski
// Modified by:
// Created:     11/05/2002
// RCS-ID:
// Copyright:   (c) John Labenki
// Licence:     wxWidgets licence
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/control.h"
    #include "wx/bitmap.h"
#endif // WX_PRECOMP

#include "wx/timer.h"
#include "wx/things/dropdown.h"
#include "wx/things/toggle.h"
#include "wx/renderer.h"

#if wxUSE_POPUPWIN

/* XPM */
static const char *down_arrow_xpm_data[] = {
/* columns rows colors chars-per-pixel */
"7 4 2 1",
"  c None",
"a c Black",
/* pixels */
"aaaaaaa",
" aaaaa ",
"  aaa  ",
"   a   "};

#define IDD_DROPDOWN_BUTTON 100

static wxBitmap s_dropdownBitmap; // all buttons share the same bitmap

// ==========================================================================
// DropDownPopup
// ==========================================================================
#define USE_POPUP_TIMER 0 // FIXME after 2.5.4 we don't need either
#define USE_POPUP_IDLE  0

IMPLEMENT_DYNAMIC_CLASS( DropDownPopup, wxPopupTransientWindow )

BEGIN_EVENT_TABLE(DropDownPopup, wxPopupTransientWindow)
    EVT_KEY_DOWN(DropDownPopup::OnKeyDown)
    EVT_MOUSE_EVENTS(DropDownPopup::OnMouse)
#if USE_POPUP_TIMER
    EVT_TIMER( wxID_ANY, DropDownPopup::OnTimer )
#endif // USE_POPUP_TIMER
#if USE_POPUP_IDLE
    EVT_IDLE( DropDownPopup::OnIdle ) // use Connect/Disconnect instead
#endif // USE_POPUP_IDLE
END_EVENT_TABLE()

bool DropDownPopup::Create(DropDownBase *parent, int style)
{
    m_owner = parent;
    return wxPopupTransientWindow::Create(parent, style);
}

void DropDownPopup::Init()
{
    m_owner          = nullptr;
    m_childWin       = nullptr;
    m_timer          = nullptr;
    m_popped_handler = false;
}

DropDownPopup::~DropDownPopup()
{
    StopTimer();
}

void DropDownPopup::StartTimer()
{
#if USE_POPUP_TIMER
    if (!m_timer)
        m_timer = new wxTimer(this, wxID_ANY);

    m_timer->Start(200, false);
#endif // USE_POPUP_TIMER
}

void DropDownPopup::StopTimer()
{
    if (m_timer)
    {
        if (m_timer->IsRunning())
            m_timer->Stop();
        delete m_timer;
        m_timer = nullptr;
    }
}

void DropDownPopup::PushPopupHandler(wxWindow* child)
{
    if (child && m_handlerPopup && m_popped_handler)
    {
        m_popped_handler = false;

        if (child->GetEventHandler() != (wxEvtHandler*)m_handlerPopup)
            child->PushEventHandler((wxEvtHandler*)m_handlerPopup);
        if (!child->HasCapture())
            child->CaptureMouse();

        child->SetFocus();
    }
}
void DropDownPopup::PopPopupHandler(wxWindow* child)
{
    if (child && m_handlerPopup && !m_popped_handler)
    {
        m_popped_handler = true;

        if (child->GetEventHandler() == (wxEvtHandler*)m_handlerPopup)
            child->PopEventHandler(false);
        if (child->HasCapture())
            child->ReleaseMouse();

        child->SetFocus();
    }
}

void DropDownPopup::OnTimer( wxTimerEvent &WXUNUSED(event) )
{
    if (!IsShown()) return;

    m_mouse = ScreenToClient(wxGetMousePosition());

    wxWindow *child = GetChild();
    if (!child) return; // nothing to do

    wxRect clientRect(GetClientRect());
    //wxPrintf(wxT("**DropDownPopup::OnTimer mouse %d %d -- %d %d %d\n"), m_mouse.x, m_mouse.y, m_popped_handler, m_child, m_handlerPopup); fflush(stdout);
    // pop the event handler if inside the child window or
    // restore the event handler if not in the child window
    if (clientRect.Contains(m_mouse))
        PopPopupHandler(child);
    else
        PushPopupHandler(child);
}

void DropDownPopup::OnIdle( wxIdleEvent& event )
{
    if (IsShown())
    {
        m_mouse = ScreenToClient(wxGetMousePosition());
        wxPrintf(wxT("OnIdle mouse %d %d\n"), m_mouse.x, m_mouse.y);

        wxWindow *child = GetChild();
        if (!child) return; // nothing to do

        wxRect clientRect(GetClientRect());
        //wxPrintf(wxT("**DropDownPopup::OnIdle mouse %d %d -- %d %d %d\n"), m_mouse.x, m_mouse.y, m_popped_handler, m_child, m_handlerPopup); fflush(stdout);
        // pop the event handler if inside the child window or
        // restore the event handler if not in the child window
        if (clientRect.Contains(m_mouse))
            PopPopupHandler(child);
        else
            PushPopupHandler(child);
    }
    event.Skip();
}

void DropDownPopup::OnMouse( wxMouseEvent& event )
{
    m_mouse = event.GetPosition();
    event.Skip();
}

void DropDownPopup::OnKeyDown( wxKeyEvent &event )
{
    if (GetChild() && GetChild()->ProcessEvent(event))
        event.Skip(false);
    else
        event.Skip(true);
}

void DropDownPopup::SetChild(wxWindow *win)
{
    m_childWin = win;
}

void DropDownPopup::Popup(wxWindow *focus)
{
    wxPopupTransientWindow::Popup(focus);

#if USE_POPUP_IDLE
    Connect(wxID_ANY, wxEVT_IDLE,
           (wxObjectEventFunction)(wxEventFunction)(wxIdleEventFunction)
            &DropDownPopup::OnIdle, 0, this);
#endif //USE_POPUP_IDLE

#if USE_POPUP_TIMER
    // start the timer to track the mouse position
    StartTimer();
#endif //USE_POPUP_TIMER
}

void DropDownPopup::Dismiss()
{
#if USE_POPUP_IDLE
    Disconnect(wxID_ANY, wxEVT_IDLE,
               (wxObjectEventFunction)(wxEventFunction)(wxIdleEventFunction)
               &DropDownPopup::OnIdle, 0, this);
#endif //USE_POPUP_IDLE

#if USE_POPUP_TIMER
    StopTimer();
#endif //USE_POPUP_TIMER

    // restore the event handler if necessary for the base class Dismiss
    wxWindow *child = GetChild();
    if (child) PushPopupHandler(child);

    m_popped_handler = false;

    wxPopupTransientWindow::Dismiss();
}

bool DropDownPopup::ProcessLeftDown( wxMouseEvent &event )
{
    m_mouse = event.GetPosition();
    //wxPrintf(wxT("DropDownPopup::ProcessLeftDown %d %d\n"), m_mouse.x, m_mouse.y); fflush(stdout);

    if (m_popped_handler) return true; // shouldn't ever get here, but just in case

    StopTimer();

    // don't let the click on the dropdown button actually press it
    wxCustomButton *dropBut = m_owner->GetDropDownButton();
    if (dropBut)
    {
        wxPoint dropMousePt = dropBut->ScreenToClient(ClientToScreen(m_mouse));
        if (dropBut->HitTest(dropMousePt) == wxHT_WINDOW_INSIDE)
        {
            m_ignore_popup = true;
            Dismiss();
            return true;
        }
    }

    if (GetClientRect().Contains(m_mouse))
        return false;

    Dismiss();
    return true;
}

// ============================================================================
// DropDownBase
// ============================================================================

IMPLEMENT_DYNAMIC_CLASS( DropDownBase, wxControl )

BEGIN_EVENT_TABLE(DropDownBase, wxControl)
    EVT_BUTTON(IDD_DROPDOWN_BUTTON, DropDownBase::OnDropButton)
    EVT_SIZE( DropDownBase::OnSize )
END_EVENT_TABLE()

DropDownBase::~DropDownBase()
{
}

void DropDownBase::Init()
{
    m_popupWin = nullptr;
    m_dropdownButton = nullptr;
}

bool DropDownBase::Create( wxWindow* parent, wxWindowID id,
                           const wxPoint& pos, const wxSize& size,
                           long style, const wxValidator& val,
                           const wxString& name)
{
    if (!wxControl::Create(parent,id,pos,size,wxNO_BORDER|wxCLIP_CHILDREN|style,val,name))
        return false;

    if (!s_dropdownBitmap.Ok())
        s_dropdownBitmap = wxBitmap(down_arrow_xpm_data);

    m_dropdownButton = new wxCustomButton(this, IDD_DROPDOWN_BUTTON,
		                                  s_dropdownBitmap,
										  wxDefaultPosition,
										  wxSize(DROPDOWN_DROP_WIDTH, wxDefaultCoord),
										  wxCUSTBUT_BUTTON);

    return true;
}

void DropDownBase::OnSize( wxSizeEvent& event )
{
    event.Skip();
/*
    if (!m_dropdownButton) return;

	wxSize size = GetClientSize();
    wxPrintf(wxT("DropDownBase %d %d\n"), size.x, size.y);

	wxSize dropSize = m_dropdownButton->GetSize();
    m_dropdownButton->SetSize(size.x-dropSize.x, 0, dropSize.x, size.y);
*/
}

void DropDownBase::DoSetSize(int x, int y, int width, int height, int sizeFlags)
{
    wxSize curSize( GetSize() );

    if (width == -1)
        width = curSize.GetWidth();
    if (height == -1)
        height = curSize.GetHeight();

    wxControl::DoSetSize(x, y, width, height, sizeFlags);

	wxSize dropSize = m_dropdownButton->GetSize();
    m_dropdownButton->SetSize(width-dropSize.x, 0, dropSize.x, height);
}

wxSize DropDownBase::DoGetBestSize() const
{
    return wxSize(95, DROPDOWN_DROP_HEIGHT);
}

bool DropDownBase::ShowPopup()
{
    int x = 0, y = GetSize().y;
    ClientToScreen( &x, &y );

    // control too low, can't show scrollbar, don't bother displaying
    wxRect displayRect = wxGetClientDisplayRect();
    if (displayRect.GetBottom() - y < DROPDOWN_DROP_HEIGHT) return false;

    int width = GetSize().x;
    int height = DoGetBestDropHeight(displayRect.GetBottom() - y);
    if (height < 1) return false;

	m_popupWin = new DropDownPopup(this);

    m_popupWin->SetSize(x, y, width, height);
    if (m_popupWin->GetChild())
        m_popupWin->GetChild()->SetSize(width, height);

    //wxPrintf(wxT("ShowPopup %d %d, %d %d -- %d\n"), width, height, m_popupWin->GetSize().x, m_popupWin->GetSize().y, m_popupWin->GetMinHeight());

    return DoShowPopup();
}

bool DropDownBase::DoShowPopup()
{
    if (m_popupWin)
    {
        if (m_popupWin->GetChild())
            m_popupWin->GetChild()->SetSize(m_popupWin->GetClientSize());

        m_popupWin->Popup(this);
        return true;
    }

    return false;
}

void DropDownBase::HidePopup()
{
    if (m_popupWin)
    {
        m_popupWin->Dismiss();
        m_popupWin->Destroy();
        m_popupWin = nullptr;
    }

    m_dropdownButton->Refresh(true);  // MSW help in toolbar
}

bool DropDownBase::IsPopupShown()
{
    return m_popupWin && m_popupWin->IsShown();
}

void DropDownBase::OnDropButton( wxCommandEvent &WXUNUSED(event))
{
    if (m_popupWin && m_popupWin->m_ignore_popup)
    {
        m_popupWin->m_ignore_popup = false;
        return;
    }

    if (IsPopupShown())
        HidePopup();
    else
        ShowPopup();
}

#endif // wxUSE_POPUPWIN
