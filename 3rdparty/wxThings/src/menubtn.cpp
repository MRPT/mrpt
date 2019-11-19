/////////////////////////////////////////////////////////////////////////////
// Name:        wxMenuButton
// Purpose:     A button with a dropdown wxMenu
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
    #include "wx/menu.h"
    #include "wx/settings.h"
    #include "wx/bitmap.h"
    #include "wx/pen.h"
    #include "wx/dc.h"
#endif // WX_PRECOMP

#include "wx/things/toggle.h"
#include "wx/things/menubtn.h"

/* XPM */
static const char *down_arrow_xpm_data[] = {
/* columns rows colors chars-per-pixel */
"5 3 2 1",
"  c None",
"a c Black",
/* pixels */
"aaaaa",
" aaa ",
"  a  "};

static wxBitmap s_dropdownBitmap; // all buttons share the same bitmap

enum
{
    IDD_DROPDOWN_BUTTON = 100
};

//-----------------------------------------------------------------------------
// wxMenuButtonEvents
//-----------------------------------------------------------------------------

DEFINE_LOCAL_EVENT_TYPE(wxEVT_MENUBUTTON_OPEN)

// ==========================================================================
// MenuDropButton
// ==========================================================================

class MenuDropButton : public wxCustomButton
{
public:
    MenuDropButton( wxWindow *parent, wxWindowID id, long style) : wxCustomButton()
    {
        if (!s_dropdownBitmap.Ok())
            s_dropdownBitmap = wxBitmap(down_arrow_xpm_data);

        Create( parent, id, wxEmptyString, s_dropdownBitmap, wxDefaultPosition,
                wxSize(wxMENUBUTTON_DROP_WIDTH, wxMENUBUTTON_DROP_HEIGHT), style);
    }

    virtual void Paint( wxDC &dc )
    {
        wxCustomButton *labelBut = ((wxMenuButton*)GetParent())->GetLabelButton();

        // pretend that both buttons have focus (for flat style)
        if (labelBut)
        {
            wxPoint p = GetParent()->ScreenToClient(wxGetMousePosition());

            if (GetRect().Contains(p) || labelBut->GetRect().Contains(p))
            {
                m_focused = true;

                if (!labelBut->GetFocused())
                    labelBut->SetFocused(true);
            }
            else
            {
                m_focused = false;

                if (labelBut->GetFocused())
                    labelBut->SetFocused(false);
            }
        }

        wxCustomButton::Paint(dc);
    }
};

// ==========================================================================
// MenuLabelButton
// ==========================================================================

class MenuLabelButton : public wxCustomButton
{
public:
    MenuLabelButton( wxWindow* parent, wxWindowID id,
                     const wxString &label,
                     const wxBitmap &bitmap,
                     long style ) : wxCustomButton()
    {
        Create(parent, id, label, bitmap, wxDefaultPosition, wxDefaultSize, style);
    }

    virtual void Paint( wxDC &dc )
    {
        wxCustomButton *dropBut = ((wxMenuButton*)GetParent())->GetDropDownButton();

        // pretend that both buttons have focus (for flat style)
        if (dropBut)
        {
            wxPoint p = GetParent()->ScreenToClient(wxGetMousePosition());

            if (GetRect().Contains(p) || dropBut->GetRect().Contains(p))
            {
                m_focused = true;

                if (!dropBut->GetFocused())
                    dropBut->SetFocused(true);
            }
            else
            {
                m_focused = false;

                if (dropBut->GetFocused())
                    dropBut->SetFocused(false);
            }
        }

        wxCustomButton::Paint(dc);
    }
};

// ==========================================================================
// wxMenuButton
// ==========================================================================

IMPLEMENT_DYNAMIC_CLASS( wxMenuButton, wxControl )

BEGIN_EVENT_TABLE(wxMenuButton,wxControl)
    EVT_BUTTON(wxID_ANY, wxMenuButton::OnButton)

#ifdef __WXMSW__
    EVT_MENU(wxID_ANY, wxMenuButton::OnMenu)
#endif
END_EVENT_TABLE()

wxMenuButton::~wxMenuButton()
{
    AssignMenu(nullptr, true);
}

void wxMenuButton::Init()
{
    m_labelButton    = nullptr;
    m_dropdownButton = nullptr;
    m_menu           = nullptr;
    m_menu_static    = false;
    m_style          = 0;
}

bool wxMenuButton::Create( wxWindow* parent, wxWindowID id,
                           const wxString &label,
                           const wxBitmap &bitmap,
                           const wxPoint& pos,
                           const wxSize& size,
                           long style,
                           const wxValidator& val,
                           const wxString& name)
{
    m_style = style;

    long flat = style & wxMENUBUT_FLAT;

    wxControl::Create(parent,id,pos,size,wxNO_BORDER|wxCLIP_CHILDREN,val,name);
    wxControl::SetLabel(label);
    SetBackgroundColour(parent->GetBackgroundColour());
    SetForegroundColour(parent->GetForegroundColour());
    SetFont(parent->GetFont());

    m_labelButton = new MenuLabelButton(this, id, label, bitmap, wxCUSTBUT_BUTTON|flat);
    m_dropdownButton = new MenuDropButton(this, IDD_DROPDOWN_BUTTON, wxCUSTBUT_BUTTON|flat);

    wxSize bestSize = DoGetBestSize();
    SetSize( wxSize(size.x < 0 ? bestSize.x : size.x,
                    size.y < 0 ? bestSize.y : size.y) );

    //SetBestSize(GetSize());

    return true;
}

#ifdef __WXMSW__
// FIXME - I think there was a patch to fix this
void wxMenuButton::OnMenu( wxCommandEvent &event )
{
    event.Skip();
    wxMenuItem *mi = m_menu->FindItem(event.GetId());
    if (mi && (mi->GetKind() == wxITEM_RADIO))
        m_menu->Check(event.GetId(), true);
}
#endif // __WXMSW__

void wxMenuButton::OnButton( wxCommandEvent &event)
{
    int win_id = event.GetId();

    if (win_id == IDD_DROPDOWN_BUTTON)
    {
        if (m_menu)
        {
            wxNotifyEvent mevent(wxEVT_MENUBUTTON_OPEN, GetId());
            mevent.SetEventObject(this);
            if (GetEventHandler()->ProcessEvent(mevent) && !mevent.IsAllowed())
                return;

            if (!m_menu)
                return;

            PopupMenu(m_menu, wxPoint(0, GetSize().y));

            m_labelButton->Refresh(false);
            m_dropdownButton->Refresh(false);
        }
    }
    else if (win_id == m_labelButton->GetId())
    {
        if (!m_menu) return;

        const wxMenuItemList &items = m_menu->GetMenuItems();
        int first_radio_id = -1;
        int checked_id = -1;
        bool check_next = false;

        // find the next available radio item to check
        wxMenuItemList::compatibility_iterator node;
        for (node = items.GetFirst(); node; node = node->GetNext())
        {
            wxMenuItem *mi = (wxMenuItem*)node->GetData();
            if (mi && (mi->GetKind() == wxITEM_RADIO))
            {
                if (first_radio_id == -1)
                    first_radio_id = mi->GetId();

                if (check_next)
                {
                    check_next = false;
                    checked_id = mi->GetId();
                    break;
                }
                else if (mi->IsChecked())
                    check_next = true;
            }
        }
        // the last item was checked, go back to the first
        if (check_next && (first_radio_id != -1))
            checked_id = first_radio_id;

        if (checked_id != -1)
        {
            m_menu->Check(checked_id, true);

            wxCommandEvent mevent( wxEVT_COMMAND_MENU_SELECTED, checked_id);
            mevent.SetEventObject( m_menu );
            mevent.SetInt(1);
            GetEventHandler()->ProcessEvent(mevent);
        }
    }
}

int wxMenuButton::GetSelection() const
{
    wxCHECK_MSG(m_menu != nullptr, wxNOT_FOUND, wxT("No attached menu in wxMenuButton::GetSelection"));

    const wxMenuItemList &items = m_menu->GetMenuItems();

    wxMenuItemList::compatibility_iterator node;
    for (node = items.GetFirst(); node; node = node->GetNext())
    {
        wxMenuItem *mi = (wxMenuItem*)node->GetData();
        if (mi && (mi->GetKind() == wxITEM_RADIO))
        {
            if (mi->IsChecked())
                return mi->GetId();
        }
    }

    return wxNOT_FOUND;
}

void wxMenuButton::AssignMenu(wxMenu *menu, bool static_menu)
{
    if (!m_menu_static && m_menu)
        delete m_menu;

    m_menu = menu;
    m_menu_static = static_menu;
}

void wxMenuButton::SetToolTip(const wxString &tip)
{
    wxWindow::SetToolTip(tip);
    ((wxWindow*)m_labelButton)->SetToolTip(tip);
    ((wxWindow*)m_dropdownButton)->SetToolTip(tip);
}
void wxMenuButton::SetToolTip(wxToolTip *tip)
{
    wxWindow::SetToolTip(tip);
    ((wxWindow*)m_labelButton)->SetToolTip(tip);
    ((wxWindow*)m_dropdownButton)->SetToolTip(tip);
}

void wxMenuButton::DoSetSize(int x, int y, int width, int height, int sizeFlags)
{
    wxSize curSize( GetSize() );
    wxSize bestSize( DoGetBestSize() );

    if (width == -1)
        width = curSize.GetWidth();
    if (width < 10)
        width = bestSize.GetWidth();

    if (height == -1)
        height = curSize.GetHeight();
    if (height < 5)
        height = bestSize.GetHeight();

    wxWindow::DoSetSize(x, y, width, height, sizeFlags);

    if (m_labelButton)
        m_labelButton->SetSize(0, 0, width - wxMENUBUTTON_DROP_WIDTH, height);
    if (m_dropdownButton)
        m_dropdownButton->SetSize(width-wxMENUBUTTON_DROP_WIDTH, 0, wxMENUBUTTON_DROP_WIDTH, height);
}

wxSize wxMenuButton::DoGetBestSize()
{
    if (!m_labelButton || !m_dropdownButton)
        return wxSize(wxMENUBUTTON_DROP_WIDTH+wxMENUBUTTON_DROP_HEIGHT, wxMENUBUTTON_DROP_HEIGHT);

    wxSize size = m_labelButton->GetBestSize();
    size.x += wxMENUBUTTON_DROP_WIDTH;
    return size;
}
