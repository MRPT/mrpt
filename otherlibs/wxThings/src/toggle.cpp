/////////////////////////////////////////////////////////////////////////////
// Name:        wxCustomButton based on wxCustomToggleCtrl.cpp
// Purpose:     a toggle button
// Author:      Bruce Phillips
// Modified by: John Labenski
// Created:     11/05/2002
// RCS-ID:
// Copyright:   (c) Bruce Phillips, John Labenki
// Licence:     wxWidgets licence
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/control.h"
    #include "wx/settings.h"
    #include "wx/bitmap.h"
    #include "wx/timer.h"
    #include "wx/dc.h"
    #include "wx/dcclient.h"
#endif // WX_PRECOMP

#include "wx/tglbtn.h"
#include "wx/image.h"
#include "wx/renderer.h"
#include "wx/things/toggle.h"

// ==========================================================================
// wxCustomButton
// ==========================================================================
IMPLEMENT_DYNAMIC_CLASS( wxCustomButton, wxControl )

BEGIN_EVENT_TABLE(wxCustomButton,wxControl)
    EVT_MOUSE_EVENTS ( wxCustomButton::OnMouseEvents )
    EVT_PAINT        ( wxCustomButton::OnPaint )
    EVT_TIMER        ( wxID_ANY, wxCustomButton::OnTimer)
    EVT_SIZE         ( wxCustomButton::OnSize )
END_EVENT_TABLE()

wxCustomButton::~wxCustomButton()
{
    if (HasCapture()) ReleaseMouse();
    if (m_timer) delete m_timer;
}

void wxCustomButton::Init()
{
    m_focused      = false;
    m_labelMargin  = wxSize(4,4);
    m_bitmapMargin = wxSize(2,2);
    m_down         = 0;
    m_timer        = nullptr;
    m_eventType    = 0;
    m_button_style = wxCUSTBUT_TOGGLE|wxCUSTBUT_BOTTOM;
}

bool wxCustomButton::Create(wxWindow* parent, wxWindowID id,
                            const wxString& label, const wxBitmap &bitmap,
                            const wxPoint& pos, const wxSize& size_,
                            long style, const wxValidator& val,
                            const wxString& name)
{
    m_labelString = label;
    if (bitmap.Ok()) m_bmpLabel = bitmap;
    wxSize bestSize = DoGetBestSize_(parent);
    wxSize size(size_.x<0 ? bestSize.x:size_.x, size_.y<0 ? bestSize.y:size_.y);

    //SetInitialSize(size);

    if (!wxControl::Create(parent,id,pos,size,wxNO_BORDER|wxCLIP_CHILDREN,val,name))
        return false;

    wxControl::SetBackgroundColour(parent->GetBackgroundColour());
    wxControl::SetForegroundColour(parent->GetForegroundColour());
    wxControl::SetFont(parent->GetFont());

    if (!SetButtonStyle(style)) return false;

    //SetBestSize(size);

    CalcLayout(true);
    return true;
}

void wxCustomButton::SetValue(bool depressed)
{
    wxCHECK_RET(!(m_button_style & wxCUSTBUT_NOTOGGLE), wxT("can't set button state"));
    m_down = depressed ? 1 : 0;
    Refresh(false);
}

bool wxCustomButton::SetButtonStyle(long style)
{
    int n_styles = 0;
    if ((style & wxCUSTBUT_LEFT) != 0)   n_styles++;
    if ((style & wxCUSTBUT_RIGHT) != 0)  n_styles++;
    if ((style & wxCUSTBUT_TOP) != 0)    n_styles++;
    if ((style & wxCUSTBUT_BOTTOM) != 0) n_styles++;
    wxCHECK_MSG(n_styles < 2, false, wxT("Only one wxCustomButton label position allowed"));

    n_styles = 0;
    if ((style & wxCUSTBUT_NOTOGGLE) != 0)       n_styles++;
    if ((style & wxCUSTBUT_BUTTON) != 0)         n_styles++;
    if ((style & wxCUSTBUT_TOGGLE) != 0)         n_styles++;
    if ((style & wxCUSTBUT_BUT_DCLICK_TOG) != 0) n_styles++;
    if ((style & wxCUSTBUT_TOG_DCLICK_BUT) != 0) n_styles++;
    wxCHECK_MSG(n_styles < 2, false, wxT("Only one wxCustomButton style allowed"));

    m_button_style = style;

    if ((m_button_style & wxCUSTBUT_BUTTON) != 0)
        m_down = 0;

    CalcLayout(true);
    return true;
}

void wxCustomButton::SetLabel( const wxString &label )
{
    m_labelString = label;
    InvalidateBestSize();
    CalcLayout(true);
}

// sequence of events in GTK is up, dclick, up.

void wxCustomButton::OnMouseEvents(wxMouseEvent& event)
{
    if (m_button_style & wxCUSTBUT_NOTOGGLE) return;

    if (event.LeftDown() || event.RightDown())
    {
        if (!HasCapture())
            CaptureMouse(); // keep depressed until up

        m_down++;
        Redraw();
    }
    else if (event.LeftDClick() || event.RightDClick())
    {
        m_down++; // GTK eats second down event
        Redraw();
    }
    else if (event.LeftUp())
    {
        if (HasCapture())
            ReleaseMouse();

        m_eventType = wxEVT_LEFT_UP;

        if (wxRect(wxPoint(0,0), GetSize()).Contains(event.GetPosition()))
        {
            if ((m_button_style & wxCUSTBUT_BUTTON) && (m_down > 0))
            {
                m_down = 0;
                Redraw();
                SendEvent();
                return;
            }
            else
            {
                if (!m_timer)
                {
                    m_timer = new wxTimer(this, m_down+1);
                    m_timer->Start(200, true);
                }
                else
                {
                    m_eventType = wxEVT_LEFT_DCLICK;
                }

                if ((m_button_style & wxCUSTBUT_TOGGLE) &&
                    (m_button_style & wxCUSTBUT_TOG_DCLICK_BUT)) m_down++;
            }
        }

        Redraw();
    }
    else if (event.RightUp())
    {
        if (HasCapture())
            ReleaseMouse();

        m_eventType = wxEVT_RIGHT_UP;

        if (wxRect(wxPoint(0,0), GetSize()).Contains(event.GetPosition()))
        {
            if ((m_button_style & wxCUSTBUT_BUTTON) && (m_down > 0))
            {
                m_down = 0;
                Redraw();
                SendEvent();
                return;
            }
            else
            {
                m_down++;

                if (!m_timer)
                {
                    m_timer = new wxTimer(this, m_down);
                    m_timer->Start(250, true);
                }
                else
                {
                    m_eventType = wxEVT_RIGHT_DCLICK;
                }
            }
        }

        Redraw();
    }
    else if (event.Entering())
    {
        m_focused = true;
        if ((event.LeftIsDown() || event.RightIsDown()) && HasCapture())
            m_down++;

        Redraw();
    }
    else if (event.Leaving())
    {
        m_focused = false;
        if ((event.LeftIsDown() || event.RightIsDown()) && HasCapture())
            m_down--;

        Redraw();
    }
}

void wxCustomButton::OnTimer( wxTimerEvent &event )
{
    m_timer->Stop();
    delete m_timer;
    m_timer = nullptr;

    // Clean up the button presses
    // FIXME - GTK eats second left down for a DClick, who know about the others?
    if (m_button_style & wxCUSTBUT_BUTTON)
    {
        m_down = 0;
    }
    else if (m_button_style & wxCUSTBUT_TOGGLE)
    {
        if (m_eventType == wxEVT_LEFT_UP)
            m_down = event.GetId()%2 ? 0 : 1;
        else
            m_down = event.GetId()%2 ? 1 : 0;
    }
    else if (m_button_style & wxCUSTBUT_BUT_DCLICK_TOG)
    {
        if (m_eventType == wxEVT_LEFT_DCLICK)
            m_down = event.GetId()%2 ? 0 : 1;
        else
            m_down = event.GetId()%2 ? 1 : 0;
    }
    else if (m_button_style & wxCUSTBUT_TOG_DCLICK_BUT)
    {
        if (m_eventType == wxEVT_LEFT_UP)
            m_down = event.GetId()%2 ? 0 : 1;
        else
            m_down = event.GetId()%2 ? 1 : 0;
    }

    Refresh(false);
    SendEvent();
}

void wxCustomButton::SendEvent()
{
    if (((m_button_style & wxCUSTBUT_TOGGLE) && (m_eventType == wxEVT_LEFT_UP)) ||
        ((m_button_style & wxCUSTBUT_BUT_DCLICK_TOG) && (m_eventType == wxEVT_LEFT_DCLICK)) ||
        ((m_button_style & wxCUSTBUT_TOG_DCLICK_BUT) && (m_eventType == wxEVT_LEFT_UP)))
    {
        wxCommandEvent eventOut(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, GetId());
        eventOut.SetInt(m_down%2 ? 1 : 0);
        eventOut.SetExtraLong(m_eventType);
        eventOut.SetEventObject(this);
        GetEventHandler()->ProcessEvent(eventOut);
    }
    else
    {
        wxCommandEvent eventOut(wxEVT_COMMAND_BUTTON_CLICKED, GetId());
        eventOut.SetInt(0);
        eventOut.SetExtraLong(m_eventType);
        eventOut.SetEventObject(this);
        GetEventHandler()->ProcessEvent(eventOut);
    }
}

wxBitmap wxCustomButton::CreateBitmapDisabled(const wxBitmap &bitmap) const
{
    wxCHECK_MSG(bitmap.Ok(), wxNullBitmap, wxT("invalid bitmap"));

    unsigned char br = GetBackgroundColour().Red();
    unsigned char bg = GetBackgroundColour().Green();
    unsigned char bb = GetBackgroundColour().Blue();

    wxImage image = bitmap.ConvertToImage();
    int pos, width = image.GetWidth(), height = image.GetHeight();
    unsigned char *img_data = image.GetData();

    for (int j=0; j<height; j++)
    {
        for (int i=j%2; i<width; i+=2)
        {
            pos = (j*width+i)*3;
            img_data[pos  ] = br;
            img_data[pos+1] = bg;
            img_data[pos+2] = bb;
        }
    }

    return wxBitmap(image);

/*      // FIXME why bother creating focused wxCustomButton's bitmap
        wxImage imgFoc = bitmap.ConvertToImage();

        bool mask = false;
        unsigned char mr=0, mg=0, mb=0;
        if (img.HasMask())
        {
            mask = true;
            mr = imgDis.GetMaskRed();
            mg = imgDis.GetMaskGreen();
            mb = imgDis.GetMaskBlue();
        }
        unsigned char *r, *g, *b;
        unsigned char *focData = imgFoc.GetData();
        r = imgFoc.GetData();
        g = imgFoc.GetData() + 1;
        b = imgFoc.GetData() + 2;
        for (int j=0; j<h; j++)
        {
            for (int i=0; i<w; i++)
            {
                if ((!mask || ((*r!=mr)&&(*b!=mb)&&(*g!=mg))) &&
                    ((*r<236)&&(*b<236)&&(*g<236)))
                {
                    *r += 20; *g += 20; *b += 20;
                }
                r += 3; g += 3; b += 3;
            }
        }
        m_bmpFocus = wxBitmap(imgFoc);
*/

}

void wxCustomButton::SetBitmapLabel(const wxBitmap& bitmap)
{
    m_bmpLabel = bitmap;
    InvalidateBestSize();
    CalcLayout(true);
}

void wxCustomButton::OnPaint(wxPaintEvent& WXUNUSED(event))
{
    wxPaintDC dc(this);
    Paint(dc);
}

void wxCustomButton::Redraw()
{
    wxClientDC dc(this);
    Paint(dc);
}

void wxCustomButton::Paint( wxDC &dc )
{
    int w, h;
    GetSize(&w,&h);

    wxColour foreColour = GetForegroundColour();
    wxColour backColour = GetBackgroundColour();

    if (m_focused)
    {
        backColour.Set( wxMin(backColour.Red()   + 20, 255),
                        wxMin(backColour.Green() + 20, 255),
                        wxMin(backColour.Blue()  + 20, 255) );
    }

    wxBitmap bitmap;

    if (IsEnabled())
    {
        if (GetValue() && m_bmpSelected.Ok())
            bitmap = m_bmpSelected;
        else if (m_focused && m_bmpFocus.Ok())
            bitmap = m_bmpFocus;
        else if (m_bmpLabel.Ok())
            bitmap = m_bmpLabel;
    }
    else
    {
        // try to create disabled if it doesn't exist
        if (!m_bmpDisabled.Ok() && m_bmpLabel.Ok())
            m_bmpDisabled = CreateBitmapDisabled(m_bmpLabel);

        if (m_bmpDisabled.Ok())
            bitmap = m_bmpDisabled;
        else if (m_bmpLabel.Ok())
            bitmap = m_bmpLabel;

        foreColour = wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT);
    }

#if wxCHECK_VERSION(2, 8, 0)

    // wxCONTROL_DISABLED
    //flags may have the wxCONTROL_PRESSED, wxCONTROL_CURRENT or wxCONTROL_ISDEFAULT

    int ren_flags = 0;
    if (GetValue())
        ren_flags |= wxCONTROL_PRESSED;
    if (m_focused)
        ren_flags |= wxCONTROL_CURRENT;
    if (!IsEnabled())
        ren_flags |= wxCONTROL_DISABLED;

    wxRendererNative::Get().DrawPushButton(this, dc, wxRect(0, 0, w, h), ren_flags);

#else

    wxBrush brush(backColour, wxSOLID);
    dc.SetBackground(brush);
    dc.SetBrush(brush);
    dc.SetPen(*wxTRANSPARENT_PEN);

    dc.DrawRectangle(0, 0, w, h);

#endif // !wxCHECK_VERSION(2, 8, 0)

    if (bitmap.Ok())
        dc.DrawBitmap(bitmap, m_bitmapPos.x, m_bitmapPos.y, true );

    if (!GetLabel().IsEmpty())
    {
        dc.SetFont(GetFont());
        dc.SetTextBackground(backColour);
        dc.SetTextForeground(foreColour);
        dc.DrawText(GetLabel(), m_labelPos.x, m_labelPos.y);
    }

#if !wxCHECK_VERSION(2, 8, 0)
    if (GetValue())                                        // draw sunken border
    {
        dc.SetPen(*wxGREY_PEN);
        dc.DrawLine(0,h-1,0,0);     dc.DrawLine(0,0,w,0);
        dc.SetPen(*wxWHITE_PEN);
        dc.DrawLine(w-1,1,w-1,h-1); dc.DrawLine(w-1,h-1,0,h-1);
        dc.SetPen(*wxBLACK_PEN);
        dc.DrawLine(1,h-2,1,1);     dc.DrawLine(1,1,w-1,1);
    }
    else if (((m_button_style & wxCUSTBUT_FLAT) == 0) || m_focused) // draw raised border
    {
        dc.SetPen(*wxWHITE_PEN);
        dc.DrawLine(0,h-2,0,0);     dc.DrawLine(0,0,w-1,0);
        dc.SetPen(*wxBLACK_PEN);
        dc.DrawLine(w-1,0,w-1,h-1); dc.DrawLine(w-1,h-1,-1,h-1);
        dc.SetPen(*wxGREY_PEN);
        dc.DrawLine(2,h-2,w-2,h-2); dc.DrawLine(w-2,h-2,w-2,1);
    }
#endif // !wxCHECK_VERSION(2, 8, 0)

    dc.SetBackground(wxNullBrush);
    dc.SetBrush(wxNullBrush);
    dc.SetPen(wxNullPen);
}

void wxCustomButton::OnSize( wxSizeEvent &event )
{
    CalcLayout(true);
    event.Skip();
}

void wxCustomButton::SetMargins(const wxSize &margin, bool fit)
{
    m_labelMargin  = margin;
    m_bitmapMargin = margin;
    if (fit) SetSize(DoGetBestSize());
    CalcLayout(true);
}
void wxCustomButton::SetLabelMargin(const wxSize &margin, bool fit)
{
    m_labelMargin = margin;
    CalcLayout(true);
    if (fit) SetSize(DoGetBestSize());
}
void wxCustomButton::SetBitmapMargin(const wxSize &margin, bool fit)
{
    m_bitmapMargin = margin;
    CalcLayout(true);
    if (fit) SetSize(DoGetBestSize());
}

wxSize wxCustomButton::DoGetBestSize() const
{
    return DoGetBestSize_((wxWindow*)this);
}

wxSize wxCustomButton::DoGetBestSize_(wxWindow* win) const
{
    //((wxWindow*)this)->InvalidateBestSize();

    int lw = 0, lh = 0;
    int bw = 0, bh = 0;
    bool has_bitmap = m_bmpLabel.Ok();
    bool has_label  = !m_labelString.IsEmpty();

    if (has_label)
    {
        win->GetTextExtent(m_labelString, &lw, &lh);
        lw += 2*m_labelMargin.x;
        lh += 2*m_labelMargin.y;
    }
    if (has_bitmap)
    {
        bw = m_bmpLabel.GetWidth()  + 2*m_bitmapMargin.x;
        bh = m_bmpLabel.GetHeight() + 2*m_bitmapMargin.y;
    }

    if (((m_button_style & wxCUSTBUT_LEFT) != 0) || ((m_button_style & wxCUSTBUT_RIGHT) != 0))
    {
        int h = (bh > lh) ? bh : lh;
        if (has_bitmap && has_label)
            lw -= wxMin(m_labelMargin.x, m_bitmapMargin.x);

        return wxSize(lw+bw, h);
    }

    int w = (bw > lw) ? bw : lw;
    if (has_bitmap && has_label)
        lh -= wxMin(m_labelMargin.y, m_bitmapMargin.y);

    return wxSize(w, lh+bh);
}

void wxCustomButton::CalcLayout(bool refresh)
{
    int w, h;
    GetSize(&w,&h);

    int bw = 0, bh = 0;
    int lw = 0, lh = 0;
    bool has_bitmap = m_bmpLabel.Ok();
    bool has_label  = !(GetLabel().IsEmpty());

    if (has_bitmap) // assume they're all the same size
    {
        bw = m_bmpLabel.GetWidth();
        bh = m_bmpLabel.GetHeight();
    }

    if (has_label)
    {
        GetTextExtent(GetLabel(), &lw, &lh);
    }

    // Center the label or bitmap if only one or the other
    if (!has_bitmap)
    {
        m_bitmapPos = wxPoint(0,0);
        m_labelPos  = wxPoint((w-lw)/2, (h-lh)/2);
    }
    else if (!has_label)
    {
        m_bitmapPos = wxPoint((w-bw)/2, (h-bh)/2);
        m_labelPos  = wxPoint(0,0);
    }
    else if ((m_button_style & wxCUSTBUT_LEFT) != 0)
    {
        int mid_margin = wxMax(m_labelMargin.x, m_bitmapMargin.x);
        m_labelPos  = wxPoint((w - (bw+lw+m_labelMargin.x+m_bitmapMargin.x+mid_margin))/2 + m_labelMargin.x, (h - lh)/2);
        m_bitmapPos = wxPoint(m_labelPos.x + lw + mid_margin,         (h - bh)/2);
    }
    else if ((m_button_style & wxCUSTBUT_RIGHT) != 0)
    {
        int mid_margin = wxMax(m_labelMargin.x, m_bitmapMargin.x);
        m_bitmapPos = wxPoint((w - (bw+lw+m_labelMargin.x+m_bitmapMargin.x+mid_margin))/2 + m_bitmapMargin.x, (h - bh)/2);
        m_labelPos  = wxPoint(m_bitmapPos.x + bw + mid_margin,        (h - lh)/2);
    }
    else if ((m_button_style & wxCUSTBUT_TOP) != 0)
    {
        int mid_margin = wxMax(m_labelMargin.y, m_bitmapMargin.y);
        m_labelPos  = wxPoint((w - lw)/2, (h - (bh+lh+m_labelMargin.y+m_bitmapMargin.y+mid_margin))/2 + m_labelMargin.y);
        m_bitmapPos = wxPoint((w - bw)/2, m_labelPos.y + lh + mid_margin);
    }
    else // if ((m_button_style & wxCUSTBUT_BOTTOM) != 0)  DEFAULT
    {
        int mid_margin = wxMax(m_labelMargin.y, m_bitmapMargin.y);
        m_bitmapPos = wxPoint((w - bw)/2, (h - (bh+lh+m_labelMargin.y+m_bitmapMargin.y+mid_margin))/2 + m_bitmapMargin.y);
        m_labelPos  = wxPoint((w - lw)/2, m_bitmapPos.y + bh + mid_margin);
    }

    if (refresh) Refresh(false);
}
