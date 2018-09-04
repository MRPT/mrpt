/////////////////////////////////////////////////////////////////////////////
// Name:        wxCustomButton.h based on wxCustomToggleCtrl.cpp
// Purpose:     a toggle button
// Author:      Bruce Phillips modified by John Labenski
// Modified by:
// Created:     11/05/2002
// RCS-ID:
// Copyright:   (c) Bruce Phillips, John Labenski
// Licence:     wxWidgets licence
/////////////////////////////////////////////////////////////////////////////

/*

wxCustomButton is a bitmap and/or text button that can toggle or not.

It can be used as a drop-in replacement for a wxButton, wxToggleButton,
    wxBitmapButton, and the non-existant "wxBitmapToggleButton."

The event's wxCommandEvent::GetExtraLong contains one of the following
    wxEVT_LEFT_UP, wxEVT_RIGHT_UP, wxEVT_LEFT_DCLICK, wxEVT_RIGHT_DCLICK
    these can be used to distinguish between the types of events sent

There are four styles the button can take.

wxCUSTBUT_BUTTON == wxButton
    Left and Right clicks and double clicks all send
        wxEVT_COMMAND_BUTTON_CLICKED => EVT_BUTTON(id,fn)

wxCUSTBUT_TOGGLE == wxToggleButton
    Left clicks sends
        wxEVT_COMMAND_TOGGLEBUTTON_CLICKED => EVT_TOGGLEBUTTON(id, fn)
    Left double clicks and Right clicks send
        wxEVT_COMMAND_BUTTON_CLICKED => EVT_BUTTON(id,fn)

wxCUSTBUT_BUT_DCLICK_TOG
    Left and Right clicks and Right double clicks send
        wxEVT_COMMAND_BUTTON_CLICKED => EVT_BUTTON(id,fn)
    Left double clicks sends
        wxEVT_COMMAND_TOGGLEBUTTON_CLICKED => EVT_TOGGLEBUTTON(id, fn)

wxCUSTBUT_TOG_DCLICK_BUT
    Left clicks sends
        wxEVT_COMMAND_TOGGLEBUTTON_CLICKED => EVT_TOGGLEBUTTON(id, fn)
    Left and Right double clicks and Right clicks send
        wxEVT_COMMAND_BUTTON_CLICKED => EVT_BUTTON(id,fn)

The event's wxCommandEvent::GetInt (IsChecked) is true (1) if the button is
    depressed, this is only useful for the wxToggleButton styles

For both types of button when double-clicked it sends this event
    wxEVT_COMMAND_BUTTON_CLICKED => EVT_BUTTON(id, fn)
    and the button state does not change. Only a single EVT_BUTTON event should
    be sent on double-click and event.GetExtraLong == wxEVT_XXX_DCLICK,
    if not then there's a bug.

If no bitmaps are set the text is centered, if only a bitmap it set then
    it's centered, if a bitmap and text are set then the text is one of the
    positions wxCUSTBUT_LEFT/RIGHT/TOP/BOTTOM

The disabled bitmap it automatically created by dithering with the background,
    the others just copy the bitmap in the constructor. The control assumes they
    are all the same size.

You have to #include "wx/tglbtn.h" for EVT_TOGGLEBUTTON

*/

#ifndef _WX_CUSTOMBUTTON_H_
#define _WX_CUSTOMBUTTON_H_

#include "wx/things/thingdef.h"

#include <wx/bitmap.h>

class /*WXDLLEXPORT*/ wxTimer;
class /*WXDLLEXPORT*/ wxTimerEvent;

//-----------------------------------------------------------------------------
// wxCustomButton styles
//-----------------------------------------------------------------------------

enum wxCustomButton_Style
{
    // Position of the label, use only one
    wxCUSTBUT_LEFT           = 0x0001,
    wxCUSTBUT_RIGHT          = 0x0002,
    wxCUSTBUT_TOP            = 0x0004,
    wxCUSTBUT_BOTTOM         = 0x0008,
    // Button style, use only one
    wxCUSTBUT_NOTOGGLE       = 0x0100,
    wxCUSTBUT_BUTTON         = 0x0200,
    wxCUSTBUT_TOGGLE         = 0x0400,
    wxCUSTBUT_BUT_DCLICK_TOG = 0x0800,
    wxCUSTBUT_TOG_DCLICK_BUT = 0x1000,
    // drawing styles
    wxCUSTBUT_FLAT           = 0x2000 // flat, mouseover raises if not depressed
};

//-----------------------------------------------------------------------------
// wxCustomButton
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxCustomButton : public wxControl
{
public:

    wxCustomButton() : wxControl() { Init(); }

    // wxToggleButton or wxButton compatible constructor (also wxTextCtrl)
    wxCustomButton(wxWindow* parent, wxWindowID id,
                   const wxString& label,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxDefaultSize,
                   long style = wxCUSTBUT_TOGGLE,
                   const wxValidator& val = wxDefaultValidator,
                   const wxString& name = wxT("wxCustomButton"))
                   : wxControl()
    {
        Init();
        Create(parent,id,label,wxNullBitmap,pos,size,style,val,name);
    }

    // wxBitmapButton compatible constructor
    wxCustomButton(wxWindow *parent, wxWindowID id,
                   const wxBitmap& bitmap,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxDefaultSize,
                   long style = wxCUSTBUT_TOGGLE,
                   const wxValidator& val = wxDefaultValidator,
                   const wxString& name = wxT("wxCustomButton"))
                   : wxControl()
    {
        Init();
        Create(parent,id,wxEmptyString,bitmap,pos,size,style,val,name);
    }

    // Native constructor
    wxCustomButton(wxWindow *parent, wxWindowID id,
                   const wxString& label, const wxBitmap& bitmap,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxDefaultSize,
                   long style = wxCUSTBUT_TOGGLE|wxCUSTBUT_BOTTOM,
                   const wxValidator& val = wxDefaultValidator,
                   const wxString& name = wxT("wxCustomButton"))
                   : wxControl()
    {
        Init();
        Create(parent,id,label,bitmap,pos,size,style,val,name);
    }

    ~wxCustomButton() override;

    bool Create(wxWindow* parent,
                wxWindowID id,
                const wxString& label,
                const wxBitmap &bitmap,
                const wxPoint& pos = wxDefaultPosition,
                const wxSize& size = wxDefaultSize,
                long style = 0,
                const wxValidator& val = wxDefaultValidator,
                const wxString& name = wxT("wxCustomButton"));

    bool GetValue() const { return m_down%2 != 0; }
    void SetValue( bool depressed );

    // Use combinations of wxCustomButton_Style(s)
    long GetButtonStyle() const { return m_button_style; }
    bool SetButtonStyle( long style );

    // Get/Set the text label, wxEmptyString for none
    wxString GetLabel() const override { return m_labelString; }
    void SetLabel( const wxString &label ) override;

    // set the bitmaps, ONLY this Label bitmap is used for calculating control size
    //   all bitmaps will be centered accordingly in any case
    //   call SetSet(GetBestSize()) if you change their size and want the control to resize appropriately
    void SetBitmapLabel(const wxBitmap& bitmap);
    void SetBitmapSelected(const wxBitmap& sel)      { m_bmpSelected = sel; CalcLayout(true); };
    void SetBitmapFocus(const wxBitmap& focus)       { m_bmpFocus = focus; CalcLayout(true); };
    void SetBitmapDisabled(const wxBitmap& disabled) { m_bmpDisabled = disabled; CalcLayout(true); };
    // wxBitmapButton compatibility
    void SetLabel(const wxBitmap& bitmap)            { SetBitmapLabel(bitmap); }

    // retrieve the bitmaps
    const wxBitmap& GetBitmapLabel()    const { return m_bmpLabel;    }
    const wxBitmap& GetBitmapSelected() const { return m_bmpSelected; }
    const wxBitmap& GetBitmapFocus()    const { return m_bmpFocus;    }
    const wxBitmap& GetBitmapDisabled() const { return m_bmpDisabled; }

    // Creates a "disabled" bitmap by dithering it with the background colour
    wxBitmap CreateBitmapDisabled(const wxBitmap &bitmap) const;

    // set/get the margins (in pixels) around the label and bitmap
    //    if fit = true then resize the button to fit
    void SetMargins(const wxSize &margin, bool fit = false);

    // set/get the margins around the text label
    //    the inter bitmap/label margin is the max of either margin, not the sum
    void SetLabelMargin(const wxSize &margin, bool fit = false);
    wxSize GetLabelMargin() const { return m_labelMargin; }
    // set/get the margins around the bitmap
    //    the inter bitmap/label margin is the max of either margin, not the sum
    void SetBitmapMargin(const wxSize &margin, bool fit = false);
    wxSize GetBitmapMargin() const { return m_bitmapMargin; }

    // can be used to activate the focused behavior (see MenuButton)
    void SetFocused(bool focused) { m_focused = focused; Refresh(false); }
    bool GetFocused() const { return m_focused; }

protected:
    void OnPaint(wxPaintEvent &event);
    void Redraw();
    virtual void Paint( wxDC &dc );

    // hack for finding the size this should be before creation
    // we cannot call SetSize() if this is a child of toolbar in GTK w/o crashing
    wxSize DoGetBestSize() const override;
    wxSize DoGetBestSize_(wxWindow* win) const;

    virtual void SendEvent();

    void OnMouseEvents(wxMouseEvent &event);

    void OnTimer(wxTimerEvent &event);
    void OnSize( wxSizeEvent &event );

    virtual void CalcLayout(bool refresh);

    int m_down;         // toggle state if m_down%2 then depressed
    bool m_focused;     // mouse in window
    long m_button_style;

    wxString m_labelString;

    // the bitmaps for various states
    wxBitmap m_bmpLabel,
             m_bmpSelected,
             m_bmpFocus,
             m_bmpDisabled;

    // the margins around the label/bitmap
    wxSize m_labelMargin,
           m_bitmapMargin;

    wxPoint m_bitmapPos,
            m_labelPos;

    wxTimer *m_timer;

    wxEventType m_eventType;     // store the mouse event type

private:
    void Init();
    DECLARE_DYNAMIC_CLASS(wxCustomButton)
    DECLARE_EVENT_TABLE()
};

#endif  // _WX_CUSTOMBUTTON_H_
