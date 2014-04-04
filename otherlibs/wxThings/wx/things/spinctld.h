/////////////////////////////////////////////////////////////////////////////
// Name:        spinctld.h
// Purpose:     A double valued spin control, compatible with wxSpinCtrl
// Author:      John Labenski
// Created:     11/05/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

/*
wxSpinCtrlDbl is a double valued wxSpinCtrl using non native (wxWidgets) widgets

It consists of a wxSpinButton and a wxTextCtrl on a wxControl and can be used
as a drop in replacement for the wxSpinCtrl. It's been tested in GTK and MSW
and should work with MAC, but you may need to fix the sizing perhaps. It
will not work in Motif as there is no wxSpinButton in Motif.

Differences to wxSpinCtrl:

    It remembers the initial value as a default value, call SetDefaultValue,
        or press <ESC> to return to it

    Shift + Arrow = *2 increment value
    Ctrl  + Arrow = *10 increment value
    Alt   + Arrow = *100 increment value
    combinations of Shift, Ctrl, Alt increment by the product of the factors

    PgUp & PgDn = *10 increment * the product of the Shift, Ctrl, Alt factors

    <SPACE> sets the control's value to the it's last valid state

    SetDigits controls the format of the text, # decimal places
        exponential uses the %.Xle format otherwise %.Xlf, where places = X
        for arbitray formats subclass control and override SyncSpinToText()
        for proper behavior when a user types in a value
*/

#ifndef __wxSPINCTRLDBL_H__
#define __wxSPINCTRLDBL_H__

#include "wx/spinbutt.h"
#include "wx/spinctrl.h" // for EVT_SPINCTRL
#include "wx/things/thingdef.h"

class WXDLLEXPORT wxTextCtrl;
class WXDLLIMPEXP_THINGS wxSpinCtrlDblTextCtrl;

enum
{
    wxSPINCTRLDBL_AUTODIGITS = -1  // try to autocalc the # of digits
};

class WXDLLIMPEXP_THINGS wxSpinCtrlDbl: public wxControl
{
public:
    wxSpinCtrlDbl() : wxControl() { Init(); }

    // Native constructor - note &parent, this is to avoid ambiguity
    wxSpinCtrlDbl( wxWindow &parent, wxWindowID id,
                   const wxString &value = wxEmptyString,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxSize(95,-1),
                   long style = 0,
                   double min = 0.0, double max = 100.0,
                   double initial = 0.0,
                   double increment = 1.0, int digits = wxSPINCTRLDBL_AUTODIGITS,
                   const wxString& name = _T("wxSpinCtrlDbl") )
    {
        Init();
        Create(&parent, id, value, pos, size, style,
               min, max, initial, increment, digits, name);
    }

    // wxSpinCtrl compatibility, call SetIncrement(increment,digits) after
    wxSpinCtrlDbl( wxWindow *parent, wxWindowID id = wxID_ANY,
                   const wxString &value = wxEmptyString,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxSize(95,-1),
                   long style = 0,
                   int min = 0, int max = 100,
                   int initial = 0,
                   const wxString& name = _T("wxSpinCtrlDbl") )
    {
        Init();
        Create(parent, id, value, pos, size, style,
               (double)min, (double)max, (double)initial, 1.0, -1, name);
    }

    bool Create( wxWindow *parent,
                 wxWindowID id = wxID_ANY,
                 const wxString &value = wxEmptyString,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxSize(100,-1),
                 long style = 0,
                 double min = 0.0, double max = 100.0,
                 double initial = 0.0,
                 double increment = 1.0, int digits = wxSPINCTRLDBL_AUTODIGITS,
                 const wxString& name = _T("wxSpinCtrlDbl") );

    virtual ~wxSpinCtrlDbl();

    // -----------------------------------------------------------------------
    // Public (normal usage) functions

    enum formatType
    {
        lf_fmt, // %lf
        le_fmt, // %le
        lg_fmt  // %lg
    };

    virtual void SetValue( double value );
    void SetValue( double value, double min, double max, double increment,
                   int digits = wxSPINCTRLDBL_AUTODIGITS, formatType fmt = lg_fmt )
        { SetRange(min, max); SetIncrement(increment); SetDigits(digits, fmt); SetValue(value); }
    // Set the value as text, if force then set text as is
    virtual void SetValue( const wxString& text, bool force );
    // Set the allowed range, if max_val < min_val then no range and all vals allowed.
    void SetRange( double min_val, double max_val );
    // Set the increment to use when the spin button or arrow keys pressed.
    void SetIncrement( double increment );
    void SetIncrement( double increment, int digits, formatType fmt = lg_fmt )
        { SetIncrement(increment); SetDigits(digits, fmt); }
    // Set the number of digits to show, use wxSPINCTRLDBL_AUTODIGITS
    //  or specify exact number to show i.e. %.[digits]lf
    //  The format type is used to create an appropriate format string.
    void SetDigits( int digits = wxSPINCTRLDBL_AUTODIGITS, formatType fmt = lg_fmt );
    // Set the format string to use, ie. format="%.2lf" for .01
    void SetFormat( const wxString& format );
    // Set the control the the default value.
    virtual void SetDefaultValue() { SetValue( m_default_value ); }
    // Set the value of the default value, default is the inital value.
    void SetDefaultValue( double default_value );
    // Force the value to always be divisible by the increment, initially off.
    //   This uses the default_value as the basis, you'll get strange results
    //   for very large differences between the current value and default value
    //   when the increment is very small.
    void SetSnapToTicks(bool forceTicks);

    double   GetValue() const { return m_value; }
    double   GetMin() const { return m_min; }
    double   GetMax() const { return m_max; }
    virtual bool HasRange() const { return m_max >= m_min; }
    virtual bool InRange(double value) const { return !HasRange() || ((value >= m_min) && (value <= m_max)); }
    double   GetIncrement() const { return m_increment; }
    int      GetDigits() const { return m_digits; }
    wxString GetFormat() const { return m_textFormat; }
    double   GetDefaultValue() const { return m_default_value; }
    bool     GetSnapToTicks() const { return m_snap_ticks; }

    bool IsDefaultValue() const { return (m_value == m_default_value); }

    virtual bool SetFont( const wxFont &font );
    wxFont GetFont() const;

    virtual bool SetBackgroundColour(const wxColour& colour);
    wxColour GetBackgroundColour() const;

    virtual bool SetForegroundColour(const wxColour& colour);
    wxColour GetForegroundColour() const;

    // for setting... stuff
    wxTextCtrl *GetTextCtrl() { return (wxTextCtrl*)m_textCtrl; }

protected:
    void OnSpinUp( wxSpinEvent &event );
    void OnSpinDown( wxSpinEvent &event );
    void OnTextEnter( wxCommandEvent &event );
    void OnText( wxCommandEvent &event );
    // the textctrl is subclassed to get at pgup/dn and then sent here
    void OnChar( wxKeyEvent &event );

    virtual void SyncSpinToText(bool send_event = true, bool force_valid = true);

    void DoSendEvent();                 // send an event based on current state

    virtual void DoSetSize(int x, int y, int width, int height,
                           int sizeFlags = wxSIZE_AUTO);

    virtual wxSize DoGetBestSize() const;
    virtual void DoSetToolTip( wxToolTip *tip );

    void OnFocus( wxFocusEvent& event );  // pass focus to textctrl, for wxTAB_TRAVERSAL
    void OnKillFocus( wxFocusEvent &event );

    wxSpinButton          *m_spinButton;
    wxSpinCtrlDblTextCtrl *m_textCtrl;

    double   m_min;           // min allowed value
    double   m_max;           // max allowed value
    double   m_value;         // current value
    double   m_default_value; // initial value, or SetDefaultValue(value)
    double   m_increment;     // how much to to add per spin
    int      m_digits;        // number of digits displayed after decimal point
    bool     m_snap_ticks;    // value is divisible by increment
    wxString m_textFormat;    // used as wxString.Printf(m_textFormat.c_str(), m_value);

private:
    friend class wxSpinCtrlDblTextCtrl;

    void Init();
    DECLARE_DYNAMIC_CLASS(wxSpinCtrlDbl)
    DECLARE_EVENT_TABLE()
};

#endif  // __wxSPINCTRLDBL_H__
