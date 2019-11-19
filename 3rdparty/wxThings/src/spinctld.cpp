/////////////////////////////////////////////////////////////////////////////
// Name:        spinctld.h
// Author:      John Labenski
// Created:     11/05/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/valtext.h"     // for wxTextValidator
    #include "wx/textctrl.h"
#endif // WX_PRECOMP

#include "wx/tooltip.h"
#include "wx/things/spinctld.h"
#include <math.h>

#if wxCHECK_VERSION(2,5,0)
    #include "wx/math.h"
#else
    #if defined(__VISUALC__) || defined(__WATCOMC__)
        #include <float.h>
        #define wxFinite(x) _finite(x)
    #elif defined(__GNUG__)||defined(__GNUWIN32__)||defined(__DJGPP__)|| \
          defined(__SGI_CC__)||defined(__SUNCC__)||defined(__XLC__)|| \
          defined(__HPUX__)||defined(__MWERKS__)
        #define wxFinite(x) finite(x)
    #else
        #define wxFinite(x) ((x) == (x))
    #endif
#endif // wxCHECK_VERSION(2,5,0)

// NOTES : if the textctrl is focused and the program is ending, a killfocus
//         event is sent in MSW, this is why m_textCtrl is set to nullptr in it's
//         destructor and there's so many checks for it not being NULL

//----------------------------------------------------------------------------
// wxSpinCtrlDbl
//----------------------------------------------------------------------------

// the textctrl used for the wxSpinCtrlDbl, needed for keypresses
class wxSpinCtrlDblTextCtrl : public wxTextCtrl
{
public:
    wxSpinCtrlDblTextCtrl( wxWindow *parent, wxWindowID id,
                           const wxString &value = wxEmptyString,
                           const wxPoint &pos = wxDefaultPosition,
                           const wxSize &size = wxDefaultSize,
                           long style = 0,
                           const wxValidator& validator = wxDefaultValidator,
                           const wxString &name = wxTextCtrlNameStr);

    // MSW sends extra kill focus event
    virtual ~wxSpinCtrlDblTextCtrl()
    {
        if (m_parent) m_parent->m_textCtrl = nullptr;
        m_parent = nullptr;
    }

    wxSpinCtrlDbl *m_parent;

    void OnChar( wxKeyEvent &event );         // pass chars to wxSpinCtrlDbl
    void OnKillFocus( wxFocusEvent &event );  // sync the spin to textctrl

private:
    DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(wxSpinCtrlDblTextCtrl,wxTextCtrl)
//  EVT_TEXT_ENTER( wxID_ANY, wxSpinCtrlDblTextCtrl::OnTextEnter ) // get them from spinctrldbl
//  EVT_TEXT( wxID_ANY, wxSpinCtrlDblTextCtrl::OnTextUpdate )      // get them from spinctrldbl
    EVT_CHAR( wxSpinCtrlDblTextCtrl::OnChar )
    EVT_KILL_FOCUS( wxSpinCtrlDblTextCtrl::OnKillFocus )
END_EVENT_TABLE()

wxSpinCtrlDblTextCtrl::wxSpinCtrlDblTextCtrl( wxWindow *parent, wxWindowID id,
                                              const wxString &value,
                                              const wxPoint &pos, const wxSize &size,
                                              long style,
                                              const wxValidator& validator,
                                              const wxString &name)
                       :wxTextCtrl( parent, id, value, pos, size, style,
                                    validator, name)
{
    m_parent = (wxSpinCtrlDbl*)parent;
}

void wxSpinCtrlDblTextCtrl::OnChar( wxKeyEvent &event )
{
    if (m_parent) m_parent->OnChar( event );
}

void wxSpinCtrlDblTextCtrl::OnKillFocus( wxFocusEvent &event )
{
    if (m_parent) m_parent->SyncSpinToText(true);
    event.Skip();
}

//----------------------------------------------------------------------------
// wxSpinCtrlDbl
//----------------------------------------------------------------------------

IMPLEMENT_DYNAMIC_CLASS( wxSpinCtrlDbl, wxControl )

BEGIN_EVENT_TABLE(wxSpinCtrlDbl,wxControl)
    EVT_SPIN_UP   ( wxID_ANY, wxSpinCtrlDbl::OnSpinUp    )
    EVT_SPIN_DOWN ( wxID_ANY, wxSpinCtrlDbl::OnSpinDown  )
    EVT_TEXT_ENTER( wxID_ANY, wxSpinCtrlDbl::OnTextEnter )
    //EVT_TEXT      ( wxID_ANY, wxSpinCtrlDbl::OnText      )
    EVT_SET_FOCUS ( wxSpinCtrlDbl::OnFocus     )
    EVT_KILL_FOCUS( wxSpinCtrlDbl::OnKillFocus )
END_EVENT_TABLE()

void wxSpinCtrlDbl::Init()
{
    m_min = 0;
    m_max = 100;
    m_value = 0;
    m_default_value = 0;
    m_increment = 1;
    m_digits = wxSPINCTRLDBL_AUTODIGITS;
    m_snap_ticks = false;
    m_spinButton = nullptr;
    m_textCtrl = nullptr;
}

bool wxSpinCtrlDbl::Create( wxWindow *parent, wxWindowID id,
                            const wxString& value,
                            const wxPoint& pos, const wxSize& size,
                            long style,
                            double min, double max,
                            double initial,
                            double increment, int digits,
                            const wxString& name)
{
    if (!wxControl::Create(parent, id, pos, size, style|wxNO_BORDER,
                           wxDefaultValidator, name))
        return false;

    wxControl::SetLabel(name);
    wxControl::SetBackgroundColour(parent->GetBackgroundColour());
    wxControl::SetForegroundColour(parent->GetForegroundColour());

    int width = size.GetWidth(), height = size.GetHeight();

    wxSize best_size( DoGetBestSize() );
    if (width  == -1) width  = best_size.GetWidth();
    if (height == -1) height = best_size.GetHeight();

    // Create a validator for numbers, +-, and eE for exponential
    wxTextValidator validator(wxFILTER_INCLUDE_CHAR_LIST);

#if wxCHECK_VERSION(2, 5, 4)
    wxArrayString list;

    wxString valid_chars(wxT(" 0123456789+-.eE"));
    size_t len = valid_chars.Length();
    for (size_t i=0; i<len; i++)
        list.Add(wxString(valid_chars.GetChar(i)));

    validator.SetIncludes(list);
#else
    wxStringList list;

    wxString valid_chars(wxT(" 0123456789+-.eE"));
    size_t len = valid_chars.Length();
    for (size_t i=0; i<len; i++)
        list.Add(wxString(valid_chars.GetChar(i)));

    validator.SetIncludeList(list);
#endif // wxCHECK_VER(2, 5, 4)

    m_spinButton = new wxSpinButton( this, id, wxPoint(0,0), wxSize(-1, height),
                                     wxSP_ARROW_KEYS|wxSP_VERTICAL|wxSP_WRAP);
    m_textCtrl = new wxSpinCtrlDblTextCtrl( this, id, value,
                      wxPoint(0,0),
                      wxSize(width-m_spinButton->GetSize().GetWidth(), height),
                      wxTE_NOHIDESEL|wxTE_PROCESS_ENTER, validator);

    DoSetSize( pos.x, pos.y, width, height );
#if wxCHECK_VERSION(2,8,2)
    SetInitialSize(wxSize(width, height));
#else
    SetBestSize(wxSize(width, height));
#endif

    m_min = min;
    m_max = max;
    m_value = initial;
    m_default_value = initial;
    m_increment = increment;
    SetDigits( digits );

    // set the value here without generating an event
    if (!value.IsEmpty())
        m_textCtrl->SetValue(value);
    else
        m_textCtrl->SetValue(wxString::Format(m_textFormat.c_str(), initial));

    return true;
}

wxSpinCtrlDbl::~wxSpinCtrlDbl()
{
    if (m_textCtrl) // null this since MSW sends KILL_FOCUS on deletion
    {
        m_textCtrl->m_parent = nullptr;

        wxSpinCtrlDblTextCtrl *text = m_textCtrl;
        m_textCtrl = nullptr;
        delete text;
    }

    delete m_spinButton;
    m_spinButton = nullptr;
}

#define wxSPINCTRLDBL_SPIN_WIDTH  15
#define wxSPINCTRLDBL_SPIN_HEIGHT 22

void wxSpinCtrlDbl::DoSetSize(int x, int y, int width, int height, int sizeFlags)
{
    //wxPrintf(wxT("DoSetSize %d, %d %d %d %d %d\n"), GetId(), x, y, width, height, sizeFlags);

    wxSize bestSize( DoGetBestSize() );
    if (width < 0)  width  = bestSize.GetWidth();
    if (height < 0) height = bestSize.GetHeight();

    wxWindow::DoSetSize(x, y, width, height, sizeFlags);

    int spinwidth  = wxSPINCTRLDBL_SPIN_WIDTH;
    int spinheight = wxSPINCTRLDBL_SPIN_HEIGHT;
    if (m_spinButton)
        m_spinButton->GetSize( &spinwidth, &spinheight );

#ifdef __WIN95__   // humm... these used to be different
    if (m_textCtrl)   m_textCtrl->SetSize( 0, 0, width - spinwidth, height );
    if (m_spinButton) m_spinButton->SetSize( width-spinwidth-2, 0, -1, height );
    //m_textCtrl->SetSize( -3, -3, width - spinwidth, height );   // old wxWin < 2.3.2
    //m_spinButton->SetSize( width-spinwidth-4, -3, -1, height-1 );
#else
    if (m_textCtrl)   m_textCtrl->SetSize( 0, 0, width - spinwidth, height );
    if (m_spinButton) m_spinButton->SetSize( width-spinwidth, 0, -1, height );
#endif
}

static wxSize s_spinctrl_bestSize(-999,-999);

wxSize wxSpinCtrlDbl::DoGetBestSize() const
{
    //wxPrintf(wxT("GetBestSize %d\n"), GetId());
    if (s_spinctrl_bestSize.x == -999)
    {
        wxSpinCtrl spin((wxWindow*)this, wxID_ANY);
        s_spinctrl_bestSize = spin.GetBestSize();
        // oops something went wrong, set to reasonable value
        if (s_spinctrl_bestSize.GetWidth()  < 20)
            s_spinctrl_bestSize.SetWidth(95);
        if (s_spinctrl_bestSize.GetHeight() < 10)
            s_spinctrl_bestSize.SetHeight(wxSPINCTRLDBL_SPIN_HEIGHT);
    }

    return s_spinctrl_bestSize;
}

void wxSpinCtrlDbl::DoSetToolTip( wxToolTip *tip )
{
    // forward tip to textctrl only since having the tip pop up on the buttons
    // is distracting.
    if (tip && m_textCtrl)
    {
        wxPrintf(wxT("TIP %s\n"), tip->GetTip().c_str());
        m_textCtrl->SetToolTip(tip->GetTip());
    }

    wxControl::DoSetToolTip(tip);
}

void wxSpinCtrlDbl::DoSendEvent()
{
    wxCommandEvent event( wxEVT_COMMAND_SPINCTRL_UPDATED, GetId() );
    event.SetEventObject( this );
    event.SetInt( (int)(m_value+0.5) );
    if (m_textCtrl) event.SetString( m_textCtrl->GetValue() );
    GetEventHandler()->ProcessEvent( event );
}

void wxSpinCtrlDbl::OnSpinUp( wxSpinEvent &WXUNUSED(event) )
{
    if (m_textCtrl && m_textCtrl->IsModified() )
        SyncSpinToText(false);

    if ( InRange(m_value + m_increment) )
    {
        m_value += m_increment;
        SetValue( m_value );
        DoSendEvent();
    }
}

void wxSpinCtrlDbl::OnSpinDown( wxSpinEvent &WXUNUSED(event) )
{
    if (m_textCtrl && m_textCtrl->IsModified() )
        SyncSpinToText(false);

    if ( InRange(m_value - m_increment) )
    {
        m_value -= m_increment;
        SetValue( m_value );
        DoSendEvent();
    }
}

void wxSpinCtrlDbl::OnTextEnter( wxCommandEvent &event )
{
    SyncSpinToText(true);
    event.Skip();
}

void wxSpinCtrlDbl::OnText( wxCommandEvent &event )
{
    //wxPrintf(wxT("Text '%s'\n"), event.GetString()); fflush(stdout);
    event.Skip();
}

void wxSpinCtrlDbl::OnChar( wxKeyEvent &event )
{
    double modifier = 1.0;
    if ( event.m_shiftDown   ) modifier  = 2.0;
    if ( event.m_controlDown ) modifier *= 10.0;
    if ( event.m_altDown     ) modifier *= 100.0;

    switch ( event.GetKeyCode() )
    {
        case WXK_UP :
        {
            if (m_textCtrl && m_textCtrl->IsModified()) SyncSpinToText(false);
            SetValue( m_value + m_increment * modifier );
            DoSendEvent();
            break;
        }
        case WXK_DOWN :
        {
            if (m_textCtrl && m_textCtrl->IsModified()) SyncSpinToText(false);
            SetValue( m_value - m_increment * modifier );
            DoSendEvent();
            break;
        }
        case WXK_PAGEUP :  // pg-up
        {
            if (m_textCtrl && m_textCtrl->IsModified()) SyncSpinToText(false);
            SetValue( m_value + m_increment * 10.0 * modifier );
            DoSendEvent();
            break;
        }
        case WXK_PAGEDOWN :  // pg-down
        {
            if (m_textCtrl && m_textCtrl->IsModified()) SyncSpinToText(false);
            SetValue( m_value - m_increment * 10.0 * modifier );
            DoSendEvent();
            break;
        }
        case WXK_SPACE :
        {
            SetValue(m_value);
            event.Skip(false);
            break;
        }
        case WXK_ESCAPE :
        {
            SetDefaultValue();
            DoSendEvent();
            break;
        }
        case WXK_TAB :
        {
            wxNavigationKeyEvent new_event;
            new_event.SetEventObject( GetParent() );
            new_event.SetDirection( !event.ShiftDown() );
            // CTRL-TAB changes the (parent) window, i.e. switch notebook page
            new_event.SetWindowChange( event.ControlDown() );
            new_event.SetCurrentFocus( this );
            GetParent()->GetEventHandler()->ProcessEvent( new_event );
            break;
        }
        default : event.Skip(); break;
    }
}

void wxSpinCtrlDbl::SetValue( double value )
{
    if (!m_textCtrl || !InRange(value))
        return;

    if ( m_snap_ticks && (m_increment != 0) )
    {
        double snap_value = (value - m_default_value) / m_increment;

        if (wxFinite(snap_value)) // FIXME what to do about a failure?
        {
            if (snap_value - floor(snap_value) < ceil(snap_value) - snap_value)
                value = m_default_value + floor(snap_value) * m_increment;
            else
                value = m_default_value + ceil(snap_value) * m_increment;
        }
    }

    wxString str(wxString::Format(m_textFormat.c_str(), value));

    if ((value != m_value) || (str != m_textCtrl->GetValue()))
    {
        m_textCtrl->SetValue( str );
        m_textCtrl->DiscardEdits();
        m_value = value;
        str.ToDouble( &m_value );    // wysiwyg for textctrl
    }
}

void wxSpinCtrlDbl::SetValue( const wxString& text, bool force )
{
    if (!m_textCtrl) return;

    double value;
    if ( text.ToDouble(&value) )
        SetValue( value );
    else if (force)
    {
        m_textCtrl->SetValue( text );
        m_textCtrl->DiscardEdits();
    }
}

void wxSpinCtrlDbl::SetRange( double min_val, double max_val )
{
    //wxCHECK_RET(max_val > min_val, wxT("invalid spinctrl range"));
    m_min = min_val;
    m_max = max_val;

    if (HasRange())
    {
        if (m_value > m_max)
            SetValue(m_max);
        else if (m_value < m_min)
            SetValue(m_min);
    }
}

void wxSpinCtrlDbl::SetIncrement( double increment )
{
    m_increment = increment;
    SetValue(m_value);
}

void wxSpinCtrlDbl::SetDigits( int digits, formatType fmt )
{
    wxCHECK_RET(digits >= -1, wxT("invalid spinctrl format"));

    if ((digits == wxSPINCTRLDBL_AUTODIGITS) && (fmt != lg_fmt))
    {
        wxString wxstr;
        int lastplace = -1, extra_digits = 0;
        if (fmt == le_fmt)
        {
            wxstr.Printf(wxT("%le"), m_increment );
            wxstr.LowerCase();
            lastplace = wxstr.Find(wxT('e')) - 2;
            long places;
            if (wxstr.AfterFirst(wxT('e')).ToLong(&places))
                extra_digits = int(labs(places));
        }
        else if (fmt == lf_fmt)
        {
            wxstr.Printf(wxT("%lf"), m_increment );
            lastplace = wxstr.Len()-1;
        }

        int decimalplace = wxstr.Find(wxT('.'));

        int i = 0;

        for ( i=lastplace; i>decimalplace; i-- )
        {
            if ( wxstr.GetChar(i) != wxT('0') )
            {
                m_digits = extra_digits + i-decimalplace;
                switch (fmt)
                {
                    case le_fmt : m_textFormat.Printf(wxT("%%.%dle"), m_digits ); break;
                    case lf_fmt :
                    default     : m_textFormat.Printf(wxT("%%.%dlg"), m_digits ); break;
                }

                SetValue(m_value);
                return;
            }
        }

        m_digits = 0;  // no digits, I guess
    }
    else
        m_digits = digits;

    switch (fmt)
    {
        case le_fmt : m_textFormat.Printf(wxT("%%.%dle"), m_digits ); break;
        case lg_fmt :
        {
            if (m_digits == -1)
                m_textFormat.Printf(wxT("%%lg") );
            else
                m_textFormat.Printf(wxT("%%.%dlg"), m_digits );
            break;
        }
        case lf_fmt :
        default     : m_textFormat.Printf(wxT("%%.%dlf"), m_digits ); break;
    }

    SetValue(m_value);
}

void wxSpinCtrlDbl::SetFormat( const wxString& format )
{
    wxString wxstr;
    if ( wxstr.Printf(format.c_str(), 123456.123456) > 0 )
        m_textFormat = format;

    SetValue(m_value);
}

void wxSpinCtrlDbl::SetDefaultValue( double default_value )
{
    if ( InRange(default_value) )
    {
        m_default_value = default_value;
        SetDefaultValue();
    }
}

void wxSpinCtrlDbl::SetSnapToTicks(bool forceTicks)
{
    if (m_snap_ticks != forceTicks)
    {
        m_snap_ticks = forceTicks;
        SetValue( m_value );
    }
}

void wxSpinCtrlDbl::OnFocus( wxFocusEvent &event )
{
    if (m_textCtrl)
        m_textCtrl->SetFocus(); // this is to pass TAB navigation

    event.Skip();
}

void wxSpinCtrlDbl::OnKillFocus( wxFocusEvent &event )
{
    SyncSpinToText(true);
    event.Skip();
}

void wxSpinCtrlDbl::SyncSpinToText(bool send_event, bool force_valid)
{
    if (!m_textCtrl)
        return;

    double txt_value;
    if ( m_textCtrl->GetValue().ToDouble( &txt_value ) )
    {
        if ( force_valid || !HasRange() || InRange(txt_value) )
        {
            if (force_valid && HasRange())
            {
                if (txt_value > GetMax())
                    txt_value = GetMax();
                else if (txt_value < GetMin())
                    txt_value = GetMin();
            }

            if (m_value != txt_value)
            {
                SetValue( txt_value );
                if (send_event) DoSendEvent();
            }
        }
    }
    else if (force_valid)
    {
        // textctrl is out of sync, discard and reset
        SetValue(GetValue());
    }
}

bool wxSpinCtrlDbl::SetFont( const wxFont &font )
{
    if (!m_textCtrl) return false;
    return m_textCtrl->SetFont( font );
}
wxFont wxSpinCtrlDbl::GetFont() const
{
    if (!m_textCtrl) return GetFont();
    return m_textCtrl->GetFont();
}

bool wxSpinCtrlDbl::SetBackgroundColour(const wxColour& colour)
{
    if (!m_textCtrl) return wxControl::SetBackgroundColour(colour);
    bool ret = false;
    ret = m_textCtrl->SetBackgroundColour(colour);
    m_textCtrl->Refresh(); // FIXME is this necessary in GTK/OSX
    return ret;
}
wxColour wxSpinCtrlDbl::GetBackgroundColour() const
{
    if (!m_textCtrl) return wxControl::GetBackgroundColour();
    return m_textCtrl->GetBackgroundColour();
}

bool wxSpinCtrlDbl::SetForegroundColour(const wxColour& colour)
{
    if (!m_textCtrl) return wxControl::SetForegroundColour(colour);
    bool ret = false;
    ret = m_textCtrl->SetForegroundColour(colour);
    m_textCtrl->Refresh();
    return ret;
}
wxColour wxSpinCtrlDbl::GetForegroundColour() const
{
    if (!m_textCtrl) return wxControl::GetForegroundColour();
    return m_textCtrl->GetForegroundColour();
}
