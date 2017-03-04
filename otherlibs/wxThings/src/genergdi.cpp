/////////////////////////////////////////////////////////////////////////////
// Name:        genergdi.cpp
// Purpose:     Generic gdi pen and colour
// Author:      John Labenski
// Modified by:
// Created:     12/01/2000
// Copyright:   (c) John Labenski
// Licence:     wxWidgets license
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#include "wx/things/genergdi.h"
#include "wx/tokenzr.h"
#include "wx/bitmap.h"

const wxGenericColour wxNullGenericColour;
const wxGenericPen    wxNullGenericPen;
const wxGenericBrush  wxNullGenericBrush;
#include "wx/arrimpl.cpp"
WX_DEFINE_OBJARRAY(wxArrayGenericColour)
WX_DEFINE_OBJARRAY(wxArrayGenericPen)
WX_DEFINE_OBJARRAY(wxArrayGenericBrush)

//----------------------------------------------------------------------------
// wxGenericColour
//----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS(wxGenericColour, wxObject)

class wxGenericColourRefData : public wxObjectRefData
{
public:
    wxGenericColourRefData(unsigned char r = 0, unsigned char g = 0,
                           unsigned char b = 0, unsigned char a = 255)
        : wxObjectRefData(), m_r(r), m_g(g), m_b(b), m_a(a) {}

    wxGenericColourRefData( const wxGenericColourRefData& data )
        : wxObjectRefData(), m_r(data.m_r), m_g(data.m_g), m_b(data.m_b), m_a(data.m_a) {}

    unsigned char m_r, m_g, m_b, m_a;
};

#define M_GCOLOURDATA ((wxGenericColourRefData*)m_refData)

//----------------------------------------------------------------------------
wxObjectRefData *wxGenericColour::CreateRefData() const
{
    return new wxGenericColourRefData;
}
wxObjectRefData *wxGenericColour::CloneRefData(const wxObjectRefData *data) const
{
    return new wxGenericColourRefData(*(const wxGenericColourRefData *)data);
}

void wxGenericColour::Create( const wxGenericColour& c )
{
    Ref(c);
}
void wxGenericColour::Create( const wxColour& c)
{
    UnRef();
    m_refData = new wxGenericColourRefData;
    Set(c);
}
void wxGenericColour::Create( unsigned char red,  unsigned char green,
                              unsigned char blue, unsigned char alpha )
{
    UnRef();
    m_refData = new wxGenericColourRefData(red, green, blue, alpha);
}
void wxGenericColour::CreateABGR( unsigned long colABGR )
{
    UnRef();
    m_refData = new wxGenericColourRefData;
    SetABGR(colABGR);
}
void wxGenericColour::CreateARGB( unsigned long colARGB )
{
    UnRef();
    m_refData = new wxGenericColourRefData;
    SetARGB(colARGB);
}
void wxGenericColour::Create( const wxString& colourName )
{
    UnRef();
    m_refData = new wxGenericColourRefData;
    Set(colourName);
}

void wxGenericColour::Set( const wxGenericColour &c )
{
    wxCHECK_RET(Ok() && c.Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_r = c.GetRed();
    M_GCOLOURDATA->m_g = c.GetGreen();
    M_GCOLOURDATA->m_b = c.GetBlue();
    M_GCOLOURDATA->m_a = c.GetAlpha();
}
void wxGenericColour::Set( const wxColour& c )
{
    wxCHECK_RET(Ok() && c.Ok(), wxT("Invalid colour"));
    M_GCOLOURDATA->m_r = c.Red();
    M_GCOLOURDATA->m_g = c.Green();
    M_GCOLOURDATA->m_b = c.Blue();
}
void wxGenericColour::Set( unsigned char red, unsigned char green,
                           unsigned char blue, unsigned char alpha )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_r = red;
    M_GCOLOURDATA->m_g = green;
    M_GCOLOURDATA->m_b = blue;
    M_GCOLOURDATA->m_a = alpha;
}
void wxGenericColour::SetABGR( unsigned long colABGR )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_r = (unsigned char)(0xFF& colABGR);
    M_GCOLOURDATA->m_g = (unsigned char)(0xFF&(colABGR >> 8));
    M_GCOLOURDATA->m_b = (unsigned char)(0xFF&(colABGR >> 16));
    M_GCOLOURDATA->m_a = (unsigned char)(0xFF&(colABGR >> 24));
}
void wxGenericColour::SetARGB( unsigned long colARGB )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_b = (unsigned char)(0xFF& colARGB);
    M_GCOLOURDATA->m_g = (unsigned char)(0xFF&(colARGB >> 8));
    M_GCOLOURDATA->m_r = (unsigned char)(0xFF&(colARGB >> 16));
    M_GCOLOURDATA->m_a = (unsigned char)(0xFF&(colARGB >> 24));
}
void wxGenericColour::Set( const wxString& colourName )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    Set(wxColour(colourName));
}
void wxGenericColour::SetRed( unsigned char r )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_r = r;
}
void wxGenericColour::SetGreen( unsigned char g )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_g = g;
}
void wxGenericColour::SetBlue( unsigned char b )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_b = b;
}
void wxGenericColour::SetAlpha( unsigned char a )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic colour"));
    M_GCOLOURDATA->m_a = a;
}

unsigned char wxGenericColour::GetRed() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic colour"));
    return M_GCOLOURDATA->m_r;
}
unsigned char wxGenericColour::GetGreen() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic colour"));
    return M_GCOLOURDATA->m_g;
}
unsigned char wxGenericColour::GetBlue() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic colour"));
    return M_GCOLOURDATA->m_b;
}
unsigned char wxGenericColour::GetAlpha() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic colour"));
    return M_GCOLOURDATA->m_a;
}

bool wxGenericColour::IsSameAs( const wxGenericColour& c ) const
{
    wxCHECK_MSG(Ok() && c.Ok(), false, wxT("Invalid generic colour"));
    wxGenericColourRefData *cData = (wxGenericColourRefData*)c.GetRefData();
    return (M_GCOLOURDATA->m_r == cData->m_r) && (M_GCOLOURDATA->m_g == cData->m_g) &&
           (M_GCOLOURDATA->m_b == cData->m_b) && (M_GCOLOURDATA->m_a == cData->m_a);
}

bool wxGenericColour::IsSameAs( const wxColour& c ) const
{
    wxCHECK_MSG(Ok() && c.Ok(), false, wxT("Invalid colour"));
    return (M_GCOLOURDATA->m_r == c.Red()) &&
           (M_GCOLOURDATA->m_g == c.Green()) &&
           (M_GCOLOURDATA->m_b == c.Blue());
}

// This code is assumed to be public domain, originally from Paul Bourke, July 1996
// http://astronomy.swin.edu.au/~pbourke/colour/colourramp/source1.c

wxGenericColour wxGenericColour::GetHotColdColour(double v) const
{
    wxGenericColour c(255, 255, 255);
    const double vmin = 0.0, vmax = 255.0, dv = vmax - vmin;

    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;

    if (v < (vmin + 0.25 * dv))
    {
        c.SetRed(0);
        c.SetGreen(int(vmax*(4.0 * (v - vmin) / dv) + 0.5));
    }
    else if (v < (vmin + 0.5 * dv))
    {
        c.SetRed(0);
        c.SetBlue(int(vmax*(1.0 + 4.0 * (vmin + 0.25 * dv - v) / dv) + 0.5));
    }
    else if (v < (vmin + 0.75 * dv))
    {
        c.SetRed(int(vmax*(4.0 * (v - vmin - 0.5 * dv) / dv) + 0.5));
        c.SetBlue(0);
    }
    else
    {
        c.SetGreen(int(vmax*(1.0 + 4.0 * (vmin + 0.75 * dv - v) / dv) + 0.5));
        c.SetBlue(0);
    }

    return c;
}
/*
wxString wxGenericColour::WriteString(const wxString& format) const
{
    return wxString::Format(format.c_str(), m_r, m_g, m_b, m_a);
}
bool wxGenericColour::ReadString(const wxString& str, const wxString& format)
{
    int r,g,b,a;
    if (4 == wxSscanf(str, format.c_str(), &r, &g, &b, &a))
    {
        m_r = r;
        m_g = g;
        m_b = b;
        m_a = a;
        return true;
    }

    return false;
}
*/

//----------------------------------------------------------------------------
// wxGenericPen
//----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS(wxGenericPen, wxObject)

class wxGenericPenRefData : public wxObjectRefData
{
public:
    wxGenericPenRefData(int width = 1, int style = wxSOLID,
                        int cap = wxCAP_ROUND, int join = wxJOIN_ROUND)
          : wxObjectRefData(), m_width(width), m_style(style),
                               m_cap(cap), m_join(join),
                               m_dash_count(0), m_dash(nullptr) {}

    wxGenericPenRefData(const wxGenericPenRefData& data) : wxObjectRefData(),
        m_colour(data.m_colour), m_width(data.m_width), m_style(data.m_style),
        m_cap(data.m_cap), m_join(data.m_join),
        m_dash_count(data.m_dash_count), m_dash(nullptr)
    {
        if (data.m_dash)
        {
            m_dash = (wxDash*)malloc(m_dash_count*sizeof(wxDash));
            memcpy(m_dash, data.m_dash, m_dash_count*sizeof(wxDash));
        }
    }

    ~wxGenericPenRefData() { if (m_dash) free(m_dash); }

    wxGenericColour m_colour;
    int m_width;
    int m_style;
    int m_cap;
    int m_join;

    int m_dash_count; // don't arbitrarily adjust these!
    wxDash *m_dash;
};

#define M_GPENDATA ((wxGenericPenRefData*)m_refData)

//----------------------------------------------------------------------------
wxObjectRefData *wxGenericPen::CreateRefData() const
{
    return new wxGenericPenRefData;
}
wxObjectRefData *wxGenericPen::CloneRefData(const wxObjectRefData *data) const
{
    return new wxGenericPenRefData(*(const wxGenericPenRefData *)data);
}

void wxGenericPen::Create( const wxGenericPen &pen )
{
    Ref(pen);
}
void wxGenericPen::Create( const wxPen &pen )
{
    UnRef();
    m_refData = new wxGenericPenRefData;
    Set(pen);
}
void wxGenericPen::Create(const wxGenericColour &colour, int width, int style,
                          int cap, int join )
{
    UnRef();
    m_refData = new wxGenericPenRefData(width, style, cap, join);
    M_GPENDATA->m_colour = colour;
}
void wxGenericPen::Create(const wxColour &colour, int width, int style,
                          int cap, int join )
{
    Create(wxGenericColour(colour), width, style, cap, join);
}

void wxGenericPen::Set( const wxGenericPen& pen )
{
    wxCHECK_RET(Ok() && pen.Ok(), wxT("Invalid generic pen"));
    SetColour(pen.GetColour());
    M_GPENDATA->m_width = pen.GetWidth();
    M_GPENDATA->m_style = pen.GetStyle();
    M_GPENDATA->m_cap   = pen.GetCap();
    M_GPENDATA->m_join  = pen.GetJoin();

    wxDash* dash;
    int n_dashes = pen.GetDashes(&dash);
    SetDashes(n_dashes, dash);
}
void wxGenericPen::Set( const wxPen &pen )
{
    wxCHECK_RET(Ok() && pen.Ok(), wxT("Invalid generic pen"));
    SetColour(pen.GetColour());
    M_GPENDATA->m_width = pen.GetWidth();
    M_GPENDATA->m_style = pen.GetStyle();
    M_GPENDATA->m_cap   = pen.GetCap();
    M_GPENDATA->m_join  = pen.GetJoin();

    wxDash* dash;
    int n_dashes = pen.GetDashes(&dash);
    SetDashes(n_dashes, dash);

    // or SetDashes(pen.GetDashCount(), pen.GetDash()); not in msw 2.4
}
void wxGenericPen::SetColour( const wxGenericColour &colour )
{
    wxCHECK_RET(Ok() && colour.Ok(), wxT("Invalid generic pen or colour"));
    M_GPENDATA->m_colour = colour;
}
void wxGenericPen::SetColour( const wxColour &colour )
{
    SetColour(wxGenericColour(colour));
}
void wxGenericPen::SetColour( int red, int green, int blue, int alpha )
{
    SetColour(wxGenericColour(red, green, blue, alpha));
}
void wxGenericPen::SetCap( int capStyle )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic pen"));
    M_GPENDATA->m_cap = capStyle;
}
void wxGenericPen::SetJoin( int joinStyle )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic pen"));
    M_GPENDATA->m_join = joinStyle;
}
void wxGenericPen::SetStyle( int style )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic pen"));
    M_GPENDATA->m_style = style;
}
void wxGenericPen::SetWidth( int width )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic pen"));
    M_GPENDATA->m_width = width;
}
void wxGenericPen::SetDashes( int number_of_dashes, const wxDash *dash )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic pen"));
    wxCHECK_RET(((number_of_dashes == 0) && !dash) ||
                ((number_of_dashes >  0) &&  dash), wxT("Invalid dashes for pen"));

    // internal double check to see if somebody's messed with this
    //wxCHECK_RET(((M_GPENDATA->m_dash_count == 0) && !M_GPENDATA->m_dash) ||
    //            ((M_GPENDATA->m_dash_count != 0) &&  M_GPENDATA->m_dash), wxT("Invalid internal dashes for pen"));

    if (M_GPENDATA->m_dash)
    {
        free(M_GPENDATA->m_dash);
        M_GPENDATA->m_dash = nullptr;
        M_GPENDATA->m_dash_count = 0;
    }

    if (!dash)
        return;

    M_GPENDATA->m_dash_count = number_of_dashes;
    M_GPENDATA->m_dash = (wxDash*)malloc(number_of_dashes*sizeof(wxDash));
    memcpy(M_GPENDATA->m_dash, dash, number_of_dashes*sizeof(wxDash));
}

wxPen wxGenericPen::GetPen() const
{
    wxCHECK_MSG(Ok(), wxNullPen, wxT("Invalid generic pen"));
    wxPen pen(M_GPENDATA->m_colour.GetColour(), M_GPENDATA->m_width, M_GPENDATA->m_style);
    pen.SetCap(M_GPENDATA->m_cap);
    pen.SetJoin(M_GPENDATA->m_join);
    if (M_GPENDATA->m_dash_count > 0)
        pen.SetDashes(M_GPENDATA->m_dash_count, M_GPENDATA->m_dash);

    return pen;
}

wxGenericColour wxGenericPen::GetGenericColour() const
{
    wxCHECK_MSG(Ok(), wxNullGenericColour, wxT("Invalid generic pen"));
    return M_GPENDATA->m_colour;
}
wxColour wxGenericPen::GetColour() const
{
    wxCHECK_MSG(Ok(), wxNullColour, wxT("Invalid generic pen"));
    return M_GPENDATA->m_colour.GetColour();
}
int wxGenericPen::GetWidth() const
{
    wxCHECK_MSG(Ok(), 1, wxT("Invalid generic pen"));
    return M_GPENDATA->m_width;
}
int wxGenericPen::GetStyle() const
{
    wxCHECK_MSG(Ok(), wxSOLID, wxT("Invalid generic pen"));
    return M_GPENDATA->m_style;
}
int wxGenericPen::GetCap() const
{
    wxCHECK_MSG(Ok(), wxCAP_ROUND, wxT("Invalid generic pen"));
    return M_GPENDATA->m_cap;
}
int wxGenericPen::GetJoin() const
{
    wxCHECK_MSG(Ok(), wxJOIN_ROUND, wxT("Invalid generic pen"));
    return M_GPENDATA->m_join;
}
int wxGenericPen::GetDashes(wxDash **ptr) const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic pen"));
    *ptr = (wxDash*)M_GPENDATA->m_dash;
    return M_GPENDATA->m_dash_count;
}
int wxGenericPen::GetDashCount() const
{
    wxCHECK_MSG(Ok(), 0, wxT("Invalid generic pen"));
    return M_GPENDATA->m_dash_count;
}
wxDash* wxGenericPen::GetDash() const
{
    wxCHECK_MSG(Ok(), nullptr, wxT("Invalid generic pen"));
    return M_GPENDATA->m_dash;
}

bool wxGenericPen::IsSameAs(const wxGenericPen &pen) const
{
    wxCHECK_MSG(Ok() && pen.Ok(), false, wxT("Invalid generic pen"));
    wxGenericPenRefData *pData = (wxGenericPenRefData*)pen.GetRefData();

    if ((M_GPENDATA->m_colour != pData->m_colour) || (M_GPENDATA->m_width != pData->m_width) ||
        (M_GPENDATA->m_style != pData->m_style) || (M_GPENDATA->m_cap != pData->m_cap) ||
        (M_GPENDATA->m_join != pData->m_join) || (M_GPENDATA->m_dash_count != pen.GetDashCount()))
        return false;

    if (M_GPENDATA->m_dash_count > 0)
        return memcmp(M_GPENDATA->m_dash, pen.GetDash(), M_GPENDATA->m_dash_count*sizeof(wxDash)) == 0;

    return true;
}
bool wxGenericPen::IsSameAs(const wxPen &pen) const
{
    wxCHECK_MSG(Ok() && pen.Ok(), false, wxT("Invalid generic pen"));
    wxGenericPen gp(pen);
    gp.GetGenericColour().SetAlpha(M_GPENDATA->m_colour.GetAlpha());
    return IsSameAs(gp);
}




/*
wxString wxGenericPen::WriteString() const
{
    wxString str;
    str.Printf(wxT("%s,%d,%d,%d,%d,%d"), m_colour.WriteString().c_str(),
                                         m_width, m_style, m_cap, m_join,
                                         m_dash_count);

    for (int i = 0; i < m_dash_count; i++)
        str += wxString::Format(wxT(",%d"), m_dash[i]);

    return str;
}

bool wxGenericPen::ReadString(const wxString& str)
{
    wxArrayString tokens = wxStringTokenize(str, wxT(", "), wxTOKEN_DEFAULT);
    size_t n, count = tokens.GetCount();

    if (count < 9u)
        return false;

    long val;

    for (n = 0; n < count; n++)
    {
        if (!tokens[n].ToLong(&val))
            return false;

        values.Add(int(val));
    }

    m_colour.Set(values[0], values[1], values[2], values[3]);

    size_t num_dashes = values[8];

    if (num_dashes != count - 9)
        return false;

    if (num_dashes > 0)
    {
        wxDash *dash = new wxDash[num_dashes];
        for (n = 0; n < num_dashes; n++)
            dash[n] = (wxDash)values[n];

        SetDashes(num_dashes, dash);
        delete dash;
    }


    m_width = values[4];
    m_style = values[5];
    m_cap   = values[6];
    m_join  = values[7];
    //m_dash_count = values[8];

    return true;
}
*/


//----------------------------------------------------------------------------
// wxGenericBrush
//----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS(wxGenericBrush, wxObject)

class wxGenericBrushRefData : public wxObjectRefData
{
public:
    wxGenericBrushRefData(const wxGenericColour& c = wxNullGenericColour,
                          int style = wxSOLID) : wxObjectRefData(),
                          m_colour(c), m_style(style) {}

    wxGenericBrushRefData(const wxGenericBrushRefData& data) : wxObjectRefData(),
        m_colour(data.m_colour), m_style(data.m_style), m_stipple(data.m_stipple) {}

    ~wxGenericBrushRefData() { }

    wxGenericColour m_colour;
    int             m_style;
    wxBitmap        m_stipple;
};

#define M_GBRUSHDATA ((wxGenericBrushRefData*)m_refData)

//----------------------------------------------------------------------------
wxObjectRefData *wxGenericBrush::CreateRefData() const
{
    return new wxGenericBrushRefData;
}
wxObjectRefData *wxGenericBrush::CloneRefData(const wxObjectRefData *data) const
{
    return new wxGenericBrushRefData(*(const wxGenericBrushRefData *)data);
}

void wxGenericBrush::Create( const wxGenericBrush &brush )
{
    Ref(brush);
}
void wxGenericBrush::Create( const wxBrush &brush )
{
    UnRef();
    m_refData = new wxGenericBrushRefData;
    Set(brush);
}
void wxGenericBrush::Create(const wxGenericColour &colour, int style)
{
    UnRef();
    m_refData = new wxGenericBrushRefData(colour, style);
}
void wxGenericBrush::Create(const wxColour &colour, int style)
{
    Create(wxGenericColour(colour), style);
}
void wxGenericBrush::Create( const wxBitmap &stipple )
{
    UnRef();
    wxCHECK_RET(stipple.Ok(), wxT("Invalid bitmap in wxGenericBrush::Create"));

    int style = stipple.GetMask() ? wxSTIPPLE_MASK_OPAQUE : wxSTIPPLE;
    m_refData = new wxGenericBrushRefData(wxNullGenericColour, style);
    M_GBRUSHDATA->m_stipple = stipple;
}

void wxGenericBrush::Set( const wxGenericBrush& brush )
{
    wxCHECK_RET(Ok() && brush.Ok(), wxT("Invalid generic brush"));
    SetColour(brush.GetColour());
    M_GBRUSHDATA->m_style = brush.GetStyle();
    wxBitmap* stipple = brush.GetStipple();
    if (stipple && stipple->Ok())
        M_GBRUSHDATA->m_stipple = *stipple;
}
void wxGenericBrush::Set( const wxBrush &brush )
{
    wxCHECK_RET(Ok() && brush.Ok(), wxT("Invalid generic brush"));
    SetColour(brush.GetColour());
    M_GBRUSHDATA->m_style = brush.GetStyle();
    wxBitmap* stipple = brush.GetStipple();
    if (stipple && stipple->Ok())
        M_GBRUSHDATA->m_stipple = *stipple;
}
void wxGenericBrush::SetColour( const wxGenericColour &colour )
{
    wxCHECK_RET(Ok() && colour.Ok(), wxT("Invalid generic brush or colour"));
    M_GBRUSHDATA->m_colour = colour;
}
void wxGenericBrush::SetColour( const wxColour &colour )
{
    SetColour(wxGenericColour(colour));
}
void wxGenericBrush::SetColour( int red, int green, int blue, int alpha )
{
    SetColour(wxGenericColour(red, green, blue, alpha));
}
void wxGenericBrush::SetStyle( int style )
{
    wxCHECK_RET(Ok(), wxT("Invalid generic brush"));
    M_GBRUSHDATA->m_style = style;
}
void wxGenericBrush::SetStipple(const wxBitmap& stipple)
{
    wxCHECK_RET(Ok(), wxT("Invalid generic brush"));
    M_GBRUSHDATA->m_stipple = stipple;
    M_GBRUSHDATA->m_style = stipple.GetMask() ? wxSTIPPLE_MASK_OPAQUE : wxSTIPPLE;

}

wxBrush wxGenericBrush::GetBrush() const
{
    wxCHECK_MSG(Ok(), wxNullBrush, wxT("Invalid generic brush"));
    if (M_GBRUSHDATA->m_stipple.Ok())
        return wxBrush(M_GBRUSHDATA->m_stipple);

    return wxBrush(M_GBRUSHDATA->m_colour.GetColour(), M_GBRUSHDATA->m_style);
}

wxGenericColour wxGenericBrush::GetGenericColour() const
{
    wxCHECK_MSG(Ok(), wxNullGenericColour, wxT("Invalid generic brush"));
    return M_GBRUSHDATA->m_colour;
}
wxColour wxGenericBrush::GetColour() const
{
    wxCHECK_MSG(Ok(), wxNullColour, wxT("Invalid generic brush"));
    return M_GBRUSHDATA->m_colour.GetColour();
}
int wxGenericBrush::GetStyle() const
{
    wxCHECK_MSG(Ok(), wxSOLID, wxT("Invalid generic brush"));
    return M_GBRUSHDATA->m_style;
}
wxBitmap* wxGenericBrush::GetStipple() const
{
    wxCHECK_MSG(Ok(), nullptr, wxT("Invalid generic brush"));
    return &M_GBRUSHDATA->m_stipple;
}

bool wxGenericBrush::IsSameAs(const wxGenericBrush& brush) const
{
    wxCHECK_MSG(Ok() && brush.Ok(), 1, wxT("Invalid generic brush"));
    wxGenericBrushRefData *bData = (wxGenericBrushRefData*)brush.GetRefData();
    return (M_GBRUSHDATA->m_colour  == bData->m_colour) &&
           (M_GBRUSHDATA->m_style   == bData->m_style) &&
#if wxCHECK_VERSION(2,7,2)
           (M_GBRUSHDATA->m_stipple.IsSameAs(bData->m_stipple));
#else
           (M_GBRUSHDATA->m_stipple == bData->m_stipple);
#endif // wxCHECK_VERSION(2,7,2)
}
bool wxGenericBrush::IsSameAs(const wxBrush& brush) const
{
    wxCHECK_MSG(Ok() && brush.Ok(), 1, wxT("Invalid generic brush"));
    wxGenericBrush gB(brush);
    gB.GetGenericColour().SetAlpha(M_GBRUSHDATA->m_colour.GetAlpha());
    return IsSameAs(gB);
}
