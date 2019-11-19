/////////////////////////////////////////////////////////////////////////////
// Name:        optvalue.h
// Purpose:     An string option & value pair class
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets v2
/////////////////////////////////////////////////////////////////////////////

#ifndef __WXOPTIONVALUE_H__
#define __WXOPTIONVALUE_H__

#include "wx/things/thingdef.h"
class WXDLLIMPEXP_THINGS wxOptionValue;

#include "wx/dynarray.h"
WX_DECLARE_OBJARRAY_WITH_DECL(wxOptionValue, wxArrayOptionValue, class WXDLLIMPEXP_THINGS);

//----------------------------------------------------------------------------
// Global wxString utilities
//----------------------------------------------------------------------------

//extern wxArrayString wxStringToWords( const wxString &string );

//----------------------------------------------------------------------------
// wxOptionValue - a ref counted wxString key, wxString value container
//----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxOptionValue : public wxObject
{
public:
    wxOptionValue(bool create = true) : wxObject() { if (create) Create(); }
    wxOptionValue( const wxOptionValue &optValue ) : wxObject() { Create(optValue); }
    wxOptionValue( const wxString &str ) : wxObject() { Create(str); }

    // (Re)Create as an empty container
    bool Create();
    // Ref the other wxOptionValue
    bool Create( const wxOptionValue &optValue );
    // Create from a string with this structure
    //   [type]  # optional
    //   key = value
    bool Create( const wxString &string );

    // Make a true copy of the source wxOptionValue (not refed)
    bool Copy( const wxOptionValue &source );

    // Is there any ref data
    bool Ok() const;
    // Unref the data
    void Destroy();

    //-------------------------------------------------------------------------

    // Get/Set the "type", which can mean whatever you want
    wxString GetType() const;
    void SetType( const wxString &type );

    //-------------------------------------------------------------------------

    // does this have a wxOptionValueArray filled with children
    size_t GetChildrenCount() const;
    wxArrayOptionValue *GetChildren() const;

    bool AddChild( const wxOptionValue& child );
    void DeleteChildren();

    //-------------------------------------------------------------------------

    // Get the number of different option name/value combinations
    size_t GetOptionCount() const;
    // Access the arrays themselves
    wxArrayString GetOptionNames() const;
    wxArrayString GetOptionValues() const;
    // Get a specific option name or value
    wxString GetOptionName( size_t n ) const;
    wxString GetOptionValue( size_t n ) const;

    // returns the index of the option >= 0 or -1 (wxNOT_FOUND) if not found
    int HasOption(const wxString &name) const;
    // Search through the option names for this part returning the first match
    int FindOption(const wxString &part_of_option_name) const;
    // delete this option, returns sucess
    bool DeleteOption(const wxString &name);
    bool DeleteOption( size_t n );

    // Option functions (arbitrary name/value mapping)
    void SetOption(const wxString& name, const wxString& value, bool force=true );
    void SetOption(const wxString& name, int value,             bool force=true ) { SetOption(name, wxString::Format(wxT("%d"), value), force); }
    void SetOption(const wxString& name, double value,          bool force=true ) { SetOption(name, wxString::Format(wxT("%lf"), value), force); }

    // printf style for numeric values SetOption("Name", true, "%d %f", 2, 2.5)
    void SetOption(const wxString& name, bool update, const wxChar* format, ...);

    void SetOption(const wxString& name, int    v1, int    v2, int    v3, bool force=true ) { SetOption(name, wxString::Format(wxT("%d %d %d"), v1, v2, v3), force); }
    void SetOption(const wxString& name, double v1, double v2, double v3, bool force=true ) { SetOption(name, wxString::Format(wxT("%lf %lf %lf"), v1, v2, v3), force); }
    void SetOption(const wxString& name, int   *v, int count,             bool force=true ) { if(v) { wxString s; for (int i=0; i<count; i++) s += wxString::Format(wxT("%d "), v[i]); SetOption(name, s, force); }}
    void SetOption(const wxString& name, float *v, int count,             bool force=true ) { if(v) { wxString s; for (int i=0; i<count; i++) s += wxString::Format(wxT("%f "), v[i]); SetOption(name, s, force); }}
    void SetOption(const wxString& name, const wxPoint &value,            bool force=true ) { SetOption(name, wxString::Format(wxT("%d %d"), value.x, value.y), force); }
    void SetOption(const wxString& name, const wxSize &value,             bool force=true ) { SetOption(name, wxString::Format(wxT("%d %d"), value.x, value.y), force); }
    void SetOption(const wxString& name, const wxRect &value,             bool force=true ) { SetOption(name, wxString::Format(wxT("%d %d %d %d"), value.x, value.y, value.width, value.height), force); }

    wxString GetOption(const wxString& name) const; // returns wxEmptyString if not found
    int GetOptionInt(const wxString& name) const;   // returns 0 if not found

    // These return true on sucess otherwise the value is not modified
    bool GetOption(const wxString& name, wxString &value ) const;
    bool GetOption(const wxString& name, int *value ) const;
    bool GetOption(const wxString& name, float *value ) const;
    bool GetOption(const wxString& name, double *value ) const;

    // sscanf style for numeric values GetOption("Name", "%d %f", &n_int, &n_float)
    int GetOption(const wxString& name, const wxChar* format, ...) const;

    // Get int values filling a wxArrayInt, if count == -1 get all, else count.
    //   delims are the possible separators between values.
    int GetOption(const wxString& name, wxArrayInt &values, int count = -1,
                  const wxString& delims = wxT(" ,\t\r\n")) const;

    // Get values and fill arrays up to count items
    bool GetOption(const wxString& name, unsigned char *value, int count,
                   const wxString& delims = wxT(" ,\t\r\n")) const;
    bool GetOption(const wxString& name, int    *value, int count,
                   const wxString& delims = wxT(" ,\t\r\n")) const;
    bool GetOption(const wxString& name, long   *value, int count,
                   const wxString& delims = wxT(" ,\t\r\n")) const;
    bool GetOption(const wxString& name, float  *value, int count,
                   const wxString& delims = wxT(" ,\t\r\n")) const;
    bool GetOption(const wxString& name, double *value, int count,
                   const wxString& delims = wxT(" ,\t\r\n")) const;

    // Convience function to easily get common values
    bool GetOption(const wxString& name, int   *v1, int   *v2,
                   const wxString& delims = wxT(" ,\t\r\n")) const;

    bool GetOption(const wxString& name, int   *v1, int   *v2, int   *v3,
                   const wxString& delims = wxT(" ,\t\r\n")) const;
    bool GetOption(const wxString& name, float *v1, float *v2, float *v3,
                   const wxString& delims = wxT(" ,\t\r\n")) const;

    bool GetOption(const wxString& name, wxPoint &value,
                   const wxString& delims = wxT(" ,\t\r\n")) const
        { return GetOption(name, &value.x, &value.y, delims); }
    bool GetOption(const wxString& name, wxSize &value,
                   const wxString& delims = wxT(" ,\t\r\n")) const
        { return GetOption(name, &value.x, &value.y, delims); }
    bool GetOption(const wxString& name, wxRect &value,
                   const wxString& delims = wxT(" ,\t\r\n")) const;

    //-------------------------------------------------------------------------
    // Operators

    wxOptionValue& operator = (const wxOptionValue& optValue)
    {
        if ( (*this) != optValue ) wxObject::Ref(optValue);
        return *this;
    }
    bool operator == (const wxOptionValue& optValue) const
        { return m_refData == optValue.m_refData; }
    bool operator != (const wxOptionValue& optValue) const
        { return m_refData != optValue.m_refData; }

private :
    // ref counting code
    virtual wxObjectRefData *CreateRefData() const;
    virtual wxObjectRefData *CloneRefData(const wxObjectRefData *data) const;

    DECLARE_DYNAMIC_CLASS(wxOptionValue);
};

#endif // __WXOPTIONVALUE_H__
