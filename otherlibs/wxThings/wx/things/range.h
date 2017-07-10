///////////////////////////////////////////////////////////////////////////////
// Name:        range.h
// Purpose:     Simple min-max range class and associated selection array class
// Author:      John Labenski
// Created:     12/01/2000
// Copyright:   (c) John Labenski 2004
// Licence:     wxWidgets
///////////////////////////////////////////////////////////////////////////////

#ifndef __WX_RANGE_H__
#define __WX_RANGE_H__

#include "wx/things/thingdef.h"

class WXDLLIMPEXP_THINGS wxRangeInt;
class WXDLLIMPEXP_THINGS wxRangeDouble;
class WXDLLIMPEXP_THINGS wxRangeIntSelection;
class WXDLLIMPEXP_THINGS wxRangeDoubleSelection;

#include "wx/dynarray.h"
WX_DECLARE_OBJARRAY_WITH_DECL(wxRangeInt, wxArrayRangeInt, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxRangeDouble, wxArrayRangeDouble, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxRangeIntSelection, wxArrayRangeIntSelection, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxRangeDoubleSelection, wxArrayRangeDoubleSelection, class WXDLLIMPEXP_THINGS);

// Empty versions of ranges (0, -1)
WXDLLIMPEXP_DATA_THINGS(extern const wxRangeInt) wxEmptyRangeInt;
WXDLLIMPEXP_DATA_THINGS(extern const wxRangeDouble) wxEmptyRangeDouble;

//=============================================================================
// wxRangeInt
//=============================================================================

class WXDLLIMPEXP_THINGS wxRangeInt
{
public:
    inline wxRangeInt(int min_=0, int max_=0) : m_min(min_), m_max(max_) {}

    // Get the width of the range
    inline int GetRange() const { return m_max - m_min + 1; }
    // Get/Set the min/max values of the range
    inline int GetMin() const { return m_min; }
    inline int GetMax() const { return m_max; }
    inline void SetMin(int min_) { m_min = min_; }
    inline void SetMax(int max_) { m_max = max_; }
    inline void Set(int min_, int max_) { m_min = min_, m_max = max_; }

    // Shift the range by i
    void Shift(int i) { m_min += i; m_max += i; }

    // Is the range empty, min < max
    inline bool IsEmpty() const { return m_min > m_max; }

    // Swap the min and max values
    inline void SwapMinMax() { int temp = m_min; m_min = m_max; m_max = temp; }

    // returns -1 for i < min, 0 for in range, +1 for i > m_max
    inline int Position(int i) const { return i < m_min ? -1 : (i > m_max ? 1 : 0); }

    // Is this point or the range within this range
    inline bool Contains( int i ) const { return (i >= m_min) && (i <= m_max); }
    inline bool Contains( const wxRangeInt &r ) const
        { return (r.m_min >= m_min) && (r.m_max <= m_max) && !IsEmpty() && !r.IsEmpty(); }

    // returns if the range intersects the given range
    inline bool Intersects(const wxRangeInt& r) const
        { return !Intersect(r).IsEmpty();  }
    // returns the intersection of the range with the other, check IsEmpty()
    inline wxRangeInt Intersect(const wxRangeInt& r) const
        { return wxRangeInt(wxMax(m_min, r.m_min), wxMin(m_max, r.m_max)); }
    // returns the union of the range with the other, the min and max of the two
    //   regardless of whether they don't overlap
    inline wxRangeInt Union(const wxRangeInt& r) const
        { return (IsEmpty() || r.IsEmpty()) ? wxEmptyRangeInt : wxRangeInt(wxMin(m_min, r.m_min), wxMax(m_max, r.m_max)); }

    // Is this point inside or touches +/- 1 of the range
    inline bool Touches( int i ) const
        { return !IsEmpty() && wxRangeInt(m_min-1, m_max+1).Contains(i); }
    // Is the range inside or +/- 1 of this range (eg. is it adjoining?)
    inline bool Touches( const wxRangeInt &r ) const
         { return (IsEmpty() || r.IsEmpty()) ? false : r.Intersects(wxRangeInt(m_min-1, m_max+1)); }

    // combine this single point with the range by expanding the m_min/m_max to contain it
    //  if only_if_touching then only combine if i is just outside the range by +/-1
    //  returns true if the range has been changed at all, false if not
    bool Combine(int i, bool only_if_touching = false);
    bool Combine(const wxRangeInt &r, bool only_if_touching = false);

    // delete range r from this, return true is anything was done
    //   if r spans this then this and right become wxEmptyRangeInt
    //   else if r is inside of this then this is the left side and right is the right
    //   else if r.m_min > m_min then this is the left side
    //   else if r.m_min < m_min this is the right side
    bool Delete( const wxRangeInt &r, wxRangeInt *right = nullptr );

    // operators
    // no copy ctor or assignment operator - the defaults are ok

    // comparison
    inline bool operator==(const wxRangeInt& r) const { return (m_min == r.m_min)&&(m_max == r.m_max); }
    inline bool operator!=(const wxRangeInt& r) const { return !(*this == r); }

    // Adding ranges unions them to create the largest range
    inline wxRangeInt operator+(const wxRangeInt& r) const { return Union(r); }
    inline wxRangeInt& operator+=(const wxRangeInt& r) { if(r.m_min<m_min) m_min=r.m_min; if(r.m_max>m_max) m_max=r.m_max; return *this; }
    // Subtracting ranges intersects them to get the smallest range
    inline wxRangeInt operator-(const wxRangeInt& r) const { return Intersect(r); }
    inline wxRangeInt& operator-=(const wxRangeInt& r) { if(r.m_min>m_min) m_min=r.m_min; if(r.m_max<m_max) m_max=r.m_max; return *this; }

    // Adding/Subtracting with an int shifts the range
    inline wxRangeInt operator+(const int i) const { return wxRangeInt(m_min+i, m_max+i); }
    inline wxRangeInt operator-(const int i) const { return wxRangeInt(m_min-i, m_max-i); }
    inline wxRangeInt& operator+=(const int i) { Shift( i); return *this; }
    inline wxRangeInt& operator-=(const int i) { Shift(-i); return *this; }

    int m_min, m_max;
};

//=============================================================================
// wxRangeIntSelection - ordered 1D array of wxRangeInts, combines to minimze size
//=============================================================================

class WXDLLIMPEXP_THINGS wxRangeIntSelection
{
public :
    wxRangeIntSelection() {}
    wxRangeIntSelection(const wxRangeInt& range) { if (!range.IsEmpty()) m_ranges.Add(range); }
    wxRangeIntSelection(const wxRangeIntSelection &ranges) { Copy(ranges); }

    // Make a full copy of the source
    void Copy(const wxRangeIntSelection &source)
        {
            m_ranges.Clear();
            WX_APPEND_ARRAY(m_ranges, source.GetRangeArray());
        }

    // Get the number of individual ranges
    inline int GetCount() const { return m_ranges.GetCount(); }
    // Get total number of items selected in all ranges, ie. sum of all wxRange::GetWidths
    int GetItemCount() const;
    // Get the ranges themselves to iterate though for example
    const wxArrayRangeInt& GetRangeArray() const { return m_ranges; }
    // Get a single range
    const wxRangeInt& GetRange( int index ) const;
    inline const wxRangeInt& Item( int index ) const { return GetRange(index); }
    // Get a range of the min range value and max range value
    wxRangeInt GetBoundingRange() const;
    // Clear all the ranges
    void Clear() { m_ranges.Clear(); }

    // Is this point or range contained in the selection
    inline bool Contains( int i ) const { return Index(i) != wxNOT_FOUND; }
    inline bool Contains( const wxRangeInt &range ) const { return Index(range) != wxNOT_FOUND; }
    // Get the index of the range that contains this, or wxNOT_FOUND
    int Index( int i ) const;
    int Index( const wxRangeInt &range ) const;

    // Get the nearest index of a range, index returned contains i or is the one just below
    //   returns -1 if it's below all the selected ones, or no ranges
    //   returns GetCount() if it's above all the selected ones
    int NearestIndex( int i ) const;

    // Add the range to the selection, returning if anything was done, false if already selected
    bool SelectRange( const wxRangeInt &range );

    // Remove the range to the selection, returning if anything was done, false if not already selected
    bool DeselectRange( const wxRangeInt &range );

    // Set the min and max bounds of the ranges, returns true if anything was done
    bool BoundRanges( const wxRangeInt &range );

    // operators
    inline const wxRangeInt& operator[](int index) const { return GetRange(index); }

    wxRangeIntSelection& operator = (const wxRangeIntSelection& other) { Copy(other); return *this; }

protected :
    wxArrayRangeInt m_ranges;
};

//=============================================================================
// wxRangeDouble
//=============================================================================

class WXDLLIMPEXP_THINGS wxRangeDouble
{
public:
    inline wxRangeDouble(wxDouble min_=0, wxDouble max_=0) : m_min(min_), m_max(max_) {}

    // Get the width of the range
    inline wxDouble GetRange() const { return m_max - m_min; }
    // Get/Set the min/max values of the range
    inline wxDouble GetMin() const { return m_min; }
    inline wxDouble GetMax() const { return m_max; }
    inline void SetMin(wxDouble min_) { m_min = min_; }
    inline void SetMax(wxDouble max_) { m_max = max_; }
    inline void Set(wxDouble min_, wxDouble max_) { m_min = min_, m_max = max_; }

    // Shift the range by i
    void Shift(wxDouble i) { m_min += i; m_max += i; }

    // Is the range empty, min < max
    inline bool IsEmpty() const { return m_min > m_max; }

    // Swap the min and max values
    inline void SwapMinMax() { wxDouble temp = m_min; m_min = m_max; m_max = temp; }

    // returns -1 for i < min, 0 for in range, +1 for i > m_max
    inline int Position(wxDouble i) const { return i < m_min ? -1 : (i > m_max ? 1 : 0); }

    // Is this point or the range within this range
    inline bool Contains( wxDouble i ) const { return (i >= m_min) && (i <= m_max); }
    inline bool Contains( const wxRangeDouble &r ) const
         { return (r.m_min >= m_min) && (r.m_max <= m_max) && !IsEmpty() && !r.IsEmpty(); }

    // returns if the range intersects the given range
    inline bool Intersects(const wxRangeDouble& r) const
        { return !Intersect(r).IsEmpty();  }
    // returns the intersection of the range with the other, check IsEmpty()
    inline wxRangeDouble Intersect(const wxRangeDouble& r) const
        { return wxRangeDouble(wxMax(m_min, r.m_min), wxMin(m_max, r.m_max)); }
    // returns the union of the range with the other, the min and max of the two
    //   regardless of whether they don't overlap
    inline wxRangeDouble Union(const wxRangeDouble& r) const
        { return (IsEmpty() || r.IsEmpty()) ? wxEmptyRangeDouble : wxRangeDouble(wxMin(m_min, r.m_min), wxMax(m_max, r.m_max)); }

    // no touches for double since what would be a good eps value?

    // combine this single point with the range by expanding the m_min/m_max to contain it
    //  if only_if_touching then only combine if there is overlap
    //  returns true if the range has been changed at all, false if not
    bool Combine(wxDouble i);
    bool Combine( const wxRangeDouble &r, bool only_if_touching = false );

    // delete range r from this, return true is anything was done
    //   if r spans this then this and right become wxEmptyRangeInt
    //   else if r is inside of this then this is the left side and right is the right
    //   else if r.m_min > m_min then this is the left side
    //   else if r.m_min < m_min this is the right side
    bool Delete( const wxRangeDouble &r, wxRangeDouble *right = nullptr );

    // operators
    // no copy ctor or assignment operator - the defaults are ok

    // comparison
    inline bool operator==(const wxRangeDouble& r) const { return (m_min == r.m_min)&&(m_max == r.m_max); }
    inline bool operator!=(const wxRangeDouble& r) const { return !(*this == r); }

    // Adding ranges unions them to create the largest range
    inline wxRangeDouble operator+(const wxRangeDouble& r) const { return Union(r); }
    inline wxRangeDouble& operator+=(const wxRangeDouble& r) { if(r.m_min<m_min) m_min=r.m_min; if(r.m_max>m_max) m_max=r.m_max; return *this; }
    // Subtracting ranges intersects them to get the smallest range
    inline wxRangeDouble operator-(const wxRangeDouble& r) const { return Intersect(r); }
    inline wxRangeDouble& operator-=(const wxRangeDouble& r) { if(r.m_min>m_min) m_min=r.m_min; if(r.m_max<m_max) m_max=r.m_max; return *this; }

    // Adding/Subtracting with a double shifts the range
    inline wxRangeDouble operator+(const wxDouble i) const { return wxRangeDouble(m_min+i, m_max+i); }
    inline wxRangeDouble operator-(const wxDouble i) const { return wxRangeDouble(m_min-i, m_max-i); }
    inline wxRangeDouble& operator+=(const wxDouble i) { Shift( i); return *this; }
    inline wxRangeDouble& operator-=(const wxDouble i) { Shift(-i); return *this; }

    wxDouble m_min, m_max;
};

//=============================================================================
// wxRangeDoubleSelection - ordered 1D array of wxRangeDoubles, combines to minimze size
//=============================================================================

class WXDLLIMPEXP_THINGS wxRangeDoubleSelection
{
public :
    wxRangeDoubleSelection() {}
    wxRangeDoubleSelection(const wxRangeDouble& range) { if (!range.IsEmpty()) m_ranges.Add(range); }
    wxRangeDoubleSelection(const wxRangeDoubleSelection &ranges) { Copy(ranges); }

    // Make a full copy of the source
    void Copy(const wxRangeDoubleSelection &source)
        {
            m_ranges.Clear();
            WX_APPEND_ARRAY(m_ranges, source.GetRangeArray());
        }

    // Get the number of individual ranges
    inline int GetCount() const { return m_ranges.GetCount(); }
    // Get the ranges themselves to iterate though for example
    const wxArrayRangeDouble& GetRangeArray() const { return m_ranges; }
    // Get a single range
    const wxRangeDouble& GetRange( int index ) const;
    inline const wxRangeDouble& Item( int index ) const { return GetRange(index); }
    // Get a range of the min range value and max range value
    wxRangeDouble GetBoundingRange() const;
    // Clear all the ranges
    void Clear() { m_ranges.Clear(); }

    // Is this point or range contained in the selection
    inline bool Contains( wxDouble i ) const { return Index(i) != wxNOT_FOUND; }
    inline bool Contains( const wxRangeDouble &range ) const { return Index(range) != wxNOT_FOUND; }
    // Get the index of the range that contains this, or wxNOT_FOUND
    int Index( wxDouble i ) const;
    int Index( const wxRangeDouble &range ) const;

    // Get the nearest index of a range, index returned contains i or is the one just below
    //   returns -1 if it's below all the selected ones, or no ranges
    //   returns GetCount() if it's above all the selected ones
    int NearestIndex( wxDouble i ) const;

    // Add the range to the selection, returning if anything was done, false if already selected
    bool SelectRange( const wxRangeDouble &range);
    // Remove the range to the selection, returning if anything was done, false if not already selected
    bool DeselectRange( const wxRangeDouble &range);

    // Set the min and max bounds of the ranges, returns true if anything was done
    bool BoundRanges( const wxRangeDouble &range );

    // operators
    inline const wxRangeDouble& operator[](int index) const { return GetRange(index); }

    wxRangeDoubleSelection& operator = (const wxRangeDoubleSelection& other) { Copy(other); return *this; }

protected :
    wxArrayRangeDouble m_ranges;
};


#endif // __WX_RANGE_H__
