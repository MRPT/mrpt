/////////////////////////////////////////////////////////////////////////////
// Name:        block.h
// Purpose:     Rectangular selection storage classes for ints and doubles
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   (c) John Labenski, 2004
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

#ifndef __wxBLOCK_H__
#define __wxBLOCK_H__

#include "wx/geometry.h"
#include "wx/things/thingdef.h"

//#define USE_wxRANGE

#ifdef USE_wxRANGE
    #include "wx/things/range.h"
#endif

// Note: Why are these not just wxRect2DXXX with m_x and m_width?
//       because the double blocks need to match up at the edges and x+width
//       does not always exactly equal the edge of an adjoining block

class WXDLLIMPEXP_THINGS wxBlockInt;
class WXDLLIMPEXP_THINGS wxBlockDouble;
class WXDLLIMPEXP_THINGS wxBlockIntSelection;
class WXDLLIMPEXP_THINGS wxBlockDoubleSelection;

#include "wx/dynarray.h"
WX_DECLARE_OBJARRAY_WITH_DECL(wxBlockInt, wxArrayBlockInt, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxBlockDouble, wxArrayBlockDouble, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxBlockIntSelection, wxArrayBlockIntSelection, class WXDLLIMPEXP_THINGS);
WX_DECLARE_OBJARRAY_WITH_DECL(wxBlockDoubleSelection, wxArrayBlockDoubleSelection, class WXDLLIMPEXP_THINGS);

//=============================================================================
// wxBlockXXX constants
//=============================================================================

// wxEmptyBlockXXX = (0,0,-1,-1)
WXDLLIMPEXP_DATA_THINGS(extern const wxBlockInt) wxEmptyBlockInt;
WXDLLIMPEXP_DATA_THINGS(extern const wxBlockDouble) wxEmptyBlockDouble;

//=============================================================================
// wxBlockXXX sorting functions
//=============================================================================

enum wxBlockSort_Type
{
    wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT,
    wxBLOCKSORT_TOPRIGHT_BOTTOMLEFT,
    wxBLOCKSORT_BOTTOMLEFT_TOPRIGHT,
    wxBLOCKSORT_BOTTOMRIGHT_TOPLEFT,
    wxBLOCKSORT_SMALLEST_TO_LARGEST,
    wxBLOCKSORT_LARGEST_TO_SMALLEST
};

// functions to sort an array of blocks from any corner
extern void wxArrayBlockIntSort(wxArrayBlockInt &blocks,
                                wxBlockSort_Type type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT);

extern void wxArrayBlockDoubleSort(wxArrayBlockDouble &blocks,
                                   wxBlockSort_Type type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT);

//=============================================================================
// wxBlockInt - a rectangle bounded by the corner points that can combine with
//              other wxBlockInts
//=============================================================================

class WXDLLIMPEXP_THINGS wxBlockInt
{
public:
    inline wxBlockInt(wxInt32 x1=0, wxInt32 y1=0, wxInt32 x2=0, wxInt32 y2=0) : m_x1(x1), m_y1(y1), m_x2(x2), m_y2(y2) {}
    inline wxBlockInt(const wxRect2DInt &rect) : m_x1(rect.m_x), m_y1(rect.m_y), m_x2(rect.GetRight()), m_y2(rect.GetBottom()) {}

    inline wxInt32 GetLeft() const   { return m_x1; }
    inline wxInt32 GetRight() const  { return m_x2; }
    inline wxInt32 GetTop() const    { return m_y1; }
    inline wxInt32 GetBottom() const { return m_y2; }

    inline wxInt32 GetWidth() const  { return m_x2 - m_x1 + 1; }
    inline wxInt32 GetHeight() const { return m_y2 - m_y1 + 1; }

    inline wxPoint2DInt GetLeftTop() const     { return wxPoint2DInt(m_x1, m_y1); }
    inline wxPoint2DInt GetLeftBottom() const  { return wxPoint2DInt(m_x1, m_y2); }
    inline wxPoint2DInt GetRightTop() const    { return wxPoint2DInt(m_x2, m_y1); }
    inline wxPoint2DInt GetRightBottom() const { return wxPoint2DInt(m_x2, m_y2); }

    inline wxRect2DInt GetRect2DInt() const { return wxRect2DInt(m_x1, m_y1, m_x2-m_x1+1, m_y2-m_y1+1); }
    inline void SetRect2DInt(const wxRect2DInt &r) { m_x1=r.m_x; m_y1=r.m_y, m_x2=r.GetRight(); m_y2=r.GetBottom(); }

    inline bool Contains( wxInt32 x, wxInt32 y ) const
        { return ((x >= m_x1) && (x <= m_x2) && (y >= m_y1) && (y <= m_y2)); }
    inline bool Contains( const wxPoint2DInt &pt ) const { return Contains(pt.m_x, pt.m_y); }
    inline bool Contains( const wxBlockInt &b ) const
        { return ((m_x1 <= b.m_x1) && (b.m_x2 <= m_x2) && (m_y1 <= b.m_y1) && (b.m_y2 <= m_y2)); }

    inline bool Intersects( const wxBlockInt &b ) const
        { return (wxMax(m_x1, b.m_x1)<=wxMin(m_x2, b.m_x2)) && (wxMax(m_y1, b.m_y1)<=wxMin(m_y2, b.m_y2)); }
    inline void Intersect( const wxBlockInt &otherBlock ) { Intersect( *this, otherBlock, this ); }
    inline void Intersect( const wxBlockInt &src1 , const wxBlockInt &src2 , wxBlockInt *dest ) const
    {
        dest->m_x1 = wxMax(src1.m_x1, src2.m_x1);
        dest->m_x2 = wxMin(src1.m_x2, src2.m_x2);
        dest->m_y1 = wxMax(src1.m_y1, src2.m_y1);
        dest->m_y2 = wxMin(src1.m_y2, src2.m_y2);
    }

    inline void Union( const wxBlockInt &otherBlock ) { Union(*this, otherBlock, this); }
    inline void Union( const wxBlockInt &src1, const wxBlockInt &src2, wxBlockInt *dest ) const
    {
        dest->m_x1 = wxMin(src1.m_x1, src2.m_x1);
        dest->m_x2 = wxMax(src1.m_x2, src2.m_x2);
        dest->m_y1 = wxMin(src1.m_y1, src2.m_y1);
        dest->m_y2 = wxMax(src1.m_y2, src2.m_y2);
    }

    // is this block larger than input block, return 1 = larger, 0 = equal, -1 = smaller
    int IsLarger(const wxBlockInt &b) const;
    bool IsEmpty() const { return (m_x1 > m_x2) || (m_y1 > m_y2); }

    // Unlike Intersects this also includes just touching the other block
    bool Touches(const wxBlockInt &block) const;

    // Try to combine these blocks, they must touch and fit to make a single larger block
    //    this block is expanded if possible
    bool Combine(const wxBlockInt &block);

    // test combining the input block with this one, returning the
    // remainder of block in top, bottom, left, right - each may be IsEmpty()
    // returns false if blocks don't touch or this block already contains block
    // |---------------------------|
    // |           top             |
    // |---------------------------|
    // |   left  |block|  right    |
    // |---------------------------|
    // |          bottom           |
    // |---------------------------|
    bool Combine( const wxBlockInt &block,
                  wxBlockInt &top, wxBlockInt &bottom,
                  wxBlockInt &left, wxBlockInt &right) const;

    // test removal of a portion or all of this contained in block returning the
    // remainder in top, bottom, left, right - each may be IsEmpty()
    // returns false if nothing to delete, this cell is not changed
    bool Delete(const wxBlockInt &block, wxBlockInt &top,  wxBlockInt &bottom,
                                         wxBlockInt &left, wxBlockInt &right) const;

    // operators
    inline bool operator == (const wxBlockInt& b)
        { return (m_x1==b.m_x1) && (m_y1==b.m_y1) && (m_x2==b.m_x2) && (m_y2==b.m_y2); }
    inline bool operator != (const wxBlockInt& b)
        { return !(*this == b); }

    wxInt32 m_x1, m_y1, m_x2, m_y2;
};

//=============================================================================
// wxBlockDouble
//=============================================================================

class WXDLLIMPEXP_THINGS wxBlockDouble
{
public:
    inline wxBlockDouble(wxDouble x1=0, wxDouble y1=0, wxDouble x2=0, wxDouble y2=0)
                      : m_x1(x1), m_y1(y1), m_x2(x2), m_y2(y2) {}
    inline wxBlockDouble(const wxRect2DDouble &rect)
                      { m_x1=rect.m_x; m_y1=rect.m_y; m_x2=rect.GetRight(); m_y2=rect.GetBottom(); }

    inline wxDouble GetLeft() const   { return m_x1; }
    inline wxDouble GetRight() const  { return m_x2; }
    inline wxDouble GetTop() const    { return m_y1; }
    inline wxDouble GetBottom() const { return m_y2; }

    inline wxDouble GetWidth() const  { return m_x2 - m_x1; }
    inline wxDouble GetHeight() const { return m_y2 - m_y1; }

    inline wxPoint2DDouble GetLeftTop() const     { return wxPoint2DDouble(m_x1, m_y1); }
    inline wxPoint2DDouble GetLeftBottom() const  { return wxPoint2DDouble(m_x1, m_y2); }
    inline wxPoint2DDouble GetRightTop() const    { return wxPoint2DDouble(m_x2, m_y1); }
    inline wxPoint2DDouble GetRightBottom() const { return wxPoint2DDouble(m_x2, m_y2); }

    inline wxRect2DDouble GetRect2DDouble() const { return wxRect2DDouble(m_x1, m_y1, m_x2-m_x1, m_y2-m_y1); }
    inline void SetRect2DDouble(const wxRect2DDouble &r) { m_x1=r.m_x; m_y1=r.m_y, m_x2=r.GetRight(); m_y2=r.GetBottom(); }

    inline bool Contains( wxDouble x, wxDouble y ) const
        { return ((x >= m_x1) && (x <= m_x2) && (y >= m_y1) && (y <= m_y2)); }
    inline bool Contains( const wxPoint2DDouble &pt ) const { return Contains(pt.m_x, pt.m_y); }
    inline bool Contains( const wxBlockDouble &b ) const
        { return ((m_x1 <= b.m_x1) && (b.m_x2 <= m_x2) && (m_y1 <= b.m_y1) && (b.m_y2 <= m_y2)); }

    inline bool Intersects( const wxBlockDouble &b ) const
        { return (wxMax(m_x1, b.m_x1)<wxMin(m_x2, b.m_x2)) && (wxMax(m_y1, b.m_y1)<wxMin(m_y2, b.m_y2)); }
    inline void Intersect( const wxBlockDouble &otherBlock ) { Intersect( *this, otherBlock, this ); }
    inline void Intersect( const wxBlockDouble &src1 , const wxBlockDouble &src2 , wxBlockDouble *dest ) const
    {
        dest->m_x1 = wxMax(src1.m_x1, src2.m_x1);
        dest->m_x2 = wxMin(src1.m_x2, src2.m_x2);
        dest->m_y1 = wxMax(src1.m_y1, src2.m_y1);
        dest->m_y2 = wxMin(src1.m_y2, src2.m_y2);
    }

    inline void Union( const wxBlockDouble &otherBlock ) { Union( *this, otherBlock, this ); }
    inline void Union( const wxBlockDouble &src1, const wxBlockDouble &src2, wxBlockDouble *dest ) const
    {
        dest->m_x1 = wxMin(src1.m_x1, src2.m_x1);
        dest->m_x2 = wxMax(src1.m_x2, src2.m_x2);
        dest->m_y1 = wxMin(src1.m_y1, src2.m_y1);
        dest->m_y2 = wxMax(src1.m_y2, src2.m_y2);
    }

    // is this block larger than input block, return 1 - larger, 0 = equal, -1 smaller
    int IsLarger(const wxBlockDouble &b) const;
    inline bool IsEmpty() const { return (m_x1 > m_x2) || (m_y1 > m_y2); }

    // Unlike Intersects this also includes just touching the other block
    bool Touches(const wxBlockDouble &block) const;

    // Try to combine these blocks, they must touch and fit to make a single larger block
    //    this block is expanded if possible
    bool Combine(const wxBlockDouble &block);

    // test combining the input block with this one, returning the
    // remainder of block in top, bottom, left, right - each may be IsEmpty()
    // returns false if blocks don't touch or this block already contains block
    // |---------------------------|
    // |           top             |
    // |---------------------------|
    // |   left  |block|  right    |
    // |---------------------------|
    // |          bottom           |
    // |---------------------------|
    bool Combine( const wxBlockDouble &block,
                  wxBlockDouble &top, wxBlockDouble &bottom,
                  wxBlockDouble &left, wxBlockDouble &right) const;

    // test removal of a portion or all of this contained in block returning the
    // remainder in top, bottom, left, right - each may be IsEmpty()
    // returns false if nothing to delete, this cell is not changed
    bool Delete( const wxBlockDouble &block,
                 wxBlockDouble &top,  wxBlockDouble &bottom,
                 wxBlockDouble &left, wxBlockDouble &right) const;

    // operators
    inline bool operator == (const wxBlockDouble& b)
        { return (m_x1==b.m_x1) && (m_y1==b.m_y1) && (m_x2==b.m_x2) && (m_y2==b.m_y2); }
    inline bool operator != (const wxBlockDouble& b)
        { return !(*this == b); }

    wxDouble m_x1, m_y1, m_x2, m_y2;
};

//=============================================================================
// wxBlockIntSelection - ordered 2D array of wxBlockInts, combines to minimize size
//                       blocks never overlap each other
//=============================================================================

class WXDLLIMPEXP_THINGS wxBlockIntSelection
{
public :
    wxBlockIntSelection(wxBlockSort_Type sort_type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT)
                        : m_sort(sort_type) {}
    wxBlockIntSelection(const wxBlockIntSelection &blocks,
                        wxBlockSort_Type sort_type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT)
                        : m_sort(sort_type) { Copy(blocks); }

    // Make a full copy of the source
    void Copy(const wxBlockIntSelection &source)
        {
            m_blocks.Clear();
            WX_APPEND_ARRAY(m_blocks, source.GetBlockArray());
            m_sort = source.GetSortType();
        }

    inline int GetCount() const { return m_blocks.GetCount(); }

    inline void Clear() { m_blocks.Clear(); }

    wxArrayBlockInt GetBlockArray() const { return m_blocks; }

#ifdef USE_wxRANGE
    // Get an array of ranges cutting though these blocks
    wxArrayRangeInt GetBlockCol(int col) const;
    wxArrayRangeInt GetBlockRow(int row) const;
#endif

    wxBlockInt GetBlock( int index ) const;
    inline wxBlockInt Item( int index ) const { return GetBlock(index); }

    // Get a block that bounds the selection
    wxBlockInt GetBoundingBlock() const;

    // do any of the blocks contains elements
    inline bool Contains( int x, int y ) const { return Index(x,y) != wxNOT_FOUND; }
    inline bool Contains( const wxPoint2DInt &pt ) const { return Index(pt) != wxNOT_FOUND; }
    inline bool Contains( const wxBlockInt &b ) const { return Index(b) != wxNOT_FOUND; }

    // what is the index of a block that contains element
    int Index( int x, int y ) const;
    inline int Index( const wxPoint2DInt &pt ) const { return Index(pt.m_x, pt.m_y); }
    int Index( const wxBlockInt &b ) const;

    // Sorts the blocks according to the wxBlockIntSort_Type
    void Sort(wxBlockSort_Type type = wxBLOCKSORT_TOPRIGHT_BOTTOMLEFT);
    wxBlockSort_Type GetSortType() const { return m_sort; }

    // Add the block to the selection, returns false if nothing was done
    //   use combineNow=false to make quick additions, when done call Minimize()
    //   addedBlocks (if !nullptr) will be filled with the actual changed selections
    //   by removing the previous selections from the input block
    bool SelectBlock( const wxBlockInt &block, bool combineNow=true, wxArrayBlockInt *addedBlocks=nullptr);

    // Remove the block to the selection, return false if nothing was done
    //   use combineNow=false to make quick additions, when done call Minimize()
    bool DeselectBlock( const wxBlockInt &block, bool combineNow=true);

    // Try to combine the blocks if possible, returns if anything was done
    //   only need to call this if you've called (De)SelectBlock(..., false)
    bool Minimize();

    // Operators
    inline wxBlockInt operator[](int index) const { return GetBlock(index); }

    wxBlockIntSelection& operator = (const wxBlockIntSelection& other) { Copy(other); return *this; }

    // generic routine using if (b1.Combine(b2)) remove b2 to cleanup array
    //   sort top_left_bottom_right first (internal use)
    static bool DoMinimize( wxArrayBlockInt &blocks );
    // DoMinimize calls this internally
    static bool DoDoMinimize( wxArrayBlockInt &blocks );

protected :
    wxArrayBlockInt m_blocks;
    wxBlockSort_Type m_sort;
};

//=============================================================================
// wxBlockDoubleSelection - ordered 2D array of wxBlockDoubles, combines to minimze size
//                          blocks never overlap each other
//=============================================================================

class WXDLLIMPEXP_THINGS wxBlockDoubleSelection
{
public :
    wxBlockDoubleSelection(wxBlockSort_Type sort_type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT)
                           : m_sort(sort_type) {}
    wxBlockDoubleSelection(const wxBlockDoubleSelection &blocks,
                           wxBlockSort_Type sort_type = wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT)
                           : m_sort(sort_type) { Copy(blocks); }

    // Make a full copy of the source
    void Copy(const wxBlockDoubleSelection &source)
        {
            m_blocks.Clear();
            WX_APPEND_ARRAY(m_blocks, source.GetBlockArray());
            m_sort = source.GetSortType();
        }

    inline int GetCount() const { return m_blocks.GetCount(); }

    inline void Clear() { m_blocks.Clear(); }

    wxArrayBlockDouble GetBlockArray() const { return m_blocks; }

#ifdef USE_wxRANGE
    // Get an array of ranges cutting though these blocks
    wxArrayRangeDouble GetBlockCol(wxDouble col) const;
    wxArrayRangeDouble GetBlockRow(wxDouble row) const;
#endif

    wxBlockDouble GetBlock( int index ) const;
    inline wxBlockDouble Item( int index ) const { return GetBlock(index); }

    // Get a block that bounds the selection
    wxBlockDouble GetBoundingBlock() const;

    // do any of the blocks contains elements
    inline bool Contains( wxDouble x, wxDouble y ) const { return Index(wxPoint2DDouble(x,y)) != wxNOT_FOUND; }
    inline bool Contains( const wxPoint2DInt &pt ) const { return Index(pt) != wxNOT_FOUND; }
    inline bool Contains( const wxBlockDouble &b ) const { return Index(b) != wxNOT_FOUND; }

    // what is the index of a block that contains element
    int Index( wxDouble x, wxDouble y ) const;
    inline int Index( const wxPoint2DDouble &pt ) const { return Index(pt.m_x, pt.m_y); }
    int Index( const wxBlockDouble &b ) const;

    // Sorts the blocks according to the wxBlockIntSort_Type
    void Sort(wxBlockSort_Type type = wxBLOCKSORT_TOPRIGHT_BOTTOMLEFT);
    wxBlockSort_Type GetSortType() const { return m_sort; }

    // Add the block to the selection, returns false if nothing was done
    //   use combineNow=false to make quick additions, when done call Minimize()
    bool SelectBlock( const wxBlockDouble &block, bool combineNow=true);

    // Remove the block to the selection, return false if nothing was done
    //   use combineNow=false to make quick additions, when done call Minimize()
    bool DeselectBlock( const wxBlockDouble &block, bool combineNow=true);

    // Try to combine the blocks if possible, returns if anything was done
    //   only need to call this if you've called (De)SelectBlock(..., false)
    bool Minimize();

    // Operators
    inline wxBlockDouble operator[](int index) const { return GetBlock(index); }

    wxBlockDoubleSelection& operator = (const wxBlockDoubleSelection& other) { Copy(other); return *this; }

    // generic routine using if (b1.Combine(b2)) remove b2 to cleanup array
    //   sort top_left_bottom_right first (internal use)
    static bool DoMinimize( wxArrayBlockDouble &blocks );
    // DoMinimize calls this internally
    static bool DoDoMinimize( wxArrayBlockDouble &blocks );

protected :
    wxArrayBlockDouble m_blocks;
    wxBlockSort_Type m_sort;
};

//=============================================================================
// wxBlockIntSelectionIterator - iterates through a wxBlockIntSelection
//=============================================================================
enum wxBLOCKINT_SELITER_Type
{
    wxBLOCKINT_SELITER_POINT, // wxBlockIntSelectionIterator::SetType go point by point
    wxBLOCKINT_SELITER_BLOCK  //                                      go block by block
};

class WXDLLIMPEXP_THINGS wxBlockIntSelectionIterator
{
public :
    wxBlockIntSelectionIterator( const wxBlockIntSelection &sel, wxBLOCKINT_SELITER_Type type = wxBLOCKINT_SELITER_POINT );
    wxBlockIntSelectionIterator( const wxArrayBlockInt &blocks, wxBLOCKINT_SELITER_Type type = wxBLOCKINT_SELITER_POINT );

    // resets the iterating to start at the beginning
    void Reset();
    // Set the method to get the blocks, either point by point or each whole block
    //   also resets the iteration to the beginning
    void SetType( wxBLOCKINT_SELITER_Type type ) { m_type = type; Reset(); }
    wxBLOCKINT_SELITER_Type GetType() const { return m_type; }

    // Get next selection, returns false if at end (only valid for wxBLOCKINT_SELITER_POINT)
    bool GetNext(wxPoint2DInt &pt);
    // Get next selection, returns false if at end (only valid for wxBLOCKINT_SELITER_BLOCK)
    bool GetNext(wxBlockInt &block);

    // checks if this row and col are in this selection
    bool IsInSelection(const wxPoint2DInt &pt) const;
    inline bool IsInSelection( int x, int y ) const { return IsInSelection(wxPoint2DInt(x,y)); }

protected :
    wxBLOCKINT_SELITER_Type m_type;
    int m_block_index;
    wxPoint2DInt m_pt;
    wxArrayBlockInt m_blocks;
};

//=============================================================================
// wxBlockDoubleSelectionIterator - iterates through a wxBlockDoubleSelection
//=============================================================================

class WXDLLIMPEXP_THINGS wxBlockDoubleSelectionIterator
{
public :
    wxBlockDoubleSelectionIterator( const wxBlockDoubleSelection &sel );
    wxBlockDoubleSelectionIterator( const wxArrayBlockDouble &blocks );

    // resets the iterating to start at the beginning
    void Reset();
    // Get next selection, returns false if at the end
    bool GetNext(wxBlockDouble &block);
    // checks if this row and col are in this selection
    bool IsInSelection(const wxPoint2DDouble &pt) const;
    inline bool IsInSelection( wxDouble x, wxDouble y ) const { return IsInSelection(wxPoint2DDouble(x,y)); }

protected :
    size_t m_block_index;
    wxArrayBlockDouble m_blocks;
};

#endif // __wxBLOCK_H__
