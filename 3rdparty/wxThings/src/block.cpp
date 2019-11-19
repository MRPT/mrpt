/////////////////////////////////////////////////////////////////////////////
// Name:        block.cpp
// Purpose:     Rectangular selection storage classes for ints and doubles
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   (c) John Labenski 2004
// Licence:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    //#include "wx/object.h"
#endif // WX_PRECOMP

#include "wx/things/block.h"

// use this to check to see if there is any overlap after minimizing
//#define CHECK_BLOCK_OVERLAP 1

#define PRINT_BLOCK(msg, b) { wxPrintf(wxT("Block '%s' %lg %lg %lg %lg\n"), msg, (double)(b).m_x1, (double)(b).m_y1, (double)(b).m_x2, (double)(b).m_y2); }

wxBlockInt const wxEmptyBlockInt(0, 0, -1, -1);
wxBlockDouble const wxEmptyBlockDouble(0, 0, -1, -1);

#include "wx/arrimpl.cpp"
WX_DEFINE_OBJARRAY(wxArrayBlockInt);
WX_DEFINE_OBJARRAY(wxArrayBlockDouble);
WX_DEFINE_OBJARRAY(wxArrayBlockIntSelection);
WX_DEFINE_OBJARRAY(wxArrayBlockDoubleSelection);

// ----------------------------------------------------------------------------
// Sorting functions for wxBlockInt
// ----------------------------------------------------------------------------

static int wxCMPFUNC_CONV wxblockint_sort_topleft_bottomright( wxBlockInt **a, wxBlockInt **b)
{
    int y = ((*a)->m_y1 - (*b)->m_y1);

    if (y < 0) return -1;
    if (y == 0) return  ((*a)->m_x1 - (*b)->m_x1);
    return 1;
}
static int wxCMPFUNC_CONV wxblockint_sort_topright_bottomleft( wxBlockInt **a, wxBlockInt **b)
{
    int y = ((*a)->m_y1 - (*b)->m_y1);

    if (y < 0) return -1;
    if (y == 0) return  ((*a)->m_x2 - (*b)->m_x2);
    return 1;
}

static int wxCMPFUNC_CONV wxblockint_sort_bottomleft_topright( wxBlockInt **a, wxBlockInt **b)
{
    int y = ((*a)->m_y2 - (*b)->m_y2);

    if (y > 0) return -1;
    if (y == 0) return  ((*a)->m_x1 - (*b)->m_x1);
    return 1;
}
static int wxCMPFUNC_CONV wxblockint_sort_bottomright_topleft( wxBlockInt **a, wxBlockInt **b)
{
    int y = ((*a)->m_y2 - (*b)->m_y2);

    if (y > 0) return -1;
    if (y == 0) return  ((*a)->m_x2 - (*b)->m_x2);
    return 1;
}
static int wxCMPFUNC_CONV wxblockint_sort_largest_to_smallest( wxBlockInt **a, wxBlockInt **b)
{
    return (*a)->IsLarger(**b);
}
static int wxCMPFUNC_CONV wxblockint_sort_smallest_to_largest( wxBlockInt **a, wxBlockInt **b)
{
    return -(*a)->IsLarger(**b);
}

void wxArrayBlockIntSort(wxArrayBlockInt &blocks, wxBlockSort_Type type)
{
    switch (type)
    {
        case wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT : blocks.Sort(wxblockint_sort_topleft_bottomright); break;
        case wxBLOCKSORT_TOPRIGHT_BOTTOMLEFT : blocks.Sort(wxblockint_sort_topright_bottomleft); break;
        case wxBLOCKSORT_BOTTOMLEFT_TOPRIGHT : blocks.Sort(wxblockint_sort_bottomleft_topright); break;
        case wxBLOCKSORT_BOTTOMRIGHT_TOPLEFT : blocks.Sort(wxblockint_sort_bottomright_topleft); break;
        case wxBLOCKSORT_SMALLEST_TO_LARGEST : blocks.Sort(wxblockint_sort_smallest_to_largest); break;
        case wxBLOCKSORT_LARGEST_TO_SMALLEST : blocks.Sort(wxblockint_sort_largest_to_smallest); break;
        default : wxFAIL_MSG(wxT("unknown block sort type"));
    }
}

// ----------------------------------------------------------------------------
// Sorting functions for wxBlockDouble
// ----------------------------------------------------------------------------

static int wxCMPFUNC_CONV wxblockdouble_sort_topleft_bottomright( wxBlockDouble **a, wxBlockDouble **b)
{
    wxDouble y = ((*a)->m_y1 - (*b)->m_y1);

    if (y < 0) return -1;
    if (y == 0) return  int((*a)->m_x1 - (*b)->m_x1);
    return 1;
}
static int wxCMPFUNC_CONV wxblockdouble_sort_topright_bottomleft( wxBlockDouble **a, wxBlockDouble **b)
{
    wxDouble y = ((*a)->m_y1 - (*b)->m_y1);

    if (y < 0) return -1;
    if (y == 0) return  int((*a)->m_x2 - (*b)->m_x2);
    return 1;
}

static int wxCMPFUNC_CONV wxblockdouble_sort_bottomleft_topright( wxBlockDouble **a, wxBlockDouble **b)
{
    wxDouble y = ((*a)->m_y2 - (*b)->m_y2);

    if (y > 0) return -1;
    if (y == 0) return  int((*a)->m_x1 - (*b)->m_x1);
    return 1;
}
static int wxCMPFUNC_CONV wxblockdouble_sort_bottomright_topleft( wxBlockDouble **a, wxBlockDouble **b)
{
    wxDouble y = ((*a)->m_y2 - (*b)->m_y2);

    if (y > 0) return -1;
    if (y == 0) return  int((*a)->m_x2 - (*b)->m_x2);
    return 1;
}
static int wxCMPFUNC_CONV wxblockdouble_sort_largest_to_smallest( wxBlockDouble **a, wxBlockDouble **b)
{
    return (*a)->IsLarger(**b);
}
static int wxCMPFUNC_CONV wxblockdouble_sort_smallest_to_largest( wxBlockDouble **a, wxBlockDouble **b)
{
    return -(*a)->IsLarger(**b);
}

void wxArrayBlockDoubleSort(wxArrayBlockDouble &blocks, wxBlockSort_Type type)
{
    switch (type)
    {
        case wxBLOCKSORT_TOPLEFT_BOTTOMRIGHT : blocks.Sort(wxblockdouble_sort_topleft_bottomright); break;
        case wxBLOCKSORT_TOPRIGHT_BOTTOMLEFT : blocks.Sort(wxblockdouble_sort_topright_bottomleft); break;
        case wxBLOCKSORT_BOTTOMLEFT_TOPRIGHT : blocks.Sort(wxblockdouble_sort_bottomleft_topright); break;
        case wxBLOCKSORT_BOTTOMRIGHT_TOPLEFT : blocks.Sort(wxblockdouble_sort_bottomright_topleft); break;
        case wxBLOCKSORT_SMALLEST_TO_LARGEST : blocks.Sort(wxblockdouble_sort_smallest_to_largest); break;
        case wxBLOCKSORT_LARGEST_TO_SMALLEST : blocks.Sort(wxblockdouble_sort_largest_to_smallest); break;
        default : wxFAIL_MSG(wxT("unknown block sort type"));
    }
}

//=============================================================================
// wxBlockInt
//=============================================================================

#define TEST_BLOCKS

#ifdef TEST_BLOCKS
void TestBlocks()
{
    printf("Start Testing blocks -----------------------------------------\n");
    wxBlockInt b1(1,1,4,4);
    wxBlockInt b2(5,4,10,11);
    PRINT_BLOCK("b1", b1)
    PRINT_BLOCK("b2", b2)

    wxBlockInt iB;
    iB.Intersect(b1, b2, &iB);
    PRINT_BLOCK("Intersect b1 b2", iB)

    wxBlockInt uB;
    uB.Union(b1, b2, &uB);
    PRINT_BLOCK("Union b1 b2", uB)

    printf("Touches b1 b2 %d %d\n", b1.Touches(b2), b2.Touches(b1));

    b1 = wxBlockInt(2,3,7,9);
    b2 = wxBlockInt(8,3,8,3);
    printf("Touches b1 b2 %d %d\n", b1.Touches(b2), b2.Touches(b1));

    b1 = wxBlockInt(2,3,7,9);
    b2 = wxBlockInt(1,3,1,3);
    printf("Touches b1 b2 %d %d\n", b1.Touches(b2), b2.Touches(b1));
    iB.Intersect(b1, b2, &iB);
    PRINT_BLOCK("Intersect b1 b2", iB)

    b1 = wxBlockInt(2,3,7,9);
    b2 = wxBlockInt(2,2,2,2);
    printf("Touches b1 b2 %d %d\n", b1.Touches(b2), b2.Touches(b1));

    b1 = wxBlockInt(2,3,7,9);
    b2 = wxBlockInt(7,10,7,10);
    printf("Touches b1 b2 %d %d\n", b1.Touches(b2), b2.Touches(b1));

    printf("End Testing blocks -----------------------------------------\n");
    fflush(stdout);
}
#endif //TEST_BLOCKS

int wxBlockInt::IsLarger(const wxBlockInt &b) const
{
    wxInt32 width    = m_x2 - m_x1 + 1,
            height   = m_y2 - m_y1 + 1,
            b_width  = b.m_x2 - b.m_x1 + 1,
            b_height = b.m_y2 - b.m_y1 + 1;

    if ((width <= 0) || (height <= 0))
        return (b_width > 0) && (b_height > 0) ? -1 : 0;
    if ((b_width <= 0) || (b_height <= 0))
        return (width > 0) && (height > 0) ? 1 : 0;

    wxDouble w_bw = wxDouble(width)/b_width,
             bh_h = wxDouble(b_height)/height;

    return (w_bw == bh_h) ? 0 : ((w_bw > bh_h) ? 1 : -1);
}

bool wxBlockInt::Touches(const wxBlockInt &b) const // see Intersects
{
    //if (((wxMax(m_x1, b.m_x1)) <= (wxMin(m_x2, b.m_x2))) &&
    //    ((wxMax(m_y1, b.m_y1)) <= (wxMin(m_y2, b.m_y2))))
    //    return true;

    return Intersects(wxBlockInt(b.m_x1-1, b.m_y1-1, b.m_x2+1, b.m_y2+1));

/*
    wxInt32 left  = wxMax( m_x1, b.m_x1 );
    wxInt32 right = wxMin( m_x2, b.m_x2 );

    if (labs(left - right) <= 1)
    {
        wxInt32 top    = wxMax( m_y1, b.m_y1 );
        wxInt32 bottom = wxMin( m_y2, b.m_y2 );
        if (labs(top - bottom) <= 1)
            return true;
    }
    return false;
*/
}

bool wxBlockInt::Combine(const wxBlockInt &b)
{
    if (!Touches(b)) return false;
    if (Contains(b)) return true;
    if (b.Contains(*this))
    {
        *this = b;
        return true;
    }

    wxBlockInt unionBlock;
    Union( *this, b, &unionBlock );

    if (unionBlock.IsEmpty()) return false;

    // at least one of the two blocks has to be at each corner of the union
    if (((unionBlock.GetLeftTop() == GetLeftTop()) || (unionBlock.GetLeftTop() == b.GetLeftTop())) &&
        ((unionBlock.GetRightTop() == GetRightTop()) || (unionBlock.GetRightTop() == b.GetRightTop())) &&
        ((unionBlock.GetLeftBottom() == GetLeftBottom()) || (unionBlock.GetLeftBottom() == b.GetLeftBottom())) &&
        ((unionBlock.GetRightBottom() == GetRightBottom()) || (unionBlock.GetRightBottom() == b.GetRightBottom())) )
    {
        *this = unionBlock;
        return true;
    }

    return false;
}

bool wxBlockInt::Combine( const wxBlockInt &block,
                          wxBlockInt &top, wxBlockInt &bottom,
                          wxBlockInt &left, wxBlockInt &right) const
{
    top = bottom = left = right = wxEmptyBlockInt;

    wxBlockInt iBlock;
    Intersect(*this, block, &iBlock);

    if (iBlock.IsEmpty()) return false; // nothing to combine
    if (iBlock == *this) return true;   // can combine all of this, no leftover

    bool combined = false;

    if ( block.m_y1 < m_y1 )
    {
        top = wxBlockInt( block.m_x1, block.m_y1, block.m_x2, m_y1-1 );
        combined = true;
    }
    if ( block.m_y2 > m_y2 )
    {
        bottom = wxBlockInt( block.m_x1, m_y2+1, block.m_x2, block.m_y2 );
        combined = true;
    }
    if ( block.m_x1 < m_x1 )
    {
        left = wxBlockInt( block.m_x1, iBlock.m_y1, m_x1-1, iBlock.m_y2 );
        combined = true;
    }
    if ( block.m_x2 > m_x2 )
    {
        right = wxBlockInt( m_x2+1, iBlock.m_y1, block.m_x2, iBlock.m_y2 );
        combined = true;
    }

    return combined;
}

bool wxBlockInt::Delete( const wxBlockInt &block,
                         wxBlockInt &top, wxBlockInt &bottom,
                         wxBlockInt &left, wxBlockInt &right) const
{
    top = bottom = left = right = wxEmptyBlockInt;

    wxBlockInt iBlock;
    Intersect(*this, block, &iBlock);

    if (iBlock.IsEmpty()) return false; // nothing to delete
    if (iBlock == *this) return true;   // can delete all of this, no leftover

    bool deleted = false;

    if ( m_y1 < iBlock.m_y1 )
    {
        top = wxBlockInt( m_x1, m_y1, m_x2, iBlock.m_y1-1 );
        deleted = true;
    }
    if ( GetBottom() > iBlock.GetBottom() )
    {
        bottom = wxBlockInt( m_x1, iBlock.m_y2+1, m_x2, m_y2 );
        deleted = true;
    }
    if ( m_x1 < iBlock.m_x1 )
    {
        left = wxBlockInt( m_x1, iBlock.m_y1, iBlock.m_x1-1, iBlock.m_y2 );
        deleted = true;
    }
    if ( GetRight() > iBlock.GetRight() )
    {
        right = wxBlockInt( iBlock.m_x2+1, iBlock.m_y1, m_x2, iBlock.m_y2 );
        deleted = true;
    }

    return deleted;
}

//=============================================================================
// wxBlockDouble
//=============================================================================

int wxBlockDouble::IsLarger(const wxBlockDouble &b) const
{
    wxDouble width    = m_x2 - m_x1,
             height   = m_y2 - m_y1,
             b_width  = b.m_x2 - b.m_x1,
             b_height = b.m_y2 - b.m_y1;

    if ((width <= 0) || (height <= 0))
        return (b_width > 0) && (b_height > 0) ? -1 : 0;
    if ((b_width <= 0) || (b_height <= 0))
        return (width > 0) && (height > 0) ? 1 : 0;

    wxDouble w_bw = width/b_width,
             bh_h = b_height/height;
    return (w_bw == bh_h) ? 0 : ((w_bw > bh_h) ? 1 : -1);
}

bool wxBlockDouble::Touches(const wxBlockDouble &b) const // see Intersects
{
    if (((wxMax(m_x1, b.m_x1)) <= (wxMin(m_x2, b.m_x2))) &&
        ((wxMax(m_y1, b.m_y1)) <= (wxMin(m_y2, b.m_y2))))
        return true;

    return false;
}

bool wxBlockDouble::Combine(const wxBlockDouble &b)
{
    if (!Touches(b)) return false;
    if (Contains(b)) return true;
    if (b.Contains(*this))
    {
        *this = b;
        return true;
    }

    wxBlockDouble unionBlock;
    Union( *this, b, &unionBlock );

    if (unionBlock.IsEmpty()) return false;

    // at least one of the two blocks has to be at each corner of the union
    if (((unionBlock.GetLeftTop() == GetLeftTop()) || (unionBlock.GetLeftTop() == b.GetLeftTop())) &&
        ((unionBlock.GetRightTop() == GetRightTop()) || (unionBlock.GetRightTop() == b.GetRightTop())) &&
        ((unionBlock.GetLeftBottom() == GetLeftBottom()) || (unionBlock.GetLeftBottom() == b.GetLeftBottom())) &&
        ((unionBlock.GetRightBottom() == GetRightBottom()) || (unionBlock.GetRightBottom() == b.GetRightBottom())) )
    {
        *this = unionBlock;
        return true;
    }

    return false;
}

bool wxBlockDouble::Combine( const wxBlockDouble &block,
                             wxBlockDouble &top, wxBlockDouble &bottom,
                             wxBlockDouble &left, wxBlockDouble &right) const
{
    top = bottom = left = right = wxEmptyBlockDouble;

    wxBlockDouble iBlock;
    Intersect(*this, block, &iBlock);

    if (iBlock.IsEmpty()) return false; // nothing to combine
    if (iBlock == *this) return true;   // can combine all of this, no leftover

    bool combined = false;

    if ( block.m_y1 < m_y1 )
    {
        top = wxBlockDouble( block.m_x1, block.m_y1, block.m_x2, m_y1 );
        combined = true;
    }
    if ( block.m_y2 > m_y2 )
    {
        bottom = wxBlockDouble( block.m_x1, m_y2, block.m_x2, block.m_y2 );
        combined = true;
    }
    if ( block.m_x1 < m_x1 )
    {
        left = wxBlockDouble( block.m_x1, iBlock.m_y1, m_x1, iBlock.m_y2 );
        combined = true;
    }
    if ( block.m_x2 > m_x2 )
    {
        right = wxBlockDouble( m_x2, iBlock.m_y1, block.m_x2, iBlock.m_y2 );
        combined = true;
    }

    return combined;
}

bool wxBlockDouble::Delete( const wxBlockDouble &block,
                            wxBlockDouble &top, wxBlockDouble &bottom,
                            wxBlockDouble &left, wxBlockDouble &right) const
{
    top = bottom = left = right = wxEmptyBlockDouble;

    wxBlockDouble iBlock;
    Intersect(*this, block, &iBlock);

    if (iBlock.IsEmpty()) return false; // nothing to delete
    if (iBlock == *this) return true;   // can delete all of this, no leftover

    bool deleted = false;

    if ( m_y1 < iBlock.m_y1 )
    {
        top = wxBlockDouble( m_x1, m_y1, m_x2, iBlock.m_y1 );
        deleted = true;
    }
    if ( m_y2 > iBlock.m_y2 )
    {
        bottom = wxBlockDouble( m_x1, iBlock.m_y2, m_x2, m_y2 );
        deleted = true;
    }
    if ( m_x1 < iBlock.m_x1 )
    {
        left = wxBlockDouble( m_x1, iBlock.m_y1, iBlock.m_x1, iBlock.m_y2 );
        deleted = true;
    }
    if ( m_x2 > iBlock.m_x2 )
    {
        right = wxBlockDouble( iBlock.m_x2, iBlock.m_y1, m_x2, iBlock.m_y2 );
        deleted = true;
    }

    return deleted;
}

//=============================================================================
// wxBlockIntSelection
//=============================================================================
wxBlockInt wxBlockIntSelection::GetBlock( int index ) const
{
    wxCHECK_MSG((index>=0) && (index<int(m_blocks.GetCount())), wxEmptyBlockInt, wxT("Invalid index"));
    return m_blocks[index];
}

#ifdef USE_wxRANGE
wxArrayRangeInt wxBlockIntSelection::GetBlockCol(int col) const
{
    wxArrayRangeInt ranges;
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ((col >= m_blocks[n].m_x1) && (col <= m_blocks[n].m_x2))
        {
            wxRangeInt range(m_blocks[n].m_y1, m_blocks[n].m_y2);
            ranges.Add(range);
        }
    }
    return ranges;
}

wxArrayRangeInt wxBlockIntSelection::GetBlockRow(int row) const
{
    wxArrayRangeInt ranges;
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ((row >= m_blocks[n].m_y1) && (row <= m_blocks[n].m_y2))
            ranges.Add(wxRangeInt(m_blocks[n].m_x1, m_blocks[n].m_x2));
    }
    return ranges;
}
#endif // USE_wxRANGE

wxBlockInt wxBlockIntSelection::GetBoundingBlock() const
{
    int n, count = m_blocks.GetCount();
    if (count == 0) return wxEmptyBlockInt;
    wxBlockInt bound = m_blocks[0];
    for (n=1; n<count; n++) bound.Union(m_blocks[n]);
    return bound;
}

int wxBlockIntSelection::Index( int x, int y ) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ( m_blocks[n].Contains(x, y) )
            return n;
    }
    return wxNOT_FOUND;
}

int wxBlockIntSelection::Index( const wxBlockInt &b ) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if (m_blocks[n].Intersects(b))
            return n;
    }
    return wxNOT_FOUND;
}

void wxBlockIntSelection::Sort(wxBlockSort_Type type)
{
    m_sort = type;
    wxArrayBlockIntSort(m_blocks, type);
}

bool wxBlockIntSelection::DeselectBlock( const wxBlockInt &block, bool combineNow)
{
    wxCHECK_MSG(!block.IsEmpty(), false, wxT("Invalid block") );

    bool done = false;

    wxBlockInt top, bottom, left, right;
    for (int n=0; n<int(m_blocks.GetCount()); n++)
    {
        if (m_blocks[n].Delete(block, top, bottom, left, right))
        {
            done = true;
            m_blocks.RemoveAt(n);
            n = (n > 0) ? n - 1 : -1;

            if (!top.IsEmpty())    m_blocks.Add(top);
            if (!bottom.IsEmpty()) m_blocks.Add(bottom);
            if (!left.IsEmpty())   m_blocks.Add(left);
            if (!right.IsEmpty())  m_blocks.Add(right);
        }
    }

    if (combineNow)
        Minimize();

    return done;
}

bool wxBlockIntSelection::SelectBlock( const wxBlockInt &block, bool combineNow,
                                       wxArrayBlockInt *addedBlocks )
{
    wxCHECK_MSG(!block.IsEmpty(), false, wxT("Invalid block") );

    //TestBlocks();

    wxArrayBlockInt extraBlocks;
    wxArrayBlockInt *extra = &extraBlocks;

    if (addedBlocks != nullptr)
    {
        addedBlocks->Clear();
        extra = addedBlocks;
    }

    extra->Add(block);

    int n, count = m_blocks.GetCount();
    wxBlockInt top, bottom, left, right;

    for (n=0; n<count; n++)
    {
        for (int k=0; k<int(extra->GetCount()); k++)
        {
            if (m_blocks[n].Combine(extra->Item(k), top, bottom, left, right))
            {
                extra->RemoveAt(k);
                if (!top.IsEmpty())    extra->Add(top);
                if (!bottom.IsEmpty()) extra->Add(bottom);
                if (!left.IsEmpty())   extra->Add(left);
                if (!right.IsEmpty())  extra->Add(right);
                //DoMinimize( *extra );
                n = -1;
                break;
            }
        }
    }

    if (extra->GetCount() > 0u)
    {
        WX_APPEND_ARRAY(m_blocks, *extra);
        if (combineNow)
            Minimize();

        return true;
    }

    return false;
}

bool wxBlockIntSelection::Minimize()
{
    bool ret = DoMinimize(m_blocks);
    Sort(m_sort);
    return ret;
}

bool wxBlockIntSelection::DoMinimize(wxArrayBlockInt &blocks)
{
    int n;
    for (n=0; n<1000; n++) // should probably just take a few
    {
        if (!DoDoMinimize(blocks)) break;
    }

#ifdef CHECK_BLOCK_OVERLAP
    for (size_t a=0; a<blocks.GetCount(); a++)
    {
        for (size_t b=a+1; b<blocks.GetCount(); b++)
        {
            if (blocks[a].Intersects(blocks[b]))
            {
                printf("Intersecting blocks in wxBlockIntSelection::DoMinimize"); fflush(stdout);
                wxBell();
            }
        }
    }
#endif

    return n != 0;
}

bool wxBlockIntSelection::DoDoMinimize(wxArrayBlockInt &blocks)
{
//    wxBlockInt top, bottom, left, right;
    bool done = false;
    for (int i=0; i<int(blocks.GetCount())-1; i++)
    {
        for (int j=i+1; j<int(blocks.GetCount()); j++)
        {
            if (blocks[i].Combine(blocks[j]))
            {
                blocks.RemoveAt(j);
                j--;
                done = true;
                //return true;
            }
/*
            else if (blocks[i].Combine(blocks[j], top, bottom, left, right))
            {
                printf("INTERSECTION!?---------------------------\n"); fflush(stdout);
                blocks.RemoveAt(j);
                if (!top.IsEmpty())    blocks.Add(top);
                if (!bottom.IsEmpty()) blocks.Add(bottom);
                if (!left.IsEmpty())   blocks.Add(left);
                if (!right.IsEmpty())  blocks.Add(right);
                return true;
            }
*/
        }
    }
    return done;
}

//=============================================================================
// wxBlockDoubleSelection
//=============================================================================
wxBlockDouble wxBlockDoubleSelection::GetBlock( int index ) const
{
    wxCHECK_MSG((index>=0) && (index<int(m_blocks.GetCount())), wxEmptyBlockDouble, wxT("Invalid index"));
    return m_blocks[index];
}

#ifdef USE_wxRANGE
wxArrayRangeDouble wxBlockDoubleSelection::GetBlockCol(wxDouble col) const
{
    wxArrayRangeDouble ranges;
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ((col >= m_blocks[n].m_x1) && (col <= m_blocks[n].m_x2))
        {
            wxRangeDouble range(m_blocks[n].m_y1, m_blocks[n].m_y2);
            ranges.Add(range);
        }
    }
    return ranges;
}

wxArrayRangeDouble wxBlockDoubleSelection::GetBlockRow(wxDouble row) const
{
    wxArrayRangeDouble ranges;
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ((row >= m_blocks[n].m_y1) && (row <= m_blocks[n].m_y2))
            ranges.Add(wxRangeDouble(m_blocks[n].m_x1, m_blocks[n].m_x2));
    }
    return ranges;
}
#endif // USE_wxRANGE

wxBlockDouble wxBlockDoubleSelection::GetBoundingBlock() const
{
    int n, count = m_blocks.GetCount();
    if (count == 0) return wxEmptyBlockDouble;
    wxBlockDouble bound = m_blocks[0];
    for (n=1; n<count; n++) bound.Union(m_blocks[n]);
    return bound;
}

int wxBlockDoubleSelection::Index( wxDouble x, wxDouble y ) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if ( (x >= m_blocks[n].m_x1) && (y >= m_blocks[n].m_y1) &&
             (x <= m_blocks[n].m_x2) && (y <= m_blocks[n].m_y2) )
            return true;
    }
    return wxNOT_FOUND;
}

int wxBlockDoubleSelection::Index( const wxBlockDouble &b ) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if (m_blocks[n].Intersects(b))
            return n;
    }
    return wxNOT_FOUND;
}

void wxBlockDoubleSelection::Sort(wxBlockSort_Type type)
{
    m_sort = type;
    wxArrayBlockDoubleSort(m_blocks, type);
}

bool wxBlockDoubleSelection::DeselectBlock( const wxBlockDouble &block, bool combineNow)
{
    //wxCHECK_MSG(!block.IsEmpty(), false, wxT("Invalid block") );

    bool done = false;

    wxBlockDouble top, bottom, left, right;
    for (int n=0; n<int(m_blocks.GetCount()); n++)
    {
        if (m_blocks[n].Delete(block, top, bottom, left, right))
        {
            done = true;
            m_blocks.RemoveAt(n);
            n = (n > 0) ? n - 1 : -1;

            if (!top.IsEmpty())    m_blocks.Add(top);
            if (!bottom.IsEmpty()) m_blocks.Add(bottom);
            if (!left.IsEmpty())   m_blocks.Add(left);
            if (!right.IsEmpty())  m_blocks.Add(right);
        }
    }

    if (combineNow)
        Minimize();

    return done;
}

bool wxBlockDoubleSelection::SelectBlock( const wxBlockDouble &block, bool combineNow)
{
    // It's valid to select a block with a width and height 0 since that means that point
    //wxCHECK_MSG(!block.IsEmpty(), false, wxT("Invalid block") );

    wxArrayBlockDouble extra;
    extra.Add(block);
    wxBlockDouble top, bottom, left, right;

    for (int n=0; n<int(m_blocks.GetCount()); n++)
    {
        for (int k=0; k<int(extra.GetCount()); k++)
        {
            bool done = false;

            // Doubles are different than ints - roundoff error problems
            // always use the bigger block to soak up the smaller blocks
            // this reduces problems with tiny roundoff error produced blocks
            if (m_blocks[n].Intersects(extra[k]))
            {
                if (m_blocks[n].Contains(extra[k]))
                {
                    extra.RemoveAt(k);
                    k--;
                    continue;
                }
                else if (extra[k].Contains(m_blocks[n]))
                {
                    m_blocks.RemoveAt(n);
                    n = -1;
                    break;
                }
                else if (m_blocks[n].IsLarger(extra[k]) > 0)
                {
                    done = m_blocks[n].Combine(extra[k], top, bottom, left, right);
                    if (done)
                    {
                        extra.RemoveAt(k);
                        k--;
                    }
                }
                else
                {
                    done = extra[k].Combine(m_blocks[n], top, bottom, left, right);
                    if (done)
                    {
                        m_blocks.RemoveAt(n);
                        n = -1;
                    }
                }
            }

            if (done)
            {
                if (!top.IsEmpty())    extra.Add(top);
                if (!bottom.IsEmpty()) extra.Add(bottom);
                if (!left.IsEmpty())   extra.Add(left);
                if (!right.IsEmpty())  extra.Add(right);
                //DoMinimize( extra );
                if (n == -1)
                    break;
            }
        }
    }

    if (extra.GetCount() > 0u)
    {
        WX_APPEND_ARRAY(m_blocks, extra);
        if (combineNow)
            Minimize();

        return true;
    }

    return false;
}

bool wxBlockDoubleSelection::Minimize()
{
    bool ret = DoMinimize(m_blocks);
    Sort(m_sort);
    return ret;
}

bool wxBlockDoubleSelection::DoMinimize(wxArrayBlockDouble &blocks)
{
    int n;
    for (n=0; n<1000; n++) // should probably just take < 10 at most
    {
        if (!DoDoMinimize(blocks)) break;
    }

#ifdef CHECK_BLOCK_OVERLAP
    for (size_t a=0; a<blocks.GetCount(); a++)
    {
        printf("Checking wxBlockDoubleSelection::DoMinimize %d =", a); PRINT_BLOCK("", blocks[a])
        for (size_t b=a+1; b<blocks.GetCount(); b++)
        {
            if (blocks[a].Intersects(blocks[b]))
            {
                printf("Intersecting blocks in wxBlockDoubleSelection::DoMinimize\n"); fflush(stdout);
                PRINT_BLOCK("",blocks[a])
                PRINT_BLOCK("",blocks[b])
                wxBell();
            }
        }
    }
#endif

    return n != 0;
}

bool wxBlockDoubleSelection::DoDoMinimize(wxArrayBlockDouble &blocks)
{
    //wxBlockDouble top, bottom, left, right;
    bool done = false;

    for (int i=0; i<int(blocks.GetCount())-1; i++)
    {
        for (int j=i+1; j<int(blocks.GetCount()); j++)
        {
            if (blocks[i].Combine(blocks[j]))
            {
                blocks.RemoveAt(j);
                done = true;
                j--;
            }
/*
            else if (blocks[i].Combine(blocks[j], top, bottom, left, right))
            {
                blocks.RemoveAt(j);
                if (!top.IsEmpty())    blocks.Add(top);
                if (!bottom.IsEmpty()) blocks.Add(bottom);
                if (!left.IsEmpty())   blocks.Add(left);
                if (!right.IsEmpty())  blocks.Add(right);
                return true;
            }
*/
        }
    }
    return done;
}

//=============================================================================
// wxBlockIntSelectionIterator - iterates through a wxBlockIntSelection
//=============================================================================

wxBlockIntSelectionIterator::wxBlockIntSelectionIterator( const wxBlockIntSelection &sel,
                                                          wxBLOCKINT_SELITER_Type type )
{
    m_type = type;
    WX_APPEND_ARRAY(m_blocks, sel.GetBlockArray());
    m_blocks.Sort(wxblockint_sort_topleft_bottomright);
    Reset();
}

wxBlockIntSelectionIterator::wxBlockIntSelectionIterator( const wxArrayBlockInt &blocks,
                                                          wxBLOCKINT_SELITER_Type type )
{
    m_type = type;
    WX_APPEND_ARRAY(m_blocks, blocks);
    m_blocks.Sort(wxblockint_sort_topleft_bottomright);
    Reset();
}

void wxBlockIntSelectionIterator::Reset()
{
    m_block_index = -1;
    m_pt = wxPoint2DInt(0, 0);
}

bool wxBlockIntSelectionIterator::GetNext(wxBlockInt &block)
{
    wxCHECK_MSG(m_type == wxBLOCKINT_SELITER_BLOCK, false, wxT("wrong selection type"));
    if (m_block_index+1 < int(m_blocks.GetCount()))
    {
        ++m_block_index;
        block = m_blocks[m_block_index];
        return true;
    }

    return false;
}

bool wxBlockIntSelectionIterator::GetNext(wxPoint2DInt &pt)
{
    wxCHECK_MSG(m_type == wxBLOCKINT_SELITER_POINT, false, wxT("wrong selection type"));
    if ((m_blocks.GetCount() < 1u) || (m_block_index >= int(m_blocks.GetCount())))
        return false;

    // first time here
    if (m_block_index < 0)
    {
        m_block_index = 0;
        pt = m_pt = m_blocks[m_block_index].GetLeftTop();
        return true;
    }

    // at end of block swap to new one
    if (m_pt == m_blocks[m_block_index].GetRightBottom())
    {
        ++m_block_index;
        if (int(m_blocks.GetCount()) > m_block_index)
        {
            pt = m_pt = m_blocks[m_block_index].GetLeftTop();
            return true;
        }
        else  // past end nothing more to check
            return  false;
    }
    // at end of col, down to next row
    if (m_pt.m_x == m_blocks[m_block_index].GetRight())
    {
        m_pt.m_x = m_blocks[m_block_index].m_x1;
        m_pt.m_y++;

        pt = m_pt;
        return true;
    }

    // increment the col
    m_pt.m_x++;
    pt = m_pt;

    return true;
}

bool wxBlockIntSelectionIterator::IsInSelection(const wxPoint2DInt &pt) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if (m_blocks[n].Contains(pt))
            return true;
    }
    return false;
}

//=============================================================================
// wxBlockDoubleSelectionIterator - iterates through a wxBlockDoubleSelection
//=============================================================================

wxBlockDoubleSelectionIterator::wxBlockDoubleSelectionIterator( const wxBlockDoubleSelection &sel )
{
    WX_APPEND_ARRAY(m_blocks, sel.GetBlockArray());
    m_blocks.Sort(wxblockdouble_sort_topleft_bottomright);
    Reset();
}

wxBlockDoubleSelectionIterator::wxBlockDoubleSelectionIterator( const wxArrayBlockDouble &blocks )
{
    WX_APPEND_ARRAY(m_blocks, blocks);
    m_blocks.Sort(wxblockdouble_sort_topleft_bottomright);
    Reset();
}

void wxBlockDoubleSelectionIterator::Reset()
{
    m_block_index = 0;
}

bool wxBlockDoubleSelectionIterator::GetNext(wxBlockDouble &block)
{
    if (m_block_index < m_blocks.GetCount())
    {
        block = m_blocks[m_block_index];
        m_block_index++;
        return true;
    }

    return false;
}

bool wxBlockDoubleSelectionIterator::IsInSelection(const wxPoint2DDouble &pt) const
{
    int n, count = m_blocks.GetCount();
    for (n=0; n<count; n++)
    {
        if (m_blocks[n].Contains(pt))
            return true;
    }
    return false;
}

// ============================================================================
// ============================================================================
// ============================================================================
// ============================================================================
// ============================================================================
// Unit testing, sortof
