/////////////////////////////////////////////////////////////////////////////
// Name:        range.cpp
// Purpose:     Simple min-max range class and associated selection array class
// Author:      John Labenski
// Created:     12/01/2000
// Copyright:   (c) John Labenski 2004
// Licence:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/utils.h"
#endif // WX_PRECOMP

#include "wx/things/range.h"
#include <stdio.h>

const wxRangeInt wxEmptyRangeInt(0, -1);
const wxRangeDouble wxEmptyRangeDouble(0, -1);

#include "wx/arrimpl.cpp"
WX_DEFINE_OBJARRAY(wxArrayRangeInt);
WX_DEFINE_OBJARRAY(wxArrayRangeDouble);
WX_DEFINE_OBJARRAY(wxArrayRangeIntSelection);
WX_DEFINE_OBJARRAY(wxArrayRangeDoubleSelection);


// set this if you want to double check that that ranges are really working
//#define CHECK_RANGES

//=============================================================================
// wxRangeInt
//=============================================================================

bool wxRangeInt::Combine(int i, bool only_if_touching)
{
    if (only_if_touching)
    {
        if      (i == m_min-1) { m_min = i; return true; }
        else if (i == m_max+1) { m_max = i; return true; }
    }
    else
    {
        if      (i < m_min) { m_min = i; return true; }
        else if (i > m_max) { m_max = i; return true; }
    }
    return false;
}

bool wxRangeInt::Combine( const wxRangeInt &r, bool only_if_touching )
{
    if (only_if_touching)
    {
        if (Touches(r))
        {
            *this += r;
            return true;
        }
    }
    else if (!IsEmpty() && !r.IsEmpty())
    {
        bool added = false;
        if (r.m_min < m_min) { m_min = r.m_min; added = true; }
        if (r.m_max > m_max) { m_max = r.m_max; added = true; }
        return added;
    }
    return false;
}

bool wxRangeInt::Delete( const wxRangeInt &r, wxRangeInt *right )
{
    if (!Contains(r))
        return false;

    if (right) *right = wxEmptyRangeInt;

    if (r.m_min <= m_min)
    {
        if (r.m_max >= m_max)
        {
            *this = wxEmptyRangeInt;
            return true;
        }

        m_min = r.m_max + 1;
        return true;
    }

    if (r.m_max >= m_max)
    {
        m_max = r.m_min - 1;
        return true;
    }

    if (right)
        *right = wxRangeInt(r.m_max + 1, m_max);

    m_max = r.m_min - 1;
    return true;
}

//=============================================================================
// wxRangeIntSelection
//=============================================================================
const wxRangeInt& wxRangeIntSelection::GetRange( int index ) const
{
    wxCHECK_MSG((index>=0) && (index<int(m_ranges.GetCount())), wxEmptyRangeInt, wxT("Invalid index"));
    return m_ranges[index];
}

wxRangeInt wxRangeIntSelection::GetBoundingRange() const
{
    if (int(m_ranges.GetCount()) < 1) return wxEmptyRangeInt;
    return wxRangeInt(m_ranges[0].m_min, m_ranges[m_ranges.GetCount()-1].m_max);
}

int wxRangeIntSelection::Index( int i ) const
{
    int count = m_ranges.GetCount();
    if (count < 1) return wxNOT_FOUND;

    if (i < m_ranges[0].m_min) return wxNOT_FOUND;
    if (i > m_ranges[count-1].m_max) return wxNOT_FOUND;

    // Binary search
    int res, tmp, lo = 0, hi = count;

    while ( lo < hi )
    {
        tmp = (lo + hi)/2;
        res = m_ranges[tmp].Position(i);

        if (res == 0)
            return tmp;
        else if ( res < 0 )
            hi = tmp;
        else //if ( res > 0 )
            lo = tmp + 1;
    }

    return wxNOT_FOUND;
}

int wxRangeIntSelection::Index( const wxRangeInt &r ) const
{
    int i, count = m_ranges.GetCount();
    for (i=0; i<count; i++) if (m_ranges[i].Contains(r)) return i;
    return wxNOT_FOUND;
}

int wxRangeIntSelection::NearestIndex( int i ) const
{
    int count = m_ranges.GetCount();
    if (count < 1) return -1;

    if (i < m_ranges[0].m_min) return -1;
    if (i > m_ranges[count-1].m_max) return count;

    // Binary search
    int res, tmp, lo = 0, hi = count;

    while ( lo < hi )
    {
        tmp = (lo + hi)/2;
        res = m_ranges[tmp].Position(i);

        if ( res == 0 )
            return tmp;
        else if ((i >= m_ranges[tmp].m_max) && (i < m_ranges[wxMin(tmp+1, count-1)].m_min))
            return tmp;
        else if ( res < 0 )
            hi = tmp;
        else //if ( res > 0 )
            lo = tmp + 1;
    }

    // oops shouldn't get here
    wxCHECK_MSG(0, -1, wxT("Error calculating NearestIndex in wxRangeIntSelection"));
}

int wxRangeIntSelection::GetItemCount() const
{
    int i, items = 0, count = m_ranges.GetCount();
    for (i=0; i<count; i++) items += m_ranges[i].GetRange();
    return items;
}

bool wxRangeIntSelection::DeselectRange(const wxRangeInt &range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Selection Range") );

    bool done = false;
    int i, count = m_ranges.GetCount();
    int nearest = count > 0 ? NearestIndex(range.m_min) : -1;

    if ((nearest < 0) || (nearest == count))
        return false;

    wxRangeInt r;
    for (i=nearest; i<int(m_ranges.GetCount()); i++)
    {
        if (range.m_max < m_ranges[i].m_min)
            break;
        else if (m_ranges[i].Delete(range, &r))
        {
            if (m_ranges[i].IsEmpty())
            {
                m_ranges.RemoveAt(i);
                i = (i > 0) ? i-1 : -1;
            }
            else if (!r.IsEmpty())
                m_ranges.Insert(r, i+1);

            done = true;
        }
    }

    return done;
}

bool wxRangeIntSelection::SelectRange(const wxRangeInt &range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Selection Range") );

    // Try to find a range that includes this one and combine it, else insert it, else append it
    bool done = false;
    int i, count = m_ranges.GetCount();
    int nearest = count > 0 ? NearestIndex(range.m_min) : -1;

    if (nearest < 0)
    {
        if (!((count > 0) && m_ranges[0].Combine(range, true)))
            m_ranges.Insert(range, 0);
        return true;
    }
    else if (nearest == count)
    {
        if (!((count > 0) && m_ranges[count-1].Combine(range, true)))
            m_ranges.Add(range);
        return true;
    }
    else
    {
        if (m_ranges[nearest].Contains(range))
            return false;

        for (i=nearest; i<count; i++)
        {
            if (m_ranges[i].Combine(range, true))
            {
                done = true;
                break;
            }
            else if (range.m_max < m_ranges[i].m_min)
            {
                m_ranges.Insert(range, i);
                return true;
            }
        }

        count = m_ranges.GetCount();
        for (i=wxMax(nearest-1, 1); i<count; i++)
        {
            if (range.m_max+1 < m_ranges[i-1].m_min)
                break;
            else if (m_ranges[i-1].Combine(m_ranges[i], true))
            {
                m_ranges.RemoveAt(i);
                count--;
                i--;
            }
        }
    }

#ifdef CHECK_RANGES
    printf("Selecting ranges %d %d count %d\n", range.m_min, range.m_max, m_ranges.GetCount());

    for (i=1; i<int(m_ranges.GetCount()); i++)
    {
        if (m_ranges[i-1].Contains(m_ranges[i]))
            printf("Error in Selecting ranges %d %d, %d %d count %d\n", m_ranges[i-1].m_min, m_ranges[i-1].m_max, m_ranges[i].m_min, m_ranges[i].m_max, m_ranges.GetCount());
        if (m_ranges[i-1].Touches(m_ranges[i]))
            printf("Could have minimzed ranges %d %d, %d %d count %d\n", m_ranges[i-1].m_min, m_ranges[i-1].m_max, m_ranges[i].m_min, m_ranges[i].m_max, m_ranges.GetCount());
    }
    fflush(stdout);
#endif // CHECK_RANGES

    return done;
}

bool wxRangeIntSelection::BoundRanges(const wxRangeInt& range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Bounding Range") );
    int i, count = m_ranges.GetCount();
    bool done = false;

    for (i = 0; i < count; i++)
    {
        if (m_ranges[i].m_min >= range.m_min)
            break;

        if (m_ranges[i].m_max < range.m_min) // range is out of bounds
        {
            done = true;
            m_ranges.RemoveAt(i);
            count--;
            i--;
        }
        else
        {
            done = true;
            m_ranges[i].m_min = range.m_min;
            break;
        }
    }

    for (i = m_ranges.GetCount() - 1; i >= 0; i--)
    {
        if (m_ranges[i].m_max <= range.m_max)
            break;

        if (m_ranges[i].m_min > range.m_max) // range is out of bounds
        {
            done = true;
            m_ranges.RemoveAt(i);
        }
        else
        {
            done = true;
            m_ranges[i].m_max = range.m_max;
            break;
        }
    }

    return done;
}

//=============================================================================
// wxRangeDouble
//=============================================================================

bool wxRangeDouble::Combine(double i)
{
    if      (i < m_min) { m_min = i; return true; }
    else if (i > m_max) { m_max = i; return true; }
    return false;
}

bool wxRangeDouble::Combine( const wxRangeDouble &r, bool only_if_touching )
{
    if (only_if_touching)
    {
        if ((r.m_min <= m_max) && (r.m_max >= m_min))//Contains(r))
        {
            *this+=r;
            return true;
        }
    }
    else if (!IsEmpty() && !r.IsEmpty())
    {
        bool added = false;
        if (r.m_min < m_min) { m_min = r.m_min; added = true; }
        if (r.m_max > m_max) { m_max = r.m_max; added = true; }
        return added;
    }
    return false;
}

bool wxRangeDouble::Delete( const wxRangeDouble &r, wxRangeDouble *right )
{
    if (!Contains(r))
        return false;

    if (right) *right = wxEmptyRangeDouble;

    if (r.m_min <= m_min)
    {
        if (r.m_max >= m_max)
        {
            *this = wxEmptyRangeDouble;
            return true;
        }

        m_min = r.m_max;
        return true;
    }

    if (r.m_max >= m_max)
    {
        m_max = r.m_min;
        return true;
    }

    if (right)
        *right = wxRangeDouble(r.m_max, m_max);

    m_max = r.m_min;
    return true;
}

//=============================================================================
// wxRangeDoubleSelection
//=============================================================================
const wxRangeDouble& wxRangeDoubleSelection::GetRange( int index ) const
{
    wxCHECK_MSG((index>=0) && (index<int(m_ranges.GetCount())), wxEmptyRangeDouble, wxT("Invalid index"));
    return m_ranges[index];
}

wxRangeDouble wxRangeDoubleSelection::GetBoundingRange() const
{
    if (int(m_ranges.GetCount()) < 1) return wxEmptyRangeDouble;
    return wxRangeDouble(m_ranges[0].m_min, m_ranges[m_ranges.GetCount()-1].m_max);
}

int wxRangeDoubleSelection::Index( wxDouble i ) const
{
    int count = m_ranges.GetCount();
    if (count < 1) return wxNOT_FOUND;

    if (i < m_ranges[0].m_min) return wxNOT_FOUND;
    if (i > m_ranges[count-1].m_max) return wxNOT_FOUND;

    // Binary search
    int res, tmp, lo = 0, hi = count;

    while ( lo < hi )
    {
        tmp = (lo + hi)/2;
        res = m_ranges[tmp].Position(i);

        if ( res == 0 )
            return tmp;
        else if ( res < 0 )
            hi = tmp;
        else //if ( res > 0 )
            lo = tmp + 1;
    }

    return wxNOT_FOUND;

/*
    for (int j=0; j<count; j++)
    {
        if (m_ranges[j].Contains(i)) return j;
    }
*/
}

int wxRangeDoubleSelection::Index( const wxRangeDouble &r ) const
{
    int i, count = m_ranges.GetCount();
    for (i=0; i<count; i++) if (m_ranges[i].Contains(r)) return i;
    return wxNOT_FOUND;
}

int wxRangeDoubleSelection::NearestIndex( wxDouble i ) const
{
    int count = m_ranges.GetCount();
    if (count < 1) return -1;

    if (i < m_ranges[0].m_min) return -1;
    if (i > m_ranges[count-1].m_max) return count;

    // Binary search
    int res, tmp, lo = 0, hi = count;

    while ( lo < hi )
    {
        tmp = (lo + hi)/2;
        res = m_ranges[tmp].Position(i);

        if ( res == 0 )
            return tmp;
        else if ((i >= m_ranges[tmp].m_max) && (i < m_ranges[wxMin(tmp+1, count-1)].m_min))
            return tmp;
        else if ( res < 0 )
            hi = tmp;
        else //if ( res > 0 )
            lo = tmp + 1;
    }

    // oops shouldn't get here
    wxCHECK_MSG(0, -1, wxT("Error calculating NearestIndex in wxRangeDoubleSelection"));
}

bool wxRangeDoubleSelection::SelectRange(const wxRangeDouble &range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Selection Range") );

    // Try to find a range that includes this one and combine it, else insert it, else append it
    bool done = false;
    int i, count = m_ranges.GetCount();
    int nearest = count > 0 ? NearestIndex(range.m_min) : -1;

    if (nearest < 0)
    {
        if (!((count > 0) && m_ranges[0].Combine(range, true)))
            m_ranges.Insert(range, 0);
        return true;
    }
    else if (nearest == count)
    {
        if (!((count > 0) && m_ranges[count-1].Combine(range, true)))
            m_ranges.Add(range);
        return true;
    }
    else
    {
        if (m_ranges[nearest].Contains(range))
            return false;

        for (i=nearest; i<count; i++)
        {
            if (m_ranges[i].Combine(range, true))
            {
                done = true;
                break;
            }
            else if (range.m_max < m_ranges[i].m_min)
            {
                m_ranges.Insert(range, i);
                return true;
            }
        }
        for (i=wxMax(nearest-1, 1); i<int(m_ranges.GetCount()); i++)
        {
            if (range.m_max+1 < m_ranges[i-1].m_min)
                break;
            else if (m_ranges[i-1].Combine(m_ranges[i], true))
            {
                m_ranges.RemoveAt(i);
                i--;
            }
        }
    }

#ifdef CHECK_RANGES
    printf("Selecting ranges %g %g count %d\n", range.m_min, range.m_max, m_ranges.GetCount());

    for (i=1; i<int(m_ranges.GetCount()); i++)
    {
        if (m_ranges[i-1].Contains(m_ranges[i]))
            printf("Error in Selecting ranges %g %g, %g %g count %d\n", m_ranges[i-1].m_min, m_ranges[i-1].m_max, m_ranges[i].m_min, m_ranges[i].m_max, m_ranges.GetCount());
        //if (m_ranges[i-1].Touches(m_ranges[i]))
        //    printf("Could have minimzed ranges %g %g, %g %g count %d\n", m_ranges[i-1].m_min, m_ranges[i-1].m_max, m_ranges[i].m_min, m_ranges[i].m_max, m_ranges.GetCount());
    }
    fflush(stdout);
#endif // CHECK_RANGES

    return done;
}

bool wxRangeDoubleSelection::DeselectRange(const wxRangeDouble &range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Selection Range") );

    bool done = false;
    int i, count = m_ranges.GetCount();
    int nearest = count > 0 ? NearestIndex(range.m_min) : -1;

    if ((nearest < 0) || (nearest == count))
        return false;

    wxRangeDouble r;
    for (i=nearest; i<int(m_ranges.GetCount()); i++)
    {
        if (range.m_max < m_ranges[i].m_min)
            break;
        else if (m_ranges[i].Delete(range, &r))
        {
            if (m_ranges[i].IsEmpty())
            {
                m_ranges.RemoveAt(i);
                i = (i > 0) ? i-1 : -1;
            }
            else if (!r.IsEmpty())
                m_ranges.Insert(r, i+1);

            done = true;
        }
    }

    return done;
}

bool wxRangeDoubleSelection::BoundRanges(const wxRangeDouble& range)
{
    wxCHECK_MSG(!range.IsEmpty(), false, wxT("Invalid Bounding Range") );
    int i, count = m_ranges.GetCount();
    bool done = false;

    for (i = 0; i < count; i++)
    {
        if (m_ranges[i].m_min >= range.m_min)
            break;

        if (m_ranges[i].m_max < range.m_min) // range is out of bounds
        {
            done = true;
            m_ranges.RemoveAt(i);
            count--;
            i--;
        }
        else
        {
            done = true;
            m_ranges[i].m_min = range.m_min;
            break;
        }
    }

    for (i = m_ranges.GetCount() - 1; i >= 0; i--)
    {
        if (m_ranges[i].m_max <= range.m_max)
            break;

        if (m_ranges[i].m_min > range.m_max) // range is out of bounds
        {
            done = true;
            m_ranges.RemoveAt(i);
        }
        else
        {
            done = true;
            m_ranges[i].m_max = range.m_max;
            break;
        }
    }

    return done;
}
