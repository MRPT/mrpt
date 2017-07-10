///////////////////////////////////////////////////////////////////////////////
// Name:        geometry.h
// Purpose:     Additional geometry functions for wxWidgets (see wx/geometry.h)
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets v2
///////////////////////////////////////////////////////////////////////////////

#ifndef __WXIMAGER_GEOMETRY_H__
#define __WXIMAGER_GEOMETRY_H__

#include "wx/geometry.h"
#include "wx/things/thingdef.h"

#define wxGEOMETRY_INF 1E100

//-----------------------------------------------------------------------------
// wxRay2DDouble uses point slope line format
//
//    y = mx+b, m=(x-x0)/(y-y0)
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxRay2DDouble : public wxPoint2DDouble
{
public :

    inline wxRay2DDouble(wxDouble x=0, wxDouble y=0, wxDouble slope=0) : wxPoint2DDouble(x, y), m_slope(slope) {}
    inline wxRay2DDouble(const wxPoint2DDouble &pt, wxDouble slope) : wxPoint2DDouble(pt), m_slope(slope) {}
    inline wxRay2DDouble(wxDouble x1, wxDouble y1, wxDouble x2, wxDouble y2) : wxPoint2DDouble(x1, y1)
        { m_slope = GetSlope(x1, y1, x2, y2); }
    inline wxRay2DDouble(const wxPoint2DDouble &pt1, const wxPoint2DDouble &pt2) : wxPoint2DDouble(pt1)
        { m_slope = GetSlope(pt1, pt2); }
    inline wxRay2DDouble(const wxRay2DDouble &line) : wxPoint2DDouble(line.m_x, line.m_y), m_slope(line.m_slope) {}

    inline wxDouble GetX() const { return m_x; }
    inline wxDouble GetY() const { return m_y; }
    inline wxPoint2DDouble GetPoint() const { return (wxPoint2DDouble)(*this); }
    inline wxDouble GetSlope() const { return m_slope; }

    inline void SetX(wxDouble x) { m_x = x; }
    inline void SetY(wxDouble y) { m_y = y; }
    inline void SetPoint(const wxPoint2DDouble &pt ) { m_x = pt.m_x; m_y = pt.m_y; }
    inline void SetSlope(wxDouble slope) { m_slope = slope; }

    inline wxDouble GetYFromX(wxDouble x) const {return m_slope*(x-m_x) + m_y;}
    inline wxDouble GetXFromY(wxDouble y) const {return (y-m_y)/m_slope + m_x;}

    // Get a point along the line at pos x or y
    inline wxPoint2DDouble GetPointOnRayFromX(wxDouble x) const { return wxPoint2DDouble(x, GetYFromX(x)); }
    inline wxPoint2DDouble GetPointOnRayFromY(wxDouble y) const
        { if (m_slope == 0) return (*this);
          return wxPoint2DDouble(GetXFromY(y), y); }

    // Translate the point m_pt along the line to pos x or y
    inline void TranslatePointByX(wxDouble x) { m_y = GetYFromX(x); m_x = x; }
    inline void TranslatePointByY(wxDouble y) { m_x = GetXFromY(y); m_y = y; }
    inline wxRay2DDouble GetTranslatedLineByX(wxDouble x) const { return wxRay2DDouble(x, GetYFromX(x), m_slope); }
    inline wxRay2DDouble GetTranslatedLineByY(wxDouble y) const { return wxRay2DDouble(GetXFromY(y), y, m_slope); }

    inline wxDouble GetDistanceToPoint(const wxPoint2DDouble &pt, wxPoint2DDouble *closestPt=nullptr) const
    {
        wxPoint2DDouble l1(m_x, m_y);
        wxPoint2DDouble l2(GetPointOnRayFromX(m_x+pt.m_x));
        wxPoint2DDouble v = l2 - l1;
        wxPoint2DDouble w = pt - l1;
        wxDouble c1 = w.GetDotProduct(v);
        wxDouble c2 = v.GetDotProduct(v);
        wxDouble b = c1 / c2;
        wxPoint2DDouble pb = l1 + b*v;
        if (closestPt) *closestPt = pb;
        return pb.GetDistance( pt );

/*
        wxPoint2DDouble ll2 = l2;
        double l = ll2.GetDistanceSquare(l1);
        double u = ((p.m_x-l1.m_x)*(l2.m_x-l1.m_x)+(p.m_y-l1.m_y)*(l2.m_y-l1.m_y))/l;
        wxPoint2DDouble i = l1 + u*(l2-l1);

        //printf("distance %.9lf %.9lf %d\n\n", i.GetDistance(p), pb.GetDistance(p), int(pb==i));
*/

        //                   this x for y on this line
        // y(on this line) = m_slope*(x-m_pt.m_x) + m_pt.m_y
        //                 = (-1/m_slope)(x-pt.m_x) + pt.m_y
        // so
        // x(on this line) = (y-m_pt.m_y)/m_slope + m_pt.m_x
        //                 = (y-pt.m_y)/(-1/m_slope) + m_pt.m_x

        //(y-m_pt.m_y)/m_slope + m_pt.m_x = (y-pt.m_y)(-1/m_slope) + m_pt.m_x
        //(y-m_pt.m_y)/m_slope - m_slope*(y-pt.m_y) = + m_pt.m_x - m_pt.m_x
        //y*(1/m_slope - m_slope) = + m_pt.m_x - m_pt.m_x +m_slope*pt.m_y + m_slope*m_pt.m_y
        //y = (m_pt.m_x - m_pt.m_x +m_slope*pt.m_y + m_slope*m_pt.m_y)/(1/m_slope - m_slope)
/*
        wxDouble x = (m_x + m_slope*m_slope*m_x - m_slope*(m_y - m_y))/(m_slope*m_slope+1.0);
        wxPoint2DDouble pl(x, m_slope*(x-m_x) + m_y);
        if (closestPt) *closestPt = pl;
        return pl.GetDistance(pt);
*/
    }

  inline wxDouble GetDistanceToRay( const wxRay2DDouble &ray ) const
        {
            // FIXME - unchecked, just quickly translated from some other code
            if (m_slope != ray.m_slope) return 0;
            if (m_slope == 0) return fabs(m_y - ray.m_y);
            wxPoint2DDouble p1 = GetPointOnRayFromX(0);
            wxPoint2DDouble p2 = ray.GetPointOnRayFromX(0);
            // y = (-1/s1)*x+p1.m_y = s2*x + p2.m_y
            wxDouble dx = (p1.m_y - p2.m_y)/(m_slope + (1.0/m_slope));
            wxDouble dy = (m_slope*dx+p2.m_y) - p1.m_y;
            return sqrt(dx*dx + dy*dy);
        }

    inline static wxDouble GetSlope(wxDouble x1, wxDouble y1, wxDouble x2, wxDouble y2)
        { return (y2 - y1)/(x2 - x1); }
    inline static wxDouble GetSlope(const wxPoint2DDouble &pt1, const wxPoint2DDouble &pt2)
        { return (pt2.m_y-pt1.m_y)/(pt2.m_x-pt1.m_x); }

    // find the point where the two rays meet, return false if parallel
    bool Intersect(const wxRay2DDouble& other, wxPoint2DDouble& pt) const
    {
        // (y1-y0)/(x1-x0)=m for both lines, equate y1's first
        if (m_slope == other.m_slope) return false;
        pt.m_x = (m_slope*m_x - other.m_slope*other.m_x + other.m_y - m_y)/(m_slope - other.m_slope);
        pt.m_y = GetYFromX(pt.m_x);
        return true;
    }

    // Operators

    inline wxRay2DDouble operator=(const wxRay2DDouble& r) { m_x = r.m_x; m_y = r.m_y; m_slope = r.m_slope; return *this; }
    inline bool operator==(const wxRay2DDouble& r) const { return (m_x == r.m_x)&&(m_y == r.m_y)&&(m_slope == r.m_slope); }
    inline bool operator!=(const wxRay2DDouble& r) const { return !(*this == r); }

    inline wxRay2DDouble operator+(const wxPoint2DDouble& rel_pos) const { return wxRay2DDouble(m_x+rel_pos.m_x, m_y+rel_pos.m_y, m_slope); }
    inline wxRay2DDouble operator-(const wxPoint2DDouble& rel_pos) const { return wxRay2DDouble(m_x-rel_pos.m_x, m_y-rel_pos.m_y, m_slope); }
    inline wxRay2DDouble operator*(const wxPoint2DDouble& rel_pos) const { return wxRay2DDouble(m_x*rel_pos.m_x, m_y*rel_pos.m_y, m_slope); }
    inline wxRay2DDouble operator/(const wxPoint2DDouble& rel_pos) const { return wxRay2DDouble(m_x/rel_pos.m_x, m_y/rel_pos.m_y, m_slope); }

    inline wxRay2DDouble& operator+=(const wxPoint2DDouble& rel_pos) { m_x += rel_pos.m_x; m_y += rel_pos.m_y; return *this; }
    inline wxRay2DDouble& operator-=(const wxPoint2DDouble& rel_pos) { m_x -= rel_pos.m_x; m_y -= rel_pos.m_y; return *this; }
    inline wxRay2DDouble& operator*=(const wxPoint2DDouble& rel_pos) { m_x *= rel_pos.m_x; m_y *= rel_pos.m_y; return *this; }
    inline wxRay2DDouble& operator/=(const wxPoint2DDouble& rel_pos) { m_x /= rel_pos.m_x; m_y /= rel_pos.m_y; return *this; }

    inline wxRay2DDouble operator+(const wxDouble& rel_slope) const { return wxRay2DDouble(m_x, m_y, m_slope+rel_slope); }
    inline wxRay2DDouble operator-(const wxDouble& rel_slope) const { return wxRay2DDouble(m_x, m_y, m_slope-rel_slope); }
    inline wxRay2DDouble operator*(const wxDouble& rel_slope) const { return wxRay2DDouble(m_x, m_y, m_slope*rel_slope); }
    inline wxRay2DDouble operator/(const wxDouble& rel_slope) const { return wxRay2DDouble(m_x, m_y, m_slope/rel_slope); }

    inline wxRay2DDouble& operator+=(const wxDouble& rel_slope) { m_slope += rel_slope; return *this; }
    inline wxRay2DDouble& operator-=(const wxDouble& rel_slope) { m_slope -= rel_slope; return *this; }
    inline wxRay2DDouble& operator*=(const wxDouble& rel_slope) { m_slope *= rel_slope; return *this; }
    inline wxRay2DDouble& operator/=(const wxDouble& rel_slope) { m_slope /= rel_slope; return *this; }

    wxDouble m_slope;
};

//-----------------------------------------------------------------------------
// wxLine2DInt uses point slope line format
//
//    y = mx+b, m=(x-x0)/(y-y0)
//-----------------------------------------------------------------------------
/*
class WXDLLIMPEXP_THINGS wxLine2DInt : wxRect2DInt
{
public :

    inline wxLine2DInt(wxInt32 x1=0, wxInt32 y1=0, wxInt32 x2=0, wxInt32 y2=0)
        { m_x = x1; m_y = y1; m_width = x2 - x1; m_height = y2 - y1; }
    inline wxLine2DInt(const wxPoint2DInt &pt1, const wxPoint2DInt &pt2)
        { m_x = pt1.m_x; m_y = pt1.m_y; m_width = pt2.m_x - pt1.m_x; m_height = pt2.m_y - pt1.m_y; }
    inline wxLine2DInt(const wxLine2DInt &line)
        { m_x = line.m_x; m_y = line.m_y; m_width = line.m_width; m_height = line.m_height; }

    inline wxInt32 GetX1() const { return m_x; }
    inline wxInt32 GetY1() const { return m_y; }
    inline wxInt32 GetX2() const { return m_x + m_width; }
    inline wxInt32 GetY2() const { return m_y + m_height; }
    inline wxPoint2DInt Get1Point() const { return GetTopLeft(); }
    inline wxPoint2DInt Get2Point() const { return GetBottomRight(); }
    inline wxDouble GetSlope() const { return wxDouble(m_height)/wxDouble(m_width); }

    inline void SetX1(wxInt32 x) { m_x = x; }
    inline void SetY1(wxInt32 y) { m_y = y; }
    inline void SetX2(wxInt32 x) { m_width = m_x - x; }
    inline void SetY2(wxInt32 y) { m_height = m_y - y; }
    inline void SetPoint1(const wxPoint2DInt &pt ) { SetTopLeft(pt); }
    inline void SetPoint2(const wxPoint2DInt &pt ) { SetBottomRight(pt); }

    inline wxDouble GetYFromX(wxDouble x) const {return (wxDouble(m_height)/m_width)*(x-m_x) + m_y;}
    inline wxDouble GetXFromY(wxDouble y) const {return (y-m_y)*(wxDouble(m_height)/m_width) + m_x;}

    // Get a point along the line at pos x or y
    inline wxPoint2DInt GetPointOnLineFromX(wxInt32 x) const
        { return wxPoint2DInt(x, GetYFromX(x)); }
    inline wxPoint2DInt GetPointOnLineFromY(wxInt32 y) const
        { return wxPoint2DInt(GetXFromY(y), y); }

    // Translate the point m_pt along the line to pos x or y
    inline void TranslatePointByX(wxDouble x)
        { m_pt.m_y = GetYFromX(x); m_pt.m_x = x; }
    inline void TranslatePointByY(wxDouble y)
        { m_pt.m_x = GetXFromY(y); m_pt.m_y = y; }
    inline wxLine2DInt GetTranslatedLineByX(wxDouble x)
        { return wxLine2DInt(x, GetYFromX(x), m_slope); }
    inline wxLine2DInt GetTranslatedLineByY(wxDouble y)
        { return wxLine2DInt(GetXFromY(y), y, m_slope); }

    inline wxDouble GetDistanceToPoint(const wxPoint2DDouble &pt, wxPoint2DDouble *closestPt=nullptr) const
    {
        wxPoint2DDouble l1(m_pt);
        wxPoint2DDouble l2(GetPointOnLineFromX(m_pt.m_x+pt.m_x));
        wxPoint2DDouble v = l2 - l1;
        wxPoint2DDouble w = pt - l1;
        double c1 = w.GetDotProduct(v);
        double c2 = v.GetDotProduct(v);
        double b = c1 / c2;
        wxPoint2DDouble pb = l1 + b*v;
        if (closestPt) *closestPt = pb;
        return pb.GetDistance( pt );

    }


    inline static wxDouble GetSlope(wxDouble x1, wxDouble y1, wxDouble x2, wxDouble y2)
        { return (y2 - y1)/(x2 - x1); }
    inline static wxDouble GetSlope(const wxPoint2DDouble &pt1, const wxPoint2DDouble &pt2)
        { return (pt2.m_y-pt1.m_y)/(pt2.m_x-pt1.m_x); }


    // Default copy operator is ok

    wxPoint2DDouble m_pt;
    wxDouble m_slope;
};
*/

//-----------------------------------------------------------------------------
// wxCircleDouble   m_r*m_r = (x-m_origin.m_x)^2 + (y-m_origin.m_y)^2
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxCircleDouble : public wxPoint2DDouble
{
public :

    inline wxCircleDouble(const wxCircleDouble &circle) : wxPoint2DDouble(circle.m_x, circle.m_y), m_r(circle.m_r) {}
    inline wxCircleDouble(wxDouble x=0, wxDouble y=0, wxDouble r=0) : wxPoint2DDouble(x, y), m_r(r) {}
    inline wxCircleDouble(const wxPoint2DDouble &origin, wxDouble r) : wxPoint2DDouble(origin), m_r(r) {}

    inline wxCircleDouble(const wxPoint2DDouble &p1,
                          const wxPoint2DDouble &p2,
                          const wxPoint2DDouble &p3);

    inline bool IsEmpty() const { return m_r <= 0; }

    inline wxDouble GetX() const { return m_x; }
    inline wxDouble GetY() const { return m_y; }
    inline wxPoint2DDouble GetOrigin() const { return wxPoint2DDouble(m_x, m_y); }
    inline wxDouble GetRadius() const { return m_r; }

    // Get a bounding rect
    inline wxRect2DDouble GetRect() const
        { return wxRect2DDouble(m_x-m_r, m_y-m_r, 2.0*m_r, 2.0*m_r); }

    inline void SetX(wxDouble x) { m_x = x; }
    inline void SetY(wxDouble y) { m_y = y; }
    inline void SetOrigin(const wxPoint2DDouble &origin) { m_x = origin.m_x; m_y = origin.m_y; }
    inline void SetRadius(wxDouble r) { m_r = r; }

    inline bool GetPointInCircle(wxDouble x, wxDouble y) const
        { if (IsEmpty()) return false;
          return ((x-m_x)*(x-m_x) + (y-m_y)*(y-m_y) <= m_r*m_r); }

    inline bool GetPointInCircle(const wxPoint2DDouble &pt) const
        { return GetPointInCircle(pt.m_x, pt.m_y); }

    inline bool Intersects(const wxCircleDouble &circle) const
        { return GetDistance(circle) <= m_r + circle.m_r;  }

    int IntersectLine( const wxRay2DDouble &line,
                       wxPoint2DDouble *pt1=nullptr,
                       wxPoint2DDouble *pt2=nullptr ) const;

    // Operators

    inline wxCircleDouble operator=(const wxCircleDouble& c) { m_x = c.m_x; m_y = c.m_y; m_r = c.m_r; return *this; }
    inline bool operator==(const wxCircleDouble& c) const { return (m_x == c.m_x)&&(m_y == c.m_y)&&(m_r == c.m_r); }
    inline bool operator!=(const wxCircleDouble& c) const { return !(*this == c); }

    inline wxCircleDouble operator+(const wxPoint2DDouble& rel_origin) const { return wxCircleDouble(m_x+rel_origin.m_x, m_y+rel_origin.m_y, m_r); }
    inline wxCircleDouble operator-(const wxPoint2DDouble& rel_origin) const { return wxCircleDouble(m_x-rel_origin.m_x, m_y-rel_origin.m_y, m_r); }
    inline wxCircleDouble operator*(const wxPoint2DDouble& rel_origin) const { return wxCircleDouble(m_x*rel_origin.m_x, m_y*rel_origin.m_y, m_r); }
    inline wxCircleDouble operator/(const wxPoint2DDouble& rel_origin) const { return wxCircleDouble(m_x/rel_origin.m_x, m_y/rel_origin.m_y, m_r); }

    inline wxCircleDouble& operator+=(const wxPoint2DDouble& rel_origin) { m_x += rel_origin.m_x; m_y += rel_origin.m_y; return *this; }
    inline wxCircleDouble& operator-=(const wxPoint2DDouble& rel_origin) { m_x -= rel_origin.m_x; m_y -= rel_origin.m_y; return *this; }
    inline wxCircleDouble& operator*=(const wxPoint2DDouble& rel_origin) { m_x *= rel_origin.m_x; m_y *= rel_origin.m_y; return *this; }
    inline wxCircleDouble& operator/=(const wxPoint2DDouble& rel_origin) { m_x /= rel_origin.m_x; m_y /= rel_origin.m_y; return *this; }

    inline wxCircleDouble operator+(const wxDouble& rel_radius) const { return wxCircleDouble(m_x, m_y, m_r+rel_radius); }
    inline wxCircleDouble operator-(const wxDouble& rel_radius) const { return wxCircleDouble(m_x, m_y, m_r-rel_radius); }
    inline wxCircleDouble operator*(const wxDouble& rel_radius) const { return wxCircleDouble(m_x, m_y, m_r*rel_radius); }
    inline wxCircleDouble operator/(const wxDouble& rel_radius) const { return wxCircleDouble(m_x, m_y, m_r/rel_radius); }

    inline wxCircleDouble& operator+=(const wxDouble& rel_radius) { m_r += rel_radius; return *this; }
    inline wxCircleDouble& operator-=(const wxDouble& rel_radius) { m_r -= rel_radius; return *this; }
    inline wxCircleDouble& operator*=(const wxDouble& rel_radius) { m_r *= rel_radius; return *this; }
    inline wxCircleDouble& operator/=(const wxDouble& rel_radius) { m_r /= rel_radius; return *this; }

    wxDouble m_r;
};

//-----------------------------------------------------------------------------
// wxCircleInt   m_r*m_r = (x-m_origin.m_x)^2 + (y-m_origin.m_y)^2
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxCircleInt : public wxPoint2DInt
{
public :

    inline wxCircleInt(wxInt32 x=0, wxInt32 y=0, wxInt32 r=0) : wxPoint2DInt(x, y), m_r(r) {}
    inline wxCircleInt(const wxPoint2DInt &origin, wxInt32 r) : wxPoint2DInt(origin), m_r(r) {}
    inline wxCircleInt(const wxCircleInt &circle) : wxPoint2DInt(circle.m_x, circle.m_y), m_r(circle.m_r) {}

    inline bool IsEmpty() const { return m_r <= 0; }

    inline wxInt32 GetX() const { return m_x; }
    inline wxInt32 GetY() const { return m_y; }
    inline wxPoint2DInt GetOrigin() const { return wxPoint2DInt(m_x, m_y); }
    inline wxInt32 GetRadius() const { return m_r; }

    // Get a bounding rect
    inline wxRect2DInt GetRect() const
        { return wxRect2DInt(m_x-m_r, m_y-m_r, 2*m_r, 2*m_r); }

    inline void SetX(wxInt32 x) { m_x = x; }
    inline void SetY(wxInt32 y) { m_y = y; }
    inline void SetOrigin(const wxPoint2DInt &origin) { m_x = origin.m_x; m_y = origin.m_y; }
    inline void SetRadius(wxInt32 r) { m_r = r; }

    inline bool GetPointInCircle(wxInt32 x, wxInt32 y) const
        { if (IsEmpty()) return false;
          return ((x-m_x)*(x-m_x) + (y-m_y)*(y-m_y) <= m_r*m_r); }

    inline bool GetPointInCircle(const wxPoint2DInt &pt) const
        { return GetPointInCircle(pt.m_x, pt.m_y); }

    inline bool Intersects(const wxCircleInt &circle) const
        { return GetDistance(circle) <= m_r + circle.m_r; }

//  int IntersectLine( const wxRay2DDouble &line,
//                     wxPoint2DInt *pt1=nullptr,
//                     wxPoint2DInt *pt2=nullptr ) const;

    // Operators

    inline wxCircleInt operator=(const wxCircleInt& c) { m_x = c.m_x; m_y = c.m_y; m_r = c.m_r; return *this; }
    inline bool operator==(const wxCircleInt& c) const { return (m_x == c.m_x)&&(m_y == c.m_y)&&(m_r == c.m_r); }
    inline bool operator!=(const wxCircleInt& c) const { return !(*this == c); }

    inline wxCircleInt operator+(const wxPoint2DInt& rel_origin) const { return wxCircleInt(m_x+rel_origin.m_x, m_y+rel_origin.m_y, m_r); }
    inline wxCircleInt operator-(const wxPoint2DInt& rel_origin) const { return wxCircleInt(m_x-rel_origin.m_x, m_y-rel_origin.m_y, m_r); }
    inline wxCircleInt operator*(const wxPoint2DInt& rel_origin) const { return wxCircleInt(m_x*rel_origin.m_x, m_y*rel_origin.m_y, m_r); }
    inline wxCircleInt operator/(const wxPoint2DInt& rel_origin) const { return wxCircleInt(m_x/rel_origin.m_x, m_y/rel_origin.m_y, m_r); }

    inline wxCircleInt& operator+=(const wxPoint2DInt& rel_origin) { m_x += rel_origin.m_x; m_y += rel_origin.m_y; return *this; }
    inline wxCircleInt& operator-=(const wxPoint2DInt& rel_origin) { m_x -= rel_origin.m_x; m_y -= rel_origin.m_y; return *this; }
    inline wxCircleInt& operator*=(const wxPoint2DInt& rel_origin) { m_x *= rel_origin.m_x; m_y *= rel_origin.m_y; return *this; }
    inline wxCircleInt& operator/=(const wxPoint2DInt& rel_origin) { m_x /= rel_origin.m_x; m_y /= rel_origin.m_y; return *this; }

    inline wxCircleInt operator+(const wxInt32& rel_radius) const { return wxCircleInt(m_x, m_y, m_r+rel_radius); }
    inline wxCircleInt operator-(const wxInt32& rel_radius) const { return wxCircleInt(m_x, m_y, m_r-rel_radius); }
    inline wxCircleInt operator*(const wxInt32& rel_radius) const { return wxCircleInt(m_x, m_y, m_r*rel_radius); }
    inline wxCircleInt operator/(const wxInt32& rel_radius) const { return wxCircleInt(m_x, m_y, m_r/rel_radius); }

    inline wxCircleInt& operator+=(const wxInt32& rel_radius) { m_r += rel_radius; return *this; }
    inline wxCircleInt& operator-=(const wxInt32& rel_radius) { m_r -= rel_radius; return *this; }
    inline wxCircleInt& operator*=(const wxInt32& rel_radius) { m_r *= rel_radius; return *this; }
    inline wxCircleInt& operator/=(const wxInt32& rel_radius) { m_r /= rel_radius; return *this; }

    wxInt32 m_r;
};

//-----------------------------------------------------------------------------
// wxEllipseInt   m_r*m_r = (x-m_origin.m_x)^2 + (y-m_origin.m_y)^2
//-----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS wxEllipseInt
{
public :

    inline wxEllipseInt(wxInt32 x=0, wxInt32 y=0, wxInt32 r_x=0, wxInt32 r_y=0)
        { m_origin.m_x=x; m_origin.m_y=y; m_radius.m_x=r_x; m_radius.m_y=r_y; }
    inline wxEllipseInt(const wxPoint2DInt &origin, const wxPoint2DInt radius)
        { m_origin = origin; m_radius = radius; }
    inline wxEllipseInt(const wxEllipseInt &ellipse)
        { m_origin = ellipse.m_origin; m_radius = ellipse.m_radius; }

    inline bool IsEmpty() const { return m_radius.m_x<=0 || m_radius.m_y<=0; }

    inline wxInt32 GetX() const { return m_origin.m_x; }
    inline wxInt32 GetY() const { return m_origin.m_y; }
    inline wxPoint2DInt GetOrigin() const { return m_origin; }
    inline wxInt32 GetXRadius() const { return m_radius.m_x; }
    inline wxInt32 GetYRadius() const { return m_radius.m_y; }
    inline wxPoint2DInt GetRadius() const { return m_radius; }

    // Get a bounding rect
    inline wxRect2DInt GetRect() const
        { return wxRect2DInt(m_origin.m_x-m_radius.m_x,
                             m_origin.m_y-m_radius.m_y,
                             2*m_radius.m_x, 2*m_radius.m_y); }

    inline void SetX(wxInt32 x) { m_origin.m_x = x; }
    inline void SetY(wxInt32 y) { m_origin.m_y = y; }
    inline void SetOrigin(const wxPoint2DInt &origin) { m_origin = origin; }
    inline void SetXRadius(wxInt32 r_x) { m_radius.m_x = r_x; }
    inline void SetYRadius(wxInt32 r_y) { m_radius.m_y = r_y; }
    inline void SetRadius(const wxPoint2DInt &radius) { m_radius = radius; }

    inline bool GetPointInEllipse(wxInt32 x, wxInt32 y) const
        { if (IsEmpty()) return false;
          return (((x-m_origin.m_x)*(x-m_origin.m_x))/m_radius.m_x +
                  ((y-m_origin.m_y)*(y-m_origin.m_y))/m_radius.m_y <= 1); }

    inline bool GetPointInEllipse(const wxPoint2DInt &pt) const
        { return GetPointInEllipse(pt.m_x, pt.m_y); }

//  int IntersectLine( const wxRay2DDouble &line,
//                     wxPoint2DInt *pt1=nullptr,
//                     wxPoint2DInt *pt2=nullptr ) const;


    inline bool operator==(const wxEllipseInt& c) const { return (m_origin == c.m_origin)&&(m_radius == c.m_radius); }
    inline bool operator!=(const wxEllipseInt& c) const { return !(*this == c); }

    inline wxEllipseInt operator+(const wxPoint2DInt& rel_origin) const { return wxEllipseInt(m_origin+rel_origin, m_radius); }
    inline wxEllipseInt& operator+=(const wxPoint2DInt& rel_origin) { m_origin += rel_origin; return *this; }
    inline wxEllipseInt operator-(const wxPoint2DInt& rel_origin) const { return wxEllipseInt(m_origin-rel_origin, m_radius); }
    inline wxEllipseInt& operator-=(const wxPoint2DInt& rel_origin) { m_origin -= rel_origin; return *this; }

    wxPoint2DInt m_radius;
    wxPoint2DInt m_origin;
};


#endif // __WXIMAGER_GEOMETRY_H__
