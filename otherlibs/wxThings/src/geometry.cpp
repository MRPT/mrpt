/////////////////////////////////////////////////////////////////////////////
// Name:        geometry.cpp
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets v2
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wxprec.h>

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#include "wx/things/geometry.h"

/*
2.4 : How do I generate a circle through three points?
Let the three given points be a, b, c. Use _0 and _1 to represent x and y coordinates. The coordinates of the center p=(p_0,p_1) of the circle determined by a, b, and c are:
A = b_0 - a_0; B = b_1 - a_1; C = c_0 - a_0; D = c_1 - a_1;
E = A*(a_0 + b_0) + B*(a_1 + b_1); F = C*(a_0 + c_0) + D*(a_1 + c_1);
G = 2.0*(A*(c_1 - b_1)-B*(c_0 - b_0));
p_0 = (D*E - B*F) / G; p_1 = (A*F - C*E) / G;
If G is zero then the three points are collinear and no finite-radius circle through them exists. Otherwise, the radius of the circle is:
r^2 = (a_0 - p_0)^2 + (a_1 - p_1)^2
[O' Rourke (C)] p. 201. Simplified by Jim Ward.
*/

wxCircleDouble::wxCircleDouble(const wxPoint2DDouble &p1,
                               const wxPoint2DDouble &p2,
                               const wxPoint2DDouble &p3)
{
    wxDouble A = p2.m_x - p1.m_x,
             B = p2.m_y - p1.m_y,
             C = p3.m_x - p1.m_x,
             D = p3.m_y - p1.m_y;

    wxDouble E = A*(p1.m_x + p2.m_x) + B*(p1.m_y + p2.m_y),
             F = C*(p1.m_x + p3.m_x) + D*(p1.m_y + p3.m_y),
             G = 2.0*(A*(p3.m_y - p2.m_y)-B*(p3.m_x - p2.m_x));

    if (G == 0)
    {
        m_x = m_y = m_r = 0;
        return;
    }

    m_x = (D*E - B*F) / G,
    m_y = (A*F - C*E) / G;
    m_r = sqrt( (p1.m_x - m_x)*(p1.m_x - m_x) + (p1.m_y - m_y)*(p1.m_y - m_y) );
}

int wxCircleDouble::IntersectLine( const wxRay2DDouble &line,
                                   wxPoint2DDouble *pt1,
                                   wxPoint2DDouble *pt2 ) const
{
    //if (line.GetDistanceToPoint(m_origin) > m_r) return 0;

    wxDouble l1_x = m_x-m_r, l1_y = line.GetYFromX(l1_x);
    wxDouble l2_x = m_x+m_r, l2_y = line.GetYFromX(l2_x);

    // quick check to see it it intersects at all
    //wxDouble top = m_origin.m_y-m_r, bot = m_origin.m_y+m_r;
    //if (((l1_y < top)&&(l2_y < top))||((l1_y > bot)&&(l2_y > bot))) return 0;

    wxDouble l2_l1_x = l2_x - l1_x, l2_l1_y = l2_y - l1_y;

    wxDouble a  = l2_l1_x*l2_l1_x + l2_l1_y*l2_l1_y;
    wxDouble b  = 2.0 * (l2_l1_x * (l1_x - m_x) + l2_l1_y * (l1_y - m_y) );

    wxDouble c = m_x*m_x + m_y*m_y + l1_x*l1_x + l1_y*l1_y - 2.0*(m_x*l1_x + m_y*l1_y) - m_r*m_r;
    wxDouble det = b*b - 4.0*a*c;

    if ( det < 0 )
    {
        return 0;
    }
    else if ( det == 0 )
    {
        if (pt1)
        {
            wxDouble u = -b/(2.0*a);
            pt1->m_x = l2_x + u*l2_l1_x;
            pt1->m_y = l2_y + u*l2_l1_y;
        }
        return 1;
    }
    // else det > 0 so 2 points intersect
    wxDouble e  = sqrt(det);

    if (pt1)
    {
        wxDouble u1 = (-b - e)/( 2.0*a );
        pt1->m_x = l1_x + u1*l2_l1_x;
        pt1->m_y = l1_y + u1*l2_l1_y;
    }
    if (pt2)
    {
        wxDouble u2 = (-b + e)/( 2.0*a );
        pt2->m_x = l1_x + u2*l2_l1_x;
        pt2->m_y = l1_y + u2*l2_l1_y;
    }

    return 2;
}








/*
int wxEllipseInt::IntersectLine( const wxLine2DInt &line,
                                 wxPoint2DInt &pt1,
                                 wxPoint2DInt &pt2 ) const
{

    //Intersection.intersectEllipseLine = function(c, rx, ry, a1, a2) {
    //var result;
    //line.m_pt    // var origin = new Vector2D(a1.x, a1.y);
    wxPoint2DInt dir = pt2 - p21;   // var dir = Vector2D.fromPoints(a1, a2);
    //m_origin   //var center = new Vector2D(c.x, c.y);
    wxPoint2DInt diff = line.m_pt - m_origin; //var diff   = origin.subtract(center);

    //var mDir   = new Vector2D( dir.x/(rx*rx),  dir.y/(ry*ry)  );
    wxPoint2DDouble mDir = wxPoint2DDouble(wxDouble(dir.m_x)/(m_radius.m_x*m_radius.m_x),
                                           wxDouble(dir.m_y)/(m_radius.m_y*m_radius.m_y));
    //var mDiff  = new Vector2D( diff.x/(rx*rx), diff.y/(ry*ry) );
    wxPoint2DDouble mDiff = wxPoint2DDouble(wxDouble(diff.m_x)/(m_radius.m_x*m_radius.m_x),
                                            wxDouble(diff.m_y)/(m_radius.m_y*m_radius.m_y));

    wxDouble a = dir.GetDotProduct(mDir);    //var a = dir.dot(mDir);
    wxDouble b = dir.GetDotProduct(mDiff);   //var b = dir.dot(mDiff);
    wxDouble c = diff.GetDotProduct(m_diff); //var c = diff.dot(mDiff) - 1.0;
    wxDouble d = b*b - a*c;                  //var d = b*b - a*c;

    if ( d < 0 )
    {
        return 0; //result = new Intersection("Outside");
    }
    else if ( d > 0 )
    {
        wxDouble root = sqrt(d);      //var root = Math.sqrt(d);
        wxDouble t_a = (-b - root)/a; //var t_a  = (-b - root) / a;
        wxDouble t_b = (-b + root)/a; //var t_b  = (-b + root) / a;

        if ( (t_a < 0 || 1 < t_a) && (t_b < 0 || 1 < t_b) )
        {
            if ( (t_a < 0 && t_b < 0) || (t_a > 1 && t_b > 1) )
                result = new Intersection("Outside");
            else
                result = new Intersection("Inside");
        }
        else
        {
            result = new Intersection("Intersection");
            if ( 0 <= t_a && t_a <= 1 )
                result.appendPoint( a1.lerp(a2, t_a) );
            if ( 0 <= t_b && t_b <= 1 )
                result.appendPoint( a1.lerp(a2, t_b) );
        }
    }
    else
    {
        var t = -b/a;
        if ( 0 <= t && t <= 1 )
        {
            result = new Intersection("Intersection");
            result.appendPoint( a1.lerp(a2, t) );
        } else
        {
            result = new Intersection("Outside");
        }
    }

    return result;
}
*/
/*
> I am looking for the algorithm to calculate the intersection of a line
with
> an ellipse, that is not necessarily parrallel to x,y axis.
>
> I cannot find anything that is easy to understand.

The ellipse can be represented by a quadratic polynomial,
a00+a10*x+a01*y+a20*x^2+a11*x*y+a02*y^2 = 0.
The line is b0+b1*x+b2*y = 0 where one of b1 or b2 is
not zero.  For the sake of argument, suppose b2 is not
zero.  Solve for y = -(b0+b1*x)/b2.  Replace this in the
quadratic equation and multiply through by b2^2 to get
Q(x) = c0 + c1*x + c2*x^2 = 0 where
  c0 = a02*b0^2-a01*b0*b2+a00*b2^2
  c1 = 2*a02*b0*b1-a11*b0*b2-a01*b1*b2+a10*b2^2
  c2 = a02*b1^2-a11*b1*b2+a02*b2^2
If Q(x) has no real roots, the line and ellipse do not intersect.
If Q(x) has a single, repeated real root x0, the line and ellipse
are tangent.  The y-value is y0 = -(b0+b1*x0)/b2.  If Q(x)
has two distinct real roots x0 and x1, the line and ellipse
intersect in two points (x0,y0) and (x1,y1) where
y0 = -(b0+b1*x0)/b2 and y1 = -(b0+b1*x1)/b2.

*/
