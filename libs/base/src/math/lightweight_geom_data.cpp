/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>

using namespace std; // For min/max, etc...

namespace mrpt	{	namespace math	{
	using namespace mrpt::poses; // For the +,- operators
	using namespace mrpt::utils;


	namespace detail
	{
		// Convert a pose into a light-weight structure (functional form, needed for forward declarations)
		TPoint2D lightFromPose(const mrpt::poses::CPoint2D &p) { return TPoint2D(p); }
		TPoint3D lightFromPose(const mrpt::poses::CPoint3D &p) { return TPoint3D(p); }
		TPose2D lightFromPose(const mrpt::poses::CPose2D &p)   { return TPose2D(p); }
		TPose3D lightFromPose(const mrpt::poses::CPose3D &p)   { return TPose3D(p); }
		TPose3DQuat lightFromPose(const mrpt::poses::CPose3DQuat &p)   { return TPose3DQuat(p); }

	} // end detail


TPoint2D::TPoint2D(const TPose2D &p):x(p.x),y(p.y)	{}
TPoint2D::TPoint2D(const TPoint3D &p):x(p.x),y(p.y)	{}
TPoint2D::TPoint2D(const TPose3D &p):x(p.x),y(p.y)	{}
TPoint2D::TPoint2D(const mrpt::poses::CPoint2D &p):x(p.x()),y(p.y())	{}
bool TPoint2D::operator<(const TPoint2D &p) const	{
	if (x<p.x) return true;
	else if (x>p.x) return false;
	else return y<p.y;
}
void TPoint2D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==2, "Wrong size of vector in ::fromString");
	x = m.get_unsafe(0,0);
	y = m.get_unsafe(0,1);
}

TPose2D::TPose2D(const TPoint2D &p):x(p.x),y(p.y),phi(0.0)	{}
TPose2D::TPose2D(const TPoint3D &p):x(p.x),y(p.y),phi(0.0)	{}
TPose2D::TPose2D(const TPose3D &p):x(p.x),y(p.y),phi(p.yaw)	{}
TPose2D::TPose2D(const mrpt::poses::CPose2D &p):x(p.x()),y(p.y()),phi(p.phi())	{}
void TPose2D::asString(std::string &s) const {
	s = mrpt::format("[%f %f %f]",x,y,mrpt::utils::RAD2DEG(phi));
}

void TPose2D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==3, "Wrong size of vector in ::fromString");
	x = m.get_unsafe(0,0);
	y = m.get_unsafe(0,1);
	phi = DEG2RAD(m.get_unsafe(0,2));
}

// ----
void TTwist2D::asString(std::string &s) const {
	s = mrpt::format("[%f %f %f]",vx,vy,mrpt::utils::RAD2DEG(omega));
}
void TTwist2D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==3, "Wrong size of vector in ::fromString");
	vx = m.get_unsafe(0,0);
	vy = m.get_unsafe(0,1);
	omega = DEG2RAD(m.get_unsafe(0,2));
}
// Transform the (vx,vy) components for a counterclockwise rotation of `ang` radians
void TTwist2D::rotate(const double ang)
{
	const double nvx = vx * cos(ang) - vy * sin(ang);
	const double nvy = vx * sin(ang) + vy * cos(ang);
	vx = nvx;
	vy = nvy;
}

// ----
void TTwist3D::asString(std::string &s) const {
	s = mrpt::format("[%f %f %f  %f %f %f]",vx,vy,vz, mrpt::utils::RAD2DEG(wx),mrpt::utils::RAD2DEG(wy),mrpt::utils::RAD2DEG(wz));
}
void TTwist3D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==6, "Wrong size of vector in ::fromString");
	for (int i=0;i<3;i++) (*this)[i] = m.get_unsafe(0,i);
	for (int i=0;i<3;i++) (*this)[3+i] = DEG2RAD(m.get_unsafe(0,3+i));
}
// Transform all 6 components for a change of reference frame from "A" to 
// another frame "B" whose rotation with respect to "A" is given by `rot`. The translational part of the pose is ignored
void TTwist3D::rotate(const mrpt::poses::CPose3D & rot)
{
	const TTwist3D t = *this;
	const mrpt::math::CMatrixDouble33 & R = rot.getRotationMatrix();
	vx=R(0,0)*t.vx+R(0,1)*t.vy+R(0,2)*t.vz;
	vy=R(1,0)*t.vx+R(1,1)*t.vy+R(1,2)*t.vz;
	vz=R(2,0)*t.vx+R(2,1)*t.vy+R(2,2)*t.vz;

	wx=R(0,0)*t.wx+R(0,1)*t.wy+R(0,2)*t.wz;
	wy=R(1,0)*t.wx+R(1,1)*t.wy+R(1,2)*t.wz;
	wz=R(2,0)*t.wx+R(2,1)*t.wy+R(2,2)*t.wz;
}


TPoint3D::TPoint3D(const TPoint2D &p):x(p.x),y(p.y),z(0.0)	{}
TPoint3D::TPoint3D(const TPose2D &p):x(p.x),y(p.y),z(0.0)	{}
TPoint3D::TPoint3D(const TPose3D &p):x(p.x),y(p.y),z(p.z)	{}
TPoint3D::TPoint3D(const mrpt::poses::CPoint3D &p):x(p.x()),y(p.y()),z(p.z())	{}
TPoint3D::TPoint3D(const mrpt::poses::CPose3D &p):x(p.x()),y(p.y()),z(p.z())	{}
bool TPoint3D::operator<(const TPoint3D &p) const	{
	if (x<p.x) return true;
	else if (x>p.x) return false;
	else if (y<p.y) return true;
	else if (y>p.y) return false;
	else return z<p.z;
}
void TPoint3D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==3, "Wrong size of vector in ::fromString");
	x = m.get_unsafe(0,0);
	y = m.get_unsafe(0,1);
	z = m.get_unsafe(0,2);
}

TPose3D::TPose3D(const TPoint2D &p):x(p.x),y(p.y),z(0.0),yaw(0.0),pitch(0.0),roll(0.0)	{}
TPose3D::TPose3D(const TPose2D &p):x(p.x),y(p.y),z(0.0),yaw(p.phi),pitch(0.0),roll(0.0)	{}
TPose3D::TPose3D(const TPoint3D &p):x(p.x),y(p.y),z(p.z),yaw(0.0),pitch(0.0),roll(0.0)	{}
TPose3D::TPose3D(const mrpt::poses::CPose3D &p):x(p.x()),y(p.y()),z(p.z()),yaw(p.yaw()),pitch(p.pitch()),roll(p.roll())	{}
void TPose3D::asString(std::string &s) const {
	s = mrpt::format("[%f %f %f %f %f %f]",x,y,z,RAD2DEG(yaw),RAD2DEG(pitch),RAD2DEG(roll));
}
void TPose3D::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==6, "Wrong size of vector in ::fromString");
	x=m.get_unsafe(0,0);
	y=m.get_unsafe(0,1);
	z=m.get_unsafe(0,2);
	yaw=DEG2RAD(m.get_unsafe(0,3));
	pitch=DEG2RAD(m.get_unsafe(0,4));
	roll=DEG2RAD(m.get_unsafe(0,5));
}

TPose3DQuat::TPose3DQuat(const mrpt::poses::CPose3DQuat &p):x(p.x()),y(p.y()),z(p.z()),qr(p.quat().r()),qx(p.quat().x()),qy(p.quat().y()),qz(p.quat().z())
{
}
void TPose3DQuat::fromString(const std::string &s)
{
	CMatrixDouble  m;
	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==7, "Wrong size of vector in ::fromString");
	for (size_t i=0;i<m.getColCount();i++)
		(*this)[i] = m.get_unsafe(0,i);
}


// Text streaming:
std::ostream & operator << (std::ostream& o, const TPoint2D & p) { return (o << CPoint2D(p)); }
std::ostream & operator << (std::ostream& o, const TPoint3D & p) { return (o << CPoint3D(p)); }
std::ostream & operator << (std::ostream& o, const TPose2D & p)  { return (o << CPose2D(p)); }
std::ostream & operator << (std::ostream& o, const TPose3D & p)  { return (o << CPose3D(p)); }
std::ostream & operator << (std::ostream& o, const TPose3DQuat & p) { return (o << CPose3DQuat(p)); }

CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TSegment2D &s)	{ return in>>s.point1>>s.point2; }
CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TSegment2D &s)	{ return out<<s.point1<<s.point2; }
CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TLine2D &l)	{ return in>>l.coefs[0]>>l.coefs[1]>>l.coefs[2]; }
CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TLine2D &l)	{ return out<<l.coefs[0]<<l.coefs[1]<<l.coefs[2]; }

mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TSegment3D &s)	{ return in>>s.point1>>s.point2; }
mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TSegment3D &s)	{ return out<<s.point1<<s.point2; }
mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TLine3D &l)	{ return in>>l.pBase>>l.director[0]>>l.director[1]>>l.director[2]; }
mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TLine3D &l)	{ return out<<l.pBase<<l.director[0]<<l.director[1]<<l.director[2]; }
mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TPlane &p)	{ return in>>p.coefs[0]>>p.coefs[1]>>p.coefs[2]>>p.coefs[3]; }
mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TPlane &p)	{ return out<<p.coefs[0]<<p.coefs[1]<<p.coefs[2]<<p.coefs[3]; }

double TSegment2D::length() const	{
	return math::distance(point1,point2);
}
double TSegment2D::distance(const TPoint2D &point) const {
	return std::abs(signedDistance(point));
}
double TSegment2D::signedDistance(const TPoint2D &point) const	{
	//It is reckoned whether the perpendicular line to the TSegment2D which passes through point crosses or not the referred segment,
	//or what is the same, whether point makes an obtuse triangle with the segment or not (being the longest segment one between the point and either end of TSegment2D).
	double d1, d2, d3, ds1, ds2, ds3;
	d1 = math::distance(point,point1);
	d2 = math::distance(point,point2);
	d3 = length();
	ds1 = square(d1);
	ds2 = square(d2);
	ds3 = square(d3);
	if ( ds1 > (ds2 + ds3) || ds2 > (ds1 + ds3) )
		 // Fix sign:
		 return min(d1,d2) * ( TLine2D(*this).signedDistance(point)<0 ? -1:1);
	else return TLine2D(*this).signedDistance(point);
}
bool TSegment2D::contains(const TPoint2D &point) const	{
	return abs(math::distance(point1,point)+math::distance(point2,point)-math::distance(point1,point2))<geometryEpsilon;
}
void TSegment2D::generate3DObject(TSegment3D &s) const	{
	s=TSegment3D(*this);
}
TSegment2D::TSegment2D(const TSegment3D &s)	{
	point1=TPoint2D(s.point1);
	point2=TPoint2D(s.point2);
	if (point1==point2) throw std::logic_error("Segment is normal to projection plane");
}

bool TSegment2D::operator<(const TSegment2D &s) const	{
	if (point1<s.point1) return true;
	else if (s.point1<point1) return false;
	else return point2<s.point2;
}

double TSegment3D::length() const	{
	return math::distance(point1,point2);
}
double TSegment3D::distance(const TPoint3D &point) const	{
	return min(min(math::distance(point,point1),math::distance(point,point2)),TLine3D(*this).distance(point));
}
double TSegment3D::distance(const TSegment3D &segment) const	{
    Eigen::Vector3d u, v, w;
    TPoint3D diff_vect = point2 - point1;
    diff_vect.getAsVector(u);
    diff_vect = segment.point2 - segment.point1;
    diff_vect.getAsVector(v);
    diff_vect = point1 - segment.point1;
    diff_vect.getAsVector(w);
    double a = u.dot(u);        // always >= 0
    double b = u.dot(v);
    double c = v.dot(v);        // always >= 0
    double d = u.dot(w);
    double e = v.dot(w);
    double D = a*c - b*b;       // always >= 0
    double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < 0.00000001) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabs(sN) < 0.00000001 ? 0.0 : sN / sD);
    tc = (fabs(tN) < 0.00000001 ? 0.0 : tN / tD);

    // get the difference of the two closest points
    CVectorDouble dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

    return dP.norm();   // return the closest distance
}
bool TSegment3D::contains(const TPoint3D &point) const	{
	//Not very intuitive, but very fast, method.
	return abs(math::distance(point1,point)+math::distance(point2,point)-math::distance(point1,point2))<geometryEpsilon;
}

bool TSegment3D::operator<(const TSegment3D &s) const	{
	if (point1<s.point1) return true;
	else if (s.point1<point1) return false;
	else return point2<s.point2;
}

double TLine2D::evaluatePoint(const TPoint2D &point) const	{
	return coefs[0]*point.x+coefs[1]*point.y+coefs[2];
}
bool TLine2D::contains(const TPoint2D &point) const	{
	return abs(distance(point))<geometryEpsilon;
}
double TLine2D::distance(const TPoint2D &point) const	{
	return abs(evaluatePoint(point))/sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]);
}
double TLine2D::signedDistance(const TPoint2D &point) const	{
	return evaluatePoint(point)/sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]);
}
void TLine2D::getNormalVector(double (&vector)[2]) const	{
	vector[0]=coefs[0];
	vector[1]=coefs[1];
}
void TLine2D::unitarize()	{
	double s=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]);
	for (size_t i=0;i<3;i++) coefs[i]/=s;
}
void TLine2D::getDirectorVector(double (&vector)[2]) const	{
	vector[0]=-coefs[1];
	vector[1]=coefs[0];
}
void TLine2D::generate3DObject(TLine3D &l) const	{
	l=TLine3D(*this);
}
void TLine2D::getAsPose2D(mrpt::poses::CPose2D &outPose) const	{
	//Line's director vector is (-coefs[1],coefs[0]).
	//If line is horizontal, force x=0. Else, force y=0. In both cases, we'll find a suitable point.
	outPose.phi( atan2(coefs[0],-coefs[1]) );
	if (abs(coefs[0])<geometryEpsilon)	{
		outPose.x(0);
		outPose.y(-coefs[2]/coefs[1]);
	}	else	{
		outPose.x(-coefs[2]/coefs[0]);
		outPose.y(0);
	}
}
void TLine2D::getAsPose2DForcingOrigin(const TPoint2D &origin,mrpt::poses::CPose2D &outPose) const	{
	if (!contains(origin)) throw std::logic_error("Base point is not contained in the line");
	outPose=mrpt::poses::CPose2D(TPose2D(origin));
	//Line's director vector is (-coefs[1],coefs[0]).
	outPose.phi( atan2(coefs[0],-coefs[1]) );
}
TLine2D::TLine2D(const TPoint2D &p1,const TPoint2D &p2) throw(std::logic_error)	{
	if (p1==p2) throw logic_error("Both points are the same");
	coefs[0]=p2.y-p1.y;
	coefs[1]=p1.x-p2.x;
	coefs[2]=p2.x*p1.y-p2.y*p1.x;
}
TLine2D::TLine2D(const TSegment2D &s)	{
	coefs[0]=s.point2.y-s.point1.y;
	coefs[1]=s.point1.x-s.point2.x;
	coefs[2]=s.point2.x*s.point1.y-s.point2.y*s.point1.x;
	//unitarize();	//¿?
}
TLine2D::TLine2D(const TLine3D &l)	{
	//Line's projection to Z plane may be a point.
	if (hypot(l.director[0],l.director[1])<geometryEpsilon) throw std::logic_error("Line is normal to projection plane");
	coefs[0]=-l.director[1];
	coefs[1]=l.director[0];
	coefs[2]=l.pBase.x*l.director[1]-l.pBase.y*l.director[0];
}

bool TLine3D::contains(const TPoint3D &point) const	{
	double dx=point.x-pBase.x;
	double dy=point.y-pBase.y;
	double dz=point.z-pBase.z;
	if (abs(dx)<geometryEpsilon&&abs(dy)<geometryEpsilon&&abs(dz)<geometryEpsilon) return true;
	//       dx          dy          dz
	//if -----------=-----------=-----------, point is inside the line.
	//   director[0] director[1] director[2]
	return (abs(dx*director[1]-dy*director[0])<geometryEpsilon)&&(abs(dx*director[2]-dz*director[0])<geometryEpsilon)&&(abs(dy*director[2]-dz*director[1])<geometryEpsilon);
}
double TLine3D::distance(const TPoint3D &point) const	{
	//Let d be line's base point minus the argument. Then, d·director/(|d|·|director|) equals both vector's cosine.
	//So, d·director/|director| equals d's projection over director. Then, distance is sqrt(|d|²-(d·director/|director|)²).
	double d[3]={point.x-pBase.x,point.y-pBase.y,point.z-pBase.z};
	double dv=0,d2=0,v2=0;
	for (size_t i=0;i<3;i++)	{
		dv+=d[i]*director[i];
		d2+=d[i]*d[i];
		v2+=director[i]*director[i];
	}
	return sqrt(d2-(dv*dv)/v2);
}
void TLine3D::unitarize()	{
	double s=sqrt(squareNorm<3,double>(director));
	for (size_t i=0;i<3;i++) director[i]/=s;
}
TLine3D::TLine3D(const TPoint3D &p1,const TPoint3D &p2) throw(std::logic_error)	{
	if (abs(math::distance(p1,p2))<geometryEpsilon) throw logic_error("Both points are the same");
	pBase=p1;
	director[0]=p2.x-p1.x;
	director[1]=p2.y-p1.y;
	director[2]=p2.z-p1.z;
}
TLine3D::TLine3D(const TSegment3D &s)	{
	pBase=s.point1;
	director[0]=s.point2.x-s.point1.x;
	director[1]=s.point2.y-s.point1.y;
	director[2]=s.point2.z-s.point1.z;
}
TLine3D::TLine3D(const TLine2D &l)	{
	director[0]=-l.coefs[1];
	director[1]=l.coefs[0];
	director[2]=0;
	//We assume that either l.coefs[0] or l.coefs[1] is not null. Respectively, either y or x can be used as free cordinate.
	if (abs(l.coefs[0])>=geometryEpsilon)	{
		pBase.x=-l.coefs[2]/l.coefs[0];
		pBase.y=0;
	}	else	{
		pBase.x=0;
		pBase.y=-l.coefs[1]/l.coefs[0];
	}
	pBase.z=0;
}

double TPlane::evaluatePoint(const TPoint3D &point) const	{
	return dotProduct<3,double>(coefs,point)+coefs[3];
}
bool TPlane::contains(const TPoint3D &point) const	{
	return distance(point)<geometryEpsilon;
}
bool TPlane::contains(const TLine3D &line) const	{
	if (!contains(line.pBase)) return false;	//Base point must be contained
	return abs(getAngle(*this,line))<geometryEpsilon;	//Plane's normal must be normal to director vector
}
double TPlane::distance(const TPoint3D &point) const	{
	return abs(evaluatePoint(point))/sqrt(squareNorm<3,double>(coefs));
}
double TPlane::distance(const TLine3D &line) const	{
	if (abs(getAngle(*this,line))>=geometryEpsilon) return 0;	//Plane crosses with line
	else return distance(line.pBase);
}
void TPlane::getNormalVector(double (&vector)[3]) const	{
	vector[0]=coefs[0];
	vector[1]=coefs[1];
	vector[2]=coefs[2];
}
void TPlane::unitarize()	{
	double s=sqrt(squareNorm<3,double>(coefs));
	for (size_t i=0;i<4;i++) coefs[i]/=s;
}

// Returns a 6D pose such as its XY plane coincides with the plane
void TPlane::getAsPose3D(mrpt::poses::CPose3D &outPose)	{
	double normal[3];
	getUnitaryNormalVector(normal);
	CMatrixDouble AXIS;
	generateAxisBaseFromDirectionAndAxis(normal,2,AXIS);
	AXIS.set_unsafe(3,3,1);
	for (size_t i=0;i<3;i++) if (abs(coefs[i])>=geometryEpsilon)	{
		AXIS.set_unsafe(i,3,-coefs[3]/coefs[i]);
		break;
	}
	outPose=mrpt::poses::CPose3D(AXIS);
}
void TPlane::getAsPose3DForcingOrigin(const TPoint3D &newOrigin,mrpt::poses::CPose3D &pose)	{
	if (!contains(newOrigin)) throw std::logic_error("Base point is not in the plane.");
	double normal[3];
	getUnitaryNormalVector(normal);
	CMatrixDouble AXIS;
	generateAxisBaseFromDirectionAndAxis(normal,2,AXIS);
	AXIS.set_unsafe(3,3,1);
	for (size_t i=0;i<3;i++) AXIS.set_unsafe(i,3,newOrigin[i]);
	pose=mrpt::poses::CPose3D(AXIS);
}
TPlane::TPlane(const TPoint3D &p1,const TPoint3D &p2,const TPoint3D &p3)  throw(std::logic_error) {
	double dx1=p2.x-p1.x;
	double dy1=p2.y-p1.y;
	double dz1=p2.z-p1.z;
	double dx2=p3.x-p1.x;
	double dy2=p3.y-p1.y;
	double dz2=p3.z-p1.z;
	coefs[0]=dy1*dz2-dy2*dz1;
	coefs[1]=dz1*dx2-dz2*dx1;
	coefs[2]=dx1*dy2-dx2*dy1;
	if (abs(coefs[0])<geometryEpsilon&&abs(coefs[1])<geometryEpsilon&&abs(coefs[2])<geometryEpsilon) throw logic_error("Points are linearly dependent");
	coefs[3]=-coefs[0]*p1.x-coefs[1]*p1.y-coefs[2]*p1.z;
}
TPlane::TPlane(const TPoint3D &p1,const TLine3D &r2) throw(std::logic_error)	{
	double dx1=p1.x-r2.pBase.x;
	double dy1=p1.y-r2.pBase.y;
	double dz1=p1.z-r2.pBase.z;
	coefs[0]=dy1*r2.director[2]-dz1*r2.director[1];
	coefs[1]=dz1*r2.director[0]-dx1*r2.director[2];
	coefs[2]=dx1*r2.director[1]-dy1*r2.director[0];
	if (abs(coefs[0])<geometryEpsilon&&abs(coefs[1])<geometryEpsilon&&abs(coefs[2])<geometryEpsilon) throw logic_error("Point is contained in the line");
	coefs[3]=-coefs[0]*p1.x-coefs[1]*p1.y-coefs[2]*p1.z;
}
TPlane::TPlane(const TLine3D &r1,const TLine3D &r2) throw(std::logic_error)	{
	crossProduct3D(r1.director,r2.director,coefs);
	coefs[3]=-coefs[0]*r1.pBase.x-coefs[1]*r1.pBase.y-coefs[2]*r1.pBase.z;
	if (abs(coefs[0])<geometryEpsilon&&abs(coefs[1])<geometryEpsilon&&abs(coefs[2])<geometryEpsilon)	{
		//Lines are parallel
		if (r1.contains(r2.pBase)) throw std::logic_error("Lines are the same");
		//Use a line's director vector and both pBase's difference to create the plane.
		double d[3];
		for (size_t i=0;i<3;i++) d[i]=r1.pBase[i]-r2.pBase[i];
		crossProduct3D(r1.director,d,coefs);
		coefs[3]=-coefs[0]*r1.pBase.x-coefs[1]*r1.pBase.y-coefs[2]*r1.pBase.z;
	}	else if (abs(evaluatePoint(r2.pBase))>=geometryEpsilon) throw logic_error("Lines do not intersect");
}

template<class T>
inline void removeUnusedVertices(T &poly)	{
	size_t N=poly.size();
	if (N<3) return;
	std::vector<size_t> unused;
	if (abs(mrpt::math::distance(poly[N-1],poly[0])+mrpt::math::distance(poly[0],poly[1])-mrpt::math::distance(poly[N-1],poly[1]))<mrpt::math::geometryEpsilon) unused.push_back(0);
	for (size_t i=1;i<N-1;i++) if (abs(mrpt::math::distance(poly[i-1],poly[i])+mrpt::math::distance(poly[i],poly[i+1])-mrpt::math::distance(poly[i-1],poly[i+1]))<mrpt::math::geometryEpsilon) unused.push_back(i);
	if (abs(mrpt::math::distance(poly[N-2],poly[N-1])+mrpt::math::distance(poly[N-1],poly[0])-mrpt::math::distance(poly[N-2],poly[0]))<mrpt::math::geometryEpsilon) unused.push_back(N-1);
	unused.push_back(N);
	size_t diff=1;
	for (size_t i=0;i<unused.size()-1;i++)	{
		size_t last=unused[i+1];
		for (size_t j=unused[i]+1-diff;j<last-diff;j++) poly[j]=poly[j+diff];
	}
	poly.resize(N+1-unused.size());
}
template<class T>
inline void removeRepVertices(T &poly)	{
	size_t N=poly.size();
	if (N<3) return;
	std::vector<size_t> rep;
	for (size_t i=0;i<N-1;i++) if (mrpt::math::distance(poly[i],poly[i+1])<geometryEpsilon) rep.push_back(i);
	if (mrpt::math::distance(poly[N-1],poly[0])<geometryEpsilon) rep.push_back(N-1);
	rep.push_back(N);
	size_t diff=1;
	for (size_t i=0;i<rep.size()-1;i++)	{
		size_t last=rep[i+1];
		for (size_t j=rep[i]+1-diff;j<last-diff;j++) poly[j]=poly[j+diff];
	}
	poly.resize(N+1-rep.size());
}

double TPolygon2D::distance(const TPoint2D &point) const	{
	if ( contains(point) )
		return 0;
	std::vector<TSegment2D> sgs;
	getAsSegmentList(sgs);

	if (sgs.empty())
		THROW_EXCEPTION("Cannot compute distance to an empty polygon.")

	double distance = std::numeric_limits<double>::max();

	for (vector<TSegment2D>::const_iterator it=sgs.begin();it!=sgs.end();++it)
	{
		double d = (*it).distance(point);
		if( d < distance )
			distance = d;
	}
	return distance;
}

void TPolygon2D::getBoundingBox(TPoint2D &min_coords, TPoint2D&max_coords) const
{
	ASSERTMSG_(!this->empty(),"getBoundingBox() called on an empty polygon!");
	min_coords.x = min_coords.y =  std::numeric_limits<double>::max();
	max_coords.x = max_coords.y = -std::numeric_limits<double>::max();
	for (size_t i=0;i<size();i++)
	{
		mrpt::utils::keep_min( min_coords.x, (*this)[i].x);
		mrpt::utils::keep_min( min_coords.y, (*this)[i].y);
		mrpt::utils::keep_max( max_coords.x, (*this)[i].x);
		mrpt::utils::keep_max( max_coords.y, (*this)[i].y);
	}
}

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
inline double isLeft( const mrpt::math::TPoint2D &P0, const mrpt::math::TPoint2D &P1, const mrpt::math::TPoint2D &P2 )
{
	return ( (P1.x - P0.x) * (P2.y - P0.y) - (P2.x -  P0.x) * (P1.y - P0.y) );
}

bool TPolygon2D::contains(const TPoint2D &P) const	
{
	int wn = 0;    // the  winding number counter

	// loop through all edges of the polygon
	const size_t n = this->size();
	for (size_t i=0; i<n; i++) // edge from V[i] to  V[i+1]
	{
		if ((*this)[i].y <= P.y) 
		{
			// start y <= P.y
			if ((*this)[(i+1) % n].y  > P.y)      // an upward crossing
				if (isLeft( (*this)[i], (*this)[(i+1)%n], P) > 0)  // P left of  edge
					++wn;            // have  a valid up intersect
	}
	else 
	{
		// start y > P.y (no test needed)
		if ((*this)[(i+1)%n].y  <= P.y)     // a downward crossing
			if (isLeft( (*this)[i], (*this)[(i+1)%n], P) < 0)  // P right of  edge
				--wn;            // have  a valid down intersect
		}
	}

	return wn!=0;

}
void TPolygon2D::getAsSegmentList(vector<TSegment2D> &v) const	{
	size_t N=size();
	v.resize(N);
	for (size_t i=0;i<N-1;i++) v[i]=TSegment2D(operator[](i),operator[](i+1));
	v[N-1]=TSegment2D(operator[](N-1),operator[](0));
}

void TPolygon2D::generate3DObject(TPolygon3D &p) const	{
	p=TPolygon3D(*this);
}
//Auxiliar functor class to compute polygon's center
template<class T,int N>
class FAddPoint	{
public:
	T &object;
	FAddPoint(T &o):object(o)	{
		for (size_t i=0;i<N;i++) object[i]=0.0;
	}
	void operator()(const T &o)	{
		for (size_t i=0;i<N;i++) object[i]+=o[i];
	}
};
void TPolygon2D::getCenter(TPoint2D &p) const	{
	for_each(begin(),end(),FAddPoint<TPoint2D,2>(p));
	size_t N=size();
	p.x/=N;
	p.y/=N;
}
bool TPolygon2D::isConvex() const	{
	size_t N=size();
	if (N<=3) return true;
	vector<TSegment2D> sgms;
	getAsSegmentList(sgms);
	for (size_t i=0;i<N;i++)	{
		char s=0;
		TLine2D l=TLine2D(sgms[i]);
		for (size_t j=0;j<N;j++)	{
			double d=l.evaluatePoint(operator[](j));
			if (abs(d)<geometryEpsilon) continue;
			else if (!s) s=(d>0)?1:-1;
			else if (s!=((d>0)?1:-1)) return false;
		}
	}
	return true;
}
void TPolygon2D::removeRepeatedVertices()	{
	removeRepVertices(*this);
}
void TPolygon2D::removeRedundantVertices()	{
	removeRepeatedVertices();
	removeUnusedVertices(*this);
}
void TPolygon2D::getPlotData(std::vector<double> &x,std::vector<double> &y) const	{
	size_t N=size();
	x.resize(N+1);
	y.resize(N+1);
	for (size_t i=0;i<N;i++)	{
		x[i]=operator[](i).x;
		y[i]=operator[](i).y;
	}
	x[N]=operator[](0).x;
	y[N]=operator[](0).y;
}
TPolygon2D::TPolygon2D(const TPolygon3D &p):std::vector<TPoint2D>()	{
	size_t N=p.size();
	resize(N);
	for (size_t i=0;i<N;i++) operator[](i)=TPoint2D(p[i]);
}
void TPolygon2D::createRegularPolygon(size_t numEdges,double radius,TPolygon2D &poly)	{
	if (numEdges<3||abs(radius)<geometryEpsilon) throw std::logic_error("Invalid arguments for regular polygon creations");
	poly.resize(numEdges);
	for (size_t i=0;i<numEdges;i++)	{
		double angle=i*M_PI*2/numEdges;
		poly[i]=TPoint2D(radius*cos(angle),radius*sin(angle));
	}
}
inline void TPolygon2D::createRegularPolygon(size_t numEdges,double radius,TPolygon2D &poly,const mrpt::poses::CPose2D &pose)	{
	createRegularPolygon(numEdges,radius,poly);
	for (size_t i=0;i<numEdges;i++) poly[i]=pose+poly[i];
}

double TPolygon3D::distance(const TPoint3D &point) const	{
	TPlane pl;
	if (!getPlane(pl)) throw std::logic_error("Polygon does not conform a plane");
	TPoint3D newPoint;
	TPolygon3D newPoly;
	mrpt::poses::CPose3D pose;
	pl.getAsPose3DForcingOrigin(operator[](0),pose);
	project3D(point,pose,newPoint);
	project3D(*this,pose,newPoly);
	double distance2D=TPolygon2D(newPoly).distance(TPoint2D(newPoint));
	return sqrt(newPoint.z*newPoint.z+distance2D*distance2D);
}
bool TPolygon3D::contains(const TPoint3D &point) const	{
	TPoint3D pMin,pMax;
	getPrismBounds(*this,pMin,pMax);
	if (point.x+geometryEpsilon<pMin.x||point.y+geometryEpsilon<pMin.y||point.z+geometryEpsilon<pMin.z||point.x>pMax.x+geometryEpsilon||point.y>pMax.y+geometryEpsilon||point.z>pMax.z+geometryEpsilon) return false;
	TPlane plane;
	if (!getPlane(plane)) throw std::logic_error("Polygon does not conform a plane");
	TPolygon3D projectedPoly;
	TPoint3D projectedPoint;
	mrpt::poses::CPose3D pose;
	//plane.getAsPose3DForcingOrigin(operator[](0),pose);
	plane.getAsPose3D(pose);
	pose=mrpt::poses::CPose3D(0,0,0,0,0,0)-pose;
	project3D(point,pose,projectedPoint);
	if (abs(projectedPoint.z)>=geometryEpsilon) return false;	//Point is not inside the polygon's plane.
	project3D(*this,pose,projectedPoly);
	return TPolygon2D(projectedPoly).contains(TPoint2D(projectedPoint));
}
void TPolygon3D::getAsSegmentList(vector<TSegment3D> &v) const	{
	size_t N=size();
	v.resize(N);
	for (size_t i=0;i<N-1;i++) v[i]=TSegment3D(operator[](i),operator[](i+1));
	v[N-1]=TSegment3D(operator[](N-1),operator[](0));
}
bool TPolygon3D::getPlane(TPlane &p) const	{
	return conformAPlane(*this,p);
}
void TPolygon3D::getBestFittingPlane(TPlane &p) const	{
	getRegressionPlane(*this,p);
}
void TPolygon3D::getCenter(TPoint3D &p) const	{
	for_each(begin(),end(),FAddPoint<TPoint3D,3>(p));
	size_t N=size();
	p.x/=N;
	p.y/=N;
	p.z/=N;
}
bool TPolygon3D::isSkew() const	{
	return !mrpt::math::conformAPlane(*this);
}
void TPolygon3D::removeRepeatedVertices()	{
	removeRepVertices(*this);
}
void TPolygon3D::removeRedundantVertices()	{
	removeRepeatedVertices();
	removeUnusedVertices(*this);
}
TPolygon3D::TPolygon3D(const TPolygon2D &p):std::vector<TPoint3D>()	{
	size_t N=p.size();
	resize(N);
	for (size_t i=0;i<N;i++) operator[](i)=p[i];
}
void TPolygon3D::createRegularPolygon(size_t numEdges,double radius,TPolygon3D &poly)	{
	if (numEdges<3||abs(radius)<geometryEpsilon) throw std::logic_error("Invalid arguments for regular polygon creations");
	poly.resize(numEdges);
	for (size_t i=0;i<numEdges;i++)	{
		double angle=i*2*M_PI/numEdges;
		poly[i]=TPoint3D(radius*cos(angle),radius*sin(angle),0);
	}
}
inline void TPolygon3D::createRegularPolygon(size_t numEdges,double radius,TPolygon3D &poly,const mrpt::poses::CPose3D &pose)	{
	createRegularPolygon(numEdges,radius,poly);
	for (size_t i=0;i<numEdges;i++) pose.composePoint(poly[i][0],poly[i][1],poly[i][2],poly[i][0],poly[i][1],poly[i][2]);
}

void TObject2D::generate3DObject(TObject3D &obj) const	{
	switch (type)	{
		case GEOMETRIC_TYPE_POINT:
			obj=TPoint3D(data.point);
			break;
		case GEOMETRIC_TYPE_SEGMENT:
			obj=TSegment3D(data.segment);
			break;
		case GEOMETRIC_TYPE_LINE:
			obj=TLine3D(data.line);
			break;
		case GEOMETRIC_TYPE_POLYGON:
			obj=TPolygon3D(*(data.polygon));
			break;
		default:
			obj=TObject3D();
			break;
	}
}
void TObject2D::getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPoint()) pnts.push_back(it->data.point);
}
void TObject2D::getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isSegment()) sgms.push_back(it->data.segment);
}
void TObject2D::getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isLine()) lins.push_back(it->data.line);
}
void TObject2D::getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPolygon()) polys.push_back(*(it->data.polygon));
}
void TObject2D::getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts,std::vector<TObject2D> &remainder)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPoint()) pnts.push_back(it->data.point);
	else remainder.push_back(*it);
}
void TObject2D::getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms,std::vector<TObject2D> &remainder)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isSegment()) sgms.push_back(it->data.segment);
	else remainder.push_back(*it);
}
void TObject2D::getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins,std::vector<TObject2D> &remainder)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isLine()) lins.push_back(it->data.line);
	else remainder.push_back(*it);
}
void TObject2D::getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys,vector<TObject2D> &remainder)	{
	for (vector<TObject2D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPolygon()) polys.push_back(*(it->data.polygon));
	else remainder.push_back(*it);
}
void TObject3D::getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPoint()) pnts.push_back(it->data.point);
}
void TObject3D::getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isSegment()) sgms.push_back(it->data.segment);
}
void TObject3D::getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isLine()) lins.push_back(it->data.line);
}
void TObject3D::getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPlane()) plns.push_back(it->data.plane);
}
void TObject3D::getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPolygon()) polys.push_back(*(it->data.polygon));
}
void TObject3D::getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts,std::vector<TObject3D> &remainder)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPoint()) pnts.push_back(it->data.point);
	else remainder.push_back(*it);
}
void TObject3D::getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms,std::vector<TObject3D> &remainder)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isSegment()) sgms.push_back(it->data.segment);
	else remainder.push_back(*it);
}
void TObject3D::getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins,std::vector<TObject3D> &remainder)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isLine()) lins.push_back(it->data.line);
	else remainder.push_back(*it);
}
void TObject3D::getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns,std::vector<TObject3D> &remainder)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPlane()) plns.push_back(it->data.plane);
	else remainder.push_back(*it);
}
void TObject3D::getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys,vector<TObject3D> &remainder)	{
	for (vector<TObject3D>::const_iterator it=objs.begin();it!=objs.end();++it) if (it->isPolygon()) polys.push_back(*(it->data.polygon));
	else remainder.push_back(*it);
}

mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPoint2D &o)
{
	in >> o.x >> o.y;
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPoint2D &o)
{
	out << o.x << o.y;
	return out;
}

mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPoint3D &o)
{
	in >> o.x >> o.y >> o.z;
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPoint3D &o)
{
	out << o.x << o.y << o.z;
	return out;
}

mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPose2D &o)
{
	in >> o.x >> o.y >> o.phi;
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPose2D &o)
{
	out << o.x << o.y << o.phi;
	return out;
}

mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TTwist2D &o)
{
    for (unsigned int i=0;i<o.size();i++) in >> o[i];
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TTwist2D &o)
{
    for (unsigned int i=0;i<o.size();i++) out << o[i];
	return out;
}

mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TTwist3D &o)
{
    for (unsigned int i=0;i<o.size();i++) in >> o[i];
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TTwist3D &o)
{
    for (unsigned int i=0;i<o.size();i++) out << o[i];
	return out;
}

	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TTwist2D &o);
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TTwist2D &o);

	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TTwist3D &o);
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TTwist3D &o);


mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPose3D &o)
{
	in >> o.x >> o.y >> o.z >> o.yaw >> o.pitch >> o.roll;
	return in;
}
mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPose3D &o)
{
	out << o.x << o.y << o.z << o.yaw << o.pitch << o.roll;
	return out;
}

mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TObject2D &o)	{
	uint16_t type;
	in>>type;
	switch (static_cast<unsigned char>(type))	{
		case GEOMETRIC_TYPE_POINT:	{
			TPoint2D p;
			in>>p;
			o=p;
		}	break;
		case GEOMETRIC_TYPE_SEGMENT:	{
			TSegment2D s;
			in>>s;
			o=s;
		}	break;
		case GEOMETRIC_TYPE_LINE:	{
			TLine2D l;
			in>>l;
			o=l;
		}	break;
		case GEOMETRIC_TYPE_POLYGON:	{
			TPolygon2D p;
			in>>p;
			o=p;
		}	break;
		case GEOMETRIC_TYPE_UNDEFINED:	{
			o=TObject2D();
		}	break;
		default:
			throw std::logic_error("Unknown TObject2D type found while reading stream");
	}
	return in;
}
mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TObject2D &o)	{
	out<<static_cast<uint16_t>(o.getType());
	switch (o.getType())	{
		case GEOMETRIC_TYPE_POINT:	{
			TPoint2D p;
			o.getPoint(p);
			return out<<p;
		};
		case GEOMETRIC_TYPE_SEGMENT:	{
			TSegment2D s;
			o.getSegment(s);
			return out<<s;
		};
		case GEOMETRIC_TYPE_LINE:	{
			TLine2D l;
			o.getLine(l);
			return out<<l;
		};
		case GEOMETRIC_TYPE_POLYGON:	{
			TPolygon2D p;
			o.getPolygon(p);
			return out<<p;
		};
	}
	return out;
}

mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TObject3D &o)	{
	uint16_t type;
	in>>type;
	switch (static_cast<unsigned char>(type))	{
		case GEOMETRIC_TYPE_POINT:	{
			TPoint3D p;
			in>>p;
			o=p;
		}	break;
		case GEOMETRIC_TYPE_SEGMENT:	{
			TSegment3D s;
			in>>s;
			o=s;
		}	break;
		case GEOMETRIC_TYPE_LINE:	{
			TLine3D l;
			in>>l;
			o=l;
		}	break;
		case GEOMETRIC_TYPE_PLANE:	{
			TPlane p;
			in>>p;
			o=p;
		}	break;
		case GEOMETRIC_TYPE_POLYGON:	{
			TPolygon3D p;
			in>>p;
			o=p;
		}	break;
		case GEOMETRIC_TYPE_UNDEFINED:	{
			o=TObject3D();
		}	break;
		default:
			throw std::logic_error("Unknown TObject3D type found while reading stream");
	}
	return in;
}
mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TObject3D &o)	{
	out<<static_cast<uint16_t>(o.getType());
	switch (o.getType())	{
		case GEOMETRIC_TYPE_POINT:	{
			TPoint3D p;
			o.getPoint(p);
			return out<<p;
		};
		case GEOMETRIC_TYPE_SEGMENT:	{
			TSegment3D s;
			o.getSegment(s);
			return out<<s;
		};
		case GEOMETRIC_TYPE_LINE:	{
			TLine3D l;
			o.getLine(l);
			return out<<l;
		};
		case GEOMETRIC_TYPE_PLANE:	{
			TPlane p;
			o.getPlane(p);
			return out<<p;
		};
		case GEOMETRIC_TYPE_POLYGON:	{
			TPolygon3D p;
			o.getPolygon(p);
			return out<<p;
		};
	}
	return out;
}

}}	//end of namespaces
