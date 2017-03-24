/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "topography-precomp.h"  // Precompiled headers

#include <mrpt/topography/conversions.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/geometry.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;


bool mrpt::topography::operator ==(const TCoords &a, const TCoords &o) { return a.decimal_value==o.decimal_value; }
bool mrpt::topography::operator !=(const TCoords &a, const TCoords &o) { return !(a==o); }
bool mrpt::topography::operator ==(const TGeodeticCoords &a, const TGeodeticCoords &o) { return a.lat==o.lat && a.lon==o.lon && a.height==o.height; }
bool mrpt::topography::operator !=(const TGeodeticCoords &a, const TGeodeticCoords &o) { return !(a==o); }


//#define M_PI_TOPO 3.141592653589793
//inline double DEG2RAD(const double x) { return x*M_PI_TOPO/180.0;	}

// Define a generic type for "precission numbers":
#ifdef HAVE_LONG_DOUBLE
	typedef long double  precnum_t;
#else
	typedef double       precnum_t;
#endif


std::ostream& mrpt::topography::operator<<( std::ostream& out, const TCoords &o )
{
	return out << o.getAsString();
}


/*---------------------------------------------------------------
				geocentricToENU_WGS84
 ---------------------------------------------------------------*/
void  mrpt::topography::geocentricToENU_WGS84(
	const mrpt::math::TPoint3D  &P_geocentric_,
	mrpt::math::TPoint3D        &out_ENU_point,
	const TGeodeticCoords       &in_coords_origin )
{
	// Generate reference 3D point:
	TPoint3D P_geocentric_ref;
	geodeticToGeocentric_WGS84(in_coords_origin,P_geocentric_ref);

	const double clat = cos(DEG2RAD(in_coords_origin.lat)), slat = sin(DEG2RAD(in_coords_origin.lat));
	const double clon = cos(DEG2RAD(in_coords_origin.lon)), slon = sin(DEG2RAD(in_coords_origin.lon));

	// Compute the resulting relative coordinates:
	// For using smaller numbers:
	const mrpt::math::TPoint3D P_geocentric = P_geocentric_ - P_geocentric_ref;

	// Optimized calculation: Local transformed coordinates of P_geo(x,y,z)
	//   after rotation given by the transposed rotation matrix from ENU -> ECEF.
	out_ENU_point.x = -slon*P_geocentric.x + clon*P_geocentric.y;
	out_ENU_point.y = -clon*slat*P_geocentric.x -slon*slat*P_geocentric.y + clat*P_geocentric.z;
	out_ENU_point.z = clon*clat*P_geocentric.x + slon*clat*P_geocentric.y +slat*P_geocentric.z;
}

void mrpt::topography::geocentricToENU_WGS84(
	const std::vector<mrpt::math::TPoint3D> &in_geocentric_points,
	std::vector<mrpt::math::TPoint3D>       &out_ENU_points,
	const TGeodeticCoords       &in_coords_origin )
{
	// Generate reference 3D point:
	TPoint3D P_geocentric_ref;
	geodeticToGeocentric_WGS84(in_coords_origin,P_geocentric_ref);

	const double clat = cos(DEG2RAD(in_coords_origin.lat)), slat = sin(DEG2RAD(in_coords_origin.lat));
	const double clon = cos(DEG2RAD(in_coords_origin.lon)), slon = sin(DEG2RAD(in_coords_origin.lon));

	const size_t N= in_geocentric_points.size();
	out_ENU_points.resize(N);
	for (size_t i=0;i<N;i++)
	{
		// Compute the resulting relative coordinates for using smaller numbers:
		const mrpt::math::TPoint3D P_geocentric = in_geocentric_points[i] - P_geocentric_ref;

		// Optimized calculation: Local transformed coordinates of P_geo(x,y,z)
		//   after rotation given by the transposed rotation matrix from ENU -> ECEF.
		out_ENU_points[i].x = -slon*P_geocentric.x + clon*P_geocentric.y;
		out_ENU_points[i].y = -clon*slat*P_geocentric.x -slon*slat*P_geocentric.y + clat*P_geocentric.z;
		out_ENU_points[i].z = clon*clat*P_geocentric.x + slon*clat*P_geocentric.y +slat*P_geocentric.z;
	}
}

/*---------------------------------------------------------------
				geodeticToENU_WGS84
 ---------------------------------------------------------------*/
void  mrpt::topography::geodeticToENU_WGS84(
	const TGeodeticCoords		&in_coords,
	mrpt::math::TPoint3D	&out_ENU_point,
	const TGeodeticCoords		&in_coords_origin )
{
	// --------------------------------------------------------------------
	//  Explanation: We compute the earth-centric coordinates first,
	//    then make a system transformation to local XYZ coordinates
	//    using a system of three orthogonal vectors as local reference.
	//
	// See: http://en.wikipedia.org/wiki/Reference_ellipsoid
	// (JLBC 21/DEC/2006)  (Fixed: JLBC 9/JUL/2008)
	// - Oct/2013, Emilio Sanjurjo: Fixed UP vector pointing exactly normal to ellipsoid surface.
	// --------------------------------------------------------------------
	// Generate 3D point:
	TPoint3D	P_geocentric;
	geodeticToGeocentric_WGS84(in_coords,P_geocentric);

	geocentricToENU_WGS84(P_geocentric,out_ENU_point,in_coords_origin);
}

/*---------------------------------------------------------------
					ENU_axes_from_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::ENU_axes_from_WGS84(
	double		in_longitude_reference_degrees,
	double		in_latitude_reference_degrees,
	double		in_height_reference_meters,
	mrpt::math::TPose3D		&out_ENU,
	bool					only_angles
	)
{
	// See "coordinatesTransformation_WGS84" for more comments.
	TPoint3D PPref;
	mrpt::topography::geodeticToGeocentric_WGS84(
		TGeodeticCoords(in_latitude_reference_degrees, in_longitude_reference_degrees, in_height_reference_meters),
		PPref );

	const double clat = cos(DEG2RAD(in_latitude_reference_degrees)), slat = sin(DEG2RAD(in_latitude_reference_degrees));
	const double clon = cos(DEG2RAD(in_longitude_reference_degrees)), slon = sin(DEG2RAD(in_longitude_reference_degrees));

	CMatrixDouble44	 HM; // zeros by default
	HM(0,0) = -slon;   HM(0,1) = -clon*slat;  HM(0,2) = clon*clat;
	HM(1,0) = clon;    HM(1,1) = -slon*slat;  HM(1,2) = slon*clat;
	HM(2,0) = 0;       HM(2,1) = clat;       HM(2,2) = slat;
	HM(3,3)=1;

	if (!only_angles)
	{
		HM(0,3) = PPref.x;
		HM(1,3) = PPref.y;
		HM(2,3) = PPref.z;
	}

	out_ENU = mrpt::math::TPose3D(CPose3D(HM));
}

//*---------------------------------------------------------------
//			geodeticToGeocentric_WGS84
// ---------------------------------------------------------------*/
void  mrpt::topography::geodeticToGeocentric_WGS84(
	const TGeodeticCoords		&in_coords,
	mrpt::math::TPoint3D  &out_point )
{
	// --------------------------------------------------------------------
	// See: http://en.wikipedia.org/wiki/Reference_ellipsoid
	//  Constants are for WGS84
	// --------------------------------------------------------------------

	static const precnum_t a = 6378137L;		// Semi-major axis of the Earth (meters)
	static const precnum_t b = 6356752.3142L;	// Semi-minor axis:

	static const precnum_t ae = acos(b/a);  	// eccentricity:
	static const precnum_t cos2_ae_earth =  square(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
	static const precnum_t sin2_ae_earth = square(sin(ae));  // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

	const precnum_t lon  = DEG2RAD( precnum_t(in_coords.lon) );
	const precnum_t lat  = DEG2RAD( precnum_t(in_coords.lat) );

	// The radius of curvature in the prime vertical:
	const precnum_t N = a / std::sqrt( 1 - sin2_ae_earth*square( sin(lat) ) );

	// Generate 3D point:
	out_point.x = (N+in_coords.height)*cos(lat)*cos(lon);
	out_point.y = (N+in_coords.height)*cos(lat)*sin(lon);
	out_point.z = (cos2_ae_earth*N+in_coords.height)*sin(lat);
}

/*---------------------------------------------------------------
			geodeticToGeocentric_WGS84
 ---------------------------------------------------------------*/
void  mrpt::topography::geodeticToGeocentric(
	const TGeodeticCoords	&in_coords,
	TGeocentricCoords		&out_point,
	const TEllipsoid		&ellip )
{
	static const precnum_t a = ellip.sa;		// Semi-major axis of the Earth (meters)
	static const precnum_t b = ellip.sb;	// Semi-minor axis:

	static const precnum_t ae = acos(b/a);  	// eccentricity:
	static const precnum_t cos2_ae_earth =  square(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
	static const precnum_t sin2_ae_earth = square(sin(ae));  // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

	const precnum_t lon  = DEG2RAD( precnum_t(in_coords.lon) );
	const precnum_t lat  = DEG2RAD( precnum_t(in_coords.lat) );

	// The radius of curvature in the prime vertical:
	const precnum_t N = a / std::sqrt( 1 - sin2_ae_earth*square( sin(lat) ) );

	// Generate 3D point:
	out_point.x = (N+in_coords.height)*cos(lat)*cos(lon);
	out_point.y = (N+in_coords.height)*cos(lat)*sin(lon);
	out_point.z = (cos2_ae_earth*N+in_coords.height)*sin(lat);
}

/*---------------------------------------------------------------
			geocentricToGeodetic
 ---------------------------------------------------------------*/
void  mrpt::topography::geocentricToGeodetic(
	const TGeocentricCoords		&in_point,
	TGeodeticCoords				&out_coords,
	const TEllipsoid			&ellip )
{
	const double sa2	= ellip.sa*ellip.sa;
	const double sb2	= ellip.sb*ellip.sb;

	const double e2		= (sa2-sb2)/sa2;
	const double ep2	= (sa2-sb2)/sb2;
	const double p		= sqrt( in_point.x*in_point.x + in_point.y*in_point.y );
	const double theta	= atan2(in_point.z*ellip.sa,p*ellip.sb);

	out_coords.lon		= atan2( in_point.y, in_point.x );
	out_coords.lat		= atan2( in_point.z + ep2*ellip.sb*sin(theta)*sin(theta)*sin(theta),p-e2*ellip.sa*cos(theta)*cos(theta)*cos(theta) );

	const double clat	= cos( out_coords.lat );
	const double slat	= sin( out_coords.lat );
	const double N		= sa2/sqrt( sa2*clat*clat+sb2*slat*slat );

	out_coords.height	= p/clat - N;

	out_coords.lon		= RAD2DEG( out_coords.lon );
	out_coords.lat		= RAD2DEG( out_coords.lat );
}

/*---------------------------------------------------------------
					UTMToGeodesic
 ---------------------------------------------------------------*/
void mrpt::topography::UTMToGeodetic(
	double		X,
	double		Y,
	int			huso,
	char		hem,
	double		&out_lon /*degrees*/,
	double		&out_lat /*degrees*/,
	const TEllipsoid	&ellip )
{
	ASSERT_(hem=='s' || hem=='S' || hem=='n' || hem=='N');

	X = X - 5e5;
	if( hem == 's' || hem == 'S' )
		Y = Y - 1e7;

	const precnum_t lon0 = huso*6-183;
	const precnum_t a2	= ellip.sa*ellip.sa;
	const precnum_t b2	= ellip.sb*ellip.sb;
	const precnum_t ep2	= (a2-b2)/b2;
	const precnum_t c	= a2/ellip.sb;
	const precnum_t latp = Y/( 6366197.724*0.9996 );
	const precnum_t clp2 = square(cos(latp));

	const precnum_t v	= c*0.9996/sqrt( 1+ep2*clp2 );
	const precnum_t na	= X/v;
	const precnum_t A1	= sin( 2*latp );
	const precnum_t A2	= A1*clp2;
	const precnum_t J2	= latp+A1*0.5;
	const precnum_t J4	= 0.75*J2+0.25*A2;
	const precnum_t J6	= (5*J4+A2*clp2)/3;

	const precnum_t alp	= 0.75*ep2;
	const precnum_t beta = (5.0/3.0)*alp*alp;
	const precnum_t gam	= (35.0/27.0)*alp*alp*alp;
	const precnum_t B	= 0.9996*c*( latp-alp*J2+beta*J4-gam*J6 );
	const precnum_t nb	= (Y-B)/v;
	const precnum_t psi	= (ep2*square(na))/2*clp2;
	const precnum_t eps	= na*(1-psi/3);
	const precnum_t nu	= nb*(1-psi)+latp;
	const precnum_t she	= (exp(eps)-exp(-eps))/2;
	const precnum_t dlon = atan2(she,cos(nu));
	const precnum_t tau	= atan2(cos(dlon)*tan(nu),1);

	out_lon	= RAD2DEG( dlon )+lon0;
	out_lat	= RAD2DEG( latp + (1+ep2*clp2-1.5*ep2*sin(latp)*cos(latp)*(tau-latp))*(tau-latp) );

}

void  mrpt::topography::geodeticToUTM(
	const TGeodeticCoords	&GeodeticCoords,
	TUTMCoords				&UTMCoords,
	int    					&UTMZone,
	char    				&UTMLatitudeBand,
	const TEllipsoid				&ellip )
{
	const double la = GeodeticCoords.lat;
	char Letra;
	if (la<-72) Letra='C';
	else if (la<-64) Letra='D';
	else if (la<-56) Letra='E';
	else if (la<-48) Letra='F';
	else if (la<-40) Letra='G';
	else if (la<-32) Letra='H';
	else if (la<-24) Letra='J';
	else if (la<-16) Letra='K';
	else if (la<-8) Letra='L';
	else if (la<0) Letra='M';
	else if (la<8) Letra='N';
	else if (la<16) Letra='P';
	else if (la<24) Letra='Q';
	else if (la<32) Letra='R';
	else if (la<40) Letra='S';
	else if (la<48) Letra='T';
	else if (la<56) Letra='U';
	else if (la<64) Letra='V';
	else if (la<72) Letra='W';
	else Letra='X';

	const precnum_t	lat		= DEG2RAD( GeodeticCoords.lat );
	const precnum_t	lon		= DEG2RAD( GeodeticCoords.lon );
	const int		Huso	= mrpt::utils::fix( ( GeodeticCoords.lon / 6 ) + 31);
	const precnum_t lon0	= DEG2RAD(Huso*6-183);

	const precnum_t sa		= ellip.sa;
	const precnum_t sb		= ellip.sb;
//	const precnum_t e2		= (sa*sa-sb*sb)/(sa*sa);
	const precnum_t ep2		= (sa*sa-sb*sb)/(sb*sb);
	const precnum_t c		= sa*sa/sb;
//	const precnum_t alp		= (sa-sb)/sa;

	const precnum_t Dlon	= lon-lon0;
	const precnum_t clat	= cos(lat);
	const precnum_t sDlon	= sin(Dlon);
	const precnum_t cDlon	= cos(Dlon);

	const precnum_t A		= clat*sDlon;
	const precnum_t eps		= 0.5*log( (1+A)/(1-A) );
	const precnum_t nu		= atan2( tan(lat), cDlon ) - lat;
	const precnum_t v		= 0.9996*c/sqrt( 1+ep2*clat*clat );
	const precnum_t psi		= 0.5*ep2*eps*eps*clat*clat;
	const precnum_t A1		= sin(2*lat);
	const precnum_t A2		= A1*clat*clat;
	const precnum_t J2		= lat+0.5*A1;
	const precnum_t J4		= 0.75*J2+0.25*A2;
	const precnum_t J6		= (5.0*J4+A2*clat*clat)/3;
	const precnum_t nalp	= 0.75*ep2;
	const precnum_t nbet	= (5.0/3.0)*nalp*nalp;
	const precnum_t ngam	= (35.0/27.0)*nalp*nalp*nalp;
	const precnum_t B		= 0.9996*c*(lat-nalp*J2+nbet*J4-ngam*J6);

	UTMCoords.x = eps*v*(1+psi/3.0)+500000;
	UTMCoords.y = nu*v*(1+psi)+B;
	UTMCoords.z = GeodeticCoords.height;

	UTMZone = Huso;
	UTMLatitudeBand = Letra;
}


/*---------------------------------------------------------------
					LatLonToUTM
 ---------------------------------------------------------------*/
void  mrpt::topography::GeodeticToUTM(
	double		la,
	double		lo,
	double    	&xx,
	double    	&yy,
	int    		&out_UTM_zone,
	char    	&out_UTM_latitude_band,
	const TEllipsoid	&ellip )
{
	// This method is based on public code by Gabriel Ruiz Martinez and Rafael Palacios.
	//  http://www.mathworks.com/matlabcentral/fileexchange/10915

	const double sa = ellip.sa;
	const double sb = ellip.sb;
	const double e2 = (sqrt((sa*sa) - (sb*sb)))/sb;

	const double e2cuadrada = e2*e2;

	const double c = ( sa*sa )/sb;

	const double lat = DEG2RAD(la);
	const double lon = DEG2RAD(lo);

	const int Huso = mrpt::utils::fix( ( lo / 6 ) + 31);
	double S = ( ( Huso * 6 ) - 183 );
	double deltaS = lon - DEG2RAD(S);

	char Letra;

	if (la<-72) Letra='C';
	else if (la<-64) Letra='D';
	else if (la<-56) Letra='E';
	else if (la<-48) Letra='F';
	else if (la<-40) Letra='G';
	else if (la<-32) Letra='H';
	else if (la<-24) Letra='J';
	else if (la<-16) Letra='K';
	else if (la<-8) Letra='L';
	else if (la<0) Letra='M';
	else if (la<8) Letra='N';
	else if (la<16) Letra='P';
	else if (la<24) Letra='Q';
	else if (la<32) Letra='R';
	else if (la<40) Letra='S';
	else if (la<48) Letra='T';
	else if (la<56) Letra='U';
	else if (la<64) Letra='V';
	else if (la<72) Letra='W';
	else Letra='X';

	const double a = cos(lat) * sin(deltaS);
	const double epsilon = 0.5 * log( ( 1 +  a) / ( 1 - a ) );
	const double nu = atan( tan(lat) / cos(deltaS) ) - lat;
	const double v = ( c / sqrt( ( 1 + ( e2cuadrada * square(cos(lat))  ) ) ) ) * 0.9996;
	const double ta = 0.5 * e2cuadrada * square(epsilon)  * square( cos(lat) );
	const double a1 = sin( 2 * lat );
	const double a2 = a1 * square( cos(lat) );
	const double j2 = lat + 0.5*a1;
	const double j4 = ( ( 3.0 * j2 ) + a2 ) / 4.0;
	const double j6 = ( ( 5.0 * j4 ) + ( a2 * square( cos(lat) ) ) ) / 3.0;
	const double alfa = 0.75  * e2cuadrada;
	const double beta = (5.0/3.0) * pow(alfa, 2.0);
	const double gama = (35.0/27.0) * pow(alfa,3.0);
	const double Bm = 0.9996 * c * ( lat - alfa * j2 + beta * j4 - gama * j6 );

	xx = epsilon * v * ( 1 + ( ta / 3.0 ) ) + 500000;
	yy = nu * v * ( 1 + ta ) + Bm;

	if (yy<0)
	   yy += 9999999;

	out_UTM_zone = Huso;
	out_UTM_latitude_band = Letra;
}

/**  7-parameter Bursa-Wolf transformation:
  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1 ] [ X Y Z ]_local
  * \sa transform10params
  */
void  mrpt::topography::transform7params(
	const mrpt::math::TPoint3D	&p,
	const TDatum7Params			&d,
	mrpt::math::TPoint3D		&o)
{
	const double scale = (1+d.dS);

	o.x = d.dX + scale*( p.x      + p.y * d.Rz - p.z*d.Ry);
	o.y = d.dY + scale*(-p.x*d.Rz + p.y        + p.z*d.Rx);
	o.z = d.dZ + scale*( p.x*d.Ry - p.y * d.Rx + p.z);
}

/**  7-parameter Bursa-Wolf transformation TOPCON:
  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1 ] [ X Y Z ]_local
  * \sa transform10params
  */
void  mrpt::topography::transform7params_TOPCON(
	const mrpt::math::TPoint3D	&p,
	const TDatum7Params_TOPCON	&d,
	mrpt::math::TPoint3D		&o)
{
	const double scale = (1+d.dS);

	o.x = d.dX + scale*( d.m11*p.x + d.m12*p.y + d.m13*p.z );
	o.y = d.dY + scale*( d.m21*p.x + d.m22*p.y + d.m23*p.z );
	o.z = d.dZ + scale*( d.m31*p.x + d.m32*p.y + d.m33*p.z );
}


/**  10-parameter Molodensky-Badekas transformation:
  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1 ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
  * \sa transform7params
  */
void  mrpt::topography::transform10params(
	const mrpt::math::TPoint3D	&p,
	const TDatum10Params		&d,
	mrpt::math::TPoint3D		&o)
{
	const double scale = (1+d.dS);

	const double px = p.x - d.Xp;
	const double py = p.y - d.Yp;
	const double pz = p.z - d.Zp;

	o.x = d.dX + scale*( px      + py * d.Rz - pz*d.Ry) + d.Xp;
	o.y = d.dY + scale*(-px*d.Rz + py        + pz*d.Rx) + d.Yp;
	o.z = d.dZ + scale*( px*d.Ry - py * d.Rx + pz)      + d.Zp;
}

/**  Helmert 2D transformation:
  *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha); sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
  * \sa transformHelmert3D
  */
void  mrpt::topography::transformHelmert2D(
	const mrpt::math::TPoint2D	&p,
	const TDatumHelmert2D		&d,
	mrpt::math::TPoint2D		&o)
{
	const double scale = (1+d.dS);

	const double px = p.x - d.Xp;
	const double py = p.y - d.Yp;

	o.x = d.dX + scale*( px*cos(d.alpha) - py*sin(d.alpha)) + d.Xp;
	o.y = d.dY + scale*( px*sin(d.alpha) + py*cos(d.alpha)) + d.Yp;
}

/**  Helmert 2D transformation:
  *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha); sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
  * \sa transformHelmert3D
  */
void  mrpt::topography::transformHelmert2D_TOPCON(
	const mrpt::math::TPoint2D		&p,
	const TDatumHelmert2D_TOPCON	&d,
	mrpt::math::TPoint2D			&o)
{
	o.x = d.a*p.x + d.b*p.y+d.c;
	o.y = -d.b*p.x+d.a*p.y+d.d;
}

/**  Helmert 3D transformation:
  *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha); sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
  * \sa transformHelmert3D
  */
void  mrpt::topography::transformHelmert3D(
	const mrpt::math::TPoint3D	&p,
	const TDatumHelmert3D		&d,
	mrpt::math::TPoint3D		&o)
{
	TDatum7Params d2( d.dX, d.dY, d.dZ, -1*d.Rx, -1*d.Ry, -1*d.Rz, d.dS );
	transform7params( p, d2, o );
}

/**  Helmert 3D transformation:
  *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha); sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
  * \sa transformHelmert3D
  */
void  mrpt::topography::transformHelmert3D_TOPCON(
	const mrpt::math::TPoint3D		&p,
	const TDatumHelmert3D_TOPCON	&d,
	mrpt::math::TPoint3D			&o)
{
	o.x = d.a*p.x + d.b*p.y + d.c;
	o.y = d.d*p.x + d.e*p.y + d.f;
	o.z = p.z + d.g;
}

/**  1D transformation:
  *   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ
  * \sa transformHelmert3D
  */
void  mrpt::topography::transform1D(
	const mrpt::math::TPoint3D	&p,
	const TDatum1DTransf		&d,
	mrpt::math::TPoint3D		&o)
{
	o.x = p.x;
	o.y = p.y;
	o.z = (d.dY * p.x - d.dX * p.y + p.z)*(1 + d.dS) + d.DZ;
}

/**  1D transformation:
  *   [ X;Y ]_WGS84 = [X;Y]_locales+[1 -sin(d.beta);0 cos(d.beta)]*[x*d.dSx;y*d.dSy ]
  * \sa transformHelmert3D
  */
void  mrpt::topography::transfInterpolation(
	const mrpt::math::TPoint3D		&p,
	const TDatumTransfInterpolation	&d,
	mrpt::math::TPoint3D			&o)
{
	o.x = d.dX + p.x*d.dSx - p.y*d.dSy*sin(d.beta);
	o.y = d.dY + p.y*d.dSy*cos(d.beta);
	o.z = p.z;
}


/** ENU to geocentric coordinates.
  * \sa geodeticToENU_WGS84
  */
void mrpt::topography::ENUToGeocentric(
	const mrpt::math::TPoint3D	&p,
	const TGeodeticCoords		&in_coords_origin,
	TGeocentricCoords 			&out_coords,
	const TEllipsoid			&ellip
	)
{
	// Generate reference 3D point:
	TPoint3D P_geocentric_ref;
	mrpt::topography::geodeticToGeocentric(in_coords_origin,P_geocentric_ref, ellip);

	CVectorDouble   P_ref(3);
	P_ref[0] = P_geocentric_ref.x;
	P_ref[1] = P_geocentric_ref.y;
	P_ref[2] = P_geocentric_ref.z;

	// Z axis -> In direction out-ward the center of the Earth:
	CVectorDouble	REF_X(3),REF_Y(3),REF_Z(3);
	math::normalize(P_ref, REF_Z);

	// 1st column: Starting at the reference point, move in the tangent direction
	//   east-ward: I compute this as the derivative of P_ref wrt "longitude":
	//      A_east[0] =-(N+in_height_meters)*cos(lat)*sin(lon);  --> -Z[1]
	//      A_east[1] = (N+in_height_meters)*cos(lat)*cos(lon);  -->  Z[0]
	//      A_east[2] = 0;                                       -->  0
	// ---------------------------------------------------------------------------
	CVectorDouble AUX_X(3);
	AUX_X[0]=-REF_Z[1];
	AUX_X[1]= REF_Z[0];
	AUX_X[2]= 0;
	math::normalize(AUX_X, REF_X);

	// 2nd column: The cross product:
	math::crossProduct3D(REF_Z, REF_X, REF_Y);

	out_coords.x = REF_X[0]*p.x + REF_Y[0]*p.y + REF_Z[0]*p.z + P_geocentric_ref.x;
	out_coords.y = REF_X[1]*p.x + REF_Y[1]*p.y + REF_Z[1]*p.z + P_geocentric_ref.y;
	out_coords.z = REF_X[2]*p.x + REF_Y[2]*p.y + REF_Z[2]*p.z + P_geocentric_ref.z;
}
