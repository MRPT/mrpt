/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/math/geometry.h>
#include <mrpt/math/utils.h>
#include <mrpt/topography/conversions.h>

#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;

bool mrpt::topography::operator==(const TCoords& a, const TCoords& o)
{
  return a.decimal_value == o.decimal_value;
}
bool mrpt::topography::operator!=(const TCoords& a, const TCoords& o) { return !(a == o); }
bool mrpt::topography::operator==(const TGeodeticCoords& a, const TGeodeticCoords& o)
{
  return a.lat == o.lat && a.lon == o.lon && a.height == o.height;
}
bool mrpt::topography::operator!=(const TGeodeticCoords& a, const TGeodeticCoords& o)
{
  return !(a == o);
}

// Define a generic type for "precission numbers":
#ifdef HAVE_LONG_DOUBLE
using precnum_t = long double;
#else
using precnum_t = double;
#endif

std::ostream& mrpt::topography::operator<<(std::ostream& out, const TCoords& o)
{
  // Use c_str() to avoid instantiating std::operator<<(ostream, string) which
  // has a versioned ABI symbol (abi:ne190102) absent from older libc++ runtimes.
  return out << o.getAsString().c_str();
}

/*---------------------------------------------------------------
        geocentricToENU_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::geocentricToENU_WGS84(
    const mrpt::math::TPoint3D& P_geocentric_,
    mrpt::math::TPoint3D& out_ENU_point,
    const TGeodeticCoords& in_coords_origin)
{
  // Generate reference 3D point:
  TPoint3D P_geocentric_ref;
  geodeticToGeocentric_WGS84(in_coords_origin, P_geocentric_ref);

  const double clat = cos(DEG2RAD(in_coords_origin.lat));
  const double slat = sin(DEG2RAD(in_coords_origin.lat));
  const double clon = cos(DEG2RAD(in_coords_origin.lon));
  const double slon = sin(DEG2RAD(in_coords_origin.lon));

  // Compute the resulting relative coordinates:
  // For using smaller numbers:
  const mrpt::math::TPoint3D P_geocentric = P_geocentric_ - P_geocentric_ref;

  // Optimized calculation: Local transformed coordinates of P_geo(x,y,z)
  //   after rotation given by the transposed rotation matrix from ENU ->
  //   ECEF.
  out_ENU_point.x = -slon * P_geocentric.x + clon * P_geocentric.y;
  out_ENU_point.y =
      -clon * slat * P_geocentric.x - slon * slat * P_geocentric.y + clat * P_geocentric.z;
  out_ENU_point.z =
      clon * clat * P_geocentric.x + slon * clat * P_geocentric.y + slat * P_geocentric.z;
}

void mrpt::topography::geocentricToENU_WGS84(
    const std::vector<mrpt::math::TPoint3D>& in_geocentric_points,
    std::vector<mrpt::math::TPoint3D>& out_ENU_points,
    const TGeodeticCoords& in_coords_origin)
{
  // Generate reference 3D point:
  TPoint3D P_geocentric_ref;
  geodeticToGeocentric_WGS84(in_coords_origin, P_geocentric_ref);

  const double clat = cos(DEG2RAD(in_coords_origin.lat));
  const double slat = sin(DEG2RAD(in_coords_origin.lat));
  const double clon = cos(DEG2RAD(in_coords_origin.lon));
  const double slon = sin(DEG2RAD(in_coords_origin.lon));

  const size_t N = in_geocentric_points.size();
  out_ENU_points.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    // Compute the resulting relative coordinates for using smaller numbers:
    const mrpt::math::TPoint3D P_geocentric = in_geocentric_points[i] - P_geocentric_ref;

    // Optimized calculation: Local transformed coordinates of P_geo(x,y,z)
    //   after rotation given by the transposed rotation matrix from ENU ->
    //   ECEF.
    out_ENU_points[i].x = -slon * P_geocentric.x + clon * P_geocentric.y;
    out_ENU_points[i].y =
        -clon * slat * P_geocentric.x - slon * slat * P_geocentric.y + clat * P_geocentric.z;
    out_ENU_points[i].z =
        clon * clat * P_geocentric.x + slon * clat * P_geocentric.y + slat * P_geocentric.z;
  }
}

/*---------------------------------------------------------------
        geodeticToENU_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::geodeticToENU_WGS84(
    const TGeodeticCoords& in_coords,
    mrpt::math::TPoint3D& out_ENU_point,
    const TGeodeticCoords& in_coords_origin)
{
  // --------------------------------------------------------------------
  //  Explanation: We compute the earth-centric coordinates first,
  //    then make a system transformation to local XYZ coordinates
  //    using a system of three orthogonal vectors as local reference.
  //
  // See: http://en.wikipedia.org/wiki/Reference_ellipsoid
  // (JLBC 21/DEC/2006)  (Fixed: JLBC 9/JUL/2008)
  // - Oct/2013, Emilio Sanjurjo: Fixed UP vector pointing exactly normal to
  // ellipsoid surface.
  // --------------------------------------------------------------------
  // Generate 3D point:
  TPoint3D P_geocentric;
  geodeticToGeocentric_WGS84(in_coords, P_geocentric);

  geocentricToENU_WGS84(P_geocentric, out_ENU_point, in_coords_origin);
}

/*---------------------------------------------------------------
          ENU_axes_from_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::ENU_axes_from_WGS84(
    double in_longitude_reference_degrees,
    double in_latitude_reference_degrees,
    double in_height_reference_meters,
    mrpt::math::TPose3D& out_ENU,
    bool only_angles)
{
  // See "coordinatesTransformation_WGS84" for more comments.
  TPoint3D PPref;
  mrpt::topography::geodeticToGeocentric_WGS84(
      TGeodeticCoords(
          in_latitude_reference_degrees, in_longitude_reference_degrees,
          in_height_reference_meters),
      PPref);

  const double cLat = cos(DEG2RAD(in_latitude_reference_degrees));
  const double sLat = sin(DEG2RAD(in_latitude_reference_degrees));
  const double cLon = cos(DEG2RAD(in_longitude_reference_degrees));
  const double sLon = sin(DEG2RAD(in_longitude_reference_degrees));

  CMatrixDouble44 HM;  // zeros by default
  HM(0, 0) = -sLon;
  HM(0, 1) = -cLon * sLat;
  HM(0, 2) = cLon * cLat;
  HM(1, 0) = cLon;
  HM(1, 1) = -sLon * sLat;
  HM(1, 2) = sLon * cLat;
  HM(2, 0) = 0;
  HM(2, 1) = cLat;
  HM(2, 2) = sLat;
  HM(3, 3) = 1;

  if (!only_angles)
  {
    HM(0, 3) = PPref.x;
    HM(1, 3) = PPref.y;
    HM(2, 3) = PPref.z;
  }

  out_ENU.fromHomogeneousMatrix(HM);
}

//*---------------------------------------------------------------
//			geodeticToGeocentric_WGS84
// ---------------------------------------------------------------*/
void mrpt::topography::geodeticToGeocentric_WGS84(
    const TGeodeticCoords& in_coords, mrpt::math::TPoint3D& out_point)
{
  // --------------------------------------------------------------------
  // See: http://en.wikipedia.org/wiki/Reference_ellipsoid
  //  Constants are for WGS84
  // --------------------------------------------------------------------

  static constexpr precnum_t a = 6378137L;       // Semi-major axis of the Earth (meters)
  static constexpr precnum_t b = 6356752.3142L;  // Semi-minor axis:

  static const precnum_t ae = std::acos(b / a);  // angular eccentricity of the Earth
  static const precnum_t cos2_ae_earth = square(std::cos(ae));
  static const precnum_t sin2_ae_earth = square(std::sin(ae));

  const precnum_t lon = DEG2RAD(precnum_t(in_coords.lon));
  const precnum_t lat = DEG2RAD(precnum_t(in_coords.lat));

  // The radius of curvature in the prime vertical:
  const precnum_t N = a / std::sqrt(1 - sin2_ae_earth * square(sin(lat)));

  // Generate 3D point:
  out_point.x = static_cast<double>((N + in_coords.height) * cos(lat) * cos(lon));
  out_point.y = static_cast<double>((N + in_coords.height) * cos(lat) * sin(lon));
  out_point.z = static_cast<double>((cos2_ae_earth * N + in_coords.height) * sin(lat));
}

/*---------------------------------------------------------------
      geodeticToGeocentric_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::geodeticToGeocentric(
    const TGeodeticCoords& in_coords, TGeocentricCoords& out_point, const TEllipsoid& ellipsoid)
{
  static const precnum_t a = ellipsoid.sa;  // Semi-major axis of the Earth (meters)
  static const precnum_t b = ellipsoid.sb;  // Semi-minor axis:

  static const precnum_t ae = acos(b / a);  // eccentricity:
  static const precnum_t cos2_ae_earth =
      square(cos(ae));  // The cos^2 of the angular eccentricity of the Earth:
  // // 0.993305619995739L;
  static const precnum_t sin2_ae_earth =
      square(sin(ae));  // The sin^2 of the angular eccentricity of the Earth:
  // // 0.006694380004261L;

  const precnum_t lon = DEG2RAD(precnum_t(in_coords.lon));
  const precnum_t lat = DEG2RAD(precnum_t(in_coords.lat));

  // The radius of curvature in the prime vertical:
  const precnum_t N = a / std::sqrt(1 - sin2_ae_earth * square(sin(lat)));

  // Generate 3D point:
  out_point.x = static_cast<double>((N + in_coords.height) * cos(lat) * cos(lon));
  out_point.y = static_cast<double>((N + in_coords.height) * cos(lat) * sin(lon));
  out_point.z = static_cast<double>((cos2_ae_earth * N + in_coords.height) * sin(lat));
}

/*---------------------------------------------------------------
      geocentricToGeodetic
 ---------------------------------------------------------------*/
void mrpt::topography::geocentricToGeodetic(
    const TGeocentricCoords& in_point, TGeodeticCoords& out_coords, const TEllipsoid& ellipsoid)
{
  const double sa2 = ellipsoid.sa * ellipsoid.sa;
  const double sb2 = ellipsoid.sb * ellipsoid.sb;

  const double e2 = (sa2 - sb2) / sa2;
  const double ep2 = (sa2 - sb2) / sb2;
  const double p = sqrt(in_point.x * in_point.x + in_point.y * in_point.y);
  const double theta = atan2(in_point.z * ellipsoid.sa, p * ellipsoid.sb);

  out_coords.lon = atan2(in_point.y, in_point.x);
  out_coords.lat = atan2(
      in_point.z + ep2 * ellipsoid.sb * sin(theta) * sin(theta) * sin(theta),
      p - e2 * ellipsoid.sa * cos(theta) * cos(theta) * cos(theta));

  const double clat = cos(out_coords.lat.decimal_value);
  const double slat = sin(out_coords.lat.decimal_value);
  const double N = sa2 / sqrt(sa2 * clat * clat + sb2 * slat * slat);

  // Note: the more obvious "height = p/clat - N" is numerically unstable
  // close to the poles, where both p and clat tend to zero. The formula
  // below is mathematically equivalent but well-conditioned everywhere.
  out_coords.height = p * clat + in_point.z * slat - N * (1.0 - e2 * slat * slat);

  out_coords.lon = RAD2DEG(out_coords.lon.decimal_value);
  out_coords.lat = RAD2DEG(out_coords.lat.decimal_value);
}

/*---------------------------------------------------------------
          UTMToGeodesic
 ---------------------------------------------------------------*/
void mrpt::topography::UTMToGeodetic(
    double X,
    double Y,
    int huso,
    char hem,
    double& out_lon /*degrees*/,
    double& out_lat /*degrees*/,
    const TEllipsoid& ellip)
{
  ASSERT_(hem == 's' || hem == 'S' || hem == 'n' || hem == 'N');

  X = X - 5e5;
  if (hem == 's' || hem == 'S') Y = Y - 1e7;

  const precnum_t lon0 = huso * 6 - 183;
  const precnum_t a2 = ellip.sa * ellip.sa;
  const precnum_t b2 = ellip.sb * ellip.sb;
  const precnum_t ep2 = (a2 - b2) / b2;
  const precnum_t c = a2 / ellip.sb;
  const precnum_t latp = Y / (6366197.724 * 0.9996);
  const precnum_t clp2 = square(cos(latp));

  const precnum_t v = c * 0.9996 / sqrt(1 + ep2 * clp2);
  const precnum_t na = X / v;
  const precnum_t A1 = sin(2 * latp);
  const precnum_t A2 = A1 * clp2;
  const precnum_t J2 = latp + A1 * 0.5;
  const precnum_t J4 = 0.75 * J2 + 0.25 * A2;
  const precnum_t J6 = (5 * J4 + A2 * clp2) / 3;

  const precnum_t alp = 0.75 * ep2;
  const precnum_t beta = (5.0 / 3.0) * alp * alp;
  const precnum_t gam = (35.0 / 27.0) * alp * alp * alp;
  const precnum_t B = 0.9996 * c * (latp - alp * J2 + beta * J4 - gam * J6);
  const precnum_t nb = (Y - B) / v;
  const precnum_t psi = (ep2 * square(na)) / 2 * clp2;
  const precnum_t eps = na * (1 - psi / 3);
  const precnum_t nu = nb * (1 - psi) + latp;
  const precnum_t she = (exp(eps) - exp(-eps)) / 2;
  const precnum_t dlon = atan2(she, cos(nu));
  const precnum_t tau = atan2(cos(dlon) * tan(nu), 1);

  out_lon = static_cast<double>(RAD2DEG(dlon) + lon0);
  out_lat = static_cast<double>(RAD2DEG(
      latp + (1 + ep2 * clp2 - 1.5 * ep2 * sin(latp) * cos(latp) * (tau - latp)) * (tau - latp)));
}

namespace
{
char utm_zone_from_latitude(const double la)
{
  char letter = 0;
  if (la < -72)
  {
    letter = 'C';
  }
  else if (la < -64)
  {
    letter = 'D';
  }
  else if (la < -56)
  {
    letter = 'E';
  }
  else if (la < -48)
  {
    letter = 'F';
  }
  else if (la < -40)
  {
    letter = 'G';
  }
  else if (la < -32)
  {
    letter = 'H';
  }
  else if (la < -24)
  {
    letter = 'J';
  }
  else if (la < -16)
  {
    letter = 'K';
  }
  else if (la < -8)
  {
    letter = 'L';
  }
  else if (la < 0)
  {
    letter = 'M';
  }
  else if (la < 8)
  {
    letter = 'N';
  }
  else if (la < 16)
  {
    letter = 'P';
  }
  else if (la < 24)
  {
    letter = 'Q';
  }
  else if (la < 32)
  {
    letter = 'R';
  }
  else if (la < 40)
  {
    letter = 'S';
  }
  else if (la < 48)
  {
    letter = 'T';
  }
  else if (la < 56)
  {
    letter = 'U';
  }
  else if (la < 64)
  {
    letter = 'V';
  }
  else if (la < 72)
  {
    letter = 'W';
  }
  else
  {
    letter = 'X';
  }
  return letter;
}
}  // namespace

void mrpt::topography::geodeticToUTM(
    const TGeodeticCoords& GeodeticCoords,
    TUTMCoords& UTMCoords,
    int& UTMZone,
    char& UTMLatitudeBand,
    const TEllipsoid& ellipsoid)
{
  const double la = GeodeticCoords.lat;
  const auto letter = utm_zone_from_latitude(la);

  const precnum_t lat = DEG2RAD(GeodeticCoords.lat);
  const precnum_t lon = DEG2RAD(GeodeticCoords.lon);
  const int Huso = mrpt::fix((GeodeticCoords.lon / 6) + 31);
  const precnum_t lon0 = DEG2RAD(Huso * 6 - 183);

  const precnum_t sa = ellipsoid.sa;
  const precnum_t sb = ellipsoid.sb;
  //	const precnum_t e2		= (sa*sa-sb*sb)/(sa*sa);
  const precnum_t ep2 = (sa * sa - sb * sb) / (sb * sb);
  const precnum_t c = sa * sa / sb;
  //	const precnum_t alp		= (sa-sb)/sa;

  const precnum_t Dlon = lon - lon0;
  const precnum_t clat = cos(lat);
  const precnum_t sDlon = sin(Dlon);
  const precnum_t cDlon = cos(Dlon);

  const precnum_t A = clat * sDlon;
  const precnum_t eps = 0.5 * log((1 + A) / (1 - A));
  const precnum_t nu = atan2(tan(lat), cDlon) - lat;
  const precnum_t v = 0.9996 * c / sqrt(1 + ep2 * clat * clat);
  const precnum_t psi = 0.5 * ep2 * eps * eps * clat * clat;
  const precnum_t A1 = sin(2 * lat);
  const precnum_t A2 = A1 * clat * clat;
  const precnum_t J2 = lat + 0.5 * A1;
  const precnum_t J4 = 0.75 * J2 + 0.25 * A2;
  const precnum_t J6 = (5.0 * J4 + A2 * clat * clat) / 3;
  const precnum_t nalp = 0.75 * ep2;
  const precnum_t nbet = (5.0 / 3.0) * nalp * nalp;
  const precnum_t ngam = (35.0 / 27.0) * nalp * nalp * nalp;
  const precnum_t B = 0.9996 * c * (lat - nalp * J2 + nbet * J4 - ngam * J6);

  UTMCoords.x = static_cast<double>(eps * v * (1 + psi / 3.0) + 500000);
  UTMCoords.y = static_cast<double>(nu * v * (1 + psi) + B);
  UTMCoords.z = static_cast<double>(GeodeticCoords.height);

  // Apply the false northing for points south of the equator, so that
  // northing values are always positive, following the standard UTM
  // convention (and matching what UTMToGeodetic() expects on input).
  if (UTMCoords.y < 0)
  {
    UTMCoords.y += 1e7;
  }

  UTMZone = Huso;
  UTMLatitudeBand = letter;
}

/*---------------------------------------------------------------
          LatLonToUTM
 ---------------------------------------------------------------*/
void mrpt::topography::GeodeticToUTM(
    double la,
    double lo,
    double& xx,
    double& yy,
    int& out_UTM_zone,
    char& out_UTM_latitude_band,
    const TEllipsoid& ellipsoid)
{
  // This method is based on public code by Gabriel Ruiz Martinez and Rafael
  // Palacios.
  //  http://www.mathworks.com/matlabcentral/fileexchange/10915

  const double sa = ellipsoid.sa;
  const double sb = ellipsoid.sb;
  const double e2 = (sqrt((sa * sa) - (sb * sb))) / sb;

  const double e2cuadrada = e2 * e2;

  const double c = (sa * sa) / sb;

  const double lat = DEG2RAD(la);
  const double lon = DEG2RAD(lo);

  const int Huso = mrpt::fix((lo / 6) + 31);
  double S = ((Huso * 6) - 183);
  double deltaS = lon - DEG2RAD(S);

  const auto letter = utm_zone_from_latitude(la);

  const double a = cos(lat) * sin(deltaS);
  const double epsilon = 0.5 * log((1 + a) / (1 - a));
  const double nu = atan(tan(lat) / cos(deltaS)) - lat;
  const double v = (c / sqrt((1 + (e2cuadrada * square(cos(lat)))))) * 0.9996;
  const double ta = 0.5 * e2cuadrada * square(epsilon) * square(cos(lat));
  const double a1 = sin(2 * lat);
  const double a2 = a1 * square(cos(lat));
  const double j2 = lat + 0.5 * a1;
  const double j4 = ((3.0 * j2) + a2) / 4.0;
  const double j6 = ((5.0 * j4) + (a2 * square(cos(lat)))) / 3.0;
  const double alfa = 0.75 * e2cuadrada;
  const double beta = (5.0 / 3.0) * pow(alfa, 2.0);
  const double gama = (35.0 / 27.0) * pow(alfa, 3.0);
  const double Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);

  xx = epsilon * v * (1 + (ta / 3.0)) + 500000;
  yy = nu * v * (1 + ta) + Bm;

  if (yy < 0)
  {
    // Standard UTM false northing for the southern hemisphere.
    yy += 1e7;
  }

  out_UTM_zone = Huso;
  out_UTM_latitude_band = letter;
}

/**  7-parameter Bursa-Wolf transformation:
 *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1
 * ] [ X Y Z ]_local
 * \sa transform10params
 */
void mrpt::topography::transform7params(
    const mrpt::math::TPoint3D& p, const TDatum7Params& d, mrpt::math::TPoint3D& o)
{
  const double scale = (1 + d.dS);

  o.x = d.dX + scale * (p.x + p.y * d.Rz - p.z * d.Ry);
  o.y = d.dY + scale * (-p.x * d.Rz + p.y + p.z * d.Rx);
  o.z = d.dZ + scale * (p.x * d.Ry - p.y * d.Rx + p.z);
}

/**  7-parameter Bursa-Wolf transformation TOPCON:
 *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1
 * ] [ X Y Z ]_local
 * \sa transform10params
 */
void mrpt::topography::transform7params_TOPCON(
    const mrpt::math::TPoint3D& p, const TDatum7Params_TOPCON& d, mrpt::math::TPoint3D& o)
{
  const double scale = (1 + d.dS);

  o.x = d.dX + scale * (d.m11 * p.x + d.m12 * p.y + d.m13 * p.z);
  o.y = d.dY + scale * (d.m21 * p.x + d.m22 * p.y + d.m23 * p.z);
  o.z = d.dZ + scale * (d.m31 * p.x + d.m32 * p.y + d.m33 * p.z);
}

/**  10-parameter Molodensky-Badekas transformation:
 *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1
 * ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
 * \sa transform7params
 */
void mrpt::topography::transform10params(
    const mrpt::math::TPoint3D& p, const TDatum10Params& d, mrpt::math::TPoint3D& o)
{
  const double scale = (1 + d.dS);

  const double px = p.x - d.Xp;
  const double py = p.y - d.Yp;
  const double pz = p.z - d.Zp;

  o.x = d.dX + scale * (px + py * d.Rz - pz * d.Ry) + d.Xp;
  o.y = d.dY + scale * (-px * d.Rz + py + pz * d.Rx) + d.Yp;
  o.z = d.dZ + scale * (px * d.Ry - py * d.Rx + pz) + d.Zp;
}

/**  Helmert 2D transformation:
 *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha);
 * sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
 * \sa transformHelmert3D
 */
void mrpt::topography::transformHelmert2D(
    const mrpt::math::TPoint2D& p, const TDatumHelmert2D& d, mrpt::math::TPoint2D& o)
{
  const double scale = (1 + d.dS);

  const double px = p.x - d.Xp;
  const double py = p.y - d.Yp;

  o.x = d.dX + scale * (px * cos(d.alpha) - py * sin(d.alpha)) + d.Xp;
  o.y = d.dY + scale * (px * sin(d.alpha) + py * cos(d.alpha)) + d.Yp;
}

/**  Helmert 2D transformation:
 *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha);
 * sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
 * \sa transformHelmert3D
 */
void mrpt::topography::transformHelmert2D_TOPCON(
    const mrpt::math::TPoint2D& p, const TDatumHelmert2D_TOPCON& d, mrpt::math::TPoint2D& o)
{
  o.x = d.a * p.x + d.b * p.y + d.c;
  o.y = -d.b * p.x + d.a * p.y + d.d;
}

/**  Helmert 3D transformation:
 *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha);
 * sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
 * \sa transformHelmert3D
 */
void mrpt::topography::transformHelmert3D(
    const mrpt::math::TPoint3D& p, const TDatumHelmert3D& d, mrpt::math::TPoint3D& o)
{
  // Note: d.Rx/Ry/Rz and d.dS are already converted to radians/fraction by
  // the TDatumHelmert3D constructor, so the transform7params() matrix
  // formula is applied here directly (using the sign convention of this
  // Helmert3D transformation) instead of routing through the
  // TDatum7Params constructor, which expects raw arc-seconds/ppm input and
  // would otherwise convert these values a second time.
  const double scale = (1 + d.dS);
  const double Rx = -d.Rx;
  const double Ry = -d.Ry;
  const double Rz = -d.Rz;

  o.x = d.dX + scale * (p.x + p.y * Rz - p.z * Ry);
  o.y = d.dY + scale * (-p.x * Rz + p.y + p.z * Rx);
  o.z = d.dZ + scale * (p.x * Ry - p.y * Rx + p.z);
}

/**  Helmert 3D transformation:
 *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha);
 * sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
 * \sa transformHelmert3D
 */
void mrpt::topography::transformHelmert3D_TOPCON(
    const mrpt::math::TPoint3D& p, const TDatumHelmert3D_TOPCON& d, mrpt::math::TPoint3D& o)
{
  o.x = d.a * p.x + d.b * p.y + d.c;
  o.y = d.d * p.x + d.e * p.y + d.f;
  o.z = p.z + d.g;
}

/**  1D transformation:
 *   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ
 * \sa transformHelmert3D
 */
void mrpt::topography::transform1D(
    const mrpt::math::TPoint3D& p, const TDatum1DTransf& d, mrpt::math::TPoint3D& o)
{
  o.x = p.x;
  o.y = p.y;
  o.z = (d.dY * p.x - d.dX * p.y + p.z) * (1 + d.dS) + d.DZ;
}

/**  1D transformation:
 *   [ X;Y ]_WGS84 = [X;Y]_locales+[1 -sin(d.beta);0
 * cos(d.beta)]*[x*d.dSx;y*d.dSy ]
 * \sa transformHelmert3D
 */
void mrpt::topography::transfInterpolation(
    const mrpt::math::TPoint3D& p, const TDatumTransfInterpolation& d, mrpt::math::TPoint3D& o)
{
  o.x = d.dX + p.x * d.dSx - p.y * d.dSy * sin(d.beta);
  o.y = d.dY + p.y * d.dSy * cos(d.beta);
  o.z = p.z;
}

/** ENU to geocentric coordinates.
 * \sa geodeticToENU_WGS84
 */
void mrpt::topography::ENUToGeocentric(
    const mrpt::math::TPoint3D& p,
    const TGeodeticCoords& in_coords_origin,
    TGeocentricCoords& out_coords,
    const TEllipsoid& ellip)
{
  // Generate reference 3D point:
  TPoint3D P_geocentric_ref;
  mrpt::topography::geodeticToGeocentric(in_coords_origin, P_geocentric_ref, ellip);

  const double clat = cos(DEG2RAD(in_coords_origin.lat));
  const double slat = sin(DEG2RAD(in_coords_origin.lat));
  const double clon = cos(DEG2RAD(in_coords_origin.lon));
  const double slon = sin(DEG2RAD(in_coords_origin.lon));

  // ENU basis vectors expressed in the geocentric frame, with the "Up" axis
  // being the normal to the ellipsoid at the reference point (the same
  // convention used by geodeticToENU_WGS84 / geocentricToENU_WGS84), so
  // that this function is their exact inverse. This is the transpose of
  // the rotation applied in geocentricToENU_WGS84.
  out_coords.x = -slon * p.x - clon * slat * p.y + clon * clat * p.z + P_geocentric_ref.x;
  out_coords.y = clon * p.x - slon * slat * p.y + slon * clat * p.z + P_geocentric_ref.y;
  out_coords.z = clat * p.y + slat * p.z + P_geocentric_ref.z;
}
