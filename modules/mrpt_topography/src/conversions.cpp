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

#include <mrpt/core/bits_math.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <cmath>
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

// Define a generic type for "precision numbers":
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
          ENUToGeodetic_WGS84
 ---------------------------------------------------------------*/
void mrpt::topography::ENUToGeodetic_WGS84(
    const mrpt::math::TPoint3D& in_ENU_point,
    TGeodeticCoords& out_coords,
    const TGeodeticCoords& in_coords_origin)
{
  const auto wgs84 = TEllipsoid::Ellipsoid_WGS84();

  TGeocentricCoords P_geocentric;
  ENUToGeocentric(in_ENU_point, in_coords_origin, P_geocentric, wgs84);
  geocentricToGeodetic(P_geocentric, out_coords, wgs84);
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
  // See geocentricToENU_WGS84() for more comments.
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
  geodeticToGeocentric(in_coords, out_point, TEllipsoid::Ellipsoid_WGS84());
}

/*---------------------------------------------------------------
      geodeticToGeocentric
 ---------------------------------------------------------------*/
void mrpt::topography::geodeticToGeocentric(
    const TGeodeticCoords& in_coords, TGeocentricCoords& out_point, const TEllipsoid& ellipsoid)
{
  const precnum_t a = ellipsoid.sa;  // Semi-major axis of the Earth (meters)
  const precnum_t b = ellipsoid.sb;  // Semi-minor axis:

  const precnum_t ae = acos(b / a);  // eccentricity:
  // The cos^2 and sin^2 of the angular eccentricity of the Earth:
  const precnum_t cos2_ae_earth = square(cos(ae));
  const precnum_t sin2_ae_earth = square(sin(ae));

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

namespace
{
// Transverse Mercator (UTM) projection with the Krueger series to order n^6,
// see C.F.F. Karney, "Transverse Mercator with an accuracy of a few
// nanometers", J. Geodesy 85(8):475-485, 2011.
constexpr precnum_t UTM_K0 = 0.9996;
constexpr precnum_t UTM_FALSE_EASTING = 500000;
constexpr precnum_t UTM_FALSE_NORTHING_SOUTH = 1e7;

struct TMSeries
{
  precnum_t e = 0;  // First eccentricity
  precnum_t A = 0;  // Rectifying radius
  precnum_t alpha[6] = {0, 0, 0, 0, 0, 0};
  precnum_t beta[6] = {0, 0, 0, 0, 0, 0};
};

TMSeries tmSeries(const mrpt::topography::TEllipsoid& ellip)
{
  const precnum_t a = ellip.sa;
  const precnum_t b = ellip.sb;
  const precnum_t n = (a - b) / (a + b);
  const precnum_t n2 = n * n;
  const precnum_t n3 = n2 * n;
  const precnum_t n4 = n3 * n;
  const precnum_t n5 = n4 * n;
  const precnum_t n6 = n5 * n;

  TMSeries t;
  t.e = std::sqrt((a * a - b * b) / (a * a));
  t.A = a / (1 + n) * (1 + n2 / 4 + n4 / 64 + n6 / 256);

  t.alpha[0] =
      n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288 + 7891 * n6 / 37800;
  t.alpha[1] =
      13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630 - 1983433 * n6 / 1935360;
  t.alpha[2] = 61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880 + 167603 * n6 / 181440;
  t.alpha[3] = 49561 * n4 / 161280 - 179 * n5 / 168 + 6601661 * n6 / 7257600;
  t.alpha[4] = 34729 * n5 / 80640 - 3418889 * n6 / 1995840;
  t.alpha[5] = 212378941 * n6 / 319334400;

  t.beta[0] = n / 2 - 2 * n2 / 3 + 37 * n3 / 96 - n4 / 360 - 81 * n5 / 512 + 96199 * n6 / 604800;
  t.beta[1] = n2 / 48 + n3 / 15 - 437 * n4 / 1440 + 46 * n5 / 105 - 1118711 * n6 / 3870720;
  t.beta[2] = 17 * n3 / 480 - 37 * n4 / 840 - 209 * n5 / 4480 + 5569 * n6 / 90720;
  t.beta[3] = 4397 * n4 / 161280 - 11 * n5 / 504 - 830251 * n6 / 7257600;
  t.beta[4] = 4583 * n5 / 161280 - 108847 * n6 / 3991680;
  t.beta[5] = 20648693 * n6 / 638668800;
  return t;
}

// Geodetic (radians, dlon relative to the central meridian) to TM
// coordinates, without false easting/northing.
void tmForward(
    const mrpt::topography::TEllipsoid& ellip,
    precnum_t lat,
    precnum_t dlon,
    precnum_t& out_x,
    precnum_t& out_y)
{
  const TMSeries t = tmSeries(ellip);
  // Conformal latitude:
  const precnum_t sinLat = std::sin(lat);
  const precnum_t psi = std::asinh(std::tan(lat)) - t.e * std::atanh(t.e * sinLat);
  const precnum_t tauP = std::sinh(psi);
  const precnum_t cosDlon = std::cos(dlon);
  const precnum_t xiP = std::atan2(tauP, cosDlon);
  const precnum_t etaP = std::asinh(std::sin(dlon) / std::sqrt(tauP * tauP + cosDlon * cosDlon));

  precnum_t xi = xiP;
  precnum_t eta = etaP;
  for (int j = 1; j <= 6; j++)
  {
    xi += t.alpha[j - 1] * std::sin(2 * j * xiP) * std::cosh(2 * j * etaP);
    eta += t.alpha[j - 1] * std::cos(2 * j * xiP) * std::sinh(2 * j * etaP);
  }
  out_x = UTM_K0 * t.A * eta;
  out_y = UTM_K0 * t.A * xi;
}

// Inverse of tmForward()
void tmInverse(
    const mrpt::topography::TEllipsoid& ellip,
    precnum_t x,
    precnum_t y,
    precnum_t& out_lat,
    precnum_t& out_dlon)
{
  const TMSeries t = tmSeries(ellip);
  const precnum_t xi = y / (UTM_K0 * t.A);
  const precnum_t eta = x / (UTM_K0 * t.A);
  precnum_t xiP = xi;
  precnum_t etaP = eta;
  for (int j = 1; j <= 6; j++)
  {
    xiP -= t.beta[j - 1] * std::sin(2 * j * xi) * std::cosh(2 * j * eta);
    etaP -= t.beta[j - 1] * std::cos(2 * j * xi) * std::sinh(2 * j * eta);
  }
  const precnum_t sinhEtaP = std::sinh(etaP);
  const precnum_t cosXiP = std::cos(xiP);
  const precnum_t tauP = std::sin(xiP) / std::sqrt(sinhEtaP * sinhEtaP + cosXiP * cosXiP);
  out_dlon = std::atan2(sinhEtaP, cosXiP);

  // Solve tau=tan(lat) from the conformal tauP by Newton iterations:
  const precnum_t e2 = t.e * t.e;
  precnum_t tau = tauP;
  for (int it = 0; it < 10; it++)
  {
    const precnum_t sq = std::sqrt(1 + tau * tau);
    const precnum_t sigma = std::sinh(t.e * std::atanh(t.e * tau / sq));
    const precnum_t tauPi = tau * std::sqrt(1 + sigma * sigma) - sigma * sq;
    const precnum_t dtau = (tauP - tauPi) / std::sqrt(1 + tauPi * tauPi) *
                           (1 + (1 - e2) * tau * tau) / ((1 - e2) * sq);
    tau += dtau;
    if (std::abs(dtau) < 1e-15 * std::max<precnum_t>(1, std::abs(tau)))
    {
      break;
    }
  }
  out_lat = std::atan(tau);
}

// UTM zone number in [1,60] of a longitude, in degrees.
int utmZoneFromLongitude(double lon)
{
  // Longitude from the antimeridian, in [0,360):
  double l = std::fmod(lon + 180.0, 360.0);
  if (l < 0)
  {
    l += 360.0;
  }
  const int zone = static_cast<int>(std::floor(l / 6.0)) + 1;
  return std::clamp(zone, 1, 60);
}

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

void geodeticToUTMImpl(
    double lat_deg,
    double lon_deg,
    double& out_x,
    double& out_y,
    int& out_zone,
    char& out_band,
    const mrpt::topography::TEllipsoid& ellip)
{
  const int zone = utmZoneFromLongitude(lon_deg);
  const precnum_t lon0 = zone * 6 - 183;
  // Longitude relative to the central meridian, in [-180,180):
  const precnum_t dlon = mrpt::math::wrapToPi(mrpt::DEG2RAD(precnum_t(lon_deg) - lon0));

  precnum_t x = 0;
  precnum_t y = 0;
  tmForward(ellip, mrpt::DEG2RAD(precnum_t(lat_deg)), dlon, x, y);

  out_x = static_cast<double>(x + UTM_FALSE_EASTING);
  out_y = static_cast<double>(y);
  // Apply the false northing for points south of the equator, so that
  // northing values are always positive, following the standard UTM
  // convention (and matching what UTMToGeodetic() expects on input).
  if (out_y < 0)
  {
    out_y += static_cast<double>(UTM_FALSE_NORTHING_SOUTH);
  }
  out_zone = zone;
  out_band = utm_zone_from_latitude(lat_deg);
}

}  // namespace

/*---------------------------------------------------------------
          UTMToGeodetic
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

  const precnum_t x = precnum_t(X) - UTM_FALSE_EASTING;
  precnum_t y = Y;
  if (hem == 's' || hem == 'S')
  {
    y -= UTM_FALSE_NORTHING_SOUTH;
  }

  precnum_t lat = 0;
  precnum_t dlon = 0;
  tmInverse(ellip, x, y, lat, dlon);

  const precnum_t lon0 = huso * 6 - 183;
  out_lon = static_cast<double>(mrpt::RAD2DEG(dlon) + lon0);
  out_lat = static_cast<double>(mrpt::RAD2DEG(lat));
}

void mrpt::topography::geodeticToUTM(
    const TGeodeticCoords& GeodeticCoords,
    TUTMCoords& UTMCoords,
    int& UTMZone,
    char& UTMLatitudeBand,
    const TEllipsoid& ellipsoid)
{
  geodeticToUTMImpl(
      GeodeticCoords.lat, GeodeticCoords.lon, UTMCoords.x, UTMCoords.y, UTMZone, UTMLatitudeBand,
      ellipsoid);
  UTMCoords.z = GeodeticCoords.height;
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
  geodeticToUTMImpl(la, lo, xx, yy, out_UTM_zone, out_UTM_latitude_band, ellipsoid);
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

/**  7-parameter transformation, TOPCON convention:
 *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ m11 m12 m13; m21 m22 m23;
 * m31 m32 m33 ] [ X Y Z ]_local
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
 * sin(alpha) cos(alpha) ] [ X-Xp Y-Yp ]_local + [Xp Yp]
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

/**  Helmert 2D transformation, TOPCON convention:
 *   [ X Y ]_WGS84 = [ a b; -b a ] [ X Y ]_local + [ c d ]
 */
void mrpt::topography::transformHelmert2D_TOPCON(
    const mrpt::math::TPoint2D& p, const TDatumHelmert2D_TOPCON& d, mrpt::math::TPoint2D& o)
{
  o.x = d.a * p.x + d.b * p.y + d.c;
  o.y = -d.b * p.x + d.a * p.y + d.d;
}

/**  Helmert 3D transformation (position vector convention):
 *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 -RZ RY; RZ 1 -RX; -RY RX 1
 * ] [ X Y Z ]_local
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

/**  Helmert 3D transformation, TOPCON convention:
 *   [ X Y ]_WGS84 = [ a b; d e ] [ X Y ]_local + [ c f ], Z_WGS84 = Z_local + g
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

/**  Interpolation:
 *   [ X Y ]_WGS84 = [ dX dY ] + [ 1 -sin(beta); 0 cos(beta) ] [ dSx*X; dSy*Y ]
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
