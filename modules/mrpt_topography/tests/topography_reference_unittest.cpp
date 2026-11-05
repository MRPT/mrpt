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

// Checks mrpt::topography against reference values generated with PROJ 9.4
// (proj / cs2cs, WGS84) and against authoritative ellipsoid definitions.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/topography.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace mrpt::topography;

namespace
{
struct UTMRef
{
  double lat;
  double lon;
  int zone;
  char band;
  double x;
  double y;
};

// echo "lon lat" | proj -f "%.4f" +proj=utm +zone=Z [+south] +ellps=WGS84
std::vector<UTMRef> utmRefs()
{
  const std::vector<UTMRef> refs = {
      {40.3154333, -3.4857166, 30, 'T', 458730.6999, 4462881.3645},
      {   36.8340,    -2.4637, 30, 'S', 547821.5953, 4076591.8847},
      {  -33.8688,   151.2093, 56, 'H', 334368.6336, 6250948.3454},
      {  -54.8019,   -68.3030, 19, 'F', 544805.0975, 3927029.8847},
      {   -0.1807,   -78.4678, 17, 'M', 781861.4575, 9980007.5669},
      {   61.2181,  -149.9003,  6, 'V', 344247.2065, 6790536.8713},
      {      10.0,      5.999, 31, 'P', 828818.9983, 1106907.8557},
      {       0.5,      6.001, 32, 'N', 166145.5227,   55341.3373},
      {      70.0,       26.9, 35, 'W', 496182.8748, 7765876.2657},
      {      80.0,      179.5, 60, 'X', 548449.8703, 8882626.9449},
      {     -79.9,     -0.001, 30, 'C', 558687.8949, 1128063.1800},
      {      45.0,    -86.999, 16, 'T', 500078.8153, 4982950.4007},
      {     -45.0,    170.999, 59, 'G', 499921.1847, 5017049.5993},
  };
  return refs;
}

struct ECEFRef
{
  double lon;
  double lat;
  double h;
  double X;
  double Y;
  double Z;
};

// echo "lon lat h" | cs2cs -f "%.5f" +proj=longlat +datum=WGS84 +to
//   +proj=geocent +datum=WGS84
std::vector<ECEFRef> ecefRefs()
{
  const std::vector<ECEFRef> refs = {
      {       0.0,        0.0,    0.0,  6378137.00000,       0.00000,        0.00000},
      {-3.4857166, 40.3154333,  650.0,  4861605.15104, -296132.03290,  4105174.69964},
      {  151.2093,   -33.8688,   58.0, -4646093.47729, 2553229.53582, -3534404.71091},
      {      10.0,       90.0,    0.0,        0.00000,       0.00000,  6356752.31425},
      {    -120.5,    -89.999, 1234.5,      -56.69992,     -96.25735, -6357986.81327},
      {      45.0,       45.0, -400.0,  3194219.14506, 3194219.14506,  4487065.56615},
  };
  return refs;
}

}  // namespace

TEST(TopographyReference, GeodeticToGeocentricMatchesPROJ)
{
  const auto wgs84 = TEllipsoid::Ellipsoid_WGS84();
  for (const auto& r : ecefRefs())
  {
    TGeocentricCoords P;
    geodeticToGeocentric(TGeodeticCoords(r.lat, r.lon, r.h), P, wgs84);
    EXPECT_NEAR(P.x, r.X, 1e-4) << r.lat << " " << r.lon;
    EXPECT_NEAR(P.y, r.Y, 1e-4) << r.lat << " " << r.lon;
    EXPECT_NEAR(P.z, r.Z, 1e-4) << r.lat << " " << r.lon;

    mrpt::math::TPoint3D P2;
    geodeticToGeocentric_WGS84(TGeodeticCoords(r.lat, r.lon, r.h), P2);
    EXPECT_NEAR(P2.x, r.X, 1e-4) << r.lat << " " << r.lon;
    EXPECT_NEAR(P2.y, r.Y, 1e-4) << r.lat << " " << r.lon;
    EXPECT_NEAR(P2.z, r.Z, 1e-4) << r.lat << " " << r.lon;
  }
}

// The WGS84 helper must agree with the generic function used by the ENU
// inverse.
TEST(TopographyReference, GeodeticToGeocentricWGS84HelperMatchesGeneric)
{
  const auto wgs84 = TEllipsoid::Ellipsoid_WGS84();
  for (const double lat : {0.0, 30.0, 45.0, 60.0, 90.0})
  {
    const TGeodeticCoords g(lat, 12.0, 100.0);
    mrpt::math::TPoint3D a;
    TGeocentricCoords b;
    geodeticToGeocentric_WGS84(g, a);
    geodeticToGeocentric(g, b, wgs84);
    EXPECT_LT((a - b).norm(), 1e-6) << "lat=" << lat;
  }
}

TEST(TopographyReference, GeocentricToGeodeticMatchesPROJ)
{
  for (const auto& r : ecefRefs())
  {
    TGeodeticCoords g;
    geocentricToGeodetic(TGeocentricCoords(r.X, r.Y, r.Z), g);
    EXPECT_NEAR(g.lat, r.lat, 1e-9) << r.lat << " " << r.lon;
    if (std::abs(r.lat) < 90.0)
    {
      // Reference values are printed with 1e-5 m resolution:
      const double p = std::hypot(r.X, r.Y);
      const double lonTol = std::max(1e-9, 2e-5 / p * 180.0 / M_PI);
      EXPECT_NEAR(g.lon, r.lon, lonTol) << r.lat << " " << r.lon;
    }
    EXPECT_NEAR(g.height, r.h, 1e-4) << r.lat << " " << r.lon;
  }
}

TEST(TopographyReference, GeodeticGeocentricRoundTripTerrestrial)
{
  const auto wgs84 = TEllipsoid::Ellipsoid_WGS84();
  for (const double h : {-500.0, 0.0, 3000.0, 1e4})
  {
    for (double lat = -89.5; lat <= 89.5; lat += 7.3)
    {
      for (double lon = -179.0; lon <= 179.0; lon += 41.0)
      {
        TGeocentricCoords P;
        TGeodeticCoords g;
        geodeticToGeocentric(TGeodeticCoords(lat, lon, h), P, wgs84);
        geocentricToGeodetic(P, g, wgs84);
        // 1e-10 deg is about 0.01 mm on the ground.
        EXPECT_NEAR(g.lat, lat, 1e-10);
        EXPECT_NEAR(g.lon, lon, 1e-10);
        EXPECT_NEAR(g.height, h, 1e-6);
      }
    }
  }
}

TEST(TopographyReference, EllipsoidConstantsMatchDefinitions)
{
  struct Def
  {
    TEllipsoid e;
    double a;
    double b;
  };
  // b = a*(1-1/rf) for flattening-defined ellipsoids (EPSG / PROJ values).
  const auto bOf = [](double a, double rf) { return a * (1.0 - 1.0 / rf); };
  const std::vector<Def> defs = {
      {                   TEllipsoid::Ellipsoid_WGS84(),   6378137.0,   bOf(6378137.0, 298.257223563)},
      {                   TEllipsoid::Ellipsoid_GRS80(),   6378137.0,   bOf(6378137.0, 298.257222101)},
      {                   TEllipsoid::Ellipsoid_WGS72(),   6378135.0,   bOf(6378135.0,        298.26)},
      {                   TEllipsoid::Ellipsoid_WGS66(),   6378145.0,   bOf(6378145.0,        298.25)},
      {      TEllipsoid::Ellipsoid_Internacional_1924(),   6378388.0,   bOf(6378388.0,         297.0)},
      {            TEllipsoid::Ellipsoid_Hayford_1909(),   6378388.0,   bOf(6378388.0,         297.0)},
      {          TEllipsoid::Ellipsoid_Krasovsky_1940(),   6378245.0,   bOf(6378245.0,         298.3)},
      {            TEllipsoid::Ellipsoid_Helmert_1906(),   6378200.0,   bOf(6378200.0,         298.3)},
      {              TEllipsoid::Ellipsoid_Hough_1960(),   6378270.0,   bOf(6378270.0,         297.0)},
      {            TEllipsoid::Ellipsoid_Fischer_1960(),   6378166.0,   bOf(6378166.0,         298.3)},
      {            TEllipsoid::Ellipsoid_Fischer_1968(),   6378150.0,   bOf(6378150.0,         298.3)},
      {            TEllipsoid::Ellipsoid_Mercury_1960(),   6378166.0,   bOf(6378166.0,         298.3)},
      { TEllipsoid::Ellipsoid_Mercury_Modificado_1968(),   6378150.0,   bOf(6378150.0,         298.3)},
      {       TEllipsoid::Ellipsoid_Sudamericano_1969(),   6378160.0,   bOf(6378160.0,        298.25)},
      {             TEllipsoid::Ellipsoid_Bessel_1841(), 6377397.155, bOf(6377397.155,   299.1528128)},
      {               TEllipsoid::Ellipsoid_Airy_1830(), 6377563.396, bOf(6377563.396,   299.3249646)},
      {    TEllipsoid::Ellipsoid_Airy_Modificado_1965(), 6377340.189, bOf(6377340.189,   299.3249646)},
      {             TEllipsoid::Ellipsoid_Clarke_1880(), 6378249.145, bOf(6378249.145,       293.465)},
      {             TEllipsoid::Ellipsoid_Clarke_1866(),   6378206.4,       6356583.8               },
      {TEllipsoid::Ellipsoid_Nuevo_Internacional_1967(),   6378157.5,       6356772.2               },
      {            TEllipsoid::Ellipsoid_Walbeck_1817(),   6376896.0,    6355834.8467               },
  };
  for (const auto& d : defs)
  {
    EXPECT_EQ(d.e.sa, d.a) << d.e.name;
    EXPECT_NEAR(d.e.sb, d.b, 1e-5) << d.e.name;
  }
}

TEST(TopographyReference, UTMZonesAndBandsMatchPROJ)
{
  for (const auto& r : utmRefs())
  {
    double x = 0;
    double y = 0;
    int zone = 0;
    char band = 0;
    GeodeticToUTM(r.lat, r.lon, x, y, zone, band);
    EXPECT_EQ(zone, r.zone) << r.lat << " " << r.lon;
    EXPECT_EQ(band, r.band) << r.lat << " " << r.lon;

    TUTMCoords u;
    int zone2 = 0;
    char band2 = 0;
    geodeticToUTM(TGeodeticCoords(r.lat, r.lon, 5.0), u, zone2, band2);
    EXPECT_EQ(zone2, r.zone);
    EXPECT_EQ(band2, r.band);
    EXPECT_EQ(u.z, 5.0);
  }
}

TEST(TopographyReference, UTMWithin1mmOfPROJ)
{
  for (const auto& r : utmRefs())
  {
    double x = 0;
    double y = 0;
    int zone = 0;
    char band = 0;
    GeodeticToUTM(r.lat, r.lon, x, y, zone, band);
    EXPECT_NEAR(x, r.x, 1e-3) << r.lat << " " << r.lon;
    EXPECT_NEAR(y, r.y, 1e-3) << r.lat << " " << r.lon;

    double lon = 0;
    double lat = 0;
    UTMToGeodetic(r.x, r.y, r.zone, r.lat < 0 ? 'S' : 'N', lon, lat);
    // 1e-8 deg is about 1 mm.
    EXPECT_NEAR(lat, r.lat, 1e-8) << r.lat << " " << r.lon;
    EXPECT_NEAR(lon, r.lon, 1e-8 / std::cos(r.lat * M_PI / 180.0)) << r.lat << " " << r.lon;
  }
}

TEST(TopographyReference, UTMRoundTripOtherEllipsoid)
{
  const auto intl = TEllipsoid::Ellipsoid_Internacional_1924();
  for (const auto& r : utmRefs())
  {
    double x = 0;
    double y = 0;
    int zone = 0;
    char band = 0;
    GeodeticToUTM(r.lat, r.lon, x, y, zone, band, intl);
    double lon = 0;
    double lat = 0;
    UTMToGeodetic(x, y, zone, r.lat < 0 ? 's' : 'n', lon, lat, intl);
    EXPECT_NEAR(lat, r.lat, 1e-10) << r.lat << " " << r.lon;
    EXPECT_NEAR(lon, r.lon, 1e-10) << r.lat << " " << r.lon;
  }
}

// Longitudes 180 and -180 are the same meridian, in a valid zone.
TEST(TopographyReference, UTMZoneAtAntimeridian)
{
  double x1 = 0;
  double y1 = 0;
  int zone1 = 0;
  char band1 = 0;
  GeodeticToUTM(10.0, 180.0, x1, y1, zone1, band1);
  EXPECT_GE(zone1, 1);
  EXPECT_LE(zone1, 60);
  double x2 = 0;
  double y2 = 0;
  int zone2 = 0;
  char band2 = 0;
  GeodeticToUTM(10.0, -180.0, x2, y2, zone2, band2);
  EXPECT_EQ(zone1, zone2);
  EXPECT_NEAR(x1, x2, 1e-6);
  EXPECT_NEAR(y1, y2, 1e-6);
  // Zone boundaries belong to the eastern zone:
  GeodeticToUTM(10.0, 6.0, x1, y1, zone1, band1);
  EXPECT_EQ(zone1, 32);
  GeodeticToUTM(10.0, -6.0, x1, y1, zone1, band1);
  EXPECT_EQ(zone1, 30);
}

TEST(TopographyReference, ENURoundTripAndOrthonormality)
{
  const TGeodeticCoords origin(36.8340, -2.4637, 120.0);
  mrpt::math::TPose3D enu;
  ENU_axes_from_WGS84(origin, enu, true);
  mrpt::math::CMatrixDouble33 R;
  enu.getRotationMatrix(R);
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      double dot = 0;
      for (int k = 0; k < 3; k++)
      {
        dot += R(k, r) * R(k, c);
      }
      EXPECT_NEAR(dot, r == c ? 1.0 : 0.0, 1e-12);
    }
  }
  for (const double dE : {-2000.0, 0.0, 1500.0})
  {
    for (const double dN : {-800.0, 2500.0})
    {
      for (const double dU : {-30.0, 400.0})
      {
        const mrpt::math::TPoint3D p(dE, dN, dU);
        TGeodeticCoords g;
        ENUToGeodetic_WGS84(p, g, origin);
        mrpt::math::TPoint3D p2;
        geodeticToENU_WGS84(g, p2, origin);
        EXPECT_LT((p2 - p).norm(), 1e-6);
      }
    }
  }
}

// The sign must be kept for negative angles with zero whole degrees.
TEST(TopographyReference, TCoordsNegativeBelowOneDegreeRoundTrip)
{
  for (const double v : {-0.5, -0.0125, -0.99})
  {
    const TCoords c(v);
    int d = 0;
    int m = 0;
    double s = 0;
    c.getDegMinSec(d, m, s);
    TCoords c2;
    c2.setDegMinSec(d, m, s);
    EXPECT_NEAR(c2.getDecimalValue(), v, 1e-12) << "v=" << v;
    EXPECT_NE(c.getAsString().find('-'), std::string::npos) << c.getAsString();
  }
}

TEST(TopographyReference, SevenParamConventionsMatchEPSGExample)
{
  // EPSG Guidance Note 7-2, WGS 72 to WGS 84 worked example.
  const mrpt::math::TPoint3D in(3657660.66, 255768.55, 5201382.11);
  const mrpt::math::TPoint3D expected(3657660.78, 255778.43, 5201387.75);
  mrpt::math::TPoint3D o;
  // transformHelmert3D() follows the position-vector convention (EPSG:9606):
  transformHelmert3D(in, TDatumHelmert3D(0, 0, 4.5, 0, 0, 0.554, 0.219), o);
  EXPECT_LT((o - expected).norm(), 0.01);
  // transform7params() follows the coordinate-frame convention (EPSG:9607),
  // whose published parameters have the opposite rotation sign:
  transform7params(in, TDatum7Params(0, 0, 4.5, 0, 0, -0.554, 0.219), o);
  EXPECT_LT((o - expected).norm(), 0.01);
}
