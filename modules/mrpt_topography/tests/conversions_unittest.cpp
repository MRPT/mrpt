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

#include <gtest/gtest.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/topography.h>
#include <mrpt/topography/registerAllClasses.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::topography;
using namespace std;

void do_test_geodetic_geocentric(const TGeodeticCoords c1)
{
  TGeocentricCoords geo1;
  mrpt::topography::geodeticToGeocentric(c1, geo1, TEllipsoid::Ellipsoid_WGS84());

  TGeodeticCoords c2;
  mrpt::topography::geocentricToGeodetic(geo1, c2, TEllipsoid::Ellipsoid_WGS84());

  EXPECT_NEAR(c1.lat, c2.lat, 1e-8);
  EXPECT_NEAR(c1.lon, c2.lon, 1e-8);
  EXPECT_NEAR(c1.height, c2.height, 1e-4);
}

TEST(TopographyConversion, GeodeticToGeocentricToGeodetic)
{
  do_test_geodetic_geocentric(
      TGeodeticCoords(TCoords(36, 1, 30).decimal_value, TCoords(3, 2, 40).decimal_value, 20));
  do_test_geodetic_geocentric(
      TGeodeticCoords(TCoords(-36, 1, 30).decimal_value, TCoords(3, 2, 40).decimal_value, 20));
  do_test_geodetic_geocentric(
      TGeodeticCoords(TCoords(36, 1, 30).decimal_value, TCoords(-3, 2, 40).decimal_value, 20));
  do_test_geodetic_geocentric(
      TGeodeticCoords(TCoords(-36, 1, 30).decimal_value, TCoords(-3, 2, 40).decimal_value, 20));
}

TEST(TopographyConversion, geodeticToENU_WGS84)
{
  // A test of GPS WGS84 -> ENU Cartessian coordinates with known solution
  mrpt::topography::TGeodeticCoords gps_point;
  gps_point.lon = -4.475828390;
  gps_point.lat = 36.716411055;
  gps_point.height = 48.243;

  mrpt::topography::TGeodeticCoords gps_ref;
  gps_ref.lon = -4.4789588283333330;
  gps_ref.lat = 36.714459075;
  gps_ref.height = 38.8887;

  TPoint3D P;
  mrpt::topography::geodeticToENU_WGS84(gps_point, P, gps_ref);

  TPoint3D P_true(279.679067794, 216.621954, 9.34448);
  EXPECT_NEAR(P.x, P_true.x, 3e-3);  // Precision should be *much* better than
  // 1mm, but don't be so hard...
  EXPECT_NEAR(P.y, P_true.y, 3e-3);
  EXPECT_NEAR(P.z, P_true.z, 3e-3);

  // 2nd test: just modify the height and verify that ENU coordinates only
  // change in Z:
  mrpt::topography::TGeodeticCoords gps_point2 = gps_ref;
  const double A_height = 15.0;
  gps_point2.height += A_height;

  mrpt::topography::geodeticToENU_WGS84(gps_point2, P, gps_ref);

  EXPECT_NEAR(P.x, 0, 0.1e-3);  // Precision: 0.1mm
  EXPECT_NEAR(P.y, 0, 0.1e-3);
  EXPECT_NEAR(P.z, A_height, 0.1e-3);
}

TEST(TopographyConversion, geodeticToUTM_WGS84)
{
  mrpt::topography::TGeodeticCoords gps_point;
  gps_point.lon = -2.404508;
  gps_point.lat = 36.829293;
  gps_point.height = 0;

  mrpt::topography::TUTMCoords utm;
  int utm_zone = 0;
  char lat_band = 0;

  mrpt::topography::geodeticToUTM(gps_point, utm, utm_zone, lat_band);

  EXPECT_EQ(utm_zone, 30);
  EXPECT_EQ(lat_band, 'S');
  EXPECT_NEAR(utm.x, 553103.02, 0.05);
  EXPECT_NEAR(utm.y, 4076100.98, 0.05);
}

TEST(TopographyConversion, UTMToGeodetic_WGS84)
{
  const mrpt::topography::TUTMCoords utm = {553103.020, 4076100.969, 0.0};

  mrpt::topography::TGeodeticCoords gc;
  mrpt::topography::UTMToGeodetic(utm, 30, 'N', gc);

  EXPECT_NEAR(gc.lat, 36.829293, 1e-6);
  EXPECT_NEAR(gc.lon, -2.404508, 1e-6);
}

TEST(TopographyConversion, UTMToGeodetic_InvalidHemisphereThrows)
{
  double lon;
  double lat;
  EXPECT_THROW(
      mrpt::topography::UTMToGeodetic(553103.020, 4076100.969, 30, 'Z', lon, lat), std::exception);
}

TEST(TopographyConversion, geodeticToUTM_SouthernHemisphereRoundTrip)
{
  // Regression test: geodeticToUTM() must apply the standard UTM false
  // northing for the southern hemisphere, matching the convention that
  // UTMToGeodetic() expects on its input (northing always positive).
  for (const auto& p :
       {TGeodeticCoords(-33.45, -70.6667, 0.0), TGeodeticCoords(-89.9, 45.0, 0.0),
        TGeodeticCoords(-0.5, 10.0, 0.0)})
  {
    TUTMCoords utm;
    int zone = 0;
    char band = 0;
    geodeticToUTM(p, utm, zone, band);

    // Northing must be a positive value using the false-northing
    // convention for the southern hemisphere.
    EXPECT_GT(utm.y, 0.0);

    TGeodeticCoords back;
    UTMToGeodetic(utm, zone, 'S', back);
    EXPECT_NEAR(back.lat, p.lat, 1e-4);
    EXPECT_NEAR(back.lon, p.lon, 1e-4);
  }
}

TEST(TopographyConversion, geodeticToUTM_and_GeodeticToUTM_areConsistent)
{
  // The double-based (legacy) GeodeticToUTM() and the TGeodeticCoords-based
  // geodeticToUTM() implement the same projection and must agree,
  // including for points south of the equator.
  const std::vector<std::pair<double, double>> latlon = {
      {36.829293, -2.404508},
      {      0.0,       0.0},
      {   -33.45,  -70.6667},
      {  51.5074,   -0.1278},
      {    -89.9,      45.0},
      {     89.9,      45.0}
  };

  for (const auto& [lat, lon] : latlon)
  {
    double x1 = 0;
    double y1 = 0;
    int zone1 = 0;
    char band1 = 0;
    GeodeticToUTM(lat, lon, x1, y1, zone1, band1);

    TGeodeticCoords gc(lat, lon, 0.0);
    TUTMCoords utm2;
    int zone2 = 0;
    char band2 = 0;
    geodeticToUTM(gc, utm2, zone2, band2);

    EXPECT_EQ(zone1, zone2);
    EXPECT_EQ(band1, band2);
    EXPECT_NEAR(x1, utm2.x, 1e-3);
    EXPECT_NEAR(y1, utm2.y, 1e-3);
  }
}

TEST(TopographyConversion, UTMLatitudeBandsAcrossAllRanges)
{
  // Exercise the full range of UTM latitude bands (C..X, skipping I and O),
  // spanning both hemispheres.
  const std::vector<std::pair<double, char>> cases = {
      {-79, 'C'},
      {-70, 'D'},
      {-60, 'E'},
      {-50, 'F'},
      {-42, 'G'},
      {-34, 'H'},
      {-28, 'J'},
      {-20, 'K'},
      {-12, 'L'},
      { -4, 'M'},
      {  4, 'N'},
      { 12, 'P'},
      { 20, 'Q'},
      { 28, 'R'},
      { 36, 'S'},
      { 44, 'T'},
      { 52, 'U'},
      { 60, 'V'},
      { 68, 'W'},
      { 75, 'X'},
  };
  for (const auto& [lat, band] : cases)
  {
    TGeodeticCoords gc(lat, 10.0, 0.0);
    TUTMCoords utm;
    int zone = 0;
    char out_band = 0;
    geodeticToUTM(gc, utm, zone, out_band);
    EXPECT_EQ(out_band, band) << "lat=" << lat;
  }
}

TEST(TopographyConversion, geocentricToGeodetic_PoleHeightIsAccurate)
{
  // Regression test: the height computation at/near the poles must not
  // blow up numerically (a naive p/cos(lat) formula is degenerate there).
  for (double lat : {89.999, 90.0, -90.0})
  {
    TGeodeticCoords in(lat, 10.0, 100.0);
    TGeocentricCoords geo;
    geodeticToGeocentric_WGS84(in, geo);

    TGeodeticCoords out;
    geocentricToGeodetic(geo, out);

    EXPECT_NEAR(out.height, 100.0, 1e-3) << "lat=" << lat;
    EXPECT_NEAR(std::abs(out.lat.decimal_value), std::abs(lat), 1e-6);
  }
}

TEST(TopographyConversion, geocentricToGeodetic_Equator)
{
  TGeocentricCoords gc(TEllipsoid::Ellipsoid_WGS84().sa, 0.0, 0.0);
  TGeodeticCoords out;
  geocentricToGeodetic(gc, out);
  EXPECT_NEAR(out.lat.decimal_value, 0.0, 1e-9);
  EXPECT_NEAR(out.lon.decimal_value, 0.0, 1e-9);
  EXPECT_NEAR(out.height, 0.0, 1e-6);
}

TEST(TopographyConversion, geocentricToENU_WGS84_PointAndVectorOverloadsMatch)
{
  const TGeodeticCoords ref(36.714459075, -4.4789588283333330, 38.8887);

  std::vector<TPoint3D> geocentric_pts;
  for (const auto& g :
       {TGeodeticCoords(36.716411055, -4.475828390, 48.243),
        TGeodeticCoords(36.720000000, -4.470000000, 100.0),
        TGeodeticCoords(36.700000000, -4.500000000, 0.0)})
  {
    TPoint3D p;
    geodeticToGeocentric_WGS84(g, p);
    geocentric_pts.push_back(p);
  }

  std::vector<TPoint3D> enu_batch;
  geocentricToENU_WGS84(geocentric_pts, enu_batch, ref);
  ASSERT_EQ(enu_batch.size(), geocentric_pts.size());

  for (size_t i = 0; i < geocentric_pts.size(); i++)
  {
    TPoint3D enu_single;
    geocentricToENU_WGS84(geocentric_pts[i], enu_single, ref);
    EXPECT_NEAR(enu_single.x, enu_batch[i].x, 1e-9);
    EXPECT_NEAR(enu_single.y, enu_batch[i].y, 1e-9);
    EXPECT_NEAR(enu_single.z, enu_batch[i].z, 1e-9);
  }
}

TEST(TopographyConversion, ENU_axes_from_WGS84_ConsistentWithGeodeticToENU)
{
  const TGeodeticCoords ref(36.714459075, -4.4789588283333330, 38.8887);
  const TGeodeticCoords target(36.716411055, -4.475828390, 48.243);

  TPoint3D p_enu;
  geodeticToENU_WGS84(target, p_enu, ref);

  TPose3D enu_axes;
  ENU_axes_from_WGS84(ref, enu_axes, false);

  const TPoint3D p_geocentric_via_pose = enu_axes.composePoint(p_enu);

  TPoint3D p_geocentric_direct;
  geodeticToGeocentric_WGS84(target, p_geocentric_direct);

  EXPECT_NEAR(p_geocentric_via_pose.x, p_geocentric_direct.x, 1e-6);
  EXPECT_NEAR(p_geocentric_via_pose.y, p_geocentric_direct.y, 1e-6);
  EXPECT_NEAR(p_geocentric_via_pose.z, p_geocentric_direct.z, 1e-6);
}

TEST(TopographyConversion, ENU_axes_from_WGS84_OnlyAnglesZeroesTranslation)
{
  TPose3D enu_axes;
  ENU_axes_from_WGS84(TGeodeticCoords(36.7, -4.5, 100.0), enu_axes, true);
  EXPECT_DOUBLE_EQ(enu_axes.x, 0.0);
  EXPECT_DOUBLE_EQ(enu_axes.y, 0.0);
  EXPECT_DOUBLE_EQ(enu_axes.z, 0.0);
}

TEST(TopographyConversion, ENUToGeocentric_IsInverseOfGeodeticToENU)
{
  // Regression test: ENUToGeocentric() must use the same ellipsoid-normal
  // "Up" convention as geodeticToENU_WGS84()/geocentricToENU_WGS84(), so
  // that they form an exact forward/inverse pair.
  const TGeodeticCoords ref(36.714459075, -4.4789588283333330, 38.8887);
  const TGeodeticCoords target(36.716411055, -4.475828390, 48.243);

  TPoint3D p_enu;
  geodeticToENU_WGS84(target, p_enu, ref);

  TGeocentricCoords p_geo_roundtrip;
  ENUToGeocentric(p_enu, ref, p_geo_roundtrip, TEllipsoid::Ellipsoid_WGS84());

  TPoint3D p_geocentric_direct;
  geodeticToGeocentric_WGS84(target, p_geocentric_direct);

  EXPECT_NEAR(p_geo_roundtrip.x, p_geocentric_direct.x, 1e-3);
  EXPECT_NEAR(p_geo_roundtrip.y, p_geocentric_direct.y, 1e-3);
  EXPECT_NEAR(p_geo_roundtrip.z, p_geocentric_direct.z, 1e-3);
}

TEST(TopographyConversion, ENUToGeocentric_OriginMapsToReferencePoint)
{
  const TGeodeticCoords ref(10.0, 20.0, 50.0);
  TPoint3D p_geocentric_ref;
  geodeticToGeocentric_WGS84(ref, p_geocentric_ref);

  TGeocentricCoords out;
  ENUToGeocentric(TPoint3D(0, 0, 0), ref, out, TEllipsoid::Ellipsoid_WGS84());

  EXPECT_NEAR(out.x, p_geocentric_ref.x, 1e-3);
  EXPECT_NEAR(out.y, p_geocentric_ref.y, 1e-3);
  EXPECT_NEAR(out.z, p_geocentric_ref.z, 1e-3);
}

TEST(TopographyConversion, RegisterAllClasses)
{
  // Trivial smoke-test to make sure this function can be invoked without
  // errors (it is normally only needed for static-library builds).
  EXPECT_NO_THROW(mrpt::topography::registerAllClasses_mrpt_topography());
}
