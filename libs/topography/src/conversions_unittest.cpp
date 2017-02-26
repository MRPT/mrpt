/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/topography.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::topography;
using namespace std;

void do_test_geodetic_geocentric(const TGeodeticCoords  c1)
{
	TGeocentricCoords  geo1;
	mrpt::topography::geodeticToGeocentric(c1,geo1, TEllipsoid::Ellipsoid_WGS84() );

	TGeodeticCoords  c2;
	mrpt::topography::geocentricToGeodetic(geo1, c2, TEllipsoid::Ellipsoid_WGS84() );

	EXPECT_NEAR( c1.lat,c2.lat, 1e-8 );
	EXPECT_NEAR( c1.lon,c2.lon, 1e-8 );
	EXPECT_NEAR( c1.height,c2.height, 1e-4 );
}



TEST(TopographyConversion, GeodeticToGeocentricToGeodetic )
{
	do_test_geodetic_geocentric(  TGeodeticCoords( TCoords(36,1,30), TCoords(3,2,40), 20 ) );
	do_test_geodetic_geocentric(  TGeodeticCoords( TCoords(-36,1,30), TCoords(3,2,40), 20 ) );
	do_test_geodetic_geocentric(  TGeodeticCoords( TCoords(36,1,30), TCoords(-3,2,40), 20 ) );
	do_test_geodetic_geocentric(  TGeodeticCoords( TCoords(-36,1,30), TCoords(-3,2,40), 20 ) );

}

TEST(TopographyConversion, geodeticToENU_WGS84 )
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
	mrpt::topography::geodeticToENU_WGS84(gps_point, P,gps_ref);

	TPoint3D P_true(279.679067794,216.621954,9.34448);
	EXPECT_NEAR(P.x,P_true.x, 3e-3); // Precision should be *much* better than 1mm, but don't be so hard...
	EXPECT_NEAR(P.y,P_true.y, 3e-3);
	EXPECT_NEAR(P.z,P_true.z, 3e-3);


	// 2nd test: just modify the height and verify that ENU coordinates only change in Z:
	mrpt::topography::TGeodeticCoords gps_point2 = gps_ref;
	const double A_height = 15.0;
	gps_point2.height += A_height;

	mrpt::topography::geodeticToENU_WGS84(gps_point2, P,gps_ref);

	EXPECT_NEAR(P.x,0, 0.1e-3);  // Precision: 0.1mm
	EXPECT_NEAR(P.y,0, 0.1e-3);
	EXPECT_NEAR(P.z,A_height, 0.1e-3);

}
