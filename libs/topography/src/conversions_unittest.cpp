/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

	TPoint3D P_true(279.679068,216.650884,8.647818);
	EXPECT_NEAR(P.x,P_true.x, 3e-3); // Precision should be *much* better than 1mm, but don't be so hard...
	EXPECT_NEAR(P.y,P_true.y, 3e-3);
	EXPECT_NEAR(P.z,P_true.z, 3e-3);
}
