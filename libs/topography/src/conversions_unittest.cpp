/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
