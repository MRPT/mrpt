/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;


void TestGPS_coords()
{
	const double lat0 = RAD2DEG( 0.6408472493152757 );
	const double lon0 = RAD2DEG(-0.0780454933097760 );
	const double h0   = 53.200600; // meters

	const double lat1 = RAD2DEG( 0.6408615769267271 );
	const double lon1 = RAD2DEG(-0.0780621148947297 );
	const double h1   = 56.210100; // meters

	cout << "Point 0: lon=" << lon0 << " lat=" << lat0 << " h=" << h0 << endl;
	cout << "Point 1: lon=" << lon1 << " lat=" << lat1 << " h=" << h1 << endl;

	mrpt::poses::TPoint3D   p;
	mrpt::topography::coordinatesTransformation_WGS84(lon1,lat1,h1, p.x,p.y,p.z, lon0,lat0,h0);

	cout << "ENU XYZ coords: " << p << endl;

	// UTM:
	// See: http://www.mathworks.com/matlabcentral/fileexchange/10915
	cout << endl << "UTM coordinate test:" << endl;

	double lats[] = { 40.3154333, 46.283900, 37.577833, 28.645650, 38.855550, 25.061783 };
	double lons[] = { -3.4857166, 7.8012333, -119.95525, -17.759533, -94.7990166, 121.640266 };
	int    zone;
	char   band;
	for (size_t i=0;i<sizeof(lats)/sizeof(lats[0]);i++)
	{
		topography::TUTMCoords UTMCoords;
		const double la = lats[i];
		const double lo = lons[i];
		//mrpt::topography::LatLonToUTM( la,lo, x,y,zone,band );
		mrpt::topography::geodeticToUTM( topography::TGeodeticCoords( la, lo, 0 ), UTMCoords, zone, band );
		cout << "Lat: " << la << " Lon: " << lo << " x: " << UTMCoords.x << " y: " << UTMCoords.y << " zone: " << zone << " band: " << band << endl;
	}
}


void TestGeoid2Geocentric()
{
	const double lon0 = -3.733572031; // RAD2DEG( 0.6408472493152757L );
	const double lat0 = 37.89250616; // RAD2DEG(-0.0780454933097760L );
	const double h0   = 515.579; // 53.200600; // meters

	cout << endl;
	cout << format("Point: lon=%.012f lat=%.012f h=%.04f\n",lon0,lat0,h0);

	mrpt::poses::TPoint3D   p;
	mrpt::topography::coordinatesTransformation_WGS84_geocentric(lon0,lat0,h0, p.x,p.y,p.z);

	cout << "In geocentric coords: " << p << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestGPS_coords();
		TestGeoid2Geocentric();
		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}




