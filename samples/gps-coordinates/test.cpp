/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/topography.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;


void TestGPS_coords()
{
	// ENU:
	const mrpt::topography::TGeodeticCoords p0(
		RAD2DEG( 0.6408472493152757 ), // lat
		RAD2DEG(-0.0780454933097760 ), // lon
		53.200600 // height meters
		);

	const mrpt::topography::TGeodeticCoords p1(
		RAD2DEG( 0.6408615769267271 ), // lat
		RAD2DEG(-0.0780621148947297 ), // lon
		56.210100 // meters
		);

	cout << "Point 0: lat=" << p0.lat << " lon=" << p0.lon << " alt="<< p0.height << endl;
	cout << "Point 1: lat=" << p1.lat << " lon=" << p1.lon << " alt="<< p1.height << endl;

	mrpt::math::TPoint3D   p;
	mrpt::topography::geodeticToENU_WGS84(p1,p, p0);
	cout << "ENU XYZ coords: " << p << endl;

	// Geocentric:
	const mrpt::topography::TGeodeticCoords pt0(32.0, -10.0, 0.0 );
	const mrpt::topography::TGeodeticCoords pt1(32.0, -10.0, 500.0 );

	mrpt::math::TPoint3D   geo0,geo1;
	mrpt::topography::geodeticToGeocentric_WGS84(pt0,geo0);
	mrpt::topography::geodeticToGeocentric_WGS84(pt1,geo1);
	cout << "P0 XYZ geocentric coords: " << geo0 << endl;
	cout << "P1 XYZ geocentric coords: " << geo1 << endl;


	// ENU_axes_from_WGS84
	mrpt::math::TPose3D pose_ENU;
	mrpt::topography::ENU_axes_from_WGS84(pt0.lon, pt0.lat, pt0.height, pose_ENU);
	cout << "ENU system of coordinates for lat=" << pt0.lat << " lon=" << pt0.lon << " alt="<< pt0.height << " is: \n" << mrpt::poses::CPose3D(pose_ENU).getHomogeneousMatrixVal() << endl;

	cout << "P0->P1: " << (geo1-geo0) << endl;
	cout << "(p1.h-p0.h)*ENU_Z: " << (pt1.height-pt0.height)*mrpt::poses::CPose3D(pose_ENU).getRotationMatrix().block<3,1>(0,2).transpose() << endl;

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

	mrpt::math::TPoint3D   p;
	mrpt::topography::geodeticToGeocentric_WGS84(
		mrpt::topography::TGeodeticCoords(lon0,lat0,h0),
		p);

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
