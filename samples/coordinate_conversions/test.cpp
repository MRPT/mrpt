/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/os.h>
#include <mrpt/topography.h>
#include <mrpt/math/CMatrixD.h>
#include <iomanip>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::topography;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>

std::vector<string>			names;
std::vector<CMatrixD>		results;
const double				TH = 0.02;

void exampleResults()
{
	names.resize( 6 );	names[0] = "MAND";	names[1] = "OROZ";	names[2] = "SANC";	names[3] = "ARJO";	names[4] = "CANI";	names[5] = "HALC";
	results.resize( 25 );

	// Example 01
	results[0] = CMatrixD(6,3);
	results[0].set_unsafe(0,0,435699.5503);		results[0].set_unsafe(0,1,4194142.0571);	results[0].set_unsafe(0,2,515.5790);
	results[0].set_unsafe(1,0,443948.2295);		results[0].set_unsafe(1,1,4160754.7114);	results[0].set_unsafe(1,2,1489.1880);
	results[0].set_unsafe(2,0,436542.0710);		results[0].set_unsafe(2,1,4177367.6694);	results[0].set_unsafe(2,2,1054.1890);
	results[0].set_unsafe(3,0,406284.9288);		results[0].set_unsafe(3,1,4176492.3033);	results[0].set_unsafe(3,2,569.1820);
	results[0].set_unsafe(4,0,406546.3268);		results[0].set_unsafe(4,1,4156628.3641);	results[0].set_unsafe(4,2,897.6260);
	results[0].set_unsafe(5,0,408051.6599);		results[0].set_unsafe(5,1,4193733.3066);	results[0].set_unsafe(5,2,530.4460);

	// Example 02
	results[1] = CMatrixD(6,3);
	results[1].set_unsafe(0,0,435810.797);		results[1].set_unsafe(0,1,4194348.753);		results[1].set_unsafe(0,2,437.516);
	results[1].set_unsafe(1,0,444059.747);		results[1].set_unsafe(1,1,4160961.358);		results[1].set_unsafe(1,2,1410.518);
	results[1].set_unsafe(2,0,436653.444);		results[1].set_unsafe(2,1,4177574.314);		results[1].set_unsafe(2,2,975.764);
	results[1].set_unsafe(3,0,406396.235);		results[1].set_unsafe(3,1,4176698.718);		results[1].set_unsafe(3,2,490.219);
	results[1].set_unsafe(4,0,406657.783);		results[1].set_unsafe(4,1,4156834.717);		results[1].set_unsafe(4,2,818.224);
	results[1].set_unsafe(5,0,408162.836);		results[1].set_unsafe(5,1,4193939.783);		results[1].set_unsafe(5,2,451.896);

	// Example 03
	results[2] = CMatrixD(6,3);
	results[2].set_unsafe(0,0,435811.252);		results[2].set_unsafe(0,1,4194347.983);		results[2].set_unsafe(0,2,515.579);
	results[2].set_unsafe(1,0,444059.929);		results[2].set_unsafe(1,1,4160960.606);		results[2].set_unsafe(1,2,1489.188);
	results[2].set_unsafe(2,0,436653.768);		results[2].set_unsafe(2,1,4177573.581);		results[2].set_unsafe(2,2,1054.189);
	results[2].set_unsafe(3,0,406396.600);		results[2].set_unsafe(3,1,4176698.222);		results[2].set_unsafe(3,2,569.182);
	results[2].set_unsafe(4,0,406657.992);		results[2].set_unsafe(4,1,4156834.266);		results[2].set_unsafe(4,2,897.626);
	results[2].set_unsafe(5,0,408163.337);		results[2].set_unsafe(5,1,4193939.241);		results[2].set_unsafe(5,2,530.446);

	// Example 04
	results[3] = CMatrixD(6,3);
	results[3].set_unsafe(0,0,435811.073);		results[3].set_unsafe(0,1,4194347.990);		results[3].set_unsafe(0,2,466.175);
	results[3].set_unsafe(1,0,444059.962);		results[3].set_unsafe(1,1,4160960.412);		results[3].set_unsafe(1,2,1439.784);
	results[3].set_unsafe(2,0,436653.708);		results[3].set_unsafe(2,1,4177573.506);		results[3].set_unsafe(2,2,1004.785);
	results[3].set_unsafe(3,0,406396.665);		results[3].set_unsafe(3,1,4176698.318);		results[3].set_unsafe(3,2,519.778);
	results[3].set_unsafe(4,0,406658.202);		results[3].set_unsafe(4,1,4156834.269);		results[3].set_unsafe(4,2,848.222);
	results[3].set_unsafe(5,0,408163.269);		results[3].set_unsafe(5,1,4193939.405);		results[3].set_unsafe(5,2,481.042);

	// Example 05
	results[4] = CMatrixD(6,3);
	results[4].set_unsafe(0,0,435811.251);		results[4].set_unsafe(0,1,4194347.974);		results[4].set_unsafe(0,2,465.655);
	results[4].set_unsafe(1,0,444059.934);		results[4].set_unsafe(1,1,4160960.612);		results[4].set_unsafe(1,2,1439.740);
	results[4].set_unsafe(2,0,436653.771);		results[4].set_unsafe(2,1,4177573.582);		results[4].set_unsafe(2,2,1004.556);
	results[4].set_unsafe(3,0,406396.596);		results[4].set_unsafe(3,1,4176698.222);		results[4].set_unsafe(3,2,520.037);
	results[4].set_unsafe(4,0,406657.988);		results[4].set_unsafe(4,1,4156834.272);		results[4].set_unsafe(4,2,848.836);
	results[4].set_unsafe(5,0,408163.337);		results[4].set_unsafe(5,1,4193939.239);		results[4].set_unsafe(5,2,480.961);

	// Example 06
	results[5] = CMatrixD(6,3);
	results[5].set_unsafe(0,0,435811.250);		results[5].set_unsafe(0,1,4194347.994);		results[5].set_unsafe(0,2,437.516);
	results[5].set_unsafe(1,0,444059.921);		results[5].set_unsafe(1,1,4160960.599);		results[5].set_unsafe(1,2,1410.518);
	results[5].set_unsafe(2,0,436653.764);		results[5].set_unsafe(2,1,4177573.581);		results[5].set_unsafe(2,2,975.764);
	results[5].set_unsafe(3,0,406396.606);		results[5].set_unsafe(3,1,4176698.225);		results[5].set_unsafe(3,2,490.219);
	results[5].set_unsafe(4,0,406657.997);		results[5].set_unsafe(4,1,4156834.260);		results[5].set_unsafe(4,2,818.224);
	results[5].set_unsafe(5,0,408163.340);		results[5].set_unsafe(5,1,4193939.242);		results[5].set_unsafe(5,2,451.896);

	// Example 07
	results[6] = CMatrixD(6,3);
	results[6].set_unsafe(0,0,435811.074);		results[6].set_unsafe(0,1,4194347.994);		results[6].set_unsafe(0,2,466.791);
	results[6].set_unsafe(1,0,444059.961);		results[6].set_unsafe(1,1,4160960.410);		results[6].set_unsafe(1,2,1439.793);
	results[6].set_unsafe(2,0,436653.708);		results[6].set_unsafe(2,1,4177573.505);		results[6].set_unsafe(2,2,1005.039);
	results[6].set_unsafe(3,0,406396.667);		results[6].set_unsafe(3,1,4176698.320);		results[6].set_unsafe(3,2,519.494);
	results[6].set_unsafe(4,0,406658.203);		results[6].set_unsafe(4,1,4156834.270);		results[6].set_unsafe(4,2,847.499);
	results[6].set_unsafe(5,0,408163.267);		results[6].set_unsafe(5,1,4193939.401);		results[6].set_unsafe(5,2,481.171);

	// Example 08
	results[7] = CMatrixD(6,3);
	results[7].set_unsafe(0,0,435811.250);		results[7].set_unsafe(0,1,4194347.974);		results[7].set_unsafe(0,2,465.657);
	results[7].set_unsafe(1,0,444059.935);		results[7].set_unsafe(1,1,4160960.612);		results[7].set_unsafe(1,2,1439.739);
	results[7].set_unsafe(2,0,436653.772);		results[7].set_unsafe(2,1,4177573.582);		results[7].set_unsafe(2,2,1004.556);
	results[7].set_unsafe(3,0,406396.597);		results[7].set_unsafe(3,1,4176698.223);		results[7].set_unsafe(3,2,520.037);
	results[7].set_unsafe(4,0,406657.988);		results[7].set_unsafe(4,1,4156834.271);		results[7].set_unsafe(4,2,848.837);
	results[7].set_unsafe(5,0,408163.339);		results[7].set_unsafe(5,1,4193939.239);		results[7].set_unsafe(5,2,480.959);

	// Example 10
	results[9] = CMatrixD(6,3);
	results[9].set_unsafe(0,0,435811.2512);		results[9].set_unsafe(0,1,4194347.9756);		results[9].set_unsafe(0,2,465.6558);
	results[9].set_unsafe(1,0,444059.9337);		results[9].set_unsafe(1,1,4160960.6109);		results[9].set_unsafe(1,2,1439.7390);
	results[9].set_unsafe(2,0,436653.7721);		results[9].set_unsafe(2,1,4177573.5812);		results[9].set_unsafe(2,2,1004.5560);
	results[9].set_unsafe(3,0,406396.5970);		results[9].set_unsafe(3,1,4176698.2216);		results[9].set_unsafe(3,2,520.0376);
	results[9].set_unsafe(4,0,406657.9894);		results[9].set_unsafe(4,1,4156834.2734);		results[9].set_unsafe(4,2,848.8373);
	results[9].set_unsafe(5,0,408163.3366);		results[9].set_unsafe(5,1,4193939.2373);		results[9].set_unsafe(5,2,480.9593);

	// Example 11
	results[10] = CMatrixD(6,3);
	results[10].set_unsafe(0,0,435811.2512);		results[10].set_unsafe(0,1,4194347.9756);		results[10].set_unsafe(0,2,465.6558);
	results[10].set_unsafe(1,0,444059.9337);		results[10].set_unsafe(1,1,4160960.6109);		results[10].set_unsafe(1,2,1439.7390);
	results[10].set_unsafe(2,0,436653.7721);		results[10].set_unsafe(2,1,4177573.5812);		results[10].set_unsafe(2,2,1004.5560);
	results[10].set_unsafe(3,0,406396.5970);		results[10].set_unsafe(3,1,4176698.2216);		results[10].set_unsafe(3,2,520.0376);
	results[10].set_unsafe(4,0,406657.9894);		results[10].set_unsafe(4,1,4156834.2734);		results[10].set_unsafe(4,2,848.8373);
	results[10].set_unsafe(5,0,408163.3366);		results[10].set_unsafe(5,1,4193939.2373);		results[10].set_unsafe(5,2,480.9593);

	// Example 12
	results[11] = CMatrixD(6,3);
	results[11].set_unsafe(0,0,435811.156);		results[11].set_unsafe(0,1,4194347.892);		results[11].set_unsafe(0,2,466.100);
	results[11].set_unsafe(1,0,444059.719);		results[11].set_unsafe(1,1,4160960.489);		results[11].set_unsafe(1,2,1439.389);
	results[11].set_unsafe(2,0,436653.606);		results[11].set_unsafe(2,1,4177573.398);		results[11].set_unsafe(2,2,1004.594);
	results[11].set_unsafe(3,0,406396.045);		results[11].set_unsafe(3,1,4176697.926);		results[11].set_unsafe(3,2,520.238);
	results[11].set_unsafe(4,0,406658.812);		results[11].set_unsafe(4,1,4156834.183);		results[11].set_unsafe(4,2,849.050);
	results[11].set_unsafe(5,0,408163.542);		results[11].set_unsafe(5,1,4193940.012);		results[11].set_unsafe(5,2,480.414);

	// Example 13
	results[12] = CMatrixD(6,3);
	results[12].set_unsafe(0,0,435811.095);		results[12].set_unsafe(0,1,4194348.146);		results[12].set_unsafe(0,2,465.884);
	results[12].set_unsafe(1,0,444059.906);		results[12].set_unsafe(1,1,4160960.276);		results[12].set_unsafe(1,2,1439.564);
	results[12].set_unsafe(2,0,436653.748);		results[12].set_unsafe(2,1,4177573.521);		results[12].set_unsafe(2,2,1004.572);
	results[12].set_unsafe(3,0,406396.375);		results[12].set_unsafe(3,1,4176698.033);		results[12].set_unsafe(3,2,520.147);
	results[12].set_unsafe(4,0,406658.370);		results[12].set_unsafe(4,1,4156834.517);		results[12].set_unsafe(4,2,848.945);
	results[12].set_unsafe(5,0,408163.386);		results[12].set_unsafe(5,1,4193939.406);		results[12].set_unsafe(5,2,480.674);
}


void displayResults( const CVectorDouble &thisResults, const unsigned int &example, const bool color )
{
	cout << "Example " << example << ". Results" << setprecision(16) << endl;
	cout << "----------------------------------------------------------------------------" << endl;
	cout << "N\t" << "X\t\t\t" << "Y\t\t\t" << "Z" << endl;
	cout << "----------------------------------------------------------------------------" << endl;

	if( color )
	{
		const size_t N = thisResults.size()/3;
		for( unsigned int i = 0; i < N; ++i )
		{
			cout << names[i] << "\t";
			if( fabs( thisResults[3*i+0]-results[example-1].get_unsafe(i,0)) < TH )
				mrpt::system::setConsoleColor(CONCOL_GREEN);
			else
				mrpt::system::setConsoleColor(CONCOL_RED);
			cout << thisResults[3*i+0] << "\t";

			if( fabs( thisResults[3*i+1]-results[example-1].get_unsafe(i,1)) < TH )
				mrpt::system::setConsoleColor(CONCOL_GREEN);
			else
				mrpt::system::setConsoleColor(CONCOL_RED);
			cout << thisResults[3*i+1] << "\t";

			if( fabs( thisResults[3*i+2]-results[example-1].get_unsafe(i,2)) < TH )
				mrpt::system::setConsoleColor(CONCOL_GREEN);
			else
				mrpt::system::setConsoleColor(CONCOL_RED);
			cout << thisResults[3*i+2] << endl;
			mrpt::system::setConsoleColor(CONCOL_NORMAL);
		}
		cout << endl;
	}
	else
	{
		const size_t N = thisResults.size()/3;
		for( unsigned int i = 0; i < N; ++i )
			cout << names[i] << "\t" << thisResults[3*i+0] << "\t" << thisResults[3*i+1] << "\t" << thisResults[3*i+2] << endl;
		cout << endl;
	}
}

void TestCoordinatesConversions()
{
	// Initial UTM coordinates (X,Y,Z)
	mrpt::math::TPoint3D UTMCoords;
	UTMCoords.x = 435500;
	UTMCoords.y = 4194142.057;
	UTMCoords.z = 515.579;

	const int huso = 30;
	const char hem = 'n';

	TGeodeticCoords		GeodeticCoords;
	TEllipsoid			ellip = TEllipsoid::Ellipsoid_WGS84();

	// Dump to console of input values
	cout << "INPUT DATA: " << endl << "----------------------------------" << endl;
	cout << "UTM Coordinates: " << endl;
	cout << "X = " << setprecision(15) << UTMCoords.x << endl;
	cout << "Y = " << UTMCoords.y << endl;
	cout << "Z = " << UTMCoords.z << endl;
	cout << "Time zone: " << huso << endl;
	cout << "Hemisphery: " << hem << endl;
	cout << "Ellipsoid: " << ellip.name << endl;
	cout << "----------------------------------" << endl << endl;

	// UTM to Geodetic ...
	UTMToGeodetic( UTMCoords, huso, hem, GeodeticCoords, ellip );
	cout << "UTM to Geodetic" << endl;
	cout << "Geodetic Coordinates:" << endl;
	cout << "Lon = " << TCoords( GeodeticCoords.lon ) << " [" << GeodeticCoords.lon << "]" << endl;
	cout << "Lat = " << TCoords( GeodeticCoords.lat ) << " [" << GeodeticCoords.lon << "]" << endl;
	cout << "H = " << GeodeticCoords.height << endl;
	cout << "----------------------------------" << endl;

	// Geodetic to Geocentric ...
	TGeocentricCoords GeocentricCoords;
	geodeticToGeocentric( GeodeticCoords, GeocentricCoords, ellip );
	cout << "Geodetic to Geocentric" << endl;
	cout << "Geocentric Coordinates:" << endl;
	cout << "X = " << GeocentricCoords.x << endl;
	cout << "Y = " << GeocentricCoords.y << endl;
	cout << "Z = " << GeocentricCoords.z << endl;
	cout << "----------------------------------" << endl;

	// 7 parameter transformation (X,Y,Z) -> (X',Y',Z')
	// TDatum7Params datum( dx, dy, dz, rx(sec), ry(sec), rz(sec), ds(ppm) );
	TDatum7Params		datum( 109.4714, 106.7407, 141.7916, 4.7834, 7.9668, -5.3771, -6.690000 );
	TGeocentricCoords	GeocentricCoords2;
	transform7params( GeocentricCoords, datum, GeocentricCoords2 );
	//transform7params( TGeocentricCoords(5029475.945,-328201.0396,3896351.728), datum, GeocentricCoords2 );
	cout << "transform7params" << endl;
	cout << "Transformed Geocentric Coordinates:" << endl;
	cout << "X = " << GeocentricCoords2.x << endl;
	cout << "Y = " << GeocentricCoords2.y << endl;
	cout << "Z = " << GeocentricCoords2.z << endl;
	cout << "----------------------------------" << endl;

	// ... and back to Geodetic
	TGeodeticCoords		GeodeticCoords2;
	ellip = TEllipsoid::Ellipsoid_Hough_1960();
	geocentricToGeodetic( GeocentricCoords2, GeodeticCoords2, ellip );
	cout << "Geocentric to Geodetic" << endl;
	cout << "Geodetic Coordinates:" << endl;
	cout << "Lon = " << TCoords( GeodeticCoords2.lon ) << " [" << GeodeticCoords2.lon << "]" << endl;
	cout << "Lat = " << TCoords( GeodeticCoords2.lat ) << " [" << GeodeticCoords2.lat << "]" << endl;
	cout << "H = " << GeodeticCoords2.height << endl;
	cout << "----------------------------------" << endl;

	// ... and back to UTM
	TUTMCoords	UTMCoords2;
	int			time_zone2;
	char		latitude_band2;
	//geodeticToUTM( GeodeticCoords2, UTMCoords2, time_zone2, latitude_band2, ellip );
	geodeticToUTM( TGeodeticCoords(37.89604181,-3.72987289,542.8624741), UTMCoords2, time_zone2, latitude_band2, TEllipsoid(6378270,6356794.343,"USER") );
	cout << "Geodetic to UTM" << endl;
	cout << "UTM Coordinates:" << endl;
	cout << "X = " << UTMCoords2.x << endl;
	cout << "Y = " << UTMCoords2.y << endl;
	cout << "Z = " << UTMCoords2.z << endl;
	cout << "Time zone: " << time_zone2 << endl;
	cout << "----------------------------------" << endl;


	// 10 parameter transformation (X,Y,Z) -> (X',Y',Z')

	// Helmert 2D transformation (X,Y) -> (X',Y')

	// Helmert 3D transformation (X,Y,Z) -> (X',Y',Z')

	// 1D transformation (X,Y,Z) -> (X')

	// Interpolation (X,Y) -> (X',Y')

}

void Examples_01()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	// A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, TEllipsoid::Ellipsoid_WGS84() );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 1, true );
}

void Examples_02()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Internacional_1924();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, TEllipsoid::Ellipsoid_WGS84() );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, TEllipsoid::Ellipsoid_WGS84() );

	//A5
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum7Params	datum( 131.0320, 100.2510, 163.3540, -1.2438, -0.0195, -1.1436, -9.3900 );
	transform7params( MAND_Geoc, datum, MAND_Geoc2 );
	transform7params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform7params( SANC_Geoc, datum, SANC_Geoc2 );
	transform7params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform7params( CANI_Geoc, datum, CANI_Geoc2 );
	transform7params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 2, true );
}

void Examples_03()
{
	// TOPCON
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_WGS84();

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, ellip );

	//A7
	TPoint2D	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumHelmert2D_TOPCON	datum( 1.0000008551, 0.0000002900, 110.1121641064, 202.4657352579 );
	transformHelmert2D_TOPCON( TPoint2D( MAND_UTM.x, MAND_UTM.y ), datum, MAND_point );
	transformHelmert2D_TOPCON( TPoint2D( OROZ_UTM.x, OROZ_UTM.y ), datum, OROZ_point );
	transformHelmert2D_TOPCON( TPoint2D( SANC_UTM.x, SANC_UTM.y ), datum, SANC_point );
	transformHelmert2D_TOPCON( TPoint2D( ARJO_UTM.x, ARJO_UTM.y ), datum, ARJO_point );
	transformHelmert2D_TOPCON( TPoint2D( CANI_UTM.x, CANI_UTM.y ), datum, CANI_point );
	transformHelmert2D_TOPCON( TPoint2D( HALC_UTM.x, HALC_UTM.y ), datum, HALC_point );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_point.x;	thisResults[1]	= MAND_point.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_point.x;	thisResults[4]	= OROZ_point.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_point.x;	thisResults[7]	= SANC_point.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_point.x;	thisResults[10] = ARJO_point.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_point.x;	thisResults[13] = CANI_point.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_point.x;	thisResults[16] = HALC_point.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 3, true );
}

void Examples_04()
{
	// TOPCON
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_WGS84();

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, ellip );

	//A7
	TUTMCoords	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumHelmert3D_TOPCON	datum( 0.9999969509, -0.0000070551, 142.4402790576, -0.0000060616, 1.0000054540, 185.6991268722, -49.4041666667 );
	transformHelmert3D_TOPCON( MAND_UTM, datum, MAND_point );
	transformHelmert3D_TOPCON( OROZ_UTM, datum, OROZ_point );
	transformHelmert3D_TOPCON( SANC_UTM, datum, SANC_point );
	transformHelmert3D_TOPCON( ARJO_UTM, datum, ARJO_point );
	transformHelmert3D_TOPCON( CANI_UTM, datum, CANI_point );
	transformHelmert3D_TOPCON( HALC_UTM, datum, HALC_point );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_point.x;	thisResults[1]	= MAND_point.y;	thisResults[2]	= MAND_point.z;
	thisResults[3]	= OROZ_point.x;	thisResults[4]	= OROZ_point.y;	thisResults[5]	= OROZ_point.z;
	thisResults[6]	= SANC_point.x;	thisResults[7]	= SANC_point.y;	thisResults[8]	= SANC_point.z;
	thisResults[9]	= ARJO_point.x;	thisResults[10] = ARJO_point.y;	thisResults[11] = ARJO_point.z;
	thisResults[12] = CANI_point.x;	thisResults[13] = CANI_point.y;	thisResults[14] = CANI_point.z;
	thisResults[15] = HALC_point.x;	thisResults[16] = HALC_point.y;	thisResults[17] = HALC_point.z;

	displayResults( thisResults, 4, true );

}

void Examples_05()
{
	// TOPCON
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid( 6378137, 6356752.31424518, "USER" );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, ellip );

	//A5
	TUTMCoords	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatum7Params_TOPCON	datum(
		109.035107801496, 202.610276063935, 32.727057571807,
		0.9999999999, 0.0000005460, 0.0000156068,
		-0.0000005463, 0.9999999998, 0.0000180851,
		-0.0000156068, -0.0000180851, 0.9999999997,
		0.842993545636 );
	transform7params_TOPCON( MAND_UTM, datum, MAND_point );
	transform7params_TOPCON( OROZ_UTM, datum, OROZ_point );
	transform7params_TOPCON( SANC_UTM, datum, SANC_point );
	transform7params_TOPCON( ARJO_UTM, datum, ARJO_point );
	transform7params_TOPCON( CANI_UTM, datum, CANI_point );
	transform7params_TOPCON( HALC_UTM, datum, HALC_point );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_point.x;	thisResults[1]	= MAND_point.y;	thisResults[2]	= MAND_point.z;
	thisResults[3]	= OROZ_point.x;	thisResults[4]	= OROZ_point.y;	thisResults[5]	= OROZ_point.z;
	thisResults[6]	= SANC_point.x;	thisResults[7]	= SANC_point.y;	thisResults[8]	= SANC_point.z;
	thisResults[9]	= ARJO_point.x;	thisResults[10] = ARJO_point.y;	thisResults[11] = ARJO_point.z;
	thisResults[12] = CANI_point.x;	thisResults[13] = CANI_point.y;	thisResults[14] = CANI_point.z;
	thisResults[15] = HALC_point.x;	thisResults[16] = HALC_point.y;	thisResults[17] = HALC_point.z;

	displayResults( thisResults, 5, true );
}

void Examples_06()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Internacional_1924();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, ellip );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, ellip );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, ellip );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, ellip );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, ellip );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, ellip );

	//A5
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum7Params	datum( 131.0320, 100.2510, 163.3540, -1.2438, -0.0195, -1.1436, -9.3900 );
	transform7params( MAND_Geoc, datum, MAND_Geoc2 );
	transform7params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform7params( SANC_Geoc, datum, SANC_Geoc2 );
	transform7params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform7params( CANI_Geoc, datum, CANI_Geoc2 );
	transform7params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 6, false);

	//A7
	TPoint2D	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumHelmert2D_TOPCON	dat( 0.9999980621, 0.0000078756, -31.7350304827, 10.8014994871 );
	transformHelmert2D_TOPCON( TPoint2D( MAND_UTM.x, MAND_UTM.y ), dat, MAND_point );
	transformHelmert2D_TOPCON( TPoint2D( OROZ_UTM.x, OROZ_UTM.y ), dat, OROZ_point );
	transformHelmert2D_TOPCON( TPoint2D( SANC_UTM.x, SANC_UTM.y ), dat, SANC_point );
	transformHelmert2D_TOPCON( TPoint2D( ARJO_UTM.x, ARJO_UTM.y ), dat, ARJO_point );
	transformHelmert2D_TOPCON( TPoint2D( CANI_UTM.x, CANI_UTM.y ), dat, CANI_point );
	transformHelmert2D_TOPCON( TPoint2D( HALC_UTM.x, HALC_UTM.y ), dat, HALC_point );

	thisResults[0]	= MAND_point.x;	thisResults[1]	= MAND_point.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_point.x;	thisResults[4]	= OROZ_point.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_point.x;	thisResults[7]	= SANC_point.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_point.x;	thisResults[10] = ARJO_point.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_point.x;	thisResults[13] = CANI_point.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_point.x;	thisResults[16] = HALC_point.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 6, true );
}

void Examples_07()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Internacional_1924();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, ellip );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, ellip );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, ellip );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, ellip );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, ellip );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, ellip );

	//A5
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum7Params	datum( 131.0320, 100.2510, 163.3540, -1.2438, -0.0195, -1.1436, -9.3900 );
	transform7params( MAND_Geoc, datum, MAND_Geoc2 );
	transform7params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform7params( SANC_Geoc, datum, SANC_Geoc2 );
	transform7params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform7params( CANI_Geoc, datum, CANI_Geoc2 );
	transform7params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 7, false);

	//A8
	TPoint3D	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumHelmert3D_TOPCON	dat( 0.9999944446, 0.0000005236, 0.5022460081, -0.0000136628, 1.0000023039, -4.4675776973, 29.2747378118 );
	transformHelmert3D_TOPCON( MAND_UTM, dat, MAND_point );
	transformHelmert3D_TOPCON( OROZ_UTM, dat, OROZ_point );
	transformHelmert3D_TOPCON( SANC_UTM, dat, SANC_point );
	transformHelmert3D_TOPCON( ARJO_UTM, dat, ARJO_point );
	transformHelmert3D_TOPCON( CANI_UTM, dat, CANI_point );
	transformHelmert3D_TOPCON( HALC_UTM, dat, HALC_point );

	thisResults[0]	= MAND_point.x;	thisResults[1]	= MAND_point.y;	thisResults[2]	= MAND_point.z;
	thisResults[3]	= OROZ_point.x;	thisResults[4]	= OROZ_point.y;	thisResults[5]	= OROZ_point.z;
	thisResults[6]	= SANC_point.x;	thisResults[7]	= SANC_point.y;	thisResults[8]	= SANC_point.z;
	thisResults[9]	= ARJO_point.x;	thisResults[10] = ARJO_point.y;	thisResults[11] = ARJO_point.z;
	thisResults[12] = CANI_point.x;	thisResults[13] = CANI_point.y;	thisResults[14] = CANI_point.z;
	thisResults[15] = HALC_point.x;	thisResults[16] = HALC_point.y;	thisResults[17] = HALC_point.z;

	displayResults( thisResults, 7, true );
}

void Examples_08()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Internacional_1924();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, ellip );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, ellip );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, ellip );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, ellip );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, ellip );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, ellip );

	//A5
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum7Params	datum( 131.0320, 100.2510, 163.3540, -1.2438, -0.0195, -1.1436, -9.3900 );
	transform7params( MAND_Geoc, datum, MAND_Geoc2 );
	transform7params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform7params( SANC_Geoc, datum, SANC_Geoc2 );
	transform7params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform7params( CANI_Geoc, datum, CANI_Geoc2 );
	transform7params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 8, false);

	//A5
	TGeocentricCoords	MAND_Geoc3, OROZ_Geoc3, SANC_Geoc3, ARJO_Geoc3, CANI_Geoc3, HALC_Geoc3;
	//TDatum7Params	dat( -34.095438458048, 10.843234125336, 212.169692142148, -8.348308019638, 6.751893142065, -1.738975581025, -1.898114919019 );
	TDatum7Params_TOPCON	dat( -34.095438458048, 10.843234125336, 212.169692142148,
		0.9999999994, 0.0000084308, 0.0000327341,
		-0.0000084321, 0.9999999991, 0.0000404737,
		-0.0000327338, -0.0000404737, 0.9999999986,
		-1.898114919019 );
	transform7params_TOPCON( MAND_UTM, dat, MAND_Geoc3 );
	transform7params_TOPCON( OROZ_UTM, dat, OROZ_Geoc3 );
	transform7params_TOPCON( SANC_UTM, dat, SANC_Geoc3 );
	transform7params_TOPCON( ARJO_UTM, dat, ARJO_Geoc3 );
	transform7params_TOPCON( CANI_UTM, dat, CANI_Geoc3 );
	transform7params_TOPCON( HALC_UTM, dat, HALC_Geoc3 );

	thisResults[0]	= MAND_Geoc3.x;	thisResults[1]	= MAND_Geoc3.y;	thisResults[2]	= MAND_Geoc3.z;
	thisResults[3]	= OROZ_Geoc3.x;	thisResults[4]	= OROZ_Geoc3.y;	thisResults[5]	= OROZ_Geoc3.z;
	thisResults[6]	= SANC_Geoc3.x;	thisResults[7]	= SANC_Geoc3.y;	thisResults[8]	= SANC_Geoc3.z;
	thisResults[9]	= ARJO_Geoc3.x;	thisResults[10] = ARJO_Geoc3.y;	thisResults[11] = ARJO_Geoc3.z;
	thisResults[12] = CANI_Geoc3.x;	thisResults[13] = CANI_Geoc3.y;	thisResults[14] = CANI_Geoc3.z;
	thisResults[15] = HALC_Geoc3.x;	thisResults[16] = HALC_Geoc3.y;	thisResults[17] = HALC_Geoc3.z;

	displayResults( thisResults, 8, true );
}

void Examples_10()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Hayford_1909();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, ellip );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, ellip );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, ellip );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, ellip );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, ellip );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, ellip );

	//A5
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum7Params	datum( 284.2535, -116.9549, -34.8027, 4.78337, 7.96684, -5.37707, -6.6900 );
	transform7params( MAND_Geoc, datum, MAND_Geoc2 );
	transform7params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform7params( SANC_Geoc, datum, SANC_Geoc2 );
	transform7params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform7params( CANI_Geoc, datum, CANI_Geoc2 );
	transform7params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 10, true);

}

void Examples_11()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Hayford_1909();

	// A3
	TGeocentricCoords	MAND_Geoc, OROZ_Geoc, SANC_Geoc, ARJO_Geoc, CANI_Geoc, HALC_Geoc;
	geodeticToGeocentric( MAND_92633, MAND_Geoc, ellip );
	geodeticToGeocentric( OROZ_96965, OROZ_Geoc, ellip );
	geodeticToGeocentric( SANC_94744, SANC_Geoc, ellip );
	geodeticToGeocentric( ARJO_94633, ARJO_Geoc, ellip );
	geodeticToGeocentric( CANI_96833, CANI_Geoc, ellip );
	geodeticToGeocentric( HALC_92543, HALC_Geoc, ellip );

	//A6
	TGeocentricCoords	MAND_Geoc2, OROZ_Geoc2, SANC_Geoc2, ARJO_Geoc2, CANI_Geoc2, HALC_Geoc2;
	TDatum10Params	datum( 109.4714, 106.7407, 141.7916, 5039726.4242, -341417.6071, 3882515.5524, 4.78337, 7.96684, -5.37707, -6.6900 );
	transform10params( MAND_Geoc, datum, MAND_Geoc2 );
	transform10params( OROZ_Geoc, datum, OROZ_Geoc2 );
	transform10params( SANC_Geoc, datum, SANC_Geoc2 );
	transform10params( ARJO_Geoc, datum, ARJO_Geoc2 );
	transform10params( CANI_Geoc, datum, CANI_Geoc2 );
	transform10params( HALC_Geoc, datum, HALC_Geoc2 );

	//A4
	TGeodeticCoords		MAND_Geod, OROZ_Geod, SANC_Geod, ARJO_Geod, CANI_Geod, HALC_Geod;
	geocentricToGeodetic( MAND_Geoc2, MAND_Geod, ellip );
	geocentricToGeodetic( OROZ_Geoc2, OROZ_Geod, ellip );
	geocentricToGeodetic( SANC_Geoc2, SANC_Geod, ellip );
	geocentricToGeodetic( ARJO_Geoc2, ARJO_Geod, ellip );
	geocentricToGeodetic( CANI_Geoc2, CANI_Geod, ellip );
	geocentricToGeodetic( HALC_Geoc2, HALC_Geod, ellip );

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_Geod, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_Geod, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_Geod, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_Geod, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_Geod, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_Geod, HALC_UTM, UTMZone, UTMBand, ellip );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_UTM.x;	thisResults[1]	= MAND_UTM.y;	thisResults[2]	= MAND_UTM.z;
	thisResults[3]	= OROZ_UTM.x;	thisResults[4]	= OROZ_UTM.y;	thisResults[5]	= OROZ_UTM.z;
	thisResults[6]	= SANC_UTM.x;	thisResults[7]	= SANC_UTM.y;	thisResults[8]	= SANC_UTM.z;
	thisResults[9]	= ARJO_UTM.x;	thisResults[10] = ARJO_UTM.y;	thisResults[11] = ARJO_UTM.z;
	thisResults[12] = CANI_UTM.x;	thisResults[13] = CANI_UTM.y;	thisResults[14] = CANI_UTM.z;
	thisResults[15] = HALC_UTM.x;	thisResults[16] = HALC_UTM.y;	thisResults[17] = HALC_UTM.z;

	displayResults( thisResults, 11, true);

}

void Examples_12()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Hayford_1909();

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, ellip );

	//A7
	TPoint2D	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumHelmert2D	dat( 15.8778, 0.4491, TCoords(0,32,8.75881).getDecimalValue() /*degrees*/, -325.9604, 422956.6976, 4176709.7722 );
	transformHelmert2D( TPoint2D( MAND_UTM.x, MAND_UTM.y ), dat, MAND_point );
	transformHelmert2D( TPoint2D( OROZ_UTM.x, OROZ_UTM.y ), dat, OROZ_point );
	transformHelmert2D( TPoint2D( SANC_UTM.x, SANC_UTM.y ), dat, SANC_point );
	transformHelmert2D( TPoint2D( ARJO_UTM.x, ARJO_UTM.y ), dat, ARJO_point );
	transformHelmert2D( TPoint2D( CANI_UTM.x, CANI_UTM.y ), dat, CANI_point );
	transformHelmert2D( TPoint2D( HALC_UTM.x, HALC_UTM.y ), dat, HALC_point );

	//A9
	TPoint3D	MAND_point2, OROZ_point2, SANC_point2, ARJO_point2, CANI_point2, HALC_point2;
	TDatum1DTransf	datum( -0.00001847 /*rad*/, -0.00001536/*rad*/, -49.4041, 0 );
	transform1D( TPoint3D( MAND_point.x, MAND_point.y, MAND_UTM.z ), datum, MAND_point2 );
	transform1D( TPoint3D( OROZ_point.x, OROZ_point.y, OROZ_UTM.z ), datum, OROZ_point2 );
	transform1D( TPoint3D( SANC_point.x, SANC_point.y, SANC_UTM.z ), datum, SANC_point2 );
	transform1D( TPoint3D( ARJO_point.x, ARJO_point.y, ARJO_UTM.z ), datum, ARJO_point2 );
	transform1D( TPoint3D( CANI_point.x, CANI_point.y, CANI_UTM.z ), datum, CANI_point2 );
	transform1D( TPoint3D( HALC_point.x, HALC_point.y, HALC_UTM.z ), datum, HALC_point2 );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_point2.x;	thisResults[1]	= MAND_point2.y;	thisResults[2]	= MAND_point2.z;
	thisResults[3]	= OROZ_point2.x;	thisResults[4]	= OROZ_point2.y;	thisResults[5]	= OROZ_point2.z;
	thisResults[6]	= SANC_point2.x;	thisResults[7]	= SANC_point2.y;	thisResults[8]	= SANC_point2.z;
	thisResults[9]	= ARJO_point2.x;	thisResults[10] = ARJO_point2.y;	thisResults[11] = ARJO_point2.z;
	thisResults[12] = CANI_point2.x;	thisResults[13] = CANI_point2.y;	thisResults[14] = CANI_point2.z;
	thisResults[15] = HALC_point2.x;	thisResults[16] = HALC_point2.y;	thisResults[17] = HALC_point2.z;

	displayResults( thisResults, 12, true);

}

void Examples_13()
{
	TGeodeticCoords MAND_92633( TCoords(37,53,33.072573).getDecimalValue(), TCoords(-3,43,52.68965).getDecimalValue(), 515.579 );
	TGeodeticCoords OROZ_96965( TCoords(37,35,31.75505).getDecimalValue(), TCoords(-3,38,5.70754).getDecimalValue(), 1489.188 );
	TGeodeticCoords SANC_94744( TCoords(37,44,29.04253).getDecimalValue(), TCoords(-3,43,12.90285).getDecimalValue(), 1054.189 );
	TGeodeticCoords ARJO_94633( TCoords(37,43,51.28918).getDecimalValue(), TCoords(-4,3,48.64503).getDecimalValue(), 569.182 );
	TGeodeticCoords CANI_96833( TCoords(37,33,6.94237).getDecimalValue(), TCoords(-4,3,28.81611).getDecimalValue(), 897.626 );
	TGeodeticCoords HALC_92543( TCoords(37,53,11.26838).getDecimalValue(), TCoords(-4,2,44.35794).getDecimalValue(), 530.446 );

	const TEllipsoid ellip = TEllipsoid::Ellipsoid_Hayford_1909();

	//A2
	int			UTMZone;
	char		UTMBand;
	TUTMCoords	MAND_UTM, OROZ_UTM, SANC_UTM, ARJO_UTM, CANI_UTM, HALC_UTM;
	geodeticToUTM( MAND_92633, MAND_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( OROZ_96965, OROZ_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( SANC_94744, SANC_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( ARJO_94633, ARJO_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( CANI_96833, CANI_UTM, UTMZone, UTMBand, ellip );
	geodeticToUTM( HALC_92543, HALC_UTM, UTMZone, UTMBand, ellip );

	//A10
	TPoint3D	MAND_point, OROZ_point, SANC_point, ARJO_point, CANI_point, HALC_point;
	TDatumTransfInterpolation dat( 423150.0514, 4197393.9437, -334.2393 /*ppm*/, -314.2785 /*ppm*/, 1.33958 /*secs*/ );
	transfInterpolation( MAND_UTM, dat, MAND_point );
	transfInterpolation( OROZ_UTM, dat, OROZ_point );
	transfInterpolation( SANC_UTM, dat, SANC_point );
	transfInterpolation( ARJO_UTM, dat, ARJO_point );
	transfInterpolation( CANI_UTM, dat, CANI_point );
	transfInterpolation( HALC_UTM, dat, HALC_point );

	//A9
	TPoint3D	MAND_point2, OROZ_point2, SANC_point2, ARJO_point2, CANI_point2, HALC_point2;
	TDatum1DTransf	datum( -0.00001822 /*rad*/, -0.00001304/*rad*/, -49.8094, 0 );
	transform1D( MAND_point, datum, MAND_point2 );
	transform1D( OROZ_point, datum, OROZ_point2 );
	transform1D( SANC_point, datum, SANC_point2 );
	transform1D( ARJO_point, datum, ARJO_point2 );
	transform1D( CANI_point, datum, CANI_point2 );
	transform1D( HALC_point, datum, HALC_point2 );

	CVectorDouble thisResults;
	thisResults.resize(18);

	thisResults[0]	= MAND_point2.x;	thisResults[1]	= MAND_point2.y;	thisResults[2]	= MAND_point2.z;
	thisResults[3]	= OROZ_point2.x;	thisResults[4]	= OROZ_point2.y;	thisResults[5]	= OROZ_point2.z;
	thisResults[6]	= SANC_point2.x;	thisResults[7]	= SANC_point2.y;	thisResults[8]	= SANC_point2.z;
	thisResults[9]	= ARJO_point2.x;	thisResults[10] = ARJO_point2.y;	thisResults[11] = ARJO_point2.z;
	thisResults[12] = CANI_point2.x;	thisResults[13] = CANI_point2.y;	thisResults[14] = CANI_point2.z;
	thisResults[15] = HALC_point2.x;	thisResults[16] = HALC_point2.y;	thisResults[17] = HALC_point2.z;

	displayResults( thisResults, 13, true);

}


int main(int argc, char **argv)
{
	try
	{
		exampleResults();
		TestCoordinatesConversions();
		mrpt::system::setConsoleColor(CONCOL_BLUE);
		cout << "**************************************" << endl;
		cout << "\tTOPCON with th = " << TH << endl;
		cout << "**************************************" << endl;
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		Examples_01();
		Examples_02();
		Examples_03();
		Examples_04();
		Examples_05();
		Examples_06();
		Examples_07();
		Examples_08();
		mrpt::system::setConsoleColor(CONCOL_BLUE);
		cout << "**************************************" << endl;
		cout << "\tLEICA with th = " << TH << endl;
		cout << "**************************************" << endl;
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		cout << "Example 09 missing in pdf" << endl;
		Examples_10();
		Examples_11();
		Examples_12();
		Examples_13();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
