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
#ifndef  conversions_H
#define  conversions_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/topography/link_pragmas.h>

using namespace std;
using namespace mrpt::utils;

namespace mrpt
{
	/** This namespace provides topography helper functions, coordinate transformations.
	 */
	namespace topography
	{

	/** =======================================================================
	   @name Data structures
	   @{ */

		struct TOPO_IMPEXP TCoords
		{
			inline TCoords( const int _deg, const int _min, const double _sec ) : degrees(_deg), minutes(_min), seconds(_sec) {}
			inline TCoords( const double dec ) { setFromDecimal(dec); }

			int degrees;
			int minutes;
			double seconds;

			inline void setFromDecimal( const double dec )
			{
				double aux = fabs(dec);
				degrees = (int)aux;
				minutes = (int)((aux - degrees)*60.0f);
				seconds = ((aux - degrees)*60.0f - minutes)*60.0f;
				if( dec<0 )
					degrees *= -1;
			}
			inline double getDecimalValue() const
			{
				double aux = abs(degrees)+minutes/60.0f+seconds/3600.0f;
				if( degrees < 0 )
					aux *= -1;
				return aux;
			}
			inline operator double(void) const { return getDecimalValue(); }
		};

		inline std::ostream& operator<<( std::ostream& out, const TCoords &o )
		{
			out << o.degrees << "deg " << o.minutes << "' " << o.seconds << "''";
			return out;
		}

			struct TOPO_IMPEXP TEllipsoid
		{
			inline TEllipsoid() : sa( 6378137.0 ), sb( 6356752.314245 ), name("WGS84") {}
			inline TEllipsoid( const double _sa, const double _sb, const string _name ) : sa(_sa), sb(_sb), name(_name) {}

			double sa;		//!< largest semiaxis of the reference ellipsoid (in meters)
			double sb;		//!< smallest semiaxis of the reference ellipsoid (in meters)
			string name;	//!< the ellipsoid name
		};

		inline TEllipsoid Ellipsoid_WGS84() {					return TEllipsoid( 6378137.000, 6356752.314245, "WGS84" ); }
		inline TEllipsoid Ellipsoid_WGS72() {					return TEllipsoid( 6378135.000, 6356750.519915, "WGS72"  ); }
		inline TEllipsoid Ellipsoid_WGS66() {					return TEllipsoid( 6378145.000, 6356759.769356, "WGS66"  ); }
		inline TEllipsoid Ellipsoid_Walbeck_1817() {			return TEllipsoid( 6376896.000, 6355834.846700, "Walbeck_1817"  ); }
		inline TEllipsoid Ellipsoid_Sudamericano_1969() {		return TEllipsoid( 6378160.000, 6356774.720000, "Sudamericano_1969"  ); }
		inline TEllipsoid Ellipsoid_Nuevo_Internacional_1967() {return TEllipsoid( 6378157.500, 6356772.200000, "Nuevo_Internacional_1967"  ); }
		inline TEllipsoid Ellipsoid_Mercury_Modificado_1968() {	return TEllipsoid( 6378150.000, 6356768.337303, "Mercury_Modificado_1968" ); }
		inline TEllipsoid Ellipsoid_Mercury_1960() {			return TEllipsoid( 6378166.000, 6356784.283666, "Mercury_1960"  ); }
		inline TEllipsoid Ellipsoid_Krasovsky_1940() {			return TEllipsoid( 6378245.000, 6356863.018800, "Krasovsky_1940"  ); }
		inline TEllipsoid Ellipsoid_Internacional_1924() {		return TEllipsoid( 6378388.000, 6356911.946130, "Internacional_1924"  ); }
		inline TEllipsoid Ellipsoid_Internacional_1909() {		return TEllipsoid( 6378388.000, 6356911.946130, "Internacional_1909" ); }
		inline TEllipsoid Ellipsoid_Hough_1960() {				return TEllipsoid( 6378270.000, 6356794.343479, "Hough_1960"  ); }
		inline TEllipsoid Ellipsoid_Helmert_1906() {			return TEllipsoid( 6378200.000, 6356818.170000, "Helmert_1906"  ); }
		inline TEllipsoid Ellipsoid_Hayford_1909() {			return TEllipsoid( 6378388.000, 6356911.946130, "Hayford_1909"  ); }
		inline TEllipsoid Ellipsoid_GRS80() {					return TEllipsoid( 6378137.000, 6356752.314140, "GRS80"  ); }
		inline TEllipsoid Ellipsoid_Fischer_1968() {			return TEllipsoid( 6378150.000, 6356768.330000, "Fischer_1968"  ); }
		inline TEllipsoid Ellipsoid_Fischer_1960() {			return TEllipsoid( 6378166.000, 6356784.280000, "Fischer_1960" ); }
		inline TEllipsoid Ellipsoid_Clarke_1880() {				return TEllipsoid( 6378249.145, 6356514.869550, "Clarke_1880"  ); }
		inline TEllipsoid Ellipsoid_Clarke_1866() {				return TEllipsoid( 6378206.400, 6356583.800000, "Clarke_1866"  ); }
		inline TEllipsoid Ellipsoid_Bessel_1841() {				return TEllipsoid( 6377397.155, 6356078.962840, "Bessel_1841"  ); }
		inline TEllipsoid Ellipsoid_Airy_Modificado_1965() {	return TEllipsoid( 6377340.189, 6356034.447900, "Airy_Modificado_1965"  ); }
		inline TEllipsoid Ellipsoid_Airy_1830() {				return TEllipsoid( 6377563.396, 6356256.910000, "Airy_1830"  ); }

		typedef mrpt::math::TPoint3D TUTMCoords;
		typedef mrpt::math::TPoint3D TGeocentricCoords;

		/**  A set of geodetic coordinates: latitude, longitude and height, defined over a given geoid (typically, WGS84)  */
		struct TOPO_IMPEXP TGeodeticCoords
		{
			TGeodeticCoords() : lat(0),lon(0),height(0) {}
			TGeodeticCoords(const double _lat, const double _lon, const double _height) : lat(_lat),lon(_lon),height(_height) {}

			double lat;		//!< Latitude (in degrees)
			double lon;		//!< Longitude (in degrees)
			double height;	//!< Geodetic height (in meters)
		};

		/** Parameters for a topographic transfomation
		  * \sa TDatum10Params, transform7params
		  */
		struct TOPO_IMPEXP TDatum7Params
		{
			double dX, dY, dZ;		//!< Deltas (X,Y,Z)
			double Rx, Ry, Rz;		//!< Rotation components (in secs)
			double dS;				//!< Scale factor (in ppm) (Scale is 1+dS/1e6)

			inline TDatum7Params(
				const double _dX, const double _dY, const double _dZ,
				const double _Rx, const double _Ry, const double _Rz,
				const double _dS ) :
			dX(_dX), dY(_dY), dZ(_dZ)
			{
				Rx = DEG2RAD(_Rx/60/60);
				Ry = DEG2RAD(_Ry/60/60);
				Rz = DEG2RAD(_Rz/60/60);
				dS = _dS*1e-6;
			}
		};

		struct TOPO_IMPEXP TDatum7Params_TOPCON
		{
			double dX, dY, dZ;		//!< Deltas (X,Y,Z)
			double m11, m12, m13, m21, m22, m23, m31, m32, m33;
			double dS;				//!< Scale factor (in ppm) (Scale is 1+dS/1e6)

			inline TDatum7Params_TOPCON(
				const double _dX, const double _dY, const double _dZ,
				const double _m11, const double _m12, const double _m13,
				const double _m21, const double _m22, const double _m23,
				const double _m31, const double _m32, const double _m33,
				const double _dS ) :
			dX(_dX), dY(_dY), dZ(_dZ), m11(_m11), m12(_m12), m13(_m13), m21(_m21), m22(_m22), m23(_m23), m31(_m31), m32(_m32), m33(_m33)
			{
				dS = _dS*1e-6;
			}
		};


		/** Parameters for a topographic transfomation
		  * \sa TDatum7Params, transform10params
		  */
		struct TOPO_IMPEXP TDatum10Params
		{
			double dX, dY, dZ; //!< Deltas (X,Y,Z)
			double Xp, Yp, Zp; //!< To be substracted to the input point
			double Rx, Ry, Rz; //!< Rotation components
			double dS; //!< Scale factor (Scale is 1+dS)

			inline TDatum10Params(
				const double _dX, const double _dY, const double _dZ,
				const double _Xp, const double _Yp, const double _Zp,
				const double _Rx, const double _Ry, const double _Rz,
				const double _dS ) :
			dX(_dX), dY(_dY), dZ(_dZ), Xp(_Xp), Yp(_Yp), Zp(_Zp)
			{
				Rx = DEG2RAD(_Rx/60/60);
				Ry = DEG2RAD(_Ry/60/60);
				Rz = DEG2RAD(_Rz/60/60);
				dS = _dS*1e-6;
			}
		};

		/** Parameters for a topographic transfomation
		  * \sa TDatumHelmert3D, transformHelmert2D
		  */
		struct TOPO_IMPEXP TDatumHelmert2D
		{
			double dX, dY;		//!< Deltas [X,Y]
			double alpha;		// The rotation about Z-axis (degrees)
			double dS;			// Scale factor (Scale is 1+dS)
			double Xp, Yp;		// Coordinates of the rotation point

			inline TDatumHelmert2D(
				const double _dX, const double _dY,
				const double _alpha, const double _dS,
				const double _Xp, const double _Yp ) :
			dX(_dX), dY(_dY), Xp(_Xp), Yp(_Yp)
			{
				alpha = DEG2RAD(_alpha);
				dS = _dS*1e-6;
			}
		};

		struct TOPO_IMPEXP TDatumHelmert2D_TOPCON
		{
			double a,b,c,d;

			inline TDatumHelmert2D_TOPCON(
				const double _a, const double _b,
				const double _c, const double _d ) :
			a(_a), b(_b), c(_c), d(_d) {}

		};

		/** Parameters for a topographic transfomation
		  * \sa TDatumHelmert2D, transformHelmert3D
		  */
		struct TOPO_IMPEXP TDatumHelmert3D
		{
			double dX, dY, dZ; //!< Deltas (X,Y,Z)
			double Rx, Ry, Rz; //!< Rotation components
			double dS; //!< Scale factor (Scale is 1+dS)

			inline TDatumHelmert3D(
				const double _dX, const double _dY, const double _dZ,
				const double _Rx, const double _Ry, const double _Rz,
				const double _dS ) :
			dX(_dX), dY(_dY), dZ(_dZ)
			{
				Rx = DEG2RAD(_Rx/60/60);
				Ry = DEG2RAD(_Ry/60/60);
				Rz = DEG2RAD(_Rz/60/60);
				dS = _dS*1e-6;
			}
		};

		/** Parameters for a topographic transfomation
		  * \sa TDatumHelmert2D, transformHelmert3D
		  */
		struct TOPO_IMPEXP TDatumHelmert3D_TOPCON
		{
			double a,b,c,d,e,f,g;

			inline TDatumHelmert3D_TOPCON(
				const double _a, const double _b, const double _c,
				const double _d, const double _e, const double _f, const double _g ) :
			a(_a), b(_b), c(_c), d(_d), e(_e), f(_f), g(_g) { }
		};

		/** Parameters for a topographic transfomation
		  * \sa transform1D
		  */
		struct TOPO_IMPEXP TDatum1DTransf
		{
			double dX, dY, DZ; //!< Deltas (X,Y,Z)
			double dS; //!< Scale factor (Scale is 1+dS)

			inline TDatum1DTransf(
				const double _dX, const double _dY, const double _DZ,
				const double _dS ) :
			dX(_dX), dY(_dY), DZ(_DZ)
			{
				dS = _dS*1e-6;
			}
		};

		/** Parameters for a topographic transfomation
		  * \sa transform1D
		  */
		struct TOPO_IMPEXP TDatumTransfInterpolation
		{
			double dX, dY;		//!< Deltas (X,Y,Z)
			double dSx, dSy;	//!< Scale factor in X and Y
			double beta;		//!< Distortion angle

			inline TDatumTransfInterpolation(
				const double _dX, const double _dY,
				const double _dSx, const double _dSy, const double _beta ) :
			dX(_dX), dY(_dY)
			{
				dSx = _dSx*1e-6;
				dSy = _dSy*1e-6;
				beta = DEG2RAD(_beta/60/60);
			}
		};

		/** @}
	    ======================================================================= */


	/** =======================================================================
		   @name Topography coordinate conversion functions
		   @{ */

		/** Coordinates transformation from longitude/latitude/height to ENU (East-North-Up)  X/Y/Z coordinates
		  *  The WGS84 ellipsoid is used for the transformation. The coordinates are in 3D
		  *   relative to some user-provided point, with local X axis being east-ward, Y north-ward, Z up-ward.
		  *  For an explanation, refer to http://en.wikipedia.org/wiki/Reference_ellipsoid
		  * \sa coordinatesTransformation_WGS84_geocentric, ENU_axes_from_WGS84
		  */
		void  TOPO_IMPEXP geodeticToENU_WGS84(
			const TGeodeticCoords		&in_coords,
			mrpt::math::TPoint3D	&out_ENU_point,
			const TGeodeticCoords		&in_coords_origin );

		/** Coordinates transformation from longitude/latitude/height to geocentric X/Y/Z coordinates (with a WGS84 geoid).
		  *  The WGS84 ellipsoid is used for the transformation. The coordinates are in 3D
		  *   where the reference is the center of the Earth.
		  *  For an explanation, refer to http://en.wikipedia.org/wiki/Reference_ellipsoid
		  * \sa geodeticToENU_WGS84
		  */
		void  TOPO_IMPEXP geodeticToGeocentric_WGS84(
			const TGeodeticCoords		&in_coords,
			mrpt::math::TPoint3D  &out_point );

		/** Coordinates transformation from longitude/latitude/height to geocentric X/Y/Z coordinates (with an specified geoid).
		  * \sa geocentricToGeodetic
		  */
		void  TOPO_IMPEXP geodeticToGeocentric(
			const TGeodeticCoords	&in_coords,
			TGeocentricCoords		&out_point,
			const TEllipsoid		&ellip );

		/** Coordinates transformation from geocentric X/Y/Z coordinates to longitude/latitude/height.
		  * \sa geodeticToGeocentric
		  */
		void  TOPO_IMPEXP geocentricToGeodetic(
			const TGeocentricCoords		&in_point,
			TGeodeticCoords				&out_coords,
			const TEllipsoid			&ellip = Ellipsoid_WGS84() );

		/**  7-parameter Bursa-Wolf transformation:
		  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1 ] [ X Y Z ]_local
		  * \sa transform10params
		  */
		void  TOPO_IMPEXP transform7params(
			const mrpt::math::TPoint3D	&in_point,
			const TDatum7Params			&in_datum,
			mrpt::math::TPoint3D		&out_point);

		void  TOPO_IMPEXP transform7params_TOPCON(
			const mrpt::math::TPoint3D	&in_point,
			const TDatum7Params_TOPCON	&in_datum,
			mrpt::math::TPoint3D		&out_point);

		/**  10-parameter Molodensky-Badekas transformation:
		  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1 ] [ X-Xp Y-Yp Z-Zp ]_local  + [Xp Yp Zp]
		  * \sa transform7params
		  */
		void  TOPO_IMPEXP transform10params(
			const mrpt::math::TPoint3D	&in_point,
			const TDatum10Params		&in_datum,
			mrpt::math::TPoint3D		&out_point);

		/**  Helmert 2D transformation:
		  *   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha); sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]
		  * \sa transformHelmert3D
		  */
		void  TOPO_IMPEXP transformHelmert2D(
			const mrpt::math::TPoint2D	&p,
			const TDatumHelmert2D		&d,
			mrpt::math::TPoint2D		&o);

		void  TOPO_IMPEXP transformHelmert2D_TOPCON(
			const mrpt::math::TPoint2D		&p,
			const TDatumHelmert2D_TOPCON	&d,
			mrpt::math::TPoint2D			&o);

		/**  Helmert3D transformation:
		  *   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 -RZ RY; RZ 1 -RX; -RY RX 1 ] [ X Y Z ]_local
		  * \sa transformHelmert2D
		  */
		void  TOPO_IMPEXP transformHelmert3D(
			const mrpt::math::TPoint3D	&p,
			const TDatumHelmert3D		&d,
			mrpt::math::TPoint3D		&o);

		void  TOPO_IMPEXP transformHelmert3D_TOPCON(
			const mrpt::math::TPoint3D		&p,
			const TDatumHelmert3D_TOPCON	&d,
			mrpt::math::TPoint3D			&o);

		/**  1D transformation:
		  *   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ
		  */
		void  TOPO_IMPEXP transform1D(
			const mrpt::math::TPoint3D	&p,
			const TDatum1DTransf		&d,
			mrpt::math::TPoint3D		&o);

		/**  Interpolation:
		  *   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ
		  */
		void  TOPO_IMPEXP transfInterpolation(
			const mrpt::math::TPoint3D		&p,
			const TDatumTransfInterpolation	&d,
			mrpt::math::TPoint3D			&o);

		/** Returns the Geodetic coordinates of the UTM input point.
		  * \param X: East coordinate of the input point.
		  * \param Y: North coordinate of the input point.
		  * \param zone: time zone (Spanish: "huso").
		  * \param hem: hemisphere ('N'/'n' for North or 'S'/s' for South ). An exception will be raised on any other value.
		  * \param ellip: the reference ellipsoid used for the transformation (default: WGS84)
		  * \param out_lat  Out latitude, in degrees.
		  * \param out_lon  Out longitude, in degrees.
		  */
		void TOPO_IMPEXP UTMToGeodetic(
			double		X,
			double		Y,
			int			zone,
			char		hem,
			double		&out_lon /*degrees*/,
			double		&out_lat /*degrees*/,
			TEllipsoid	ellip = Ellipsoid_WGS84() );

		/** Returns the Geodetic coordinates of the UTM input point.
		  * \param UTMCoords: UTM input coordinates.
		  * \param zone: time zone (Spanish: "huso").
		  * \param hem: hemisphere ('N'/'n' for North or 'S'/s' for South ). An exception will be raised on any other value.
		  * \param GeodeticCoords: Out geodetic coordinates.
		  * \param ellip: the reference ellipsoid used for the transformation (default: WGS84)
		  */
		inline void TOPO_IMPEXP UTMToGeodetic(
			const TUTMCoords	&UTMCoords,
			const int			&zone,
			const char			&hem,
			TGeodeticCoords		&GeodeticCoords,
			TEllipsoid	ellip = Ellipsoid_WGS84() )
		{
			UTMToGeodetic( UTMCoords.x, UTMCoords.y, zone, hem, GeodeticCoords.lon, GeodeticCoords.lat, ellip );
			GeodeticCoords.height = UTMCoords.z;
		}

		/** Convert latitude and longitude coordinates into UTM coordinates, computing the corresponding UTM zone and latitude band.
		  *   This method is based on public code by Gabriel Ruiz Martinez and Rafael Palacios.
		  *   Example:
		  *   \code
		  *   Input:
		  *    Lat=40.3154333    Lon=-3.4857166
		  *   Output:
		  *    x = 458731
		  *    y = 4462881
		  *    utm_zone = 30
		  *    utm_band = T
		  *   \endcode
		  *   \sa http://www.mathworks.com/matlabcentral/fileexchange/10915
		 */
		void  TOPO_IMPEXP GeodeticToUTM(
			double		in_latitude_degrees,
			double		in_longitude_degrees,
			double    	&out_UTM_x,
			double    	&out_UTM_y,
			int    		&out_UTM_zone,
			char    	&out_UTM_latitude_band,
			TEllipsoid	ellip = Ellipsoid_WGS84());

		void  TOPO_IMPEXP geodeticToUTM(
			const TGeodeticCoords	&GeodeticCoords,
			TUTMCoords				&UTMCoords,
			int    					&UTMZone,
			char    				&UTMLatitudeBand,
			TEllipsoid				ellip = Ellipsoid_WGS84());


		/** Convert latitude and longitude coordinates into UTM coordinates, computing the corresponding UTM zone and latitude band.
		  *   This method is based on public code by Gabriel Ruiz Martinez and Rafael Palacios.
		  *   Example:
		  *   \code
		  *   Input:
		  *    Lat=40.3154333    Lon=-3.4857166
		  *   Output:
		  *    x = 458731
		  *    y = 4462881
		  *    utm_zone = 30
		  *    utm_band = T
		  *   \endcode
		  *   \sa http://www.mathworks.com/matlabcentral/fileexchange/10915
		 */
		inline void  TOPO_IMPEXP GeodeticToUTM(
			const TGeodeticCoords	&GeodeticCoords,
			TUTMCoords				&UTMCoords,
			int    					&UTMZone,
			char    				&UTMLatitudeBand,
			TEllipsoid				ellip = Ellipsoid_WGS84())
		{
			GeodeticToUTM( GeodeticCoords.lat, GeodeticCoords.lon, UTMCoords.x, UTMCoords.y, UTMZone, UTMLatitudeBand, ellip );
			UTMCoords.z = GeodeticCoords.height;
		}


	/** @}
	    ======================================================================= */


	/** =======================================================================
	   @name DEPRECATED topography coordinate conversion functions
	   @{ */

		/** Coordinates transformation from longitude/latitude/height to ENU (East-North-Up)  X/Y/Z coordinates
		  *  The WGS84 ellipsoid is used for the transformation. The coordinates are in 3D
		  *   relative to some user-provided point, with local X axis being east-ward, Y north-ward, Z up-ward.
		  *  For an explanation, refer to http://en.wikipedia.org/wiki/Reference_ellipsoid
		  * \sa coordinatesTransformation_WGS84_geocentric, ENU_axes_from_WGS84
		  */
		MRPT_DECLARE_DEPRECATED_FUNCTION("*DEPRECATED* Use geodeticToENU_WGS84 instead",
		void TOPO_IMPEXP coordinatesTransformation_WGS84(
			double		in_longitude_degrees,
			double		in_latitude_degrees,
			double		in_height_meters,
			double		&out_x_meters,
			double		&out_y_meters,
			double		&out_z_meters,
			double		in_longitude_reference_degrees,
			double		in_latitude_reference_degrees,
			double		in_height_reference_meters
			));

		/** Coordinates transformation from longitude/latitude/height to geocentric X/Y/Z coordinates (with a WGS84 geoid).
		  *  The WGS84 ellipsoid is used for the transformation. The coordinates are in 3D
		  *   where the reference is the center of the Earth.
		  *  For an explanation, refer to http://en.wikipedia.org/wiki/Reference_ellipsoid
		  * \sa geodeticToENU_WGS84
		  */
		MRPT_DECLARE_DEPRECATED_FUNCTION("*DEPRECATED* Use geodeticToENU_WGS84 instead",
		void TOPO_IMPEXP coordinatesTransformation_WGS84_geocentric(
			double		in_longitude_degrees,
			double		in_latitude_degrees,
			double		in_height_meters,
			double		&out_x_meters,
			double		&out_y_meters,
			double		&out_z_meters
			) );


	/** @}
	    ======================================================================= */


	/** =======================================================================
	   @name Miscellaneous
	   @{ */

		/** Returns the East-North-Up (ENU) coordinate system associated to the given point.
		  * This is the reference employed in geodeticToENU_WGS84
		  * \param only_angles If set to true, the (x,y,z) fields will be left zeroed.
		  * \sa geodeticToENU_WGS84
		  */
		void TOPO_IMPEXP ENU_axes_from_WGS84(
			double		in_longitude_reference_degrees,
			double		in_latitude_reference_degrees,
			double		in_height_reference_meters,
			mrpt::math::TPose3D &out_ENU,
			bool		only_angles = false
			);

		/** Returns the East-North-Up (ENU) coordinate system associated to the given point.
		  * This is the reference employed in coordinatesTransformation_WGS84
		  * \param only_angles If set to true, the (x,y,z) fields will be left zeroed.
		  * \sa geodeticToENU_WGS84
		  */
		inline void ENU_axes_from_WGS84(
			const TGeodeticCoords		&in_coords,
			mrpt::math::TPose3D &out_ENU,
			bool		only_angles = false
			)
		{
			ENU_axes_from_WGS84(in_coords.lon,in_coords.lat,in_coords.height, out_ENU,only_angles);
		}

	/** @}
	    ======================================================================= */


	} // End of namespace

} // End of namespace

#endif
