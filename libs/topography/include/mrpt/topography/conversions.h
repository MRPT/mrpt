/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  conversions_H
#define  conversions_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/topography/link_pragmas.h>

#include <mrpt/topography/data_types.h>


namespace mrpt
{
	/** This namespace provides topography helper functions, coordinate transformations.
	 * \ingroup mrpt_topography_grp
	 */
	namespace topography
	{
		/** \addtogroup mrpt_topography_grp
		  *  @{ */

		/** @name Topography coordinate conversion functions
		    @{ */

		/** Coordinates transformation from longitude/latitude/height to ENU (East-North-Up)  X/Y/Z coordinates
		  *  The WGS84 ellipsoid is used for the transformation. The coordinates are in 3D
		  *   relative to some user-provided point, with local X axis being east-ward, Y north-ward, Z up-ward.
		  *  For an explanation, refer to http://en.wikipedia.org/wiki/Reference_ellipsoid
		  * \sa coordinatesTransformation_WGS84_geocentric, ENU_axes_from_WGS84, ENUToGeocentric
		  * \note The "Up" (Z) direction in ENU is the normal to the ellipsoid, which coincides with the direction of an increasing geodetic height.
		  */
		void  TOPO_IMPEXP geodeticToENU_WGS84(
			const TGeodeticCoords		&in_coords,
			mrpt::math::TPoint3D	&out_ENU_point,
			const TGeodeticCoords		&in_coords_origin );

		/** ENU to geocentric coordinates. \sa geodeticToENU_WGS84 */
		void  TOPO_IMPEXP ENUToGeocentric(
			const mrpt::math::TPoint3D	&in_ENU_point,
			const TGeodeticCoords		&in_coords_origin,
			TGeocentricCoords			&out_coords,
			const TEllipsoid			&ellip );

		/** ENU to EFEC (Geocentric) coordinates \sa ENUToGeocentric, geodeticToENU_WGS84 */
		void TOPO_IMPEXP geocentricToENU_WGS84(
			const mrpt::math::TPoint3D  &in_geocentric_point,
			mrpt::math::TPoint3D        &out_ENU_point,
			const TGeodeticCoords       &in_coords_origin );

		/** \overload More efficient for converting a pointcloud */
		void TOPO_IMPEXP geocentricToENU_WGS84(
			const std::vector<mrpt::math::TPoint3D> &in_geocentric_points,
			std::vector<mrpt::math::TPoint3D>       &out_ENU_points,
			const TGeodeticCoords       &in_coords_origin );

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
			const TEllipsoid			&ellip = TEllipsoid::Ellipsoid_WGS84() );

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
			const TEllipsoid	&ellip = TEllipsoid::Ellipsoid_WGS84() );

		/** Returns the Geodetic coordinates of the UTM input point.
		  * \param UTMCoords: UTM input coordinates.
		  * \param zone: time zone (Spanish: "huso").
		  * \param hem: hemisphere ('N'/'n' for North or 'S'/s' for South ). An exception will be raised on any other value.
		  * \param GeodeticCoords: Out geodetic coordinates.
		  * \param ellip: the reference ellipsoid used for the transformation (default: WGS84)
		  */
		inline void UTMToGeodetic(
			const TUTMCoords	&UTMCoords,
			const int			&zone,
			const char			&hem,
			TGeodeticCoords		&GeodeticCoords,
			const TEllipsoid	&ellip = TEllipsoid::Ellipsoid_WGS84() )
		{
			UTMToGeodetic( UTMCoords.x, UTMCoords.y, zone, hem, GeodeticCoords.lon.decimal_value, GeodeticCoords.lat.decimal_value, ellip );
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
			const TEllipsoid	&ellip = TEllipsoid::Ellipsoid_WGS84());

		void  TOPO_IMPEXP geodeticToUTM(
			const TGeodeticCoords	&GeodeticCoords,
			TUTMCoords				&UTMCoords,
			int    					&UTMZone,
			char    				&UTMLatitudeBand,
			const TEllipsoid		&ellip = TEllipsoid::Ellipsoid_WGS84());


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
		inline void  GeodeticToUTM(
			const TGeodeticCoords	&GeodeticCoords,
			TUTMCoords				&UTMCoords,
			int    					&UTMZone,
			char    				&UTMLatitudeBand,
			const TEllipsoid		&ellip = TEllipsoid::Ellipsoid_WGS84())
		{
			GeodeticToUTM( GeodeticCoords.lat, GeodeticCoords.lon, UTMCoords.x, UTMCoords.y, UTMZone, UTMLatitudeBand, ellip );
			UTMCoords.z = GeodeticCoords.height;
		}

	/** @}
	    ======================================================================= */


	/** =======================================================================
	   @name Miscellaneous
	   @{ */

		/** Returns the East-North-Up (ENU) coordinate system associated to the given point.
		  * This is the reference employed in geodeticToENU_WGS84
		  * \param only_angles If set to true, the (x,y,z) fields will be left zeroed.
		  * \note The "Up" (Z) direction in ENU is the normal to the ellipsoid, which coincides with the direction of an increasing geodetic height.
		  * \sa geodeticToENU_WGS84
		  */
		void TOPO_IMPEXP ENU_axes_from_WGS84(
			double		in_longitude_reference_degrees,
			double		in_latitude_reference_degrees,
			double		in_height_reference_meters,
			mrpt::math::TPose3D &out_ENU,
			bool		only_angles = false
			);

		/** \overload */
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

		/**  @} */  // end of grouping

	} // End of namespace

} // End of namespace

#endif
