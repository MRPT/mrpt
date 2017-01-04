/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  data_types_H
#define  data_types_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/topography/link_pragmas.h>

namespace mrpt
{
	namespace topography
	{
		/** \addtogroup mrpt_topography_grp
		  *  @{ */

	/** @name Data structures
	   @{ */

		/** A coordinate that is stored as a simple "decimal" angle in degrees, but can be retrieved/set in the form of DEGREES + arc-MINUTES + arc-SECONDS.
		  */
		struct TOPO_IMPEXP TCoords
		{
			// Only keep one of the possible representations:
			double decimal_value; //!< Also obtained directly through the double(void) operator using a TCoords anywhere were a double is expected.

			inline TCoords( const int _deg, const int _min, const double _sec ) { setDegMinSec(_deg,_min,_sec); }
			inline TCoords( const double dec ) { setFromDecimal(dec); }
			inline TCoords() { setFromDecimal(0); }

			/** Automatic conversion to a double value (read-only) */
			inline operator double(void) const { return decimal_value; }

			/** Automatic conversion to a double value (read-only) */
			inline operator double& (void) { return decimal_value; }

			/** Set from a decimal value (XX.YYYYY) in degrees. */
			inline void setFromDecimal( const double dec ) { decimal_value = dec; }

			/** Get the decimal value (XX.YYYYY), in degrees - you can also use the automatic conversion between TCoords and a double.  */
			inline double getDecimalValue() const { return decimal_value; }

			/** Return the Deg Min' Sec'' representation of this value. */
			inline void getDegMinSec( int &degrees, int &minutes, double &seconds) const
			{
				double aux = std::abs(decimal_value);
				degrees = (int)aux;
				minutes = (int)((aux - degrees)*60.0f);
				seconds = ((aux - degrees)*60.0f - minutes)*60.0f;
				if( decimal_value<0 ) degrees = -degrees;
			}

			/** Set the coordinate from its Deg Min' Deg'' parts. */
			inline void setDegMinSec(const int degrees, const int minutes, const double seconds)
			{
				decimal_value = std::abs(degrees)+minutes/60.0+seconds/3600.0;
				if(degrees<0)  decimal_value  = - decimal_value;
			}

			/** Return a std::string in the format "DEGdeg MIN' SEC''" */
			inline std::string getAsString() const
			{
				int deg,min;
				double sec;
				getDegMinSec( deg,min,sec);
				return mrpt::format("%ddeg %d' %.04f''",deg,min,sec );
			}

		};

		bool TOPO_IMPEXP operator ==(const TCoords &a, const TCoords &o);
		bool TOPO_IMPEXP operator !=(const TCoords &a, const TCoords &o);

		std::ostream TOPO_IMPEXP & operator<<( std::ostream& out, const TCoords &o );

		struct TOPO_IMPEXP TEllipsoid
		{
			inline TEllipsoid() : sa( 6378137.0 ), sb( 6356752.314245 ), name("WGS84") {}
			inline TEllipsoid( const double _sa, const double _sb, const std::string &_name ) : sa(_sa), sb(_sb), name(_name) {}

			double sa;		//!< largest semiaxis of the reference ellipsoid (in meters)
			double sb;		//!< smallest semiaxis of the reference ellipsoid (in meters)
			std::string name;	//!< the ellipsoid name

			static inline TEllipsoid Ellipsoid_WGS84() {					return TEllipsoid( 6378137.000, 6356752.314245, "WGS84" ); }
			static inline TEllipsoid Ellipsoid_WGS72() {					return TEllipsoid( 6378135.000, 6356750.519915, "WGS72"  ); }
			static inline TEllipsoid Ellipsoid_WGS66() {					return TEllipsoid( 6378145.000, 6356759.769356, "WGS66"  ); }
			static inline TEllipsoid Ellipsoid_Walbeck_1817() {			return TEllipsoid( 6376896.000, 6355834.846700, "Walbeck_1817"  ); }
			static inline TEllipsoid Ellipsoid_Sudamericano_1969() {		return TEllipsoid( 6378160.000, 6356774.720000, "Sudamericano_1969"  ); }
			static inline TEllipsoid Ellipsoid_Nuevo_Internacional_1967() {return TEllipsoid( 6378157.500, 6356772.200000, "Nuevo_Internacional_1967"  ); }
			static inline TEllipsoid Ellipsoid_Mercury_Modificado_1968() {	return TEllipsoid( 6378150.000, 6356768.337303, "Mercury_Modificado_1968" ); }
			static inline TEllipsoid Ellipsoid_Mercury_1960() {			return TEllipsoid( 6378166.000, 6356784.283666, "Mercury_1960"  ); }
			static inline TEllipsoid Ellipsoid_Krasovsky_1940() {			return TEllipsoid( 6378245.000, 6356863.018800, "Krasovsky_1940"  ); }
			static inline TEllipsoid Ellipsoid_Internacional_1924() {		return TEllipsoid( 6378388.000, 6356911.946130, "Internacional_1924"  ); }
			static inline TEllipsoid Ellipsoid_Internacional_1909() {		return TEllipsoid( 6378388.000, 6356911.946130, "Internacional_1909" ); }
			static inline TEllipsoid Ellipsoid_Hough_1960() {				return TEllipsoid( 6378270.000, 6356794.343479, "Hough_1960"  ); }
			static inline TEllipsoid Ellipsoid_Helmert_1906() {			return TEllipsoid( 6378200.000, 6356818.170000, "Helmert_1906"  ); }
			static inline TEllipsoid Ellipsoid_Hayford_1909() {			return TEllipsoid( 6378388.000, 6356911.946130, "Hayford_1909"  ); }
			static inline TEllipsoid Ellipsoid_GRS80() {					return TEllipsoid( 6378137.000, 6356752.314140, "GRS80"  ); }
			static inline TEllipsoid Ellipsoid_Fischer_1968() {			return TEllipsoid( 6378150.000, 6356768.330000, "Fischer_1968"  ); }
			static inline TEllipsoid Ellipsoid_Fischer_1960() {			return TEllipsoid( 6378166.000, 6356784.280000, "Fischer_1960" ); }
			static inline TEllipsoid Ellipsoid_Clarke_1880() {				return TEllipsoid( 6378249.145, 6356514.869550, "Clarke_1880"  ); }
			static inline TEllipsoid Ellipsoid_Clarke_1866() {				return TEllipsoid( 6378206.400, 6356583.800000, "Clarke_1866"  ); }
			static inline TEllipsoid Ellipsoid_Bessel_1841() {				return TEllipsoid( 6377397.155, 6356078.962840, "Bessel_1841"  ); }
			static inline TEllipsoid Ellipsoid_Airy_Modificado_1965() {	return TEllipsoid( 6377340.189, 6356034.447900, "Airy_Modificado_1965"  ); }
			static inline TEllipsoid Ellipsoid_Airy_1830() {				return TEllipsoid( 6377563.396, 6356256.910000, "Airy_1830"  ); }
		};


		typedef mrpt::math::TPoint3D TUTMCoords;
		typedef mrpt::math::TPoint3D TGeocentricCoords;

		/**  A set of geodetic coordinates: latitude, longitude and height, defined over a given geoid (typically, WGS84)  */
		struct TOPO_IMPEXP TGeodeticCoords
		{
			TGeodeticCoords() : lat(0),lon(0),height(0) {}
			TGeodeticCoords(const double _lat, const double _lon, const double _height) : lat(_lat),lon(_lon),height(_height) {}

			inline bool isClear() const { return lat.getDecimalValue()==0 && lon.getDecimalValue()==0 && height==0; }

			TCoords lat;	//!< Latitude (in degrees)
			TCoords lon;	//!< Longitude (in degrees)
			double  height;	//!< Geodetic height (in meters)

		};

		bool TOPO_IMPEXP operator ==(const TGeodeticCoords &a, const TGeodeticCoords &o);
		bool TOPO_IMPEXP operator !=(const TGeodeticCoords &a, const TGeodeticCoords &o);

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
				Rx = mrpt::utils::DEG2RAD(_Rx/60/60);
				Ry = mrpt::utils::DEG2RAD(_Ry/60/60);
				Rz = mrpt::utils::DEG2RAD(_Rz/60/60);
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
				Rx = mrpt::utils::DEG2RAD(_Rx/60/60);
				Ry = mrpt::utils::DEG2RAD(_Ry/60/60);
				Rz = mrpt::utils::DEG2RAD(_Rz/60/60);
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
				alpha = mrpt::utils::DEG2RAD(_alpha);
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
				Rx = mrpt::utils::DEG2RAD(_Rx/60/60);
				Ry = mrpt::utils::DEG2RAD(_Ry/60/60);
				Rz = mrpt::utils::DEG2RAD(_Rz/60/60);
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
				beta = mrpt::utils::DEG2RAD(_beta/60/60);
			}
		};

		/** @} */
	
		/** @} */ // end of grouping
		
	} // End of namespace

} // End of namespace

#endif
