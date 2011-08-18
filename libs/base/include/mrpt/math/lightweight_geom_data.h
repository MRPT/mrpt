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
#ifndef LIGHTWEIGHT_GEOM_DATA_H
#define LIGHTWEIGHT_GEOM_DATA_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/stl_extensions.h>
#include <mrpt/utils/TPixelCoord.h>

#include <mrpt/math/math_frwds.h>  // Fordward declarations



namespace mrpt	{
	namespace poses	{
		template <class DERIVEDCLASS> class CPoseOrPoint;
		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;
	}
	namespace utils { class CStream; }
}

namespace mrpt	{
namespace math	{
	using namespace mrpt::utils; // For "square"

	struct TPoint2D;
	struct TPose2D;
	struct TPoint3D;
	struct TPose3D;
	struct TPose3DQuat;

	/** \addtogroup geometry_grp
	  * @{ */


	//Pragma defined to ensure no structure packing
#pragma pack(push,1)
	//Set of typedefs for lightweight geometric items.
	/**
	  * Lightweight 2D point. Allows coordinate access using [] operator.
	  * \sa mrpt::poses::CPoint2D
	  */
//#define TOBJECTS_USE_UNIONS
	struct BASE_IMPEXP TPoint2D	{
		/**
		  * X coordinate.
		  */
		double x;
		/**
		  * Y coordinate.
		  */
		double y;
		/**
		  * Constructor from TPose2D, discarding phi.
		  * \sa TPose2D
		  */
		explicit TPoint2D(const TPose2D &p);
		/**
		  * Constructor from TPoint3D, discarding z.
		  * \sa TPoint3D
		  */
		explicit TPoint2D(const TPoint3D &p);
		/**
		  * Constructor from TPose3D, discarding z and the angular coordinates.
		  * \sa TPose3D
		  */
		explicit TPoint2D(const TPose3D &p);
		/**
		  * Constructor from CPoseOrPoint, perhaps losing 3D information
		  * \sa CPoseOrPoint,CPoint3D,CPose2D,CPose3D
		  */
		template <class DERIVEDCLASS>
		explicit TPoint2D(const mrpt::poses::CPoseOrPoint<DERIVEDCLASS> &p) :x(p.x()),y(p.y())	{}

		/** Implicit transformation constructor from TPixelCoordf */
		inline TPoint2D(const mrpt::utils::TPixelCoordf &p) :x(p.x),y(p.y)	{}

		/** Implicit constructor from CPoint2D  */
		TPoint2D(const mrpt::poses::CPoint2D &p);
		/**
		  * Constructor from coordinates.
		  */
		inline TPoint2D(double xx,double yy):x(xx),y(yy)	{}
		/**
		  * Default fast constructor. Initializes to garbage.
		  */
		inline TPoint2D()	{}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline double &operator[](size_t i)	{
			return (&x)[i];
		}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline const double &operator[](size_t i) const	{
			return (&x)[i];
		}
		/**
		  * Transformation into vector.
		  */
		inline void getAsVector(vector_double &v) const	{
			v.resize(2);
			v[0]=x; v[1]=y;
		}

		bool operator<(const TPoint2D &p) const;

		inline TPoint2D &operator+=(const TPoint2D &p)	{
			x+=p.x;
			y+=p.y;
			return *this;
		}

		inline TPoint2D &operator-=(const TPoint2D &p)	{
			x-=p.x;
			y-=p.y;
			return *this;
		}

		inline TPoint2D &operator*=(double d)	{
			x*=d;
			y*=d;
			return *this;
		}

		inline TPoint2D &operator/=(double d)	{
			x/=d;
			y/=d;
			return *this;
		}

		inline TPoint2D operator+(const TPoint2D &p) const	{
			TPoint2D r(*this);
			return r+=p;
		}

		inline TPoint2D operator-(const TPoint2D &p) const	{
			TPoint2D r(*this);
			return r-=p;
		}

		inline TPoint2D operator*(double d) const	{
			TPoint2D r(*this);
			return r*=d;
		}

		inline TPoint2D operator/(double d) const	{
			TPoint2D r(*this);
			return r/=d;
		}
		 /** Returns a human-readable textual representation of the object (eg: "[0.02 1.04]" )
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f]",x,y); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s);
		 static size_t size() { return 2; }
	};

	/**
	  * Lightweight 2D pose. Allows coordinate access using [] operator.
	  * \sa mrpt::poses::CPose2D
	  */
	struct BASE_IMPEXP TPose2D	{
		/**
		  * X coordinate.
		  */
		double x;
		/**
		  * Y coordinate.
		  */
		double y;
		/**
		  * Phi coordinate.
		  */
		double phi;
		/**
		  * Implicit constructor from TPoint2D. Zeroes the phi coordinate.
		  * \sa TPoint2D
		  */
		TPose2D(const TPoint2D &p);
		/**
		  * Constructor from TPoint3D, losing information. Zeroes the phi coordinate.
		  * \sa TPoint3D
		  */
		explicit TPose2D(const TPoint3D &p);
		/**
		  * Constructor from TPose3D, losing information. The phi corresponds to the original pose's yaw.
		  * \sa TPose3D
		  */
		explicit TPose2D(const TPose3D &p);
		/**
		  * Implicit constructor from heavyweight type.
		  * \sa mrpt::poses::CPose2D
		  */
		TPose2D(const mrpt::poses::CPose2D &p);
		/**
		  * Constructor from coordinates.
		  */
		inline TPose2D(double xx,double yy,double pphi):x(xx),y(yy),phi(pphi)	{}
		/**
		  * Default fast constructor. Initializes to garbage.
		  */
		inline TPose2D()	{}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline double &operator[](size_t i)	{
			return (&x)[i];
		}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline const double &operator[](size_t i) const	{
			return (&x)[i];
		}
		/**
		  * Transformation into vector.
		  */
		inline void getAsVector(vector_double &v) const	{
			v.resize(3);
			v[0]=x; v[1]=y; v[2]=phi;
		}
		 /** Returns a human-readable textual representation of the object (eg: "[x y yaw]", yaw in degrees)
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f]",x,y,RAD2DEG(phi)); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -45.0]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s);
		 static size_t size() { return 3; }
	};

	/** Lightweight 3D point (float version).
	  * \sa mrpt::poses::CPoint3D, mrpt::math::TPoint3D
	  */
	struct BASE_IMPEXP TPoint3Df
	{
		float x;
		float y;
		float z;

		inline TPoint3Df() { }
		inline TPoint3Df(const float xx,const float yy,const float zz) : x(xx), y(yy),z(zz) { }
		inline TPoint3Df & operator +=(const TPoint3Df &p) { x+=p.x; y+=p.y; z+=p.z; return *this; }
		inline TPoint3Df   operator *(const float s) { return TPoint3Df(x*s,y*s,z*s); }
	};

	/**
	  * Lightweight 3D point. Allows coordinate access using [] operator.
	  * \sa mrpt::poses::CPoint3D, mrpt::math::TPoint3Df
	  */
	struct BASE_IMPEXP TPoint3D	{
		double x; //!< X coordinate
		double y; //!< Y coordinate
		double z; //!< Z coordinate

		/** Constructor from coordinates.  */
		inline TPoint3D(double xx,double yy,double zz):x(xx),y(yy),z(zz)	{}
		/** Default fast constructor. Initializes to garbage. */
		inline TPoint3D()	{}
		/** Explicit constructor from coordinates.  */
		explicit inline TPoint3D(const TPoint3Df &p):x(p.x),y(p.y),z(p.z) {}

		/** Implicit constructor from TPoint2D. Zeroes the z.
		  * \sa TPoint2D
		  */
		TPoint3D(const TPoint2D &p);
		/**
		  * Constructor from TPose2D, losing information. Zeroes the z.
		  * \sa TPose2D
		  */
		explicit TPoint3D(const TPose2D &p);
		/**
		  * Constructor from TPose3D, losing information.
		  * \sa TPose3D
		  */
		explicit TPoint3D(const TPose3D &p);
		/**
		  * Implicit constructor from heavyweight type.
		  * \sa mrpt::poses::CPoint3D
		  */
		TPoint3D(const mrpt::poses::CPoint3D &p);
		/**
		  * Constructor from heavyweight 3D pose.
		  * \sa mrpt::poses::CPose3D.
		  */
		explicit TPoint3D(const mrpt::poses::CPose3D &p);
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline double &operator[](size_t i)	{
			return (&x)[i];
		}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline const double &operator[](size_t i) const	{
			return (&x)[i];
		}
		/**
		  * Point-to-point distance.
		  */
		inline double distanceTo(const TPoint3D &p) const	{
			return sqrt(square(p.x-x)+square(p.y-y)+square(p.z-z));
		}
		/**
		  * Point-to-point distance, squared.
		  */
		inline double sqrDistanceTo(const TPoint3D &p) const	{
			return square(p.x-x)+square(p.y-y)+square(p.z-z);
		}
		/**
		  * Point norm.
		  */
		inline double norm() const	{
			return sqrt(square(x)+square(y)+square(z));
		}
		/**
		  * Point scale.
		  */
		inline TPoint3D &operator*=(const double f) {
			x*=f;y*=f;z*=f;
			return *this;
		}
		/**
		  * Transformation into vector.
		  */
		void getAsVector(vector_double &v) const {
			v.resize(3);
			v[0]=x; v[1]=y; v[2]=z;
		}
		/**
		  * Translation.
		  */
		inline TPoint3D &operator+=(const TPoint3D &p)	{
			x+=p.x;
			y+=p.y;
			z+=p.z;
			return *this;
		}
		/**
		  * Difference between points.
		  */
		inline TPoint3D &operator-=(const TPoint3D &p)	{
			x-=p.x;
			y-=p.y;
			z-=p.z;
			return *this;
		}
		/**
		  * Points addition.
		  */
		inline TPoint3D operator+(const TPoint3D &p) const	{
			return TPoint3D(x+p.x,y+p.y,z+p.z);
		}
		/**
		  * Points substraction.
		  */
		inline TPoint3D operator-(const TPoint3D &p) const	{
			return TPoint3D(x-p.x,y-p.y,z-p.z);
		}

		inline TPoint3D operator*(double d) const	{
			return TPoint3D(x*d,y*d,z*d);
		}

		inline TPoint3D operator/(double d) const	{
			return TPoint3D(x/d,y/d,z/d);
		}

		bool operator<(const TPoint3D &p) const;

		 /** Returns a human-readable textual representation of the object (eg: "[0.02 1.04 -0.8]" )
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f]",x,y,z); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s);
		 static size_t size() { return 3; }
	};

	/**
	  * Lightweight 3D pose (three spatial coordinates, plus three angular coordinates). Allows coordinate access using [] operator.
	  * \sa mrpt::poses::CPose3D
	  */
	struct BASE_IMPEXP TPose3D	{
		/**
		  * X coordinate.
		  */
		double x;
		/**
		  * Y coordinate.
		  */
		double y;
		/**
		  * Z coordinate.
		  */
		double z;
		/**
		  * Yaw coordinate (rotation angle over Z axis).
		  */
		double yaw;
		/**
		  * Pitch coordinate (rotation angle over Y axis).
		  */
		double pitch;
		/**
		  * Roll coordinate (rotation angle over X coordinate).
		  */
		double roll;
		/**
		  * Implicit constructor from TPoint2D. Zeroes all the unprovided information.
		  * \sa TPoint2D
		  */
		TPose3D(const TPoint2D &p);
		/**
		  * Implicit constructor from TPose2D. Gets the yaw from the 2D pose's phi, zeroing all the unprovided information.
		  * \sa TPose2D
		  */
		TPose3D(const TPose2D &p);
		/**
		  * Implicit constructor from TPoint3D. Zeroes angular information.
		  * \sa TPoint3D
		  */
		TPose3D(const TPoint3D &p);
		/**
		  * Implicit constructor from heavyweight type.
		  * \sa mrpt::poses::CPose3D
		  */
		TPose3D(const mrpt::poses::CPose3D &p);
		/**
		  * Constructor from coordinates.
		  */
		TPose3D(double _x,double _y,double _z,double _yaw,double _pitch,double _roll):x(_x),y(_y),z(_z),yaw(_yaw),pitch(_pitch),roll(_roll)	{}
		/**
		  * Default fast constructor. Initializes to garbage.
		  */
		inline TPose3D()	{}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline double &operator[](size_t i)	{
			return (&x)[i];
		}
		/**
		  * Unsafe coordinate access using operator[]. Intended for loops.
		  */
		inline const double &operator[](size_t i) const	{
			return (&x)[i];
		}
		/**
		  * Pose's spatial coordinates norm.
		  */
		double norm() const {
			return sqrt(square(x)+square(y)+square(z));
		}
		/**
		  * Gets the pose as a vector of doubles.
		  */
		void getAsVector(vector_double &v) const {
			v.resize(6);
			v[0]=x; v[1]=y; v[2]=z; v[3]=yaw; v[4]=pitch; v[5]=roll;
		}
		 /** Returns a human-readable textual representation of the object (eg: "[x y z yaw pitch roll]", angles in degrees.)
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f %f %f %f]",x,y,z,RAD2DEG(yaw),RAD2DEG(pitch),RAD2DEG(roll)); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s);
		 static size_t size() { return 6; }
	};

	/** Lightweight 3D pose (three spatial coordinates, plus a quaternion ). Allows coordinate access using [] operator.
	  * \sa mrpt::poses::CPose3DQuat
	  */
	struct BASE_IMPEXP TPose3DQuat	{
		double x;	//!< Translation in x
		double y;	//!< Translation in y
		double z;	//!< Translation in z
		double qr;  //!< Quaternion part, r
		double qx;  //!< Quaternion part, x
		double qy;  //!< Quaternion part, y
		double qz;  //!< Quaternion part, z

		/** Constructor from coordinates. */
		inline TPose3DQuat(double _x,double _y,double _z,double _qr,double _qx, double _qy, double _qz):x(_x),y(_y),z(_z),qr(_qr),qx(_qx),qy(_qy),qz(_qz) {  }
		/** Default fast constructor. Initializes to garbage. */
		inline TPose3DQuat()	{}
		/** Constructor from a CPose3DQuat */
		TPose3DQuat(const mrpt::poses::CPose3DQuat &p);

		/** Unsafe coordinate access using operator[]. Intended for loops. */
		inline double &operator[](size_t i)	{
			return (&x)[i];
		}
		/** Unsafe coordinate access using operator[]. Intended for loops. */
		inline const double &operator[](size_t i) const	{
			return (&x)[i];
		}
		/** Pose's spatial coordinates norm. */
		double norm() const {
			return sqrt(square(x)+square(y)+square(z));
		}
		/** Gets the pose as a vector of doubles. */
		void getAsVector(vector_double &v) const {
			v.resize(7);
			for (size_t i=0;i<7;i++) v[i]=(*this)[i];
		}
		 /** Returns a human-readable textual representation of the object as "[x y z qr qx qy qz]"
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f %f %f %f %f]",x,y,z,qr,qx,qy,qz); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8 1.0 0.0 0.0 0.0]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s);
		 static size_t size() { return 7; }
	};
#pragma pack(pop)

	// Text streaming functions:
	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const TPoint2D & p);
	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const TPoint3D & p);
	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const TPose2D & p);
	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const TPose3D & p);
	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const TPose3DQuat & p);


	/**
	  * Unary minus operator for 3D points.
	  */
	inline TPoint3D operator-(const TPoint3D &p1)	{
		return TPoint3D(-p1.x,-p1.y,-p1.z);
	}
	/**
	  * Exact comparison between 2D points.
	  */
	inline bool operator==(const TPoint2D &p1,const TPoint2D &p2)	{
		return (p1.x==p2.x)&&(p1.y==p2.y);
	}
	/**
	  * Exact comparison between 2D points.
	  */
	inline bool operator!=(const TPoint2D &p1,const TPoint2D &p2)	{
		return (p1.x!=p2.x)||(p1.y!=p2.y);
	}
	/**
	  * Exact comparison between 3D points.
	  */
	inline bool operator==(const TPoint3D &p1,const TPoint3D &p2)	{
		return (p1.x==p2.x)&&(p1.y==p2.y)&&(p1.z==p2.z);
	}
	/**
	  * Exact comparison between 3D points.
	  */
	inline bool operator!=(const TPoint3D &p1,const TPoint3D &p2)	{
		return (p1.x!=p2.x)||(p1.y!=p2.y)||(p1.z!=p2.z);
	}
	/**
	  * Exact comparison between 2D poses, taking possible cycles into account.
	  */
	inline bool operator==(const TPose2D &p1,const TPose2D &p2)	{
		return (p1.x==p2.x)&&(p1.y==p2.y)&&(mrpt::math::wrapTo2Pi(p1.phi)==mrpt::math::wrapTo2Pi(p2.phi));
	}
	/**
	  * Exact comparison between 2D poses, taking possible cycles into account.
	  */
	inline bool operator!=(const TPose2D &p1,const TPose2D &p2)	{
		return (p1.x!=p2.x)||(p1.y!=p2.y)||(mrpt::math::wrapTo2Pi(p1.phi)!=mrpt::math::wrapTo2Pi(p2.phi));
	}
	/**
	  * Exact comparison between 3D poses, taking possible cycles into account.
	  */
	inline bool operator==(const TPose3D &p1,const TPose3D &p2)	{
		return (p1.x==p2.x)&&(p1.y==p2.y)&&(p1.z==p2.z)&&(mrpt::math::wrapTo2Pi(p1.yaw)==mrpt::math::wrapTo2Pi(p2.yaw))&&(mrpt::math::wrapTo2Pi(p1.pitch)==mrpt::math::wrapTo2Pi(p2.pitch))&&(mrpt::math::wrapTo2Pi(p1.roll)==mrpt::math::wrapTo2Pi(p2.roll));
	}
	/**
	  * Exact comparison between 3D poses, taking possible cycles into account.
	  */
	inline bool operator!=(const TPose3D &p1,const TPose3D &p2)	{
		return (p1.x!=p2.x)||(p1.y!=p2.y)||(p1.z!=p2.z)||(mrpt::math::wrapTo2Pi(p1.yaw)!=mrpt::math::wrapTo2Pi(p2.yaw))||(mrpt::math::wrapTo2Pi(p1.pitch)!=mrpt::math::wrapTo2Pi(p2.pitch))||(mrpt::math::wrapTo2Pi(p1.roll)!=mrpt::math::wrapTo2Pi(p2.roll));
	}
	//Forward declarations
	struct BASE_IMPEXP TSegment3D;
	struct BASE_IMPEXP TLine3D;
	class BASE_IMPEXP TPolygon3D;
	struct BASE_IMPEXP TObject3D;

	//Pragma defined to ensure no structure packing
#pragma pack(push,1)
	/**
	  * 2D segment, consisting of two points.
	  * \sa TSegment3D,TLine2D,TPolygon2D,TPoint2D
	  */
	struct BASE_IMPEXP TSegment2D	{
	public:
		/**
		  * Origin point.
		  */
		TPoint2D point1;
		/**
		  * Destiny point.
		  */
		TPoint2D point2;
		/**
		  * Segment length.
		  */
		double length() const;
		/**
		  * Distance to point.
		  */
		double distance(const TPoint2D &point) const;
		/**
		  * Distance with sign to point (sign indicates which side the point is).
		  */
		double signedDistance(const TPoint2D &point) const;
		/**
		  * Check whether a point is inside a segment.
		  */
		bool contains(const TPoint2D &point) const;
		/**
		  * Unsafe point access using [] operator, intended for loops.
		  */
		inline TPoint2D &operator[](size_t i)	{
			return (&point1)[i];
		}
		/**
		  * Unsafe point access using [] operator, intended for loops.
		  */
		inline const TPoint2D &operator[](size_t i) const	{
			return (&point1)[i];
		}
		/**
		  * Project into 3D space, setting the z to 0.
		  */
		void generate3DObject(TSegment3D &s) const;
		/**
		  * Segment's central point.
		  */
		inline void getCenter(TPoint2D &p) const	{
			p.x=(point1.x+point2.x)/2;
			p.y=(point1.y+point2.y)/2;
		}
		/**
		  * Constructor from both points.
		  */
		TSegment2D(const TPoint2D &p1,const TPoint2D &p2):point1(p1),point2(p2)	{}
		/**
		  * Fast default constructor. Initializes to garbage.
		  */
		TSegment2D()	{}
		/**
		  * Explicit constructor from 3D object, discarding the z.
		  */
		explicit TSegment2D(const TSegment3D &s);

		bool operator<(const TSegment2D &s) const;
	};
	/**
	  * 3D segment, consisting of two points.
	  * \sa TSegment2D,TLine3D,TPlane,TPolygon3D,TPoint3D
	  */
	struct BASE_IMPEXP TSegment3D	{
	public:
		/**
		  * Origin point.
		  */
		TPoint3D point1;
		/**
		  * Destiny point.
		  */
		TPoint3D point2;
		/**
		  * Segment length.
		  */
		double length() const;
		/**
		  * Distance to point.
		  */
		double distance(const TPoint3D &point) const;
		/**
		  * Check whether a point is inside the segment.
		  */
		bool contains(const TPoint3D &point) const;
		/**
		  * Unsafe point access using [] operator, intended for loops.
		  */
		inline TPoint3D &operator[](size_t i)	{
			return (&point1)[i];
		}
		/**
		  * Unsafe point access using [] operator, intended for loops.
		  */
		inline const TPoint3D &operator[](size_t i) const	{
			return (&point1)[i];
		}
		/**
		  * Projection into 2D space, discarding the z.
		  */
		inline void generate2DObject(TSegment2D &s) const	{
			s=TSegment2D(*this);
		}
		/**
		  * Segment's central point.
		  */
		inline void getCenter(TPoint3D &p) const	{
			p.x=(point1.x+point2.x)/2;
			p.y=(point1.y+point2.y)/2;
			p.z=(point1.z+point2.z)/2;
		}
		/**
		  * Constructor from both points.
		  */
		TSegment3D(const TPoint3D &p1,const TPoint3D &p2):point1(p1),point2(p2)	{}
		/**
		  * Fast default constructor. Initializes to garbage.
		  */
		TSegment3D()	{}
		/**
		  * Constructor from 2D object. Sets the z to zero.
		  */
		TSegment3D(const TSegment2D &s):point1(s.point1),point2(s.point2)	{}

		bool operator<(const TSegment3D &s) const;
	};
#pragma pack(pop)

	inline bool operator==(const TSegment2D &s1,const TSegment2D &s2)	{
		return (s1.point1==s2.point1)&&(s1.point2==s2.point2);
	}

	inline bool operator!=(const TSegment2D &s1,const TSegment2D &s2)	{
		return (s1.point1!=s1.point1)||(s1.point2!=s2.point2);
	}

	inline bool operator==(const TSegment3D &s1,const TSegment3D &s2)	{
		return (s1.point1==s2.point1)&&(s1.point2==s2.point2);
	}

	inline bool operator!=(const TSegment3D &s1,const TSegment3D &s2)	{
		return (s1.point1!=s1.point1)||(s1.point2!=s2.point2);
	}

	/**
	  * 2D line without bounds, represented by its equation \f$Ax+By+C=0\f$.
	  * \sa TLine3D,TSegment2D,TPolygon2D,TPoint2D
	  */
	struct BASE_IMPEXP TLine2D	{
	public:
		/**
		  * Line coefficients, stored as an array: \f$\left[A,B,C\right]\f$.
		  */
		double coefs[3];
		/**
		  * Evaluate point in the line's equation.
		  */
		double evaluatePoint(const TPoint2D &point) const;
		/**
		  * Check whether a point is inside the line.
		  */
		bool contains(const TPoint2D &point) const;
		/**
		  * Distance from a given point.
		  */
		double distance(const TPoint2D &point) const;
		/**
		  * Distance with sign from a given point (sign indicates side).
		  */
		double signedDistance(const TPoint2D &point) const;
		/**
		  * Get line's normal vector.
		  */
		void getNormalVector(double (&vector)[2]) const;
		/**
		  * Unitarize line's normal vector.
		  */
		void unitarize();
		/**
		  * Get line's normal vector after unitarizing line.
		  */
		inline void getUnitaryNormalVector(double (&vector)[2])	{
			unitarize();
			getNormalVector(vector);
		}
		/**
		  * Get line's director vector.
		  */
		void getDirectorVector(double (&vector)[2]) const;
		/**
		  * Unitarize line and then get director vector.
		  */
		inline void getUnitaryDirectorVector(double (&vector)[2])	{
			unitarize();
			getDirectorVector(vector);
		}
		/**
		  * Project into 3D space, setting the z to 0.
		  */
		void generate3DObject(TLine3D &l) const;
		/**
		  * Get a pose2D whose X axis corresponds to the line.
		  * \sa mrpt::poses::CPose2D.
		  */
		void getAsPose2D(mrpt::poses::CPose2D &outPose) const;
		/**
		  * Get a pose2D whose X axis corresponds to the line, forcing the base point to one given.
		  * \throw logic_error if the point is not inside the line.
		  * \sa mrpt::poses::CPose2D.
		  */
		void getAsPose2DForcingOrigin(const TPoint2D &origin,mrpt::poses::CPose2D &outPose) const;
		/**
		  * Constructor from two points, through which the line will pass.
		  * \throw logic_error if both points are the same
		  */
		TLine2D(const TPoint2D &p1,const TPoint2D &p2) throw(std::logic_error);
		/**
		  * Constructor from a segment.
		  */
		explicit TLine2D(const TSegment2D &s);
		/**
		  * Fast default constructor. Initializes to garbage.
		  */
		TLine2D()	{}
		/**
		  * Constructor from line's coefficients.
		  */
		inline TLine2D(double A,double B,double C)	{
			coefs[0]=A;
			coefs[1]=B;
			coefs[2]=C;
		}
		/**
		  * Construction from 3D object, discarding the Z.
		  * \throw std::logic_error if the line is normal to the XY plane.
		  */
		explicit TLine2D(const TLine3D &l);
	};

	/**
	  * 3D line, represented by a base point and a director vector.
	  * \sa TLine2D,TSegment3D,TPlane,TPolygon3D,TPoint3D
	  */
	struct BASE_IMPEXP TLine3D	{
	public:
		/**
		  * Base point.
		  */
		TPoint3D pBase;
		/**
		  * Director vector.
		  */
		double director[3];
		/**
		  * Check whether a point is inside the line.
		  */
		bool contains(const TPoint3D &point) const;
		/**
		  * Distance between the line and a point.
		  */
		double distance(const TPoint3D &point) const;
		/**
		  * Unitarize director vector.
		  */
		void unitarize();
		/**
		  * Get director vector.
		  */
		inline void getDirectorVector(double (&vector)[3]) const	{
			for (size_t i=0;i<3;i++) vector[i]=director[i];
		}
		/**
		  * Unitarize and then get director vector.
		  */
		inline void getUnitaryDirectorVector(double (&vector)[3])	{
			unitarize();
			getDirectorVector(vector);
		}
		/**
		  * Project into 2D space, discarding the Z coordinate.
		  * \throw std::logic_error if the line's director vector is orthogonal to the XY plane.
		  */
		inline void generate2DObject(TLine2D &l) const	{
			l=TLine2D(*this);
		}
		/**
		  * Constructor from two points, through which the line will pass.
		  * \throw std::logic_error if both points are the same.
		  */
		TLine3D(const TPoint3D &p1,const TPoint3D &p2) throw(std::logic_error);
		/**
		  * Constructor from 3D segment.
		  */
		explicit TLine3D(const TSegment3D &s);
		/**
		  * Fast default constructor. Initializes to garbage.
		  */
		TLine3D()	{}
		/**
		  * Implicit constructor from 2D object. Zeroes the z.
		  */
		TLine3D(const TLine2D &l);
	};

	/**
	  * 3D Plane, represented by its equation \f$Ax+By+Cz+D=0\f$
	  * \sa TSegment3D,TLine3D,TPolygon3D,TPoint3D
	  */
	struct BASE_IMPEXP TPlane	{
	public:
		/**
		  * Plane coefficients, stored as an array: \f$\left[A,B,C,D\right]\f$
		  */
		double coefs[4];
		/**
		  * Evaluate a point in the plane's equation.
		  */
		double evaluatePoint(const TPoint3D &point) const;
		/**
		  * Check whether a point is contained into the plane.
		  */
		bool contains(const TPoint3D &point) const;
		/**
		  * Check whether a segment is fully contained into the plane.
		  */
		inline bool contains(const TSegment3D &segment) const	{
			return contains(segment.point1)&&contains(segment.point2);
		}
		/**
		  * Check whether a line is fully contained into the plane.
		  */
		bool contains(const TLine3D &line) const;
		/**
		  * Distance to 3D point.
		  */
		double distance(const TPoint3D &point) const;
		/**
		  * Distance to 3D line. Will be zero if the line is not parallel to the plane.
		  */
		double distance(const TLine3D &line) const;
		/**
		  * Get plane's normal vector.
		  */
		void getNormalVector(double (&vec)[3]) const;
		/**
		  * Unitarize normal vector.
		  */
		void unitarize();
		/**
		  * Unitarize, then get normal vector.
		  */
		inline void getUnitaryNormalVector(double (&vec)[3])	{
			unitarize();
			getNormalVector(vec);
		}
		/**
		  * Gets a pose whose XY plane corresponds to this plane.
		  */
		void getAsPose3D(mrpt::poses::CPose3D &outPose);
		/**
		  * Gets a pose whose XY plane corresponds to this plane.
		  */
		inline void getAsPose3D(mrpt::poses::CPose3D &outPose) const	{
			TPlane p=*this;
			p.getAsPose3D(outPose);
		}
		/**
		  * Gets a pose whose XY plane corresponds to this, forcing an exact point as its spatial coordinates.
		  * \throw std::logic_error if the point is not inside the plane.
		  */
		void getAsPose3DForcingOrigin(const TPoint3D &newOrigin,mrpt::poses::CPose3D &pose);
		/**
		  * Gets a pose whose XY plane corresponds to this, forcing an exact point as its spatial coordinates.
		  * \throw std::logic_error if the point is not inside the plane.
		  */
		inline void getAsPose3DForcingOrigin(const TPoint3D &newOrigin,mrpt::poses::CPose3D &pose) const	{
			TPlane p=*this;
			p.getAsPose3DForcingOrigin(newOrigin,pose);
		}
		/**
		  * Gets a plane which contains these three points.
		  * \throw std::logic_error if the points are linearly dependants.
		  */
		TPlane(const TPoint3D &p1,const TPoint3D &p2,const TPoint3D &p3) throw(std::logic_error);
		/**
		  * Gets a plane which contains this point and this line.
		  * \throw std::logic_error if the point is inside the line.
		  */
		TPlane(const TPoint3D &p1,const TLine3D &r2) throw(std::logic_error);
		/**
		  * Gets a plane which contains the two lines.
		  * \throw std::logic_error if the lines do not cross.
		  */
		TPlane(const TLine3D &r1,const TLine3D &r2) throw(std::logic_error);
		/**
		  * Fast default constructor. Initializes to garbage.
		  */
		TPlane()	{}
		/**
		  * Constructor from plane coefficients.
		  */
		inline TPlane(double A,double B,double C,double D)	{
			coefs[0]=A;
			coefs[1]=B;
			coefs[2]=C;
			coefs[3]=D;
		}
		/**
		  * Constructor from an array of coefficients.
		  */
		inline TPlane(const double (&vec)[4])	{
			for (size_t i=0;i<4;i++) coefs[i]=vec[i];
		}
	};

	typedef TPlane TPlane3D;

	/**
	  * 2D polygon, inheriting from std::vector<TPoint2D>.
	  * \sa TPolygon3D,TSegment2D,TLine2D,TPoint2D, CPolygon
	  */
	class BASE_IMPEXP TPolygon2D:public std::vector<TPoint2D>	{
	public:
		/**
		  * Distance to a point.
		  */
		double distance(const TPoint2D &point) const;
		/**
		  * Check whether a point is inside the polygon.
		  */
		bool contains(const TPoint2D &point) const;
		/**
		  * Gets as set of segments, instead of points.
		  */
		void getAsSegmentList(std::vector<TSegment2D> &v) const;
		/**
		  * Projects into 3D space, zeroing the z.
		  */
		void generate3DObject(TPolygon3D &p) const;
		/**
		  * Polygon's central point.
		  */
		void getCenter(TPoint2D &p) const;
		/**
		  * Checks whether is convex.
		  */
		bool isConvex() const;
		/**
		  * Erase repeated vertices.
		  * \sa removeRedundantVertices
		  */
		void removeRepeatedVertices();
		/**
		  * Erase every redundant vertex from the polygon, saving space.
		  * \sa removeRepeatedVertices
		  */
		void removeRedundantVertices();
		/**
		  * Gets plot data, ready to use on a 2D plot.
		  * \sa mrpt::gui::CDisplayWindowPlots
		  */
		void getPlotData(std::vector<double> &x,std::vector<double> &y) const;
		/**
		  * Default constructor.
		  */
		TPolygon2D():std::vector<TPoint2D>()	{}
		/**
		  * Constructor for a given number of vertices, intializing them as garbage.
		  */
		explicit TPolygon2D(size_t N):std::vector<TPoint2D>(N)	{}
		/**
		  * Implicit constructor from a vector of 2D points.
		  */
		TPolygon2D(const std::vector<TPoint2D> &v):std::vector<TPoint2D>(v)	{}
		/**
		  * Constructor from a 3D object.
		  */
		explicit TPolygon2D(const TPolygon3D &p);
		/**
		  * Static method to create a regular polygon, given its size and radius.
		  * \throw std::logic_error if radius is near zero or the number of edges is less than three.
		  */
		static void createRegularPolygon(size_t numEdges,double radius,TPolygon2D &poly);
		/**
		  * Static method to create a regular polygon from its size and radius. The center will correspond to the given pose.
		  * \throw std::logic_error if radius is near zero or the number of edges is less than three.
		  */
		static inline void createRegularPolygon(size_t numEdges,double radius,TPolygon2D &poly,const mrpt::poses::CPose2D &pose);
	};

	/**
	  * 3D polygon, inheriting from std::vector<TPoint3D>
	  * \sa TPolygon2D,TSegment3D,TLine3D,TPlane,TPoint3D
	  */
	class BASE_IMPEXP TPolygon3D:public std::vector<TPoint3D>	{
	public:
		/**
		  * Distance to point.
		  */
		double distance(const TPoint3D &point) const;
		/**
		  * Check whether a point is inside the polygon.
		  */
		bool contains(const TPoint3D &point) const;
		/**
		  * Gets as set of segments, instead of set of points.
		  */
		void getAsSegmentList(std::vector<TSegment3D> &v) const;
		/**
		  * Gets a plane which contains the polygon. Returns false if the polygon is skew and cannot be fit inside a plane.
		  */
		bool getPlane(TPlane &p) const;
		/**
		  * Gets the best fitting plane, disregarding whether the polygon actually fits inside or not.
		  * \sa getBestFittingPlane
		  */
		void getBestFittingPlane(TPlane &p) const;
		/**
		  * Projects into a 2D space, discarding the z.
		  * \get getPlane,isSkew
		  */
		inline void generate2DObject(TPolygon2D &p) const	{
			p=TPolygon2D(*this);
		}
		/**
		  * Get polygon's central point.
		  */
		void getCenter(TPoint3D &p) const;
		/**
		  * Check whether the polygon is skew. Returns true if there doesn't exist a plane in which the polygon can fit.
		  * \sa getBestFittingPlane
		  */
		bool isSkew() const;
		/**
		  * Remove polygon's repeated vertices.
		  */
		void removeRepeatedVertices();
		/**
		  * Erase every redundant vertex, thus saving space.
		  */
		void removeRedundantVertices();
		/**
		  * Default constructor. Creates a polygon with no vertices.
		  */
		TPolygon3D():std::vector<TPoint3D>()	{}
		/**
		  * Constructor for a given size. Creates a polygon with a fixed number of vertices, which are initialized to garbage.
		  */
		explicit TPolygon3D(size_t N):std::vector<TPoint3D>(N)	{}
		/**
		  * Implicit constructor from a 3D points vector.
		  */
		TPolygon3D(const std::vector<TPoint3D> &v):std::vector<TPoint3D>(v)	{}
		/**
		  * Constructor from a 2D object. Zeroes the z.
		  */
		TPolygon3D(const TPolygon2D &p);
		/**
		  * Static method to create a regular polygon, given its size and radius.
		  * \throw std::logic_error if number of edges is less than three, or radius is near zero.
		  */
		static void createRegularPolygon(size_t numEdges,double radius,TPolygon3D &poly);
		/**
		  * Static method to create a regular polygon, given its size and radius. The center will be located on the given pose.
		  * \throw std::logic_error if number of edges is less than three, or radius is near zero.
		  */
		static inline void createRegularPolygon(size_t numEdges,double radius,TPolygon3D &poly,const mrpt::poses::CPose3D &pose);
	};

	/**
	  * Object type identifier for TPoint2D or TPoint3D.
	  * \sa TObject2D,TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_POINT=0;
	/**
	  * Object type identifier for TSegment2D or TSegment3D.
	  * \sa TObject2D,TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_SEGMENT=1;
	/**
	  * Object type identifier for TLine2D or TLine3D.
	  * \sa TObject2D,TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_LINE=2;
	/**
	  * Object type identifier for TPolygon2D or TPolygon3D.
	  * \sa TObject2D,TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_POLYGON=3;
	/**
	  * Object type identifier for TPlane.
	  * \sa TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_PLANE=4;
	/**
	  * Object type identifier for empty TObject2D or TObject3D.
	  * \sa TObject2D,TObject3D
	  */
	const unsigned char GEOMETRIC_TYPE_UNDEFINED=255;

	/**
	  * Standard type for storing any lightweight 2D type. Do not inherit from this class.
	  * \sa TPoint2D,TSegment2D,TLine2D,TPolygon2D
	  */
#ifdef TOBJECTS_USE_UNIONS
	struct BASE_IMPEXP TObject2D	{
	private:
		/**
		  * Object type identifier.
		  */
		unsigned char type;
		/**
		  * Union type storing pointers to every allowed type.
		  */
		union	{
			TPoint2D *point;
			TSegment2D *segment;
			TLine2D *line;
			TPolygon2D *polygon;
		}	data;
		/**
		  * Destroys the object, releasing the pointer to the content (if any).
		  */
		void destroy()	{
			switch(type)	{
				case GEOMETRIC_TYPE_POINT:
					delete data.point;
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					delete data.segment;
					break;
				case GEOMETRIC_TYPE_LINE:
					delete data.line;
					break;
				case GEOMETRIC_TYPE_POLYGON:
					delete data.polygon;
					break;
			}
			type=GEOMETRIC_TYPE_UNDEFINED;
		}
	public:
		/**
		  * Implicit constructor from point.
		  */
		TObject2D(const TPoint2D &p):type(GEOMETRIC_TYPE_POINT)	{
			data.point=new TPoint2D(p);
		}
		/**
		  * Implicit constructor from segment.
		  */
		TObject2D(const TSegment2D &s):type(GEOMETRIC_TYPE_SEGMENT)	{
			data.segment=new TSegment2D(s);
		}
		/**
		  * Implicit constructor from line.
		  */
		TObject2D(const TLine2D &r):type(GEOMETRIC_TYPE_LINE)	{
			data.line=new TLine2D(r);
		}
		/**
		  * Implicit constructor from polygon.
		  */
		TObject2D(const TPolygon2D &p):type(GEOMETRIC_TYPE_POLYGON)	{
			data.polygon=new TPolygon2D(p);
		}
		/**
		  * Implicit constructor from polygon.
		  */
		TObject2D():type(GEOMETRIC_TYPE_UNDEFINED)	{}
		/**
		  * Object destruction.
		  */
		~TObject2D()	{
			destroy();
		}
		/**
		  * Checks whether content is a point.
		  */
		inline bool isPoint() const	{
			return type==GEOMETRIC_TYPE_POINT;
		}
		/**
		  * Checks whether content is a segment.
		  */
		inline bool isSegment() const	{
			return type==GEOMETRIC_TYPE_SEGMENT;
		}
		/**
		  * Checks whether content is a line.
		  */
		inline bool isLine() const	{
			return type==GEOMETRIC_TYPE_LINE;
		}
		/**
		  * Checks whether content is a polygon.
		  */
		inline bool isPolygon() const	{
			return type==GEOMETRIC_TYPE_POLYGON;
		}
		/**
		  * Gets content type.
		  */
		inline unsigned char getType() const	{
			return type;
		}
		/**
		  * Gets the content as a point, returning false if the type is inadequate.
		  */
		inline bool getPoint(TPoint2D &p) const	{
			if (isPoint())	{
				p=*(data.point);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a segment, returning false if the type is inadequate.
		  */
		inline bool getSegment(TSegment2D &s) const	{
			if (isSegment())	{
				s=*(data.segment);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a line, returning false if the type is inadequate.
		  */
		inline bool getLine(TLine2D &r) const	{
			if (isLine())	{
				r=*(data.line);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a polygon, returning false if the type is inadequate.
		  */
		inline bool getPolygon(TPolygon2D &p) const	{
			if (isPolygon())	{
				p=*(data.polygon);
				return true;
			}	else return false;
		}
		/**
		  * Assign another TObject2D. Pointers are not shared.
		  */
		void operator=(const TObject2D &obj)	{
			if (this==&obj) return;
			destroy();
			switch (type=obj.type)	{
				case GEOMETRIC_TYPE_POINT:
					data.point=new TPoint2D(*(obj.data.point));
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					data.segment=new TSegment2D(*(obj.data.segment));
					break;
				case GEOMETRIC_TYPE_LINE:
					data.line=new TLine2D(*(obj.data.line));
					break;
				case GEOMETRIC_TYPE_POLYGON:
					data.polygon=new TPolygon2D(*(obj.data.polygon));
					break;
			}
		}
		/**
		  * Assign a point to this object.
		  */
		inline void operator=(const TPoint2D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POINT;
			data.point=new TPoint2D(p);
		}
		/**
		  * Assign a segment to this object.
		  */
		inline void operator=(const TSegment2D &s)	{
			destroy();
			type=GEOMETRIC_TYPE_SEGMENT;
			data.segment=new TSegment2D(s);
		}
		/**
		  * Assign a line to this object.
		  */
		inline void operator=(const TLine2D &l)	{
			destroy();
			type=GEOMETRIC_TYPE_LINE;
			data.line=new TLine2D(l);
		}
		/**
		  * Assign a polygon to this object.
		  */
		inline void operator=(const TPolygon2D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POLYGON;
			data.polygon=new TPolygon2D(p);
		}
		/**
		  * Project into 3D space.
		  */
		void generate3DObject(TObject3D &obj) const;
		/**
		  * Constructor from another TObject2D.
		  */
		TObject2D(const TObject2D &obj):type(GEOMETRIC_TYPE_UNDEFINED)	{
			operator=(obj);
		}
		/**
		  * Static method to retrieve all the points in a vector of TObject2D.
		  */
		static void getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts);
		/**
		  * Static method to retrieve all the segments in a vector of TObject2D.
		  */
		static void getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms);
		/**
		  * Static method to retrieve all the lines in a vector of TObject2D.
		  */
		static void getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins);
		/**
		  * Static method to retrieve all the polygons in a vector of TObject2D.
		  */
		static void getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys);
		/**
		  * Static method to retrieve all the points in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the segments in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the lines in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the polygons in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys,std::vector<TObject2D> &remainder);
	};
	/**
	  * Standard object for storing any 3D lightweight object. Do not inherit from this class.
	  * \sa TPoint3D,TSegment3D,TLine3D,TPlane,TPolygon3D
	  */
	struct BASE_IMPEXP TObject3D	{
	private:
		/**
		  * Object type identifier.
		  */
		unsigned char type;
		/**
		  * Union containing pointer to actual data.
		  */
		union	{
			TPoint3D *point;
			TSegment3D *segment;
			TLine3D *line;
			TPolygon3D *polygon;
			TPlane *plane;
		}	data;
		/**
		  * Destroys the object and releases the pointer, if any.
		  */
		void destroy()	{
			switch (type)	{
				case GEOMETRIC_TYPE_POINT:
					delete data.point;
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					delete data.segment;
					break;
				case GEOMETRIC_TYPE_LINE:
					delete data.line;
					break;
				case GEOMETRIC_TYPE_POLYGON:
					delete data.polygon;
					break;
				case GEOMETRIC_TYPE_PLANE:
					delete data.plane;
					break;
				case GEOMETRIC_TYPE_UNDEFINED:
					break;
				default:
					THROW_EXCEPTION("Invalid TObject2D object");
			}
			type=GEOMETRIC_TYPE_UNDEFINED;
		}
	public:
		/**
		  * Constructor from point.
		  */
		TObject3D(const TPoint3D &p):type(GEOMETRIC_TYPE_POINT)	{
			data.point=new TPoint3D(p);
		}
		/**
		  * Constructor from segment.
		  */
		TObject3D(const TSegment3D &s):type(GEOMETRIC_TYPE_SEGMENT)	{
			data.segment=new TSegment3D(s);
		}
		/**
		  * Constructor from line.
		  */
		TObject3D(const TLine3D &r):type(GEOMETRIC_TYPE_LINE)	{
			data.line=new TLine3D(r);
		}
		/**
		  * Constructor from polygon.
		  */
		TObject3D(const TPolygon3D &p):type(GEOMETRIC_TYPE_POLYGON)	{
			data.polygon=new TPolygon3D(p);
		}
		/**
		  * Constructor from plane.
		  */
		TObject3D(const TPlane &p):type(GEOMETRIC_TYPE_PLANE)	{
			data.plane=new TPlane(p);
		}
		/**
		  * Empty constructor.
		  */
		TObject3D():type(GEOMETRIC_TYPE_UNDEFINED)	{}
		/**
		  * Destructor.
		  */
		~TObject3D()	{
			destroy();
		}
		/**
		  * Checks whether content is a point.
		  */
		inline bool isPoint() const	{
			return type==GEOMETRIC_TYPE_POINT;
		}
		/**
		  * Checks whether content is a segment.
		  */
		inline bool isSegment() const	{
			return type==GEOMETRIC_TYPE_SEGMENT;
		}
		/**
		  * Checks whether content is a line.
		  */
		inline bool isLine() const	{
			return type==GEOMETRIC_TYPE_LINE;
		}
		/**
		  * Checks whether content is a polygon.
		  */
		inline bool isPolygon() const	{
			return type==GEOMETRIC_TYPE_POLYGON;
		}
		/**
		  * Checks whether content is a plane.
		  */
		inline bool isPlane() const	{
			return type==GEOMETRIC_TYPE_PLANE;
		}
		/**
		  * Gets object type.
		  */
		inline unsigned char getType() const	{
			return type;
		}
		/**
		  * Gets the content as a point, returning false if the type is not adequate.
		  */
		inline bool getPoint(TPoint3D &p) const	{
			if (isPoint())	{
				p=*(data.point);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a segment, returning false if the type is not adequate.
		  */
		inline bool getSegment(TSegment3D &s) const	{
			if (isSegment())	{
				s=*(data.segment);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a line, returning false if the type is not adequate.
		  */
		inline bool getLine(TLine3D &r) const	{
			if (isLine())	{
				r=*(data.line);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a polygon, returning false if the type is not adequate.
		  */
		inline bool getPolygon(TPolygon3D &p) const	{
			if (isPolygon())	{
				p=*(data.polygon);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a plane, returning false if the type is not adequate.
		  */
		inline bool getPlane(TPlane &p) const	{
			if (isPlane())	{
				p=*(data.plane);
				return true;
			}	else return false;
		}
		/**
		  * Assigns another object, creating a new pointer if needed.
		  */
		void operator=(const TObject3D &obj)	{
			if (this==&obj) return;
			destroy();
			switch (type=obj.type)	{
				case GEOMETRIC_TYPE_POINT:
					data.point=new TPoint3D(*(obj.data.point));
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					data.segment=new TSegment3D(*(obj.data.segment));
					break;
				case GEOMETRIC_TYPE_LINE:
					data.line=new TLine3D(*(obj.data.line));
					break;
				case GEOMETRIC_TYPE_POLYGON:
					data.polygon=new TPolygon3D(*(obj.data.polygon));
					break;
				case GEOMETRIC_TYPE_PLANE:
					data.plane=new TPlane(*(obj.data.plane));
					break;
				case GEOMETRIC_TYPE_UNDEFINED:
					break;
				default:
					THROW_EXCEPTION("Invalid TObject3D object");
			}
		}
		/**
		  * Assigns a point to this object.
		  */
		inline void operator=(const TPoint3D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POINT;
			data.point=new TPoint3D(p);
		}
		/**
		  * Assigns a segment to this object.
		  */
		inline void operator=(const TSegment3D &s)	{
			destroy();
			type=GEOMETRIC_TYPE_SEGMENT;
			data.segment=new TSegment3D(s);
		}
		/**
		  * Assigns a line to this object.
		  */
		inline void operator=(const TLine3D &l)	{
			destroy();
			type=GEOMETRIC_TYPE_LINE;
			data.line=new TLine3D(l);
		}
		/**
		  * Assigns a polygon to this object.
		  */
		inline void operator=(const TPolygon3D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POLYGON;
			data.polygon=new TPolygon3D(p);
		}
		/**
		  * Assigns a plane to this object.
		  */
		inline void operator=(const TPlane &p)	{
			destroy();
			type=GEOMETRIC_TYPE_PLANE;
			data.plane=new TPlane(p);
		}
		/**
		  * Projects into 2D space.
		  * \throw std::logic_error if the 3D object loses its properties when projecting into 2D space (for example, it's a plane or a vertical line).
		  */
		inline void generate2DObject(TObject2D &obj) const	{
			switch (type)	{
				case GEOMETRIC_TYPE_POINT:
					obj=TPoint2D(*(data.point));
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					obj=TSegment2D(*(data.segment));
					break;
				case GEOMETRIC_TYPE_LINE:
					obj=TLine2D(*(data.line));
					break;
				case GEOMETRIC_TYPE_POLYGON:
					obj=TPolygon2D(*(data.polygon));
					break;
				case GEOMETRIC_TYPE_PLANE:
					throw std::logic_error("Too many dimensions");
				default:
					obj=TObject2D();
					break;
			}
		}
		/**
		  * Constructs from another object.
		  */
		TObject3D(const TObject3D &obj):type(GEOMETRIC_TYPE_UNDEFINED)	{
			operator=(obj);
		}
		/**
		  * Static method to retrieve every point included in a vector of objects.
		  */
		static void getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts);
		/**
		  * Static method to retrieve every segment included in a vector of objects.
		  */
		static void getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms);
		/**
		  * Static method to retrieve every line included in a vector of objects.
		  */
		static void getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins);
		/**
		  * Static method to retrieve every plane included in a vector of objects.
		  */
		static void getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns);
		/**
		  * Static method to retrieve every polygon included in a vector of objects.
		  */
		static void getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys);
		/**
		  * Static method to retrieve every point included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every segment included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every line included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every plane included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every polygon included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys,std::vector<TObject3D> &remainder);
	};
#else
	struct BASE_IMPEXP TObject2D	{
	private:
		/**
		  * Object type identifier.
		  */
		unsigned char type;
		/**
		  * Union type storing pointers to every allowed type.
		  */
		struct	{
			TPoint2D point;
			TSegment2D segment;
			TLine2D line;
			TPolygon2D *polygon;
		}	data;
		/**
		  * Destroys the object, releasing the pointer to the content (if any).
		  */
		inline void destroy()	{
			if (type==GEOMETRIC_TYPE_POLYGON) delete data.polygon;
			type=GEOMETRIC_TYPE_UNDEFINED;
		}
	public:
		/**
		  * Implicit constructor from point.
		  */
		inline TObject2D(const TPoint2D &p):type(GEOMETRIC_TYPE_POINT)	{
			data.point=p;
		}
		/**
		  * Implicit constructor from segment.
		  */
		inline TObject2D(const TSegment2D &s):type(GEOMETRIC_TYPE_SEGMENT)	{
			data.segment=s;
		}
		/**
		  * Implicit constructor from line.
		  */
		inline TObject2D(const TLine2D &r):type(GEOMETRIC_TYPE_LINE)	{
			data.line=r;
		}
		/**
		  * Implicit constructor from polygon.
		  */
		inline TObject2D(const TPolygon2D &p):type(GEOMETRIC_TYPE_POLYGON)	{
			data.polygon=new TPolygon2D(p);
		}
		/**
		  * Implicit constructor from polygon.
		  */
		TObject2D():type(GEOMETRIC_TYPE_UNDEFINED)	{}
		/**
		  * Object destruction.
		  */
		~TObject2D()	{
			destroy();
		}
		/**
		  * Checks whether content is a point.
		  */
		inline bool isPoint() const	{
			return type==GEOMETRIC_TYPE_POINT;
		}
		/**
		  * Checks whether content is a segment.
		  */
		inline bool isSegment() const	{
			return type==GEOMETRIC_TYPE_SEGMENT;
		}
		/**
		  * Checks whether content is a line.
		  */
		inline bool isLine() const	{
			return type==GEOMETRIC_TYPE_LINE;
		}
		/**
		  * Checks whether content is a polygon.
		  */
		inline bool isPolygon() const	{
			return type==GEOMETRIC_TYPE_POLYGON;
		}
		/**
		  * Gets content type.
		  */
		inline unsigned char getType() const	{
			return type;
		}
		/**
		  * Gets the content as a point, returning false if the type is inadequate.
		  */
		inline bool getPoint(TPoint2D &p) const	{
			if (isPoint())	{
				p=data.point;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a segment, returning false if the type is inadequate.
		  */
		inline bool getSegment(TSegment2D &s) const	{
			if (isSegment())	{
				s=data.segment;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a line, returning false if the type is inadequate.
		  */
		inline bool getLine(TLine2D &r) const	{
			if (isLine())	{
				r=data.line;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a polygon, returning false if the type is inadequate.
		  */
		inline bool getPolygon(TPolygon2D &p) const	{
			if (isPolygon())	{
				p=*(data.polygon);
				return true;
			}	else return false;
		}
		/**
		  * Assign another TObject2D. Pointers are not shared.
		  */
		void operator=(const TObject2D &obj)	{
			if (this==&obj) return;
			destroy();
			switch (type=obj.type)	{
				case GEOMETRIC_TYPE_POINT:
					data.point=obj.data.point;
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					data.segment=obj.data.segment;
					break;
				case GEOMETRIC_TYPE_LINE:
					data.line=obj.data.line;
					break;
				case GEOMETRIC_TYPE_POLYGON:
					data.polygon=new TPolygon2D(*(obj.data.polygon));
					break;
			}
		}
		/**
		  * Assign a point to this object.
		  */
		inline void operator=(const TPoint2D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POINT;
			data.point=p;
		}
		/**
		  * Assign a segment to this object.
		  */
		inline void operator=(const TSegment2D &s)	{
			destroy();
			type=GEOMETRIC_TYPE_SEGMENT;
			data.segment=s;
		}
		/**
		  * Assign a line to this object.
		  */
		inline void operator=(const TLine2D &l)	{
			destroy();
			type=GEOMETRIC_TYPE_LINE;
			data.line=l;
		}
		/**
		  * Assign a polygon to this object.
		  */
		inline void operator=(const TPolygon2D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POLYGON;
			data.polygon=new TPolygon2D(p);
		}
		/**
		  * Project into 3D space.
		  */
		void generate3DObject(TObject3D &obj) const;
		/**
		  * Constructor from another TObject2D.
		  */
		TObject2D(const TObject2D &obj):type(GEOMETRIC_TYPE_UNDEFINED)	{
			operator=(obj);
		}
		/**
		  * Static method to retrieve all the points in a vector of TObject2D.
		  */
		static void getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts);
		/**
		  * Static method to retrieve all the segments in a vector of TObject2D.
		  */
		static void getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms);
		/**
		  * Static method to retrieve all the lines in a vector of TObject2D.
		  */
		static void getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins);
		/**
		  * Static method to retrieve all the polygons in a vector of TObject2D.
		  */
		static void getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys);
		/**
		  * Static method to retrieve all the points in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getPoints(const std::vector<TObject2D> &objs,std::vector<TPoint2D> &pnts,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the segments in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getSegments(const std::vector<TObject2D> &objs,std::vector<TSegment2D> &sgms,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the lines in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getLines(const std::vector<TObject2D> &objs,std::vector<TLine2D> &lins,std::vector<TObject2D> &remainder);
		/**
		  * Static method to retrieve all the polygons in a vector of TObject2D, returning the remainder objects in another parameter.
		  */
		static void getPolygons(const std::vector<TObject2D> &objs,std::vector<TPolygon2D> &polys,std::vector<TObject2D> &remainder);
	};
	/**
	  * Standard object for storing any 3D lightweight object. Do not inherit from this class.
	  * \sa TPoint3D,TSegment3D,TLine3D,TPlane,TPolygon3D
	  */
	struct BASE_IMPEXP TObject3D	{
	private:
		/**
		  * Object type identifier.
		  */
		unsigned char type;
		/**
		  * Union containing pointer to actual data.
		  */
		struct	{
			TPoint3D point;
			TSegment3D segment;
			TLine3D line;
			TPolygon3D *polygon;
			TPlane plane;
		}	data;
		/**
		  * Destroys the object and releases the pointer, if any.
		  */
		void destroy()	{
			if (type==GEOMETRIC_TYPE_POLYGON) delete data.polygon;
			type=GEOMETRIC_TYPE_UNDEFINED;
		}
	public:
		/**
		  * Constructor from point.
		  */
		TObject3D(const TPoint3D &p):type(GEOMETRIC_TYPE_POINT)	{
			data.point=p;
		}
		/**
		  * Constructor from segment.
		  */
		TObject3D(const TSegment3D &s):type(GEOMETRIC_TYPE_SEGMENT)	{
			data.segment=s;
		}
		/**
		  * Constructor from line.
		  */
		TObject3D(const TLine3D &r):type(GEOMETRIC_TYPE_LINE)	{
			data.line=r;
		}
		/**
		  * Constructor from polygon.
		  */
		TObject3D(const TPolygon3D &p):type(GEOMETRIC_TYPE_POLYGON)	{
			data.polygon=new TPolygon3D(p);
		}
		/**
		  * Constructor from plane.
		  */
		TObject3D(const TPlane &p):type(GEOMETRIC_TYPE_PLANE)	{
			data.plane=p;
		}
		/**
		  * Empty constructor.
		  */
		TObject3D():type(GEOMETRIC_TYPE_UNDEFINED)	{}
		/**
		  * Destructor.
		  */
		~TObject3D()	{
			destroy();
		}
		/**
		  * Checks whether content is a point.
		  */
		inline bool isPoint() const	{
			return type==GEOMETRIC_TYPE_POINT;
		}
		/**
		  * Checks whether content is a segment.
		  */
		inline bool isSegment() const	{
			return type==GEOMETRIC_TYPE_SEGMENT;
		}
		/**
		  * Checks whether content is a line.
		  */
		inline bool isLine() const	{
			return type==GEOMETRIC_TYPE_LINE;
		}
		/**
		  * Checks whether content is a polygon.
		  */
		inline bool isPolygon() const	{
			return type==GEOMETRIC_TYPE_POLYGON;
		}
		/**
		  * Checks whether content is a plane.
		  */
		inline bool isPlane() const	{
			return type==GEOMETRIC_TYPE_PLANE;
		}
		/**
		  * Gets object type.
		  */
		inline unsigned char getType() const	{
			return type;
		}
		/**
		  * Gets the content as a point, returning false if the type is not adequate.
		  */
		inline bool getPoint(TPoint3D &p) const	{
			if (isPoint())	{
				p=data.point;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a segment, returning false if the type is not adequate.
		  */
		inline bool getSegment(TSegment3D &s) const	{
			if (isSegment())	{
				s=data.segment;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a line, returning false if the type is not adequate.
		  */
		inline bool getLine(TLine3D &r) const	{
			if (isLine())	{
				r=data.line;
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a polygon, returning false if the type is not adequate.
		  */
		inline bool getPolygon(TPolygon3D &p) const	{
			if (isPolygon())	{
				p=*(data.polygon);
				return true;
			}	else return false;
		}
		/**
		  * Gets the content as a plane, returning false if the type is not adequate.
		  */
		inline bool getPlane(TPlane &p) const	{
			if (isPlane())	{
				p=data.plane;
				return true;
			}	else return false;
		}
		/**
		  * Assigns another object, creating a new pointer if needed.
		  */
		void operator=(const TObject3D &obj)	{
			if (this==&obj) return;
			destroy();
			switch (type=obj.type)	{
				case GEOMETRIC_TYPE_POINT:
					data.point=obj.data.point;
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					data.segment=obj.data.segment;
					break;
				case GEOMETRIC_TYPE_LINE:
					data.line=obj.data.line;
					break;
				case GEOMETRIC_TYPE_POLYGON:
					data.polygon=new TPolygon3D(*(obj.data.polygon));
					break;
				case GEOMETRIC_TYPE_PLANE:
					data.plane=obj.data.plane;
					break;
				case GEOMETRIC_TYPE_UNDEFINED:
					break;
				default:
					THROW_EXCEPTION("Invalid TObject3D object");
			}
		}
		/**
		  * Assigns a point to this object.
		  */
		inline void operator=(const TPoint3D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POINT;
			data.point=p;
		}
		/**
		  * Assigns a segment to this object.
		  */
		inline void operator=(const TSegment3D &s)	{
			destroy();
			type=GEOMETRIC_TYPE_SEGMENT;
			data.segment=s;
		}
		/**
		  * Assigns a line to this object.
		  */
		inline void operator=(const TLine3D &l)	{
			destroy();
			type=GEOMETRIC_TYPE_LINE;
			data.line=l;
		}
		/**
		  * Assigns a polygon to this object.
		  */
		inline void operator=(const TPolygon3D &p)	{
			destroy();
			type=GEOMETRIC_TYPE_POLYGON;
			data.polygon=new TPolygon3D(p);
		}
		/**
		  * Assigns a plane to this object.
		  */
		inline void operator=(const TPlane &p)	{
			destroy();
			type=GEOMETRIC_TYPE_PLANE;
			data.plane=p;
		}
		/**
		  * Projects into 2D space.
		  * \throw std::logic_error if the 3D object loses its properties when projecting into 2D space (for example, it's a plane or a vertical line).
		  */
		inline void generate2DObject(TObject2D &obj) const	{
			switch (type)	{
				case GEOMETRIC_TYPE_POINT:
					obj=TPoint2D(data.point);
					break;
				case GEOMETRIC_TYPE_SEGMENT:
					obj=TSegment2D(data.segment);
					break;
				case GEOMETRIC_TYPE_LINE:
					obj=TLine2D(data.line);
					break;
				case GEOMETRIC_TYPE_POLYGON:
					obj=TPolygon2D(*(data.polygon));
					break;
				case GEOMETRIC_TYPE_PLANE:
					throw std::logic_error("Too many dimensions");
				default:
					obj=TObject2D();
					break;
			}
		}
		/**
		  * Constructs from another object.
		  */
		TObject3D(const TObject3D &obj):type(GEOMETRIC_TYPE_UNDEFINED)	{
			operator=(obj);
		}
		/**
		  * Static method to retrieve every point included in a vector of objects.
		  */
		static void getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts);
		/**
		  * Static method to retrieve every segment included in a vector of objects.
		  */
		static void getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms);
		/**
		  * Static method to retrieve every line included in a vector of objects.
		  */
		static void getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins);
		/**
		  * Static method to retrieve every plane included in a vector of objects.
		  */
		static void getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns);
		/**
		  * Static method to retrieve every polygon included in a vector of objects.
		  */
		static void getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys);
		/**
		  * Static method to retrieve every point included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPoints(const std::vector<TObject3D> &objs,std::vector<TPoint3D> &pnts,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every segment included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getSegments(const std::vector<TObject3D> &objs,std::vector<TSegment3D> &sgms,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every line included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getLines(const std::vector<TObject3D> &objs,std::vector<TLine3D> &lins,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every plane included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPlanes(const std::vector<TObject3D> &objs,std::vector<TPlane> &plns,std::vector<TObject3D> &remainder);
		/**
		  * Static method to retrieve every polygon included in a vector of objects, returning the remaining objects in another argument.
		  */
		static void getPolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys,std::vector<TObject3D> &remainder);
	};

#endif


	//Streaming functions
	/**
	  * TPoint2D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPoint2D &o);
	/**
	  * TPoint2D binary output.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPoint2D &o);

	/**
	  * TPoint3D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPoint3D &o);
	/**
	  * TPoint3D binary output.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPoint3D &o);

	/**
	  * TPose2D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPose2D &o);
	/**
	  * TPose2D binary output.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPose2D &o);

	/**
	  * TPose3D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in,mrpt::math::TPose3D &o);
	/**
	  * TPose3D binary output.
	  */
	BASE_IMPEXP mrpt::utils::CStream& operator<<(mrpt::utils::CStream& out,const mrpt::math::TPose3D &o);

	/**
	  * TSegment2D binary input.
	  */
	inline mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TSegment2D &s)	{
		return in>>s.point1>>s.point2;
	}
	/**
	  * TSegment2D binary output.
	  */
	inline mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TSegment2D &s)	{
		return out<<s.point1<<s.point2;
	}

	/**
	  * TLine2D binary input.
	  */
	inline mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TLine2D &l)	{
		return in>>l.coefs[0]>>l.coefs[1]>>l.coefs[2];
	}
	/**
	  * TLine2D binary output.
	  */
	inline mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TLine2D &l)	{
		return out<<l.coefs[0]<<l.coefs[1]<<l.coefs[2];
	}

	/**
	  * TObject2D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TObject2D &o);
	/**
	  * TObject2D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TObject2D &o);

	/**
	  * TSegment3D binary input.
	  */
	inline mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TSegment3D &s)	{
		return in>>s.point1>>s.point2;
	}
	/**
	  * TSegment3D binary output.
	  */
	inline mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TSegment3D &s)	{
		return out<<s.point1<<s.point2;
	}

	/**
	  * TLine3D binary input.
	  */
	inline mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TLine3D &l)	{
		return in>>l.pBase>>l.director[0]>>l.director[1]>>l.director[2];
	}
	/**
	  * TLine3D binary output.
	  */
	inline mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TLine3D &l)	{
		return out<<l.pBase<<l.director[0]<<l.director[1]<<l.director[2];
	}

	/**
	  * TPlane binary input.
	  */
	inline mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TPlane &p)	{
		return in>>p.coefs[0]>>p.coefs[1]>>p.coefs[2]>>p.coefs[3];
	}
	/**
	  * TPlane binary output.
	  */
	inline mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TPlane &p)	{
		return out<<p.coefs[0]<<p.coefs[1]<<p.coefs[2]<<p.coefs[3];
	}

	/**
	  * TObject3D binary input.
	  */
	BASE_IMPEXP mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,mrpt::math::TObject3D &o);
	/**
	  * TObject3D binary output.
	  */
	BASE_IMPEXP mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const mrpt::math::TObject3D &o);


	/** @} */ // end of grouping

	}	//end of namespace math

	namespace utils
	{
		using namespace ::mrpt::math;

		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME(TPoint2D)
		MRPT_DECLARE_TTYPENAME(TPoint3D)
		MRPT_DECLARE_TTYPENAME(TPose2D)
		MRPT_DECLARE_TTYPENAME(TPose3D)
		MRPT_DECLARE_TTYPENAME(TSegment2D)
		MRPT_DECLARE_TTYPENAME(TLine2D)
		MRPT_DECLARE_TTYPENAME(TPolygon2D)
		MRPT_DECLARE_TTYPENAME(TObject2D)
		MRPT_DECLARE_TTYPENAME(TSegment3D)
		MRPT_DECLARE_TTYPENAME(TLine3D)
		MRPT_DECLARE_TTYPENAME(TPlane)
		MRPT_DECLARE_TTYPENAME(TPolygon3D)
		MRPT_DECLARE_TTYPENAME(TObject3D)

	} // end of namespace utils

}	//end of namespace
#endif
