/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  GEO_H
#define  GEO_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CSparseMatrixTemplate.h>

#include <mrpt/math/math_frwds.h>  // forward declarations
#include <mrpt/math/wrap2pi.h>

namespace mrpt
{
	namespace math
	{
		/** \addtogroup geometry_grp Geometry: lines, planes, intersections, SLERP, "lightweight" point & pose classes
		  *  \ingroup mrpt_base_grp
		  * @{ */

		extern double BASE_IMPEXP geometryEpsilon; //!< Global epsilon to overcome small precision errors (Default=1e-5)

		/** Slightly heavyweight type to speed-up calculations with polygons in 3D
		  * \sa TPolygon3D,TPlane
		  */
		class BASE_IMPEXP TPolygonWithPlane	{
		public:
			TPolygon3D poly; //!< Actual polygon.
			TPlane plane; //!< Plane containing the polygon.
			mrpt::poses::CPose3D pose; //!< Plane's pose.  \sa inversePose
			mrpt::poses::CPose3D inversePose; //!< Plane's inverse pose. \sa pose
			TPolygon2D poly2D; //!< Polygon, after being projected to the plane using inversePose. \sa inversePose
			TPolygonWithPlane(const TPolygon3D &p); //!< Constructor. Takes a polygon and computes each parameter.
			/** Basic constructor. Needed to create containers  \sa TPolygonWithPlane(const TPolygon3D &) */
			TPolygonWithPlane()	{}
			/** Static method for vectors. Takes a set of polygons and creates every TPolygonWithPlane  */
			static void getPlanes(const std::vector<TPolygon3D> &oldPolys,std::vector<TPolygonWithPlane> &newPolys);
		};

		/** @name Simple intersection operations, relying basically on geometrical operations.
			@{
		 */
		/** Gets the intersection between two 3D segments. Possible outcomes:
		  *		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  *		- Segments don't intersect & are parallel: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment "in between" both segments.
		  *		- Segments don't intersect & aren't parallel: Return=false.
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TSegment3D &s1,const TSegment3D &s2,TObject3D &obj);

		/** Gets the intersection between a 3D segment and a plane. Possible outcomes:
		  *		- Don't intersect: Return=false
		  *		- s1 is within the plane: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- s1 intersects the plane at one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TSegment3D &s1,const TPlane &p2,TObject3D &obj);

		/** Gets the intersection between a 3D segment and a 3D line. Possible outcomes:
		  *		- They don't intersect : Return=false
		  *		- s1 lies within the line: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- s1 intersects the line at a point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TSegment3D &s1,const TLine3D &r2,TObject3D &obj);

		/** Gets the intersection between a plane and a 3D segment. Possible outcomes:
		  *		- Don't intersect: Return=false
		  *		- s2 is within the plane: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- s2 intersects the plane at one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		inline bool intersect(const TPlane &p1,const TSegment3D &s2,TObject3D &obj)	{
			return intersect(s2,p1,obj);
		}

		/** Gets the intersection between two planes. Possible outcomes:
		  *		- Planes are parallel: Return=false
		  *		- Planes intersect into a line: Return=true, obj.getType()=GEOMETRIC_TYPE_LINE
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TPlane &p1,const TPlane &p2,TObject3D &obj);

		/** Gets the intersection between a plane and a 3D line. Possible outcomes:
		  *		- Line is parallel to plane but not within it: Return=false
		  *		- Line is contained in the plane: Return=true, obj.getType()=GEOMETRIC_TYPE_LINE
		  *		- Line intersects the plane at one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TPlane &p1,const TLine3D &p2,TObject3D &obj);

		/** Gets the intersection between a 3D line and a 3D segment. Possible outcomes:
		  *		- They don't intersect : Return=false
		  *		- s2 lies within the line: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- s2 intersects the line at a point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		inline bool intersect(const TLine3D &r1,const TSegment3D &s2,TObject3D &obj)	{
			return intersect(s2,r1,obj);
		}

		/** Gets the intersection between a 3D line and a plane. Possible outcomes:
		  *		- Line is parallel to plane but not within it: Return=false
		  *		- Line is contained in the plane: Return=true, obj.getType()=GEOMETRIC_TYPE_LINE
		  *		- Line intersects the plane at one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		inline bool intersect(const TLine3D &r1,const TPlane &p2,TObject3D &obj)	{
			return intersect(p2,r1,obj);
		}

		/** Gets the intersection between two 3D lines. Possible outcomes:
		  *		- Lines do not intersect: Return=false
		  *		- Lines are parallel and do not coincide: Return=false
		  *		- Lines coincide (are the same): Return=true, obj.getType()=GEOMETRIC_TYPE_LINE
		  *		- Lines intesect in a point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject3D
		  */
		bool BASE_IMPEXP intersect(const TLine3D &r1,const TLine3D &r2,TObject3D &obj);

		/** Gets the intersection between two 2D lines. Possible outcomes:
		  *		- Lines do not intersect: Return=false
		  *		- Lines are parallel and do not coincide: Return=false
		  *		- Lines coincide (are the same): Return=true, obj.getType()=GEOMETRIC_TYPE_LINE
		  *		- Lines intesect in a point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject2D
		  */
		bool BASE_IMPEXP intersect(const TLine2D &r1,const TLine2D &r2,TObject2D &obj);

		/** Gets the intersection between a 2D line and a 2D segment. Possible outcomes:
		  *		- They don't intersect: Return=false
		  *		- s2 lies within the line: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- Both intersects in one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject2D
		  */
		bool BASE_IMPEXP intersect(const TLine2D &r1,const TSegment2D &s2,TObject2D &obj);

		/** Gets the intersection between a 2D line and a 2D segment. Possible outcomes:
		  *		- They don't intersect: Return=false
		  *		- s1 lies within the line: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT
		  *		- Both intersects in one point: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  * \sa TObject2D
		  */
		inline bool intersect(const TSegment2D &s1,const TLine2D &r2,TObject2D &obj)	{
			return intersect(r2,s1,obj);
		}

		/** Gets the intersection between two 2D segments. Possible outcomes:
		  *		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
		  *		- Segments don't intersect & are parallel: Return=true, obj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment "in between" both segments.
		  *		- Segments don't intersect & aren't parallel: Return=false.
		  * \sa TObject2D
		  */
		bool BASE_IMPEXP intersect(const TSegment2D &s1,const TSegment2D &s2,TObject2D &obj);

		/** @}
		 */

		/** @name Angle retrieval methods. Methods which use TSegments will automatically use TLines' implicit constructors.
			@{
		 */
		/**
		  * Computes the angle between two planes.
		  */
		double BASE_IMPEXP getAngle(const TPlane &p1,const TPlane &p2);
		/**
		  * Computes the angle between a plane and a 3D line or segment (implicit constructor will be used if passing a segment instead of a line).
		  */
		double BASE_IMPEXP getAngle(const TPlane &p1,const TLine3D &r2);
		/**
		  * Computes the angle between a 3D line or segment and a plane (implicit constructor will be used if passing a segment instead of a line).
		  */
		inline double getAngle(const TLine3D &r1,const TPlane &p2)	{
			return getAngle(p2,r1);
		}
		/**
		  * Computes the angle between two 3D lines or segments (implicit constructor will be used if passing a segment instead of a line).
		  */
		double BASE_IMPEXP getAngle(const TLine3D &r1,const TLine3D &r2);
		/**
		  * Computes the angle between two 2D lines or segments (implicit constructor will be used if passing a segment instead of a line).
		  */
		double BASE_IMPEXP getAngle(const TLine2D &r1,const TLine2D &r2);
		/** @}
		 */

		/** @name Creation of lines from poses.
			@{
		 */
		/**
		  * Gets a 3D line corresponding to the X axis in a given pose. An implicit constructor is used if a TPose3D is given.
		  * \sa createFromPoseY,createFromPoseZ,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseX(const mrpt::poses::CPose3D &p,TLine3D &r);
		/**
		  * Gets a 3D line corresponding to the Y axis in a given pose. An implicit constructor is used if a TPose3D is given.
		  * \sa createFromPoseX,createFromPoseZ,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseY(const mrpt::poses::CPose3D &p,TLine3D &r);
		/**
		  * Gets a 3D line corresponding to the Z axis in a given pose. An implicit constructor is used if a TPose3D is given.
		  * \sa createFromPoseX,createFromPoseY,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseZ(const mrpt::poses::CPose3D &p,TLine3D &r);
		/**
		  * Gets a 3D line corresponding to any arbitrary vector, in the base given by the pose. An implicit constructor is used if a TPose3D is given.
		  * \sa createFromPoseX,createFromPoseY,createFromPoseZ
		  */
		void BASE_IMPEXP createFromPoseAndVector(const mrpt::poses::CPose3D &p,const double (&vector)[3],TLine3D &r);
		/**
		  * Gets a 2D line corresponding to the X axis in a given pose. An implicit constructor is used if a CPose2D is given.
		  * \sa createFromPoseY,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseX(const TPose2D &p,TLine2D &r);
		/**
		  * Gets a 2D line corresponding to the Y axis in a given pose. An implicit constructor is used if a CPose2D is given.
		  * \sa createFromPoseX,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseY(const TPose2D &p,TLine2D &r);
		/**
		  * Gets a 2D line corresponding to any arbitrary vector, in the base given the given pose. An implicit constructor is used if a CPose2D is given.
		  * \sa createFromPoseY,createFromPoseAndVector
		  */
		void BASE_IMPEXP createFromPoseAndVector(const TPose2D &p,const double (&vector)[2],TLine2D &r);
		/** @}
		 */

		/** @name Other line or plane related methods.
			@{
		 */
		/**
		  * Checks whether this polygon or set of points acceptably fits a plane.
		  * \sa TPolygon3D,geometryEpsilon
		  */
		bool BASE_IMPEXP conformAPlane(const std::vector<TPoint3D> &points);
		/**
		  * Checks whether this polygon or set of points acceptably fits a plane, and if it's the case returns it in the second argument.
		  * \sa TPolygon3D,geometryEpsilon
		  */
		bool BASE_IMPEXP conformAPlane(const std::vector<TPoint3D> &points,TPlane &p);
		/**
		  * Checks whether this set of points acceptably fits a 2D line.
		  * \sa geometryEpsilon
		  */
		bool BASE_IMPEXP areAligned(const std::vector<TPoint2D> &points);
		/**
		  * Checks whether this set of points acceptably fits a 2D line, and if it's the case returns it in the second argument.
		  * \sa geometryEpsilon
		  */
		bool BASE_IMPEXP areAligned(const std::vector<TPoint2D> &points,TLine2D &r);
		/**
		  * Checks whether this set of points acceptably fits a 3D line.
		  * \sa geometryEpsilon
		  */
		bool BASE_IMPEXP areAligned(const std::vector<TPoint3D> &points);
		/**
		  * Checks whether this set of points acceptably fits a 3D line, and if it's the case returns it in the second argument.
		  */
		bool BASE_IMPEXP areAligned(const std::vector<TPoint3D> &points,TLine3D &r);
		/** @}
		 */

		/** @name Projections
			@{
		 */
		/** Uses the given pose 3D to project a point into a new base */
		inline void project3D(const TPoint3D &point, const mrpt::poses::CPose3D &newXYpose,TPoint3D &newPoint)	{
			newXYpose.composePoint(point.x,point.y,point.z,newPoint.x,newPoint.y,newPoint.z);
		}
		/** Uses the given pose 3D to project a segment into a new base  */
		inline void project3D(const TSegment3D &segment,const mrpt::poses::CPose3D &newXYpose,TSegment3D &newSegment)	{
			project3D(segment.point1,newXYpose,newSegment.point1);
			project3D(segment.point2,newXYpose,newSegment.point2);
		}

		void BASE_IMPEXP project3D(const TLine3D &line,const mrpt::poses::CPose3D &newXYpose,TLine3D &newLine); //!< Uses the given pose 3D to project a line into a new base
		void BASE_IMPEXP project3D(const TPlane &plane,const mrpt::poses::CPose3D &newXYpose,TPlane &newPlane); //!< Uses the given pose 3D to project a plane into a new base
		void BASE_IMPEXP project3D(const TPolygon3D &polygon,const mrpt::poses::CPose3D &newXYpose,TPolygon3D &newPolygon); //!< Uses the given pose 3D to project a polygon into a new base
		void BASE_IMPEXP project3D(const TObject3D &object,const mrpt::poses::CPose3D &newXYPose,TObject3D &newObject); //!< Uses the given pose 3D to project any 3D object into a new base.

		/** Projects any 3D object into the plane's base, using its inverse pose. If the object is exactly inside the plane, this projection will zero its Z coordinates */
		template<class T> void project3D(const T &obj,const TPlane &newXYPlane,T &newObj)	{
			mrpt::poses::CPose3D pose;
			TPlane(newXYPlane).getAsPose3D(pose);
			project3D(obj,-pose,newObj);
		}

		/** Projects any 3D object into the plane's base, using its inverse pose and forcing the position of the new coordinates origin. If the object is exactly inside the plane, this projection will zero its Z coordinates  */
		template<class T> void project3D(const T &obj,const TPlane &newXYPlane,const TPoint3D &newOrigin,T &newObj)	{
			mrpt::poses::CPose3D pose;
			//TPlane(newXYPlane).getAsPose3DForcingOrigin(newOrigin,pose);
			TPlane(newXYPlane).getAsPose3D(pose);
			project3D(obj,-pose,newObj);
		}

		/** Projects a set of 3D objects into the plane's base. */
		template<class T> void project3D(const std::vector<T> &objs,const mrpt::poses::CPose3D &newXYpose,std::vector<T> &newObjs)	{
			size_t N=objs.size();
			newObjs.resize(N);
			for (size_t i=0;i<N;i++) project3D(objs[i],newXYpose,newObjs[i]);
		}

		/** Uses the given pose 2D to project a point into a new base. */
		void BASE_IMPEXP project2D(const TPoint2D &point,const mrpt::poses::CPose2D &newXpose,TPoint2D &newPoint);

		/** Uses the given pose 2D to project a segment into a new base  */
		inline void project2D(const TSegment2D &segment,const mrpt::poses::CPose2D &newXpose,TSegment2D &newSegment)	{
			project2D(segment.point1,newXpose,newSegment.point1);
			project2D(segment.point2,newXpose,newSegment.point2);
		}


		void BASE_IMPEXP project2D(const TLine2D &line,const mrpt::poses::CPose2D &newXpose,TLine2D &newLine); //!< Uses the given pose 2D to project a line into a new base
		void BASE_IMPEXP project2D(const TPolygon2D &polygon,const mrpt::poses::CPose2D &newXpose,TPolygon2D &newPolygon); //!< Uses the given pose 2D to project a polygon into a new base.
		void BASE_IMPEXP project2D(const TObject2D &object,const mrpt::poses::CPose2D &newXpose,TObject2D &newObject); //!< Uses the given pose 2D to project any 2D object into a new base

		/** Projects any 2D object into the line's base, using its inverse pose. If the object is exactly inside the line, this projection will zero its Y coordinate.
		  * \tparam CPOSE2D set to mrpt::poses::CPose2D
		  */
		template<class T,class CPOSE2D> void project2D(const T &obj,const TLine2D &newXLine,T &newObj)	{
			CPOSE2D pose;
			newXLine.getAsPose2D(pose);
			project2D(obj,CPOSE2D(0,0,0)-pose,newObj);
		}

		/** Projects any 2D object into the line's base, using its inverse pose and forcing the position of the new coordinate origin. If the object is exactly inside the line, this projection will zero its Y coordinate.
		  * \tparam CPOSE2D set to mrpt::poses::CPose2D
		  */
		template<class T,class CPOSE2D> void project2D(const T &obj,const TLine2D &newXLine,const TPoint2D &newOrigin,T &newObj)	{
			CPOSE2D pose;
			newXLine.getAsPose2DForcingOrigin(newOrigin,pose);
			project2D(obj,CPOSE2D(0,0,0)-pose,newObj);
		}

		/** Projects a set of 2D objects into the line's base */
		template<class T> void project2D(const std::vector<T> &objs,const mrpt::poses::CPose2D &newXpose,std::vector<T> &newObjs)	{
			size_t N=objs.size();
			newObjs.resize(N);
			for (size_t i=0;i<N;i++) project2D(objs[i],newXpose,newObjs[i]);
		}
		/** @}
		 */

		/** @name Polygon intersections. These operations rely more on spatial reasoning than in raw numerical operations.
			@{
		 */
		/** Gets the intersection between a 2D polygon and a 2D segment. \sa TObject2D  */
		bool BASE_IMPEXP intersect(const TPolygon2D &p1,const TSegment2D &s2,TObject2D &obj);
		/** Gets the intersection between a 2D polygon and a 2D line. \sa TObject2D  */
		bool BASE_IMPEXP intersect(const TPolygon2D &p1,const TLine2D &r2,TObject2D &obj);
		/** Gets the intersection between two 2D polygons. \sa TObject2D */
		bool BASE_IMPEXP intersect(const TPolygon2D &p1,const TPolygon2D &p2,TObject2D &obj);
		/** Gets the intersection between a 2D segment and a 2D polygon.  \sa TObject2D  */
		inline bool intersect(const TSegment2D &s1,const TPolygon2D &p2,TObject2D &obj)	{
			return intersect(p2,s1,obj);
		}
		/** Gets the intersection between a 2D line and a 2D polygon.\sa TObject2D */
		inline bool intersect(const TLine2D &r1,const TPolygon2D &p2,TObject2D &obj)	{
			return intersect(p2,r1,obj);
		}
		/** Gets the intersection between a 3D polygon and a 3D segment. \sa TObject3D  */
		bool BASE_IMPEXP intersect(const TPolygon3D &p1,const TSegment3D &s2,TObject3D &obj);
		/** Gets the intersection between a 3D polygon and a 3D line. \sa TObject3D  */
		bool BASE_IMPEXP intersect(const TPolygon3D &p1,const TLine3D &r2,TObject3D &obj);
		/** Gets the intersection between a 3D polygon and a plane. \sa TObject3D */
		bool BASE_IMPEXP intersect(const TPolygon3D &p1,const TPlane &p2,TObject3D &obj);
		/** Gets the intersection between two 3D polygons. \sa TObject3D */
		bool BASE_IMPEXP intersect(const TPolygon3D &p1,const TPolygon3D &p2,TObject3D &obj);
		/** Gets the intersection between a 3D segment and a 3D polygon. \sa TObject3D */
		inline bool intersect(const TSegment3D &s1,const TPolygon3D &p2,TObject3D &obj)	{
			return intersect(p2,s1,obj);
		}
		/** Gets the intersection between a 3D line and a 3D polygon.\sa TObject3D  */
		inline bool intersect(const TLine3D &r1,const TPolygon3D &p2,TObject3D &obj)	{
			return intersect(p2,r1,obj);
		}
		/** Gets the intersection between a plane and a 3D polygon. \sa TObject3D */
		inline bool intersect(const TPlane &p1,const TPolygon3D &p2,TObject3D &obj)	{
			return intersect(p2,p1,obj);
		}

		/** Gets the intersection between two sets of 3D polygons. The intersection is returned as an sparse matrix with each pair of polygons' intersections, and the return value is the amount of intersections found.
		  * \sa TObject3D,CSparseMatrixTemplate */
		size_t BASE_IMPEXP intersect(const std::vector<TPolygon3D> &v1,const std::vector<TPolygon3D> &v2,CSparseMatrixTemplate<TObject3D> &objs);
		/** Gets the intersection between two sets of 3D polygons. The intersection is returned as a vector with every intersection found, and the return value is the amount of intersections found.
		  * \sa TObject3D */
		size_t BASE_IMPEXP intersect(const std::vector<TPolygon3D> &v1,const std::vector<TPolygon3D> &v2,std::vector<TObject3D> &objs);
		/** @}
		 */

		/** @name Other intersections
			@{
		 */
		/** Gets the intersection between vectors of geometric objects and returns it in a sparse matrix of either TObject2D or TObject3D.
		  * \sa TObject2D,TObject3D,CSparseMatrix */
		template<class T,class U,class O> size_t intersect(const std::vector<T> &v1,const std::vector<U> &v2,CSparseMatrixTemplate<O> &objs)	{
			size_t M=v1.size(),N=v2.size();
			O obj;
			objs.clear();
			objs.resize(M,N);
			for (size_t i=0;i<M;i++) for (size_t j=0;j<M;j++) if (intersect(v1[i],v2[j],obj)) objs(i,j)=obj;
			return objs.getNonNullElements();
		}

		/** Gets the intersection between vectors of geometric objects and returns it in a vector of either TObject2D or TObject3D.
		  * \sa TObject2D,TObject3D */
		template<class T,class U,class O> size_t intersect(const std::vector<T> &v1,const std::vector<U> &v2,std::vector<O> objs)	{
			objs.resize(0);
			O obj;
			for (typename std::vector<T>::const_iterator it1=v1.begin();it1!=v1.end();++it1)	{
				const T &elem1=*it1;
				for (typename std::vector<U>::const_iterator it2=v2.begin();it2!=v2.end();++it2) if (intersect(elem1,*it2,obj)) objs.push_back(obj);
			}
			return objs.size();
		}

		/** Gets the intersection between any pair of 2D objects.*/
		bool BASE_IMPEXP intersect(const TObject2D &o1,const TObject2D &o2,TObject2D &obj);
		/** Gets the intersection between any pair of 3D objects.*/
		bool BASE_IMPEXP intersect(const TObject3D &o1,const TObject3D &o2,TObject3D &obj);
		/** @}
		 */

		/** @name Distances
			@{
		 */
		double BASE_IMPEXP distance(const TPoint2D &p1,const TPoint2D &p2); //!< Gets the distance between two points in a 2D space.
		double BASE_IMPEXP distance(const TPoint3D &p1,const TPoint3D &p2); //!< Gets the distance between two points in a 3D space.
		double BASE_IMPEXP distance(const TLine2D &r1,const TLine2D &r2); //!<  Gets the distance between two lines in a 2D space.
		double BASE_IMPEXP distance(const TLine3D &r1,const TLine3D &r2); //!<  Gets the distance between two lines in a 3D space.
		double BASE_IMPEXP distance(const TPlane &p1,const TPlane &p2); //!<  Gets the distance between two planes. It will be zero if the planes are not parallel.
		double BASE_IMPEXP distance(const TPolygon2D &p1,const TPolygon2D &p2);//!< Gets the distance between two polygons in a 2D space.
		double BASE_IMPEXP distance(const TPolygon2D &p1,const TSegment2D &s2);//!< Gets the distance between a polygon and a segment in a 2D space.
		/** Gets the distance between a segment and a polygon in a 2D space. */
		inline double distance(const TSegment2D &s1,const TPolygon2D &p2)	{
			return distance(p2,s1);
		}
		double BASE_IMPEXP distance(const TPolygon2D &p1,const TLine2D &l2);//!< Gets the distance between a polygon and a line in a 2D space.
		inline double distance(const TLine2D &l1,const TPolygon2D &p2)	{
			return distance(p2,l1);
		}
		double BASE_IMPEXP distance(const TPolygon3D &p1,const TPolygon3D &p2);//!< Gets the distance between two polygons in a 3D space.
		double BASE_IMPEXP distance(const TPolygon3D &p1,const TSegment3D &s2);//!< Gets the distance between a polygon and a segment in a 3D space.
		/** Gets the distance between a segment and a polygon in a 3D space.*/
		inline double distance(const TSegment3D &s1,const TPolygon3D &p2)	{
			return distance(p2,s1);
		}
		double BASE_IMPEXP distance(const TPolygon3D &p1,const TLine3D &l2); //!< Gets the distance between a polygon and a line in a 3D space.
		/** Gets the distance between a line and a polygon in a 3D space */
		inline double distance(const TLine3D &l1,const TPolygon3D &p2)	{
			return distance(p2,l1);
		}
		double BASE_IMPEXP distance(const TPolygon3D &po,const TPlane &pl);//!< Gets the distance between a polygon and a plane.
		/** Gets the distance between a plane and a polygon.*/
		inline double distance(const TPlane &pl,const TPolygon3D &po)	{
			return distance(po,pl);
		}
		/** @}
		 */

		/** @name Bound checkers
			@{
		 */
		/** Gets the rectangular bounds of a 2D polygon or set of 2D points */
		void BASE_IMPEXP getRectangleBounds(const std::vector<TPoint2D> &poly,TPoint2D &pMin,TPoint2D &pMax);
		/** Gets the prism bounds of a 3D polygon or set of 3D points. */
		void BASE_IMPEXP getPrismBounds(const std::vector<TPoint3D> &poly,TPoint3D &pMin,TPoint3D &pMax);
		/** @}
		 */

		/** @name Creation of planes from poses
			@{
		 */
		/**
		  * Given a pose, creates a plane orthogonal to its Z vector.
		  * \sa createPlaneFromPoseXZ,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal
		  */
		void BASE_IMPEXP createPlaneFromPoseXY(const mrpt::poses::CPose3D &pose,TPlane &plane);
		/**
		  * Given a pose, creates a plane orthogonal to its Y vector.
		  * \sa createPlaneFromPoseXY,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal
		  */
		void BASE_IMPEXP createPlaneFromPoseXZ(const mrpt::poses::CPose3D &pose,TPlane &plane);
		/**
		  * Given a pose, creates a plane orthogonal to its X vector.
		  * \sa createPlaneFromPoseXY,createPlaneFromPoseXZ,createPlaneFromPoseAndNormal
		  */
		void BASE_IMPEXP createPlaneFromPoseYZ(const mrpt::poses::CPose3D &pose,TPlane &plane);
		/**
		  * Given a pose and any vector, creates a plane orthogonal to that vector in the pose's coordinates.
		  * \sa createPlaneFromPoseXY,createPlaneFromPoseXZ,createPlaneFromPoseYZ
		  */
		void BASE_IMPEXP createPlaneFromPoseAndNormal(const mrpt::poses::CPose3D &pose,const double (&normal)[3],TPlane &plane);
		/**
		  * Creates a rotation matrix so that the coordinate given (0 for x, 1 for y, 2 for z) corresponds to the vector.
		  */
		void BASE_IMPEXP generateAxisBaseFromDirectionAndAxis(const double (&vec)[3],char coord,CMatrixDouble &matrix);
		/** @}
		 */

		/** @name Linear regression methods
			@{
		 */
		/**
		  * Using eigenvalues, gets the best fitting line for a set of 2D points. Returns an estimation of the error.
		  * \sa spline, leastSquareLinearFit
		  */
		double BASE_IMPEXP getRegressionLine(const std::vector<TPoint2D> &points,TLine2D &line);
		/**
		  * Using eigenvalues, gets the best fitting line for a set of 3D points. Returns an estimation of the error.
		  * \sa spline, leastSquareLinearFit
		  */
		double BASE_IMPEXP getRegressionLine(const std::vector<TPoint3D> &points,TLine3D &line);
		/**
		  * Using eigenvalues, gets the best fitting plane for a set of 3D points. Returns an estimation of the error.
		  * \sa spline, leastSquareLinearFit
		  */
		double BASE_IMPEXP getRegressionPlane(const std::vector<TPoint3D> &points,TPlane &plane);
		/** @}
		 */

		/** @name Miscellaneous Geometry methods
			@{
		 */
		/**
		  * Tries to assemble a set of segments into a set of closed polygons.
		  */
		void BASE_IMPEXP assemblePolygons(const std::vector<TSegment3D> &segms,std::vector<TPolygon3D> &polys);
		/**
		  * Tries to assemble a set of segments into a set of closed polygons, returning the unused segments as another out parameter.
		  */
		void BASE_IMPEXP assemblePolygons(const std::vector<TSegment3D> &segms,std::vector<TPolygon3D> &polys,std::vector<TSegment3D> &remainder);
		/**
		  * Extracts all the polygons, including those formed from segments, from the set of objects.
		  */
		void BASE_IMPEXP assemblePolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys);
		/**
		  * Extracts all the polygons, including those formed from segments, from the set of objects.
		  */
		void BASE_IMPEXP assemblePolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys,std::vector<TObject3D> &remainder);
		/**
		  * Extracts all the polygons, including those formed from segments, from the set of objects.
		  */
		void BASE_IMPEXP assemblePolygons(const std::vector<TObject3D> &objs,std::vector<TPolygon3D> &polys,std::vector<TSegment3D> &remainder1,std::vector<TObject3D> &remainder2);

		/**
		  * Changes the value of the geometric epsilon.
		  * \sa geometryEpsilon,getEpsilon
		  */
		inline void setEpsilon(double nE)	{
			geometryEpsilon=nE;
		}
		/**
		  * Gets the value of the geometric epsilon.
		  * \sa geometryEpsilon,setEpsilon
		  */
		inline double getEpsilon()	{
			return geometryEpsilon;
		}

		/**
		  * Splits a 2D polygon into convex components.
		  */
		bool BASE_IMPEXP splitInConvexComponents(const TPolygon2D &poly,std::vector<TPolygon2D> &components);
		/**
		  * Splits a 3D polygon into convex components.
		  * \throw std::logic_error if the polygon can't be fit into a plane.
		  */
		bool BASE_IMPEXP splitInConvexComponents(const TPolygon3D &poly,std::vector<TPolygon3D> &components);

		/**
		  * Gets the bisector of a 2D segment.
		  */
		void BASE_IMPEXP getSegmentBisector(const TSegment2D &sgm,TLine2D &bis);
		/**
		  * Gets the bisector of a 3D segment.
		  */
		void BASE_IMPEXP getSegmentBisector(const TSegment3D &sgm,TPlane &bis);
		/**
		  * Gets the bisector of two lines or segments (implicit constructor will be used if necessary)
		  */
		void BASE_IMPEXP getAngleBisector(const TLine2D &l1,const TLine2D &l2,TLine2D &bis);
		/**
		  * Gets the bisector of two lines or segments (implicit constructor will be used if necessary)
		  * \throw std::logic_error if the lines do not fit in a single plane.
		  */
		void BASE_IMPEXP getAngleBisector(const TLine3D &l1,const TLine3D &l2,TLine3D &bis);

		/**
		  * Fast ray tracing method using polygons' properties.
		  * \sa CRenderizable::rayTrace
		  */
		bool BASE_IMPEXP traceRay(const std::vector<TPolygonWithPlane> &vec,const mrpt::poses::CPose3D &pose,double &dist);
		/**
		  * Fast ray tracing method using polygons' properties.
		  * \sa CRenderizable::rayTrace
		  */
		inline bool traceRay(const std::vector<TPolygon3D> &vec,const mrpt::poses::CPose3D &pose,double &dist)	{
			std::vector<TPolygonWithPlane> pwp;
			TPolygonWithPlane::getPlanes(vec,pwp);
			return traceRay(pwp,pose,dist);
		}

		/** Computes the cross product of two 3D vectors, returning a vector normal to both.
		  *  It uses the simple implementation:

		    \f[  v_out = \left(
					\begin{array}{c c c}
					\hat{i} ~ \hat{j} ~ \hat{k} \\
					x0 ~ y0 ~ z0 \\
					x1 ~ y1 ~ z1 \\
					\end{array} \right)
			\f]
		  */
		template<class T,class U,class V>
		inline void crossProduct3D(const T &v0,const U &v1,V& vOut)	{
			vOut[0]=v0[1]*v1[2]-v0[2]*v1[1];
			vOut[1]=v0[2]*v1[0]-v0[0]*v1[2];
			vOut[2]=v0[0]*v1[1]-v0[1]*v1[0];
		}

		//! \overload
		template<class T>
		inline void crossProduct3D(
			const std::vector<T> &v0,
			const std::vector<T> &v1,
			std::vector<T> &v_out )
		{
			ASSERT_(v0.size()==3)
			ASSERT_(v1.size()==3);
			v_out.resize(3);
			v_out[0] =  v0[1]*v1[2] - v0[2]*v1[1];
			v_out[1] = -v0[0]*v1[2] + v0[2]*v1[0];
			v_out[2] =  v0[0]*v1[1] - v0[1]*v1[0];
		}

		//! overload (returning a vector of size 3 by value).
		template<class VEC1,class VEC2>
		inline Eigen::Matrix<double,3,1> crossProduct3D(const VEC1 &v0,const VEC2 &v1)	{
			Eigen::Matrix<double,3,1> vOut;
			vOut[0]=v0[1]*v1[2]-v0[2]*v1[1];
			vOut[1]=v0[2]*v1[0]-v0[0]*v1[2];
			vOut[2]=v0[0]*v1[1]-v0[1]*v1[0];
			return vOut;
		}

		/** Computes the 3x3 skew symmetric matrix from a 3-vector or 3-array:
		  * \f[  M([x ~ y ~ z]^\top) = \left(
		  * 	\begin{array}{c c c}
		  * 	0 & -z & y \\
		  * 	z & 0 & -x \\
		  * 	-y & x & 0
		  * 	\end{array} \right)
		  * \f]
		  */
		template<class VECTOR,class MATRIX>
		inline void skew_symmetric3(const VECTOR &v,MATRIX& M)	{
			ASSERT_(v.size()==3)
			M.setSize(3,3);
			M.set_unsafe(0,0, 0); M.set_unsafe(0,1, -v[2]); M.set_unsafe(0,2, v[1]);
			M.set_unsafe(1,0, v[2]); M.set_unsafe(1,1, 0); M.set_unsafe(1,2, -v[0]);
			M.set_unsafe(2,0, -v[1]); M.set_unsafe(2,1, v[0]); M.set_unsafe(2,2, 0);
		}
		//! \overload
		template<class VECTOR>
		inline mrpt::math::CMatrixDouble33 skew_symmetric3(const VECTOR &v)	{
			mrpt::math::CMatrixDouble33 M(mrpt::math::UNINITIALIZED_MATRIX);
			skew_symmetric3(v,M);
			return M;
		}

		/** Computes the negative version of a 3x3 skew symmetric matrix from a 3-vector or 3-array:
		  * \f[  -M([x ~ y ~ z]^\top) = \left(
		  * 	\begin{array}{c c c}
		  * 	0 & z & -y \\
		  * 	-z & 0 & x \\
		  * 	y & -x & 0
		  * 	\end{array} \right)
		  * \f]
		  */
		template<class VECTOR,class MATRIX>
		inline void skew_symmetric3_neg(const VECTOR &v,MATRIX& M)	{
			ASSERT_(v.size()==3)
			M.setSize(3,3);
			M.set_unsafe(0,0, 0); M.set_unsafe(0,1, v[2]); M.set_unsafe(0,2, -v[1]);
			M.set_unsafe(1,0, -v[2]); M.set_unsafe(1,1, 0); M.set_unsafe(1,2, v[0]);
			M.set_unsafe(2,0, v[1]); M.set_unsafe(2,1, -v[0]); M.set_unsafe(2,2, 0);
		}
		//! \overload
		template<class VECTOR>
		inline mrpt::math::CMatrixDouble33 skew_symmetric3_neg(const VECTOR &v)	{
			mrpt::math::CMatrixDouble33 M(mrpt::math::UNINITIALIZED_MATRIX);
			skew_symmetric3_neg(v,M);
			return M;
		}


		/**
		  * Returns true if two 2D vectors are parallel. The arguments may be points, arrays, etc.
		  */
		template<class T,class U> inline bool vectorsAreParallel2D(const T &v1,const U &v2)	{
			return abs(v1[0]*v2[1]-v2[0]*v1[1])<geometryEpsilon;
		}

		/**
		  * Returns true if two 3D vectors are parallel. The arguments may be points, arrays, etc.
		  */
		template<class T,class U>
		inline bool vectorsAreParallel3D(const T &v1,const U &v2)	{
			if (abs(v1[0]*v2[1]-v2[0]*v1[1])>=geometryEpsilon) return false;
			if (abs(v1[1]*v2[2]-v2[1]*v1[2])>=geometryEpsilon) return false;
			return abs(v1[2]*v2[0]-v2[2]*v1[0])<geometryEpsilon;
		}

		/** Computes the closest point from a given point to a segment.
		  * \sa closestFromPointToLine
		  */
		void BASE_IMPEXP closestFromPointToSegment(
				const double &	Px,
				const double &	Py,
				const double &	x1,
				const double &	y1,
				const double &	x2,
				const double &	y2,
				double &out_x,
				double &out_y);

		/** Computes the closest point from a given point to a (infinite) line.
		  * \sa closestFromPointToSegment
		  */
		void BASE_IMPEXP closestFromPointToLine(
				const double &	Px,
				const double &	Py,
				const double &	x1,
				const double &	y1,
				const double &	x2,
				const double &	y2,
				double &out_x,
				double &out_y);

		/** Returns the square distance from a point to a line.
		  */
		double BASE_IMPEXP closestSquareDistanceFromPointToLine(
				const double &	Px,
				const double &	Py,
				const double &	x1,
				const double &	y1,
				const double &	x2,
				const double &	y2 );


		/** Returns the distance between 2 points in 2D. */
		template <typename T>
		T distanceBetweenPoints(const T x1,const T y1,const T x2,const T y2) {
			return std::sqrt( square(x1-x2)+square(y1-y2) );
		}

		/** Returns the distance between 2 points in 3D. */
		template <typename T>
		T distanceBetweenPoints(const T x1,const T y1,const T z1, const T x2,const T y2, const T z2) {
			return std::sqrt( square(x1-x2)+square(y1-y2)+square(z1-z2) );
		}

		/** Returns the square distance between 2 points in 2D. */
		template <typename T>
		T distanceSqrBetweenPoints(const T x1,const T y1,const T x2,const T y2) {
			return square(x1-x2)+square(y1-y2);
		}

		/** Returns the square distance between 2 points in 3D. */
		template <typename T>
		T distanceSqrBetweenPoints(const T x1,const T y1,const T z1, const T x2,const T y2, const T z2) {
			return square(x1-x2)+square(y1-y2)+square(z1-z2);
		}

		/** Computes the closest point from a given point to a segment, and returns that minimum distance.
		  */
		template <typename T>
		double minimumDistanceFromPointToSegment(
			const double Px,
			const double Py,
			const double x1,
			const double y1,
			const double x2,
			const double y2,
			T & out_x,
			T & out_y)
		{
			double ox, oy;
			closestFromPointToSegment(Px, Py, x1, y1, x2, y2, ox, oy);
			out_x = static_cast<T>(ox);
			out_y = static_cast<T>(oy);
			return distanceBetweenPoints(Px, Py, ox, oy);
		}


		/** Returns the intersection point, and if it exists, between two segments.
		  */
		bool  BASE_IMPEXP SegmentsIntersection(
					const double x1,const double y1,
					const double x2,const double y2,
					const double x3,const double y3,
					const double x4,const double y4,
					double &ix,double &iy);

		/** Returns the intersection point, and if it exists, between two segments.
		  */
		bool  BASE_IMPEXP SegmentsIntersection(
					const double x1,const double y1,
					const double x2,const double y2,
					const double x3,const double y3,
					const double x4,const double y4,
					float &ix,float &iy);

		/** Returns true if the 2D point (px,py) falls INTO the given polygon.
		  * \sa pointIntoQuadrangle
		  */
		bool  BASE_IMPEXP pointIntoPolygon2D(const double & px, const double & py, unsigned int polyEdges, const double *poly_xs, const double *poly_ys );

		/** Specialized method to check whether a point (x,y) falls into a quadrangle.
		  * \sa pointIntoPolygon2D
		  */
		template <typename T>
		bool pointIntoQuadrangle(
			T x, T y,
			T v1x, T v1y,
			T v2x, T v2y,
			T v3x, T v3y,
			T v4x, T v4y )
		{
			using mrpt::utils::sign;

			const T a1 = atan2( v1y - y , v1x - x );
			const T a2 = atan2( v2y - y , v2x - x );
			const T a3 = atan2( v3y - y , v3x - x );
			const T a4 = atan2( v4y - y , v4x - x );

			// The point is INSIDE iff all the signs of the angles between each vertex
			//  and the next one are equal.
			const T da1 = mrpt::math::wrapToPi( a2-a1 );
			const T da2 = mrpt::math::wrapToPi( a3-a2 );
			if (sign(da1)!=sign(da2)) return false;

			const T da3 = mrpt::math::wrapToPi( a4-a3 );
			if (sign(da2)!=sign(da3)) return false;

			const T da4 = mrpt::math::wrapToPi( a1-a4 );
			return (sign(da3)==sign(da4) && (sign(da4)==sign(da1)));
		}

		/** Returns the closest distance of a given 2D point to a polygon, or "0" if the point is INTO the polygon or its perimeter.
		  */
		double BASE_IMPEXP distancePointToPolygon2D(const double & px, const double & py, unsigned int polyEdges, const double *poly_xs, const double *poly_ys );

		/** Calculates the minimum distance between a pair of lines.
		  The lines are given by:
			- Line 1 = P1 + f (P2-P1)
			- Line 2 = P3 + f (P4-P3)
		  The Euclidean distance is returned in "dist", and the mid point between the lines in (x,y,z)
		  \return It returns false if there is no solution, i.e. lines are (almost, up to EPS) parallel.
		 */
		bool  BASE_IMPEXP minDistBetweenLines(
							const double &	p1_x, const double &	p1_y, const double & p1_z,
							const double &	p2_x, const double &	p2_y, const double & p2_z,
							const double &	p3_x, const double & p3_y, const double & p3_z,
							const double &	p4_x, const double & p4_y, const double & p4_z,
							double &x,   double &y,   double &z,
							double &dist);

		/** Returns whether two rotated rectangles intersect.
		 *  The first rectangle is not rotated and given by (R1_x_min,R1_x_max)-(R1_y_min,R1_y_max).
		 *  The second rectangle is given is a similar way, but it is internally rotated according
		 *   to the given coordinates translation (R2_pose_x,R2_pose_y,R2_pose_phi(radians)), relative
		 *   to the coordinates system of rectangle 1.
		 */
		bool  BASE_IMPEXP RectanglesIntersection(
					const double &	R1_x_min,	const double &	R1_x_max,
					const double &	R1_y_min,	const double &	R1_y_max,
					const double &	R2_x_min,	const double &	R2_x_max,
					const double &	R2_y_min,	const double &	R2_y_max,
					const double &	R2_pose_x,
					const double &	R2_pose_y,
					const double &	R2_pose_phi );

		/** Computes an axis base (a set of three 3D normal vectors) with the given vector being the first of them.
		  * NOTE: Make sure of passing all floats or doubles and that the template of the receiving matrix is of the same type!
		  *
		  *  If   \f$ d = [ dx ~ dy ~ dz ] \f$ is the input vector, then this function returns a matrix \f$ M \f$ such as:
		  *
		    \f[  M = \left(
					\begin{array}{c c c}
					v^1_x ~ v^2_x ~ v^3_x \\
					v^1_y ~ v^2_y ~ v^3_y \\
					v^1_z ~ v^2_z ~ v^3_z
					\end{array} \right)
			\f]
		  *
		  *   And the three normal vectors are computed as:

			  \f[ v^1 = \frac{d}{|d|}  \f]

			  If (dx!=0 or dy!=0):
				\f[ v^2 = \frac{[-dy ~ dx ~ 0 ]}{\sqrt{dx^2+dy^2}}  \f]
			  otherwise (the direction vector is vertical):
				\f[ v^2 = [1 ~ 0 ~ 0]  \f]

			  And finally, the third vector is the cross product of the others:

			    \f[ v^3 = v^1 \times v^2  \f]
		  *
		  * \return The 3x3 matrix (CMatrixTemplateNumeric<T>), containing one vector per column.
		  * \except Throws an std::exception on invalid input (i.e. null direction vector)
		  *
		  * (JLB @ 18-SEP-2007)
		  */
		template<class T>
		CMatrixTemplateNumeric<T> generateAxisBaseFromDirection( T dx, T dy, T dz )
		{
			MRPT_START

			if (dx==0 && dy==0 && dz==0)
				THROW_EXCEPTION("Invalid input: Direction vector is (0,0,0)!");

			CMatrixTemplateNumeric<T>	P(3,3);

			// 1st vector:
			T	n_xy = square(dx)+square(dy);
			T	n = sqrt(n_xy+square(dz));
			n_xy = sqrt(n_xy);
			P(0,0) = dx / n;
			P(1,0) = dy / n;
			P(2,0) = dz / n;

			// 2nd perpendicular vector:
			if (fabs(dx)>1e-4 || fabs(dy)>1e-4)
			{
				P(0,1) = -dy / n_xy;
				P(1,1) = dx / n_xy;
				P(2,1) = 0;
			}
			else
			{
				// Any vector in the XY plane will work:
				P(0,1) = 1;
				P(1,1) = 0;
				P(2,1) = 0;
			}

			// 3rd perpendicular vector: cross product of the two last vectors:
			P.col(2) = crossProduct3D(P.col(0),P.col(1));

			return P;
			MRPT_END
		}


		/** Compute a rotation exponential using the Rodrigues Formula.
		  * The rotation axis is given by \f$\vec{w}\f$, and the rotation angle must
		  * be computed using \f$ \theta = |\vec{w}|\f$. This is provided as a separate
		  * function primarily to allow fast and rough matrix exponentials using fast
		  * and rough approximations to \e A and \e B.
		  *
		  * \param w Vector about which to rotate.
		  * \param A \f$\frac{\sin \theta}{\theta}\f$
		  * \param B \f$\frac{1 - \cos \theta}{\theta^2}\f$
		  * \param R Matrix to hold the return value.
		  * \sa CPose3D
		  * \note Method from TooN (C) Tom Drummond (GNU GPL)
		  */
		template <typename VECTOR_LIKE, typename Precision, typename MATRIX_LIKE>
		inline void rodrigues_so3_exp(const VECTOR_LIKE& w, const Precision A,const Precision B,MATRIX_LIKE & R)
		{
			ASSERT_EQUAL_(w.size(),3)
			ASSERT_EQUAL_(R.getColCount(),3)
			ASSERT_EQUAL_(R.getRowCount(),3)
			{
				const Precision wx2 = (Precision)w[0]*w[0];
				const Precision wy2 = (Precision)w[1]*w[1];
				const Precision wz2 = (Precision)w[2]*w[2];
				R(0,0) = 1.0 - B*(wy2 + wz2);
				R(1,1) = 1.0 - B*(wx2 + wz2);
				R(2,2) = 1.0 - B*(wx2 + wy2);
			}
			{
				const Precision a = A*w[2];
				const Precision b = B*(w[0]*w[1]);
				R(0,1) = b - a;
				R(1,0) = b + a;
			}
			{
				const Precision a = A*w[1];
				const Precision b = B*(w[0]*w[2]);
				R(0,2) = b + a;
				R(2,0) = b - a;
			}
			{
				const Precision a = A*w[0];
				const Precision b = B*(w[1]*w[2]);
				R(1,2) = b - a;
				R(2,1) = b + a;
			}
		}

		/** @} */  // end of misc. geom. methods

		/** @} */  // end of grouping

	} // End of namespace

} // End of namespace
#endif
