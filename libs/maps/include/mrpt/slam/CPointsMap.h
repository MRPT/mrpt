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
#ifndef CPOINTSMAP_H
#define CPOINTSMAP_H

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/slam/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/PLY_import_export.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::poses;
	using namespace mrpt::math;

	class CSimplePointsMap;
	class CMultiMetricMap;
	class CColouredPointsMap;
	class COccupancyGridMap2D;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointsMap , CMetricMap, MAPS_IMPEXP )

	/** A cloud of points in 2D or 3D, which can be built from a sequence of laser scans or other sensors.
	 *  This is a virtual class, thus only a derived class can be instantiated by the user. The user most usually wants to use CSimplePointsMap.
	 * \sa CMetricMap, CPoint, mrpt::utils::CSerializable
	 */
	class MAPS_IMPEXP CPointsMap :
		public CMetricMap,
		public mrpt::utils::KDTreeCapable,
		public mrpt::utils::PLY_Importer,
		public mrpt::utils::PLY_Exporter

	{
		friend class CMultiMetricMap;
		friend class CMultiMetricMapPDF;
		friend class CSimplePointsMap;
		friend class CColouredPointsMap;
		friend class COccupancyGridMap2D;

		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CPointsMap )

	 protected:
		 std::vector<float>     x,y,z;        //!< The points coordinates
		 std::vector<uint32_t>  pointWeight;  //!< The points weights

		 CSinCosLookUpTableFor2DScans  m_scans_sincos_cache; //!< Cache of sin/cos values for the latest 2D scan geometries.

		 /** Auxiliary variables used in "getLargestDistanceFromOrigin"
		   * \sa getLargestDistanceFromOrigin
		   */
		 mutable float	m_largestDistanceFromOrigin;

		 /** Auxiliary variables used in "getLargestDistanceFromOrigin"
		   * \sa getLargestDistanceFromOrigin
		   */
		 mutable bool	m_largestDistanceFromOriginIsUpdated;

		 void mark_as_modified() const; //!< Called only by this class or children classes, set m_largestDistanceFromOriginIsUpdated=false and such.



		/** @name Virtual methods that MUST be implemented by children classes of KDTreeCapable
			@{ */
		/** Must return the number of data points */
		virtual size_t kdtree_get_point_count() const;

		/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
		virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const;

		/** @} */

	 public:
		 /** Constructor
		   */
		 CPointsMap();

		 /** Virtual destructor.
		   */
		 virtual ~CPointsMap();



		/** Returns the square distance from the 2D point (x0,y0) to the closest correspondence in the map.
		  */
		virtual float squareDistanceToClosestCorrespondence(
			float   x0,
			float   y0 ) const;

		inline float squareDistanceToClosestCorrespondenceT(const TPoint2D &p0) const	{
			return squareDistanceToClosestCorrespondence(static_cast<float>(p0.x),static_cast<float>(p0.y));
		}


		 /** With this struct options are provided to the observation insertion process.
		  * \sa CObservation::insertIntoPointsMap
		  */
		 struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		 {
			/** Initilization of default parameters */
			TInsertionOptions( );
			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source,const std::string &section);
			/** See utils::CLoadableOptions */
			void  dumpToTextStream(CStream	&out) const;

			float   minDistBetweenLaserPoints;   //!< The minimum distance between points (in 3D): If two points are too close, one of them is not inserted into the map. Default is 0.02 meters.
			bool    addToExistingPointsMap;      //!< Applicable to "loadFromRangeScan" only! If set to false, the points from the scan are loaded, clearing all previous content. Default is false.
			bool    also_interpolate;            //!< If set to true, far points (<1m) are interpolated with samples at "minDistSqrBetweenLaserPoints" intervals (Default is false).
			bool    disableDeletion;             //!< If set to false (default=true) points in the same plane as the inserted scan and inside the free space, are erased: i.e. they don't exist yet.
			bool    fuseWithExisting;            //!< If set to true (default=false), inserted points are "fused" with previously existent ones. This shrink the size of the points map, but its slower.
			bool    isPlanarMap;                 //!< If set to true, only HORIZONTAL (in the XY plane) measurements will be inserted in the map (Default value is false, thus 3D maps are generated). \sa	horizontalTolerance
			float   horizontalTolerance;	     //!< The tolerance in rads in pitch & roll for a laser scan to be considered horizontal, considered only when isPlanarMap=true (default=0).
			float   maxDistForInterpolatePoints; //!< The maximum distance between two points to interpolate between them (ONLY when also_interpolate=true)

   		 };

		TInsertionOptions insertionOptions; //!< The options used when inserting observations in the map

		 /** Options used when evaluating "computeObservationLikelihood" in the derived classes.
		  * \sa CObservation::computeObservationLikelihood
		  */
		 struct MAPS_IMPEXP TLikelihoodOptions: public utils::CLoadableOptions
		 {
			/** Initilization of default parameters
			 */
			TLikelihoodOptions( );
			virtual ~TLikelihoodOptions() {}

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** See utils::CLoadableOptions */
			void  dumpToTextStream(CStream	&out) const;

			void writeToStream(CStream &out) const;		//!< Binary dump to stream - for usage in derived classes' serialization
			void readFromStream(CStream &in);			//!< Binary dump to stream - for usage in derived classes' serialization

			double 		sigma_dist; //!< Sigma (standard deviation, in meters) of the exponential used to model the likelihood (default= 0.5meters)
			double 		max_corr_distance; //!< Maximum distance in meters to consider for the numerator divided by "sigma_dist", so that each point has a minimum (but very small) likelihood to avoid underflows (default=1.0 meters)
			uint32_t	decimation; //!< Speed up the likelihood computation by considering only one out of N rays (default=10)
		 };

		 TLikelihoodOptions  likelihoodOptions;

		 /** Virtual assignment operator, to be implemented in derived classes.
		   */
		 virtual void  copyFrom(const CPointsMap &obj) = 0;

		/** Insert the contents of another map into this one, fusing the previous content with the new one.
		 *    This means that points very close to existing ones will be "fused", rather than "added". This prevents
		 *     the unbounded increase in size of these class of maps.
		 *		NOTICE that "otherMap" is neither translated nor rotated here, so if this is desired it must done
		 *		 before calling this method.
		 * \param otherMap The other map whose points are to be inserted into this one.
		 * \param minDistForFuse Minimum distance (in meters) between two points, each one in a map, to be considered the same one and be fused rather than added.
		 * \param notFusedPoints If a pointer is supplied, this list will contain at output a list with a "bool" value per point in "this" map. This will be false/true according to that point having been fused or not.
		 * \sa loadFromRangeScan, addFrom
		 */
		virtual void  fuseWith(
			CPointsMap			*anotherMap,
			float				minDistForFuse  = 0.02f,
			std::vector<bool>	*notFusedPoints = NULL) = 0;

		/** Adds all the points from \a anotherMap to this map, without fusing.
		  *  This operation can be also invoked via the "+=" operator, for example:
		  *  \code
		  *   CSimplePointsMap m1, m2;
		  *   ...
		  *   m1.addFrom( m2 );  // Add all points of m2 to m1
		  *   m1 += m2;          // Exactly the same than above
		  *  \endcode
		  * \note The method in CPointsMap is generic but derived classes may redefine this virtual method to another one more optimized.
		  */
		virtual void  addFrom(const CPointsMap &anotherMap);

		/** This operator is synonymous with \a addFrom. \sa addFrom */
		inline void operator +=(const CPointsMap &anotherMap)
		{
			this->addFrom(anotherMap);
		}

		/** Transform the range scan into a set of cartessian coordinated
		  *	 points. The options in "insertionOptions" are considered in this method.
		  * \param rangeScan The scan to be inserted into this map
		  * \param robotPose The robot 3D pose, default to (0,0,0|0deg,0deg,0deg). It is used to compute the sensor pose relative to the robot actual pose. Recall sensor pose is embeded in the observation class.
		  *
		  *   NOTE: Only ranges marked as "valid=true" in the observation will be inserted
		  *
		  * \sa CObservation2DRangeScan
		  */
		virtual void  loadFromRangeScan(
				const CObservation2DRangeScan &rangeScan,
				const CPose3D				  *robotPose = NULL ) = 0;

		/** Load from a text file. In each line there are a point coordinates.
		 *   Returns false if any error occured, true elsewere.
		 */
		virtual bool  load2D_from_text_file(std::string file) = 0;

		/** Load from a text file. In each line there are a point coordinates.
		 *   Returns false if any error occured, true elsewere.
		 */
		virtual bool  load3D_from_text_file(std::string file) = 0;

		/**  Save to a text file. In each line there are a point coordinates.
		 *		Returns false if any error occured, true elsewere.
		 */
		bool  save2D_to_text_file(const std::string &file) const;

		/** Save to a text file. In each line there are a point coordinates.
		 *     Returns false if any error occured, true elsewere.
		 */
		bool  save3D_to_text_file(const std::string &file)const;

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			)const
		{
			std::string		fil( filNamePrefix + std::string(".txt") );
			save3D_to_text_file( fil );
		}


		/** Returns the number of stored points in the map.
		 */
		size_t size() const;

		/** Returns the number of stored points in the map (DEPRECATED, use "size()" instead better)
		 */
		size_t getPointsCount() const;

		/** Access to a given point from map, as a 2D point. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,CPoint2D &p) const;

		/** Access to a given point from map, as a 3D point. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,CPoint3D &p) const;

		/** Access to a given point from map, as a 3D point. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,mrpt::math::TPoint3D &p) const;

		/** Access to a given point from map, as a 2D point. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,mrpt::math::TPoint2D &p) const;

		/** Access to a given point from map. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,float &x,float &y) const;

		/** Access to a given point from map. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		unsigned long  getPoint(size_t index,float &x,float &y,float &z) const;


		/** Access to a given point from map, and its colors, if the map defines them (othersise, R=G=B=1.0). First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void getPoint( size_t index, float &x, float &y, float &z, float &R, float &G, float &B ) const
		{
			getPoint(index,x,y,z);
			R=G=B=1;
		}

		/** Returns true if the point map has a color field for each point */
		virtual bool hasColorPoints() const { return false; }

		/** Changes a given point from map, as a 2D point. First index is 0.
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void  setPoint(size_t index,CPoint2D &p)=0;

		/** Changes a given point from map, as a 3D point. First index is 0.
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void  setPoint(size_t index,CPoint3D &p)=0;

		/** Changes a given point from map. First index is 0.
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void  setPoint(size_t index,float x, float y)=0;

		/** Changes a given point from map. First index is 0.
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void  setPoint(size_t index,float x, float y, float z)=0;

		/** Provides a direct access to points buffer, or NULL if there is no points in the map.
		  */
		void  getPointsBuffer( size_t &outPointsCount, const float *&xs, const float *&ys, const float *&zs ) const;

		/** Provides a direct access to a read-only reference of the internal point buffer. \sa getAllPoints */
		inline const std::vector<float> & getPointsBufferRef_x() const { return x; }
		/** Provides a direct access to a read-only reference of the internal point buffer. \sa getAllPoints */
		inline const std::vector<float> & getPointsBufferRef_y() const { return y; }
		/** Provides a direct access to a read-only reference of the internal point buffer. \sa getAllPoints */
		inline const std::vector<float> & getPointsBufferRef_z() const { return z; }

		/** Returns a copy of the 2D/3D points as a std::vector of float coordinates.
		  * If decimation is greater than 1, only 1 point out of that number will be saved in the output, effectively performing a subsampling of the points.
		  * \sa getPointsBufferRef_x, getPointsBufferRef_y, getPointsBufferRef_z
		  * \tparam VECTOR can be std::vector<float or double> or any row/column Eigen::Array or Eigen::Matrix (this includes mrpt::vector_float and mrpt::vector_double).
		  */
		template <class VECTOR>
		void  getAllPoints( VECTOR &xs, VECTOR &ys, VECTOR &zs, size_t decimation = 1 ) const
		{
			MRPT_START
			ASSERT_(decimation>0)
			const size_t Nout = x.size() / decimation;
			xs.resize(Nout);
			ys.resize(Nout);
			zs.resize(Nout);
			size_t idx_in, idx_out;
			for (idx_in=0,idx_out=0;idx_out<Nout;idx_in+=decimation,++idx_out)
			{
				xs[idx_out]=x[idx_in];
				ys[idx_out]=y[idx_in];
				zs[idx_out]=z[idx_in];
			}
			MRPT_END
		}

		inline void getAllPoints(std::vector<TPoint3D> &ps,size_t decimation=1) const	{
			std::vector<float> dmy1,dmy2,dmy3;
			getAllPoints(dmy1,dmy2,dmy3,decimation);
			ps.resize(dmy1.size());
			for (size_t i=0;i<dmy1.size();i++)	{
				ps[i].x=static_cast<double>(dmy1[i]);
				ps[i].y=static_cast<double>(dmy2[i]);
				ps[i].z=static_cast<double>(dmy3[i]);
			}
		}

		/** Returns a copy of the 2D/3D points as a std::vector of float coordinates.
		  * If decimation is greater than 1, only 1 point out of that number will be saved in the output, effectively performing a subsampling of the points.
		  * \sa setAllPoints
		  */
		void  getAllPoints( std::vector<float> &xs, std::vector<float> &ys, size_t decimation = 1 ) const;

		inline void getAllPoints(std::vector<TPoint2D> &ps,size_t decimation=1) const	{
			std::vector<float> dmy1,dmy2;
			getAllPoints(dmy1,dmy2,decimation);
			ps.resize(dmy1.size());
			for (size_t i=0;i<dmy1.size();i++)	{
				ps[i].x=static_cast<double>(dmy1[i]);
				ps[i].y=static_cast<double>(dmy2[i]);
			}
		}

		/** Provides a way to insert individual points into the map */
		virtual void  insertPoint( float x, float y, float z = 0 ) = 0;

		/** Provides a way to insert individual points into the map */
		inline void  insertPoint( const CPoint3D &p ) {
			insertPoint(p.x(),p.y(),p.z());
		}

		/** Provides a way to insert individual points into the map */
		inline void  insertPoint( const mrpt::math::TPoint3D &p ) {
			insertPoint(p.x,p.y,p.z);
		}

		/** Reserves memory for a given number of points: the size of the map does not change, it only reserves the memory.
		  *  This is useful for situations where it is approximately known the final size of the map. This method is more
		  *  efficient than constantly increasing the size of the buffers. Refer to the STL C++ library's "reserve" methods.
		  */
		virtual void reserve(size_t newLength) = 0;

		/** Set all the points at once from vectors with X,Y and Z coordinates. \sa getAllPoints */
		virtual void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y,const std::vector<float> &Z) = 0;
		/** Set all the points at once from vectors with X and Y coordinates (Z=0). \sa getAllPoints */
		virtual void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y) = 0;

		/** Delete points out of the given "z" axis range have been removed.
		  */
		void  clipOutOfRangeInZ(float zMin, float zMax);

		/** Delete points which are more far than "maxRange" away from the given "point".
		  */
		void  clipOutOfRange(const CPoint2D	&point, float maxRange);

		/** Remove from the map the points marked in a bool's array as "true".
		  *
		  * \exception std::exception If mask size is not equal to points count.
		  */
		virtual void  applyDeletionMask( std::vector<bool> &mask ) = 0;

		/** Computes the matchings between this and another 2D/3D points map.
		   This includes finding:
				- The set of points pairs in each map
				- The mean squared distance between corresponding pairs.
		   This method is the most time critical one into the ICP algorithm.

		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
		 * \param  maxDistForCorrespondence [IN] Maximum 2D distance between two points to be matched.
		 * \param  maxAngularDistForCorrespondence [IN] Maximum angular distance in radians to allow far points to be matched.
		 * \param  angularDistPivotPoint      [IN] The point from which to measure the "angular distances"
		 * \param  correspondences			  [OUT] The detected matchings pairs.
		 * \param  correspondencesRatio		  [OUT] The number of correct correspondences.
		 * \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
		 * \param  covariance				  [OUT] The resulting matching covariance 3x3 matrix, or NULL if undesired.
		 * \param  onlyKeepTheClosest		  [OUT] Returns only the closest correspondence (default=false)
		 *
		 * \sa computeMatching3DWith
		 */
		void  computeMatchingWith2D(
				const CMetricMap						*otherMap,
				const CPose2D							&otherMapPose,
				float									maxDistForCorrespondence,
				float									maxAngularDistForCorrespondence,
				const CPose2D							&angularDistPivotPoint,
				TMatchingPairList						&correspondences,
				float									&correspondencesRatio,
				float									*sumSqrDist	= NULL,
				bool									onlyKeepTheClosest = false,
				bool									onlyUniqueRobust = false ) const;

		/** Computes the matchings between this and another 3D points map - method used in 3D-ICP.
		   This method finds the set of point pairs in each map.

		   The method is the most time critical one into the ICP algorithm.

		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
		 * \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
		 * \param  maxAngularDistForCorrespondence [IN] In radians: The aim is to allow larger distances to more distant correspondences.
		 * \param  angularDistPivotPoint      [IN] The point used to calculate distances from in both maps.
		 * \param  correspondences			  [OUT] The detected matchings pairs.
		 * \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
		 * \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
		 * \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
		 *
		 * \sa compute3DMatchingRatio
		 */
		void  computeMatchingWith3D(
			const CMetricMap						*otherMap,
			const CPose3D							&otherMapPose,
			float									maxDistForCorrespondence,
			float									maxAngularDistForCorrespondence,
			const CPoint3D							&angularDistPivotPoint,
			TMatchingPairList						&correspondences,
			float									&correspondencesRatio,
			float									*sumSqrDist	= NULL,
			bool									onlyKeepTheClosest = true,
			bool									onlyUniqueRobust = false ) const;


		/** Replace each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const CPose2D &b);

		/** Replace each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const CPose3D &b);

		/** Copy all the points from "other" map to "this", replacing each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const CPointsMap &other, const CPose3D &b);

		/** Returns true if the map is empty/no observation has been inserted.
		   */
		virtual bool  isEmpty() const;

		/** STL-like method to check whether the map is empty: */
		inline bool  empty() const { return isEmpty(); }

		/** Returns a 3D object representing the map.
		  *  The color of the points is given by the static variables: COLOR_3DSCENE_R,COLOR_3DSCENE_G,COLOR_3DSCENE_B
		  * \sa mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE
		  */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;


		/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
		 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
		 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
		 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
		 *
		 * \return The matching ratio [0,1]
		 * \sa computeMatchingWith2D
		 */
		float  compute3DMatchingRatio(
				const CMetricMap						*otherMap,
				const CPose3D							&otherMapPose,
				float									minDistForCorr = 0.10f,
				float									minMahaDistForCorr = 2.0f
				) const;

		/** This method returns the largest distance from the origin to any of the points, such as a sphere centered at the origin with this radius cover ALL the points in the map (the results are buffered, such as, if the map is not modified, the second call will be much faster than the first one).
		  */
		float  getLargestDistanceFromOrigin() const;

		/** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if there are no points. */
		void boundingBox( float &min_x,float &max_x,float &min_y,float &max_y,float &min_z,float &max_z ) const;

		inline void boundingBox(TPoint3D &pMin,TPoint3D &pMax) const	{
			float dmy1,dmy2,dmy3,dmy4,dmy5,dmy6;
			boundingBox(dmy1,dmy2,dmy3,dmy4,dmy5,dmy6);
			pMin.x=dmy1;
			pMin.y=dmy2;
			pMin.z=dmy3;
			pMax.x=dmy4;
			pMax.y=dmy5;
			pMax.z=dmy6;
		}

		void extractCylinder( const CPoint2D &center, const double radius, const double zmin, const double zmax, CPointsMap *outMap );


		/** The color [0,1] of points when extracted from getAs3DObject (default=blue) */
		static float COLOR_3DSCENE_R;
		static float COLOR_3DSCENE_G;
		static float COLOR_3DSCENE_B;


		/** Computes the likelihood of taking a given observation from a given pose in the world being modeled with this map.
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF
		 * \note In CPointsMap this method is virtual so it can be redefined in derived classes, if desired.
		 */
		virtual double computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

		protected:
			/** @name PLY Import virtual methods to implement in base classes
			    @{ */
			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_face */
			virtual void PLY_import_set_face_count(const size_t N) {  }

			/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color = NULL);
			/** @} */

			/** @name PLY Export virtual methods to implement in base classes
			    @{ */

			/** In a base class, return the number of vertices */
			virtual size_t PLY_export_get_vertex_count() const;

			/** In a base class, return the number of faces */
			virtual size_t PLY_export_get_face_count() const { return 0; }

			/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point.
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_export_get_vertex(
				const size_t idx,
				mrpt::math::TPoint3Df &pt,
				bool &pt_has_color,
				mrpt::utils::TColorf &pt_color) const;

			/** @} */

	}; // End of class def.

	} // End of namespace

	namespace global_settings
	{
		/** The size of points when exporting with getAs3DObject() (default=3.0)
		  * Affects to:
		  *		- mrpt::slam::CPointsMap and all its children classes.
		  */
		extern MAPS_IMPEXP float POINTSMAPS_3DOBJECT_POINTSIZE;
	}
} // End of namespace

#endif
