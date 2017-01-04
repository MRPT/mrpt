/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOINTSMAP_H
#define CPOINTSMAP_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/PLY_import_export.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/maps/link_pragmas.h>
#include <mrpt/utils/adapters.h>

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM( mrpt::maps::CPointsMap )

namespace mrpt
{
/** \ingroup mrpt_maps_grp */
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPointsMap , CMetricMap, MAPS_IMPEXP )

	// Forward decls. needed to make its static methods friends of CPointsMap
	namespace detail {
		template <class Derived> struct loadFromRangeImpl;
		template <class Derived> struct pointmap_traits;
	}


	/** A cloud of points in 2D or 3D, which can be built from a sequence of laser scans or other sensors.
	 *  This is a virtual class, thus only a derived class can be instantiated by the user. The user most usually wants to use CSimplePointsMap.
	 *
	 *  This class implements generic version of mrpt::maps::CMetric::insertObservation() accepting these types of sensory data:
	 *   - mrpt::obs::CObservation2DRangeScan: 2D range scans
	 *   - mrpt::obs::CObservation3DRangeScan: 3D range scans (Kinect, etc...)
	 *   - mrpt::obs::CObservationRange: IRs, Sonars, etc.
	 *   - mrpt::obs::CObservationVelodyneScan
	 *
	 * Loading and saving in the standard LAS LiDAR point cloud format is supported by installing `libLAS` and including the
	 * header `<mrpt/maps/CPointsMaps_liblas.h>` in your program. Since MRPT 1.5.0 there is no need to build MRPT against libLAS to use this feature.
	 * See LAS functions in \ref mrpt_maps_liblas_grp.
	 *
	 * \sa CMetricMap, CPoint, mrpt::utils::CSerializable
	  * \ingroup mrpt_maps_grp
	 */
	class MAPS_IMPEXP CPointsMap :
		public CMetricMap,
		public mrpt::math::KDTreeCapable<CPointsMap>,
		public mrpt::utils::PLY_Importer,
		public mrpt::utils::PLY_Exporter
	{
		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CPointsMap )
		// This must be added for declaration of MEX-related functions
		DECLARE_MEX_CONVERSION

	protected:
		/** Helper struct used for \a internal_loadFromRangeScan2D_prepareOneRange() */
		struct MAPS_IMPEXP TLaserRange2DInsertContext {
			TLaserRange2DInsertContext(const mrpt::obs::CObservation2DRangeScan  &_rangeScan) : HM(mrpt::math::UNINITIALIZED_MATRIX), rangeScan(_rangeScan)
			{ }
		 mrpt::math::CMatrixDouble44	HM;  //!< Homog matrix of the local sensor pose within the robot
			const mrpt::obs::CObservation2DRangeScan  &rangeScan;
			std::vector<float>         fVars;  //!< Extra variables to be used as desired by the derived class.
			std::vector<unsigned int>  uVars;
			std::vector<uint8_t>       bVars;
		};

		/** Helper struct used for \a internal_loadFromRangeScan3D_prepareOneRange() */
		struct MAPS_IMPEXP TLaserRange3DInsertContext {
			TLaserRange3DInsertContext(const mrpt::obs::CObservation3DRangeScan  &_rangeScan) : HM(mrpt::math::UNINITIALIZED_MATRIX), rangeScan(_rangeScan)
			{ }
			mrpt::math::CMatrixDouble44	HM;  //!< Homog matrix of the local sensor pose within the robot
			const mrpt::obs::CObservation3DRangeScan  &rangeScan;
			float scan_x, scan_y,scan_z; //!< In \a internal_loadFromRangeScan3D_prepareOneRange, these are the local coordinates of the scan points being inserted right now.
			std::vector<float>         fVars;  //!< Extra variables to be used as desired by the derived class.
			std::vector<unsigned int>  uVars;
			std::vector<uint8_t>       bVars;
		};

	 public:
		 CPointsMap();            //!< Ctor
		 virtual ~CPointsMap();   //!< Virtual destructor.

		// --------------------------------------------
		/** @name Pure virtual interfaces to be implemented by any class derived from CPointsMap
			@{ */

		/** Reserves memory for a given number of points: the size of the map does not change, it only reserves the memory.
		  *  This is useful for situations where it is approximately known the final size of the map. This method is more
		  *  efficient than constantly increasing the size of the buffers. Refer to the STL C++ library's "reserve" methods.
		  */
		virtual void reserve(size_t newLength) = 0;

		/** Resizes all point buffers so they can hold the given number of points: newly created points are set to default values,
		  *  and old contents are not changed.
		  * \sa reserve, setPoint, setPointFast, setSize
		  */
		virtual void resize(size_t newLength) = 0;

		/** Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents
		  *  and leaving all points to default values.
		  * \sa reserve, setPoint, setPointFast, setSize
		  */
		virtual void setSize(size_t newLength) = 0;

		/** Changes the coordinates of the given point (0-based index), *without* checking for out-of-bounds and *without* calling mark_as_modified()  \sa setPoint */
		virtual void  setPointFast(size_t index,float x, float y, float z)=0;

		/** The virtual method for \a insertPoint() *without* calling mark_as_modified()   */
		virtual void  insertPointFast( float x, float y, float z = 0 ) = 0;

		 /** Virtual assignment operator, copies as much common data (XYZ, color,...) as possible from the source map into this one. */
		 virtual void  copyFrom(const CPointsMap &obj) = 0;

		/** Get all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...
		  *  Unlike getPointAllFields(), this method does not check for index out of bounds
		  * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
		  */
		virtual void  getPointAllFieldsFast( const size_t index, std::vector<float> & point_data ) const = 0;

		/** Set all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...
		  *  Unlike setPointAllFields(), this method does not check for index out of bounds
		  * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
		  */
		virtual void  setPointAllFieldsFast( const size_t index, const std::vector<float> & point_data ) = 0;

	protected:

		/** Auxiliary method called from within \a addFrom() automatically, to finish the copying of class-specific data  */
		virtual void  addFrom_classSpecific(const CPointsMap &anotherMap, const size_t nPreviousPoints) = 0;

	public:

		/** @} */
		// --------------------------------------------


		/** Returns the square distance from the 2D point (x0,y0) to the closest correspondence in the map.
		  */
		virtual float squareDistanceToClosestCorrespondence(
			float   x0,
			float   y0 ) const MRPT_OVERRIDE;

		inline float squareDistanceToClosestCorrespondenceT(const mrpt::math::TPoint2D &p0) const	{
			return squareDistanceToClosestCorrespondence(static_cast<float>(p0.x),static_cast<float>(p0.y));
		}


		 /** With this struct options are provided to the observation insertion process.
		  * \sa CObservation::insertIntoPointsMap
		  */
		 struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		 {
			/** Initilization of default parameters */
			TInsertionOptions( );
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			float   minDistBetweenLaserPoints;   //!< The minimum distance between points (in 3D): If two points are too close, one of them is not inserted into the map. Default is 0.02 meters.
			bool    addToExistingPointsMap;      //!< Applicable to "loadFromRangeScan" only! If set to false, the points from the scan are loaded, clearing all previous content. Default is false.
			bool    also_interpolate;            //!< If set to true, far points (<1m) are interpolated with samples at "minDistSqrBetweenLaserPoints" intervals (Default is false).
			bool    disableDeletion;             //!< If set to false (default=true) points in the same plane as the inserted scan and inside the free space, are erased: i.e. they don't exist yet.
			bool    fuseWithExisting;            //!< If set to true (default=false), inserted points are "fused" with previously existent ones. This shrink the size of the points map, but its slower.
			bool    isPlanarMap;                 //!< If set to true, only HORIZONTAL (in the XY plane) measurements will be inserted in the map (Default value is false, thus 3D maps are generated). \sa	horizontalTolerance
			float   horizontalTolerance;	     //!< The tolerance in rads in pitch & roll for a laser scan to be considered horizontal, considered only when isPlanarMap=true (default=0).
			float   maxDistForInterpolatePoints; //!< The maximum distance between two points to interpolate between them (ONLY when also_interpolate=true)
			bool    insertInvalidPoints;             //!< Points with x,y,z coordinates set to zero will also be inserted

			void writeToStream(mrpt::utils::CStream &out) const;		//!< Binary dump to stream - for usage in derived classes' serialization
			void readFromStream(mrpt::utils::CStream &in);			//!< Binary dump to stream - for usage in derived classes' serialization
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
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			void writeToStream(mrpt::utils::CStream &out) const;		//!< Binary dump to stream - for usage in derived classes' serialization
			void readFromStream(mrpt::utils::CStream &in);			//!< Binary dump to stream - for usage in derived classes' serialization

            double 		sigma_dist; //!< Sigma squared (variance, in meters) of the exponential used to model the likelihood (default= 0.5^2 meters)
			double 		max_corr_distance; //!< Maximum distance in meters to consider for the numerator divided by "sigma_dist", so that each point has a minimum (but very small) likelihood to avoid underflows (default=1.0 meters)
			uint32_t	decimation; //!< Speed up the likelihood computation by considering only one out of N rays (default=10)
		 };

		 TLikelihoodOptions  likelihoodOptions;


		/** Adds all the points from \a anotherMap to this map, without fusing.
		  *  This operation can be also invoked via the "+=" operator, for example:
		  *  \code
		  *   mrpt::maps::CSimplePointsMap m1, m2;
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

		/** Insert the contents of another map into this one with some geometric transformation, without fusing close points.
		 * \param otherMap The other map whose points are to be inserted into this one.
		 * \param otherPose The pose of the other map in the coordinates of THIS map
		 * \sa fuseWith, addFrom
		 */
		void  insertAnotherMap(
			const CPointsMap	*otherMap,
			const mrpt::poses::CPose3D		&otherPose);

		// --------------------------------------------------
		/** @name File input/output methods
			@{ */

		/** Load from a text file. Each line should contain an "X Y" coordinate pair, separated by whitespaces.
		 *   Returns false if any error occured, true elsewere.
		 */
		inline bool  load2D_from_text_file(const std::string &file) { return load2Dor3D_from_text_file(file,false); }

		/** Load from a text file. Each line should contain an "X Y Z" coordinate tuple, separated by whitespaces.
		 *   Returns false if any error occured, true elsewere.
		 */
		inline bool  load3D_from_text_file(const std::string &file) { return load2Dor3D_from_text_file(file,true); }

		/** 2D or 3D generic implementation of \a load2D_from_text_file and load3D_from_text_file */
		bool  load2Dor3D_from_text_file(const std::string &file, const bool is_3D);

		/**  Save to a text file. Each line will contain "X Y" point coordinates.
		 *		Returns false if any error occured, true elsewere.
		 */
		bool  save2D_to_text_file(const std::string &file) const;

		/**  Save to a text file. Each line will contain "X Y Z" point coordinates.
		 *     Returns false if any error occured, true elsewere.
		 */
		bool  save3D_to_text_file(const std::string &file)const;

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface) */
		void  saveMetricMapRepresentationToFile(const std::string	&filNamePrefix)const MRPT_OVERRIDE
		{
			std::string		fil( filNamePrefix + std::string(".txt") );
			save3D_to_text_file( fil );
		}

		/** Save the point cloud as a PCL PCD file, in either ASCII or binary format (requires MRPT built against PCL) \return false on any error */
		virtual bool savePCDFile(const std::string &filename, bool save_as_binary) const;

		/** Load the point cloud from a PCL PCD file (requires MRPT built against PCL) \return false on any error */
		virtual bool loadPCDFile(const std::string &filename);

		/** @} */ // End of: File input/output methods
		// --------------------------------------------------

		/** Returns the number of stored points in the map.
		 */
		inline size_t size() const { return x.size(); }

		/** Access to a given point from map, as a 2D point. First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused), or 1 if weights are not used.
		 * \exception Throws std::exception on index out of bound.
		 * \sa setPoint, getPointFast
		 */
		unsigned long  getPoint(size_t index,float &x,float &y,float &z) const;
		/// \overload
		unsigned long  getPoint(size_t index,float &x,float &y) const;
		/// \overload
		unsigned long  getPoint(size_t index,double &x,double &y,double &z) const;
		/// \overload
		unsigned long  getPoint(size_t index,double &x,double &y) const;
		/// \overload
		inline unsigned long  getPoint(size_t index,mrpt::math::TPoint2D &p) const  { return getPoint(index,p.x,p.y); }
		/// \overload
		inline unsigned long  getPoint(size_t index,mrpt::math::TPoint3D &p) const  { return getPoint(index,p.x,p.y,p.z); }

		/** Access to a given point from map, and its colors, if the map defines them (othersise, R=G=B=1.0). First index is 0.
		 * \return The return value is the weight of the point (the times it has been fused)
		 * \exception Throws std::exception on index out of bound.
		 */
		virtual void getPoint( size_t index, float &x, float &y, float &z, float &R, float &G, float &B ) const
		{
			getPoint(index,x,y,z);
			R=G=B=1;
		}

		/** Just like \a getPoint() but without checking out-of-bound index and without returning the point weight, just XYZ.
		 */
		inline void getPointFast(size_t index,float &x,float &y,float &z) const { x=this->x[index]; y=this->y[index]; z=this->z[index]; }

		/** Returns true if the point map has a color field for each point */
		virtual bool hasColorPoints() const { return false; }

		/** Changes a given point from map, with Z defaulting to 0 if not provided.
		 * \exception Throws std::exception on index out of bound.
		 */
		inline void  setPoint(size_t index,float x, float y, float z) {
			ASSERT_BELOW_(index,this->size())
			setPointFast(index,x,y,z);
			mark_as_modified();
		}
		/// \overload
		inline void  setPoint(size_t index,mrpt::math::TPoint2D &p) {  setPoint(index,p.x,p.y,0); }
		/// \overload
		inline void  setPoint(size_t index,mrpt::math::TPoint3D &p)  { setPoint(index,p.x,p.y,p.z); }
		/// \overload
		inline void  setPoint(size_t index,float x, float y) { setPoint(index,x,y,0); }
		/// overload (RGB data is ignored in classes without color information)
		virtual void setPoint(size_t index,float x, float y, float z, float R, float G, float B)
		{
			MRPT_UNUSED_PARAM(R); MRPT_UNUSED_PARAM(G); MRPT_UNUSED_PARAM(B);
			setPoint(index,x,y,z);
		}

		/// Sets the point weight, which is ignored in all classes but those which actually store that field (Note: No checks are done for out-of-bounds index). \sa getPointWeight
		virtual void setPointWeight(size_t index,unsigned long w)
		{
			MRPT_UNUSED_PARAM(index); MRPT_UNUSED_PARAM(w);
		}
		/// Gets the point weight, which is ignored in all classes (defaults to 1) but in those which actually store that field (Note: No checks are done for out-of-bounds index).  \sa setPointWeight
		virtual unsigned int getPointWeight(size_t index) const { MRPT_UNUSED_PARAM(index); return 1; }


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
		  * \tparam VECTOR can be std::vector<float or double> or any row/column Eigen::Array or Eigen::Matrix (this includes mrpt::math::CVectorFloat and mrpt::math::CVectorDouble).
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

		/** Gets all points as a STL-like container.
		  * \tparam CONTAINER Any STL-like container of mrpt::math::TPoint3D, mrpt::math::TPoint3Df or anything having members `x`,`y`,`z`.
		  * Note that this method is not efficient for large point clouds. Fastest methods are getPointsBuffer() or getPointsBufferRef_x(), getPointsBufferRef_y(), getPointsBufferRef_z()
		  */
		template <class CONTAINER>
		void getAllPoints(CONTAINER &ps,size_t decimation=1) const	{
			std::vector<float> dmy1,dmy2,dmy3;
			getAllPoints(dmy1,dmy2,dmy3,decimation);
			ps.resize(dmy1.size());
			for (size_t i=0;i<dmy1.size();i++)	{
				ps[i].x=dmy1[i];
				ps[i].y=dmy2[i];
				ps[i].z=dmy3[i];
			}
		}

		/** Returns a copy of the 2D/3D points as a std::vector of float coordinates.
		  * If decimation is greater than 1, only 1 point out of that number will be saved in the output, effectively performing a subsampling of the points.
		  * \sa setAllPoints
		  */
		void  getAllPoints( std::vector<float> &xs, std::vector<float> &ys, size_t decimation = 1 ) const;

		inline void getAllPoints(std::vector<mrpt::math::TPoint2D> &ps,size_t decimation=1) const	{
			std::vector<float> dmy1,dmy2;
			getAllPoints(dmy1,dmy2,decimation);
			ps.resize(dmy1.size());
			for (size_t i=0;i<dmy1.size();i++)	{
				ps[i].x=static_cast<double>(dmy1[i]);
				ps[i].y=static_cast<double>(dmy2[i]);
			}
		}

		/** Provides a way to insert (append) individual points into the map: the missing fields of child
		  * classes (color, weight, etc) are left to their default values
		  */
		inline void  insertPoint( float x, float y, float z=0 ) { insertPointFast(x,y,z); mark_as_modified(); }
		/// \overload
		inline void  insertPoint( const mrpt::math::TPoint3D &p ) { insertPoint(p.x,p.y,p.z); }
		/// overload (RGB data is ignored in classes without color information)
		virtual void  insertPoint( float x, float y, float z, float R, float G, float B )
		{
			MRPT_UNUSED_PARAM(R); MRPT_UNUSED_PARAM(G); MRPT_UNUSED_PARAM(B);
			insertPoint(x,y,z);
		}

		/** Set all the points at once from vectors with X,Y and Z coordinates (if Z is not provided, it will be set to all zeros).
		  * \tparam VECTOR can be mrpt::math::CVectorFloat or std::vector<float> or any other column or row Eigen::Matrix.
		  */
		template <typename VECTOR>
		inline void setAllPointsTemplate(const VECTOR &X,const VECTOR &Y,const VECTOR &Z = VECTOR())
		{
			const size_t N = X.size();
			ASSERT_EQUAL_(X.size(),Y.size())
			ASSERT_(Z.size()==0 || Z.size()==X.size())
			this->setSize(N);
			const bool z_valid = !Z.empty();
			if (z_valid) for (size_t i=0;i<N;i++) { this->setPointFast(i,X[i],Y[i],Z[i]); }
			else         for (size_t i=0;i<N;i++) { this->setPointFast(i,X[i],Y[i],0); }
			mark_as_modified();
		}

		/** Set all the points at once from vectors with X,Y and Z coordinates. \sa getAllPoints */
		inline void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y,const std::vector<float> &Z) {
			setAllPointsTemplate(X,Y,Z);
		}

		/** Set all the points at once from vectors with X and Y coordinates (Z=0). \sa getAllPoints */
		inline void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y) {
			setAllPointsTemplate(X,Y);
		}

		/** Get all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...
		  * \sa getPointAllFieldsFast, setPointAllFields, setPointAllFieldsFast
		  */
		void  getPointAllFields( const size_t index, std::vector<float> & point_data ) const {
			ASSERT_BELOW_(index,this->size())
			getPointAllFieldsFast(index,point_data);
		}

		/** Set all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...
		  *  Unlike setPointAllFields(), this method does not check for index out of bounds
		  * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
		  */
		void  setPointAllFields( const size_t index, const std::vector<float> & point_data ){
			ASSERT_BELOW_(index,this->size())
			setPointAllFieldsFast(index,point_data);
		}


		/** Delete points out of the given "z" axis range have been removed.
		  */
		void  clipOutOfRangeInZ(float zMin, float zMax);

		/** Delete points which are more far than "maxRange" away from the given "point".
		  */
		void  clipOutOfRange(const mrpt::math::TPoint2D &point, float maxRange);

		/** Remove from the map the points marked in a bool's array as "true".
		  * \exception std::exception If mask size is not equal to points count.
		  */
		void  applyDeletionMask( const std::vector<bool> &mask );

		// See docs in base class.
		virtual void  determineMatching2D(
			const mrpt::maps::CMetricMap      * otherMap,
			const mrpt::poses::CPose2D         & otherMapPose,
			mrpt::utils::TMatchingPairList     & correspondences,
			const TMatchingParams & params,
			TMatchingExtraResults & extraResults ) const MRPT_OVERRIDE;

		// See docs in base class
		virtual void  determineMatching3D(
			const mrpt::maps::CMetricMap      * otherMap,
			const mrpt::poses::CPose3D         & otherMapPose,
			mrpt::utils::TMatchingPairList     & correspondences,
			const TMatchingParams & params,
			TMatchingExtraResults & extraResults ) const MRPT_OVERRIDE;

		// See docs in base class
		float compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const MRPT_OVERRIDE;

		/** Computes the matchings between this and another 3D points map.
		   This method matches each point in the other map with the centroid of the 3 closest points in 3D
		   from this map (if the distance is below a defined threshold).
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
		 * \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
		 * \param  correspondences			  [OUT] The detected matchings pairs.
		 * \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
		 *
		 * \sa determineMatching3D
		 */
		 void compute3DDistanceToMesh(
			const mrpt::maps::CMetricMap						*otherMap2,
			const mrpt::poses::CPose3D							&otherMapPose,
			float									maxDistForCorrespondence,
			mrpt::utils::TMatchingPairList                       &correspondences,
			float						            &correspondencesRatio );

		/** Transform the range scan into a set of cartessian coordinated
		  *	 points. The options in "insertionOptions" are considered in this method.
		  * \param rangeScan The scan to be inserted into this map
		  * \param robotPose Default to (0,0,0|0deg,0deg,0deg). Changes the frame of reference for the point cloud (i.e. the vehicle/robot pose in world coordinates).
		  *
		  *  Only ranges marked as "valid=true" in the observation will be inserted
		  *
		  *  \note Each derived class may enrich points in different ways (color, weight, etc..), so please refer to the description of the specific
		  *         implementation of mrpt::maps::CPointsMap you are using.
		  *  \note The actual generic implementation of this file lives in <src>/CPointsMap_crtp_common.h, but specific instantiations are generated at each derived class.
		  *
		  * \sa CObservation2DRangeScan, CObservation3DRangeScan
		  */
		virtual void  loadFromRangeScan(
				const mrpt::obs::CObservation2DRangeScan &rangeScan,
				const mrpt::poses::CPose3D				  *robotPose = NULL ) = 0;

		/** Overload of \a loadFromRangeScan() for 3D range scans (for example, Kinect observations).
		  *
		  * \param rangeScan The scan to be inserted into this map
		  * \param robotPose Default to (0,0,0|0deg,0deg,0deg). Changes the frame of reference for the point cloud (i.e. the vehicle/robot pose in world coordinates).
		  *
		  *  \note Each derived class may enrich points in different ways (color, weight, etc..), so please refer to the description of the specific
		  *         implementation of mrpt::maps::CPointsMap you are using.
		  *  \note The actual generic implementation of this file lives in <src>/CPointsMap_crtp_common.h, but specific instantiations are generated at each derived class.
		  * \sa loadFromVelodyneScan
		  */
		virtual void  loadFromRangeScan(
				const mrpt::obs::CObservation3DRangeScan &rangeScan,
				const mrpt::poses::CPose3D				  *robotPose = NULL ) = 0;

		/** Like \a loadFromRangeScan() for Velodyne 3D scans. Points are translated and rotated according to the \a sensorPose field in the observation and, if provided, to the \a robotPose parameter.
		  *
		  * \param scan The Raw LIDAR data to be inserted into this map. It MUST contain point cloud data, generated by calling to \a mrpt::obs::CObservationVelodyneScan::generatePointCloud() prior to insertion in this map.
		  * \param robotPose Default to (0,0,0|0deg,0deg,0deg). Changes the frame of reference for the point cloud (i.e. the vehicle/robot pose in world coordinates).
		  * \sa loadFromRangeScan
		  */
		void loadFromVelodyneScan(
			const mrpt::obs::CObservationVelodyneScan & scan,
			const mrpt::poses::CPose3D				  *robotPose = NULL );

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
		void  fuseWith(
			CPointsMap			*anotherMap,
			float				minDistForFuse  = 0.02f,
			std::vector<bool>	*notFusedPoints = NULL);

		/** Replace each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const mrpt::poses::CPose2D &b);

		/** Replace each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const mrpt::poses::CPose3D &b);

		/** Copy all the points from "other" map to "this", replacing each point \f$ p_i \f$ by \f$ p'_i = b \oplus p_i \f$ (pose compounding operator).
		  */
		void   changeCoordinatesReference(const CPointsMap &other, const mrpt::poses::CPose3D &b);

		/** Returns true if the map is empty/no observation has been inserted.
		   */
		virtual bool isEmpty() const MRPT_OVERRIDE;

		/** STL-like method to check whether the map is empty: */
		inline bool  empty() const { return isEmpty(); }

		/** Returns a 3D object representing the map.
		  *  The color of the points is given by the static variables: COLOR_3DSCENE_R,COLOR_3DSCENE_G,COLOR_3DSCENE_B
		  * \sa mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE
		  */
		virtual void getAs3DObject( mrpt::opengl::CSetOfObjectsPtr &outObj ) const MRPT_OVERRIDE;

		/** If the map is a simple points map or it's a multi-metric map that contains EXACTLY one simple points map, return it.
			* Otherwise, return NULL
			*/
		virtual const mrpt::maps::CSimplePointsMap * getAsSimplePointsMap() const MRPT_OVERRIDE { return NULL; }
		virtual       mrpt::maps::CSimplePointsMap * getAsSimplePointsMap()       MRPT_OVERRIDE { return NULL; }


		/** This method returns the largest distance from the origin to any of the points, such as a sphere centered at the origin with this radius cover ALL the points in the map (the results are buffered, such as, if the map is not modified, the second call will be much faster than the first one). */
		float  getLargestDistanceFromOrigin() const;

		/** Like \a getLargestDistanceFromOrigin() but returns in \a output_is_valid = false if the distance was not already computed, skipping its computation then, unlike getLargestDistanceFromOrigin() */
		float  getLargestDistanceFromOriginNoRecompute(bool &output_is_valid) const {
			output_is_valid = m_largestDistanceFromOriginIsUpdated;
			return m_largestDistanceFromOrigin;
		}

		/** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if there are no points.
		  *  Results are cached unless the map is somehow modified to avoid repeated calculations.
		  */
		void boundingBox( float &min_x,float &max_x,float &min_y,float &max_y,float &min_z,float &max_z ) const;

		inline void boundingBox(mrpt::math::TPoint3D &pMin,mrpt::math::TPoint3D &pMax) const	{
			float dmy1,dmy2,dmy3,dmy4,dmy5,dmy6;
			boundingBox(dmy1,dmy2,dmy3,dmy4,dmy5,dmy6);
			pMin.x=dmy1;
			pMin.y=dmy3;
			pMin.z=dmy5;
			pMax.x=dmy2;
			pMax.y=dmy4;
			pMax.z=dmy6;
		}

		/** Extracts the points in the map within a cylinder in 3D defined the provided radius and zmin/zmax values.
		  */
		void extractCylinder( const mrpt::math::TPoint2D &center, const double radius, const double zmin, const double zmax, CPointsMap *outMap );

		/** Extracts the points in the map within the area defined by two corners.
		  *  The points are coloured according the R,G,B input data.
		  */
        void extractPoints( const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, CPointsMap *outMap, const double &R = 1, const double &G = 1, const double &B = 1 );

		/** @name Filter-by-height stuff
			@{ */

		/** Enable/disable the filter-by-height functionality \sa setHeightFilterLevels \note Default upon construction is disabled. */
		inline void enableFilterByHeight(bool enable=true) { m_heightfilter_enabled=enable; }
		/** Return whether filter-by-height is enabled \sa enableFilterByHeight */
		inline bool isFilterByHeightEnabled() const  { return m_heightfilter_enabled; }

		/** Set the min/max Z levels for points to be actually inserted in the map (only if \a enableFilterByHeight() was called before). */
		inline void setHeightFilterLevels(const double _z_min, const double _z_max) { m_heightfilter_z_min=_z_min; m_heightfilter_z_max=_z_max; }
		/** Get the min/max Z levels for points to be actually inserted in the map \sa enableFilterByHeight, setHeightFilterLevels */
		inline void getHeightFilterLevels(double &_z_min, double &_z_max) const { _z_min=m_heightfilter_z_min; _z_max=m_heightfilter_z_max; }

		/** @} */

		/** The color [0,1] of points when extracted from getAs3DObject (default=blue) */
		static float COLOR_3DSCENE_R;
		static float COLOR_3DSCENE_G;
		static float COLOR_3DSCENE_B;


		// See docs in base class
		virtual double internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

		/** @name PCL library support
			@{ */


		/** Use to convert this MRPT point cloud object into a PCL point cloud object (PointCloud<PointXYZ>).
		  *  Usage example:
		  *  \code
		  *    mrpt::maps::CPointsCloud       pc;
		  *    pcl::PointCloud<pcl::PointXYZ> cloud;
		  *
		  *    pc.getPCLPointCloud(cloud);
		  *  \endcode
		  * \sa setFromPCLPointCloud, CColouredPointsMap::getPCLPointCloudXYZRGB (for color data)
		  */
		template <class POINTCLOUD>
		void getPCLPointCloud(POINTCLOUD &cloud) const
		{
			const size_t nThis = this->size();
			// Fill in the cloud data
			cloud.width    = nThis;
			cloud.height   = 1;
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);
			for (size_t i = 0; i < nThis; ++i) {
				cloud.points[i].x =this->x[i];
				cloud.points[i].y =this->y[i];
				cloud.points[i].z =this->z[i];
			}
		}

		/** Loads a PCL point cloud into this MRPT class (note: this method ignores potential RGB information, see CColouredPointsMap::setFromPCLPointCloudRGB() ).
		  *  Usage example:
		  *  \code
		  *    pcl::PointCloud<pcl::PointXYZ> cloud;
		  *    mrpt::maps::CPointsCloud       pc;
		  *
		  *    pc.setFromPCLPointCloud(cloud);
		  *  \endcode
		  * \sa getPCLPointCloud, CColouredPointsMap::setFromPCLPointCloudRGB()
		  */
		template <class POINTCLOUD>
		void setFromPCLPointCloud(const POINTCLOUD &cloud)
		{
			const size_t N = cloud.points.size();
			clear();
			reserve(N);
			for (size_t i=0;i<N;++i)
				this->insertPointFast(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
		}

		/** @} */

		/** @name Methods that MUST be implemented by children classes of KDTreeCapable
			@{ */

		/// Must return the number of data points
		inline size_t kdtree_get_point_count() const {  return this->size(); }

		/// Returns the dim'th component of the idx'th point in the class:
		inline float kdtree_get_pt(const size_t idx, int dim) const {
			if (dim==0) return this->x[idx];
			else if (dim==1) return this->y[idx];
			else if (dim==2) return this->z[idx]; else return 0;
		}

		/// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const
		{
			if (size==2)
			{
				const float d0 = p1[0]-x[idx_p2];
				const float d1 = p1[1]-y[idx_p2];
				return d0*d0+d1*d1;
			}
			else
			{
				const float d0 = p1[0]-x[idx_p2];
				const float d1 = p1[1]-y[idx_p2];
				const float d2 = p1[2]-z[idx_p2];
				return d0*d0+d1*d1+d2*d2;
			}
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <typename BBOX>
		bool kdtree_get_bbox(BBOX &bb) const
		{
			float min_z,max_z;
			this->boundingBox(
				bb[0].low, bb[0].high,
				bb[1].low, bb[1].high,
				min_z,max_z);
			if (bb.size()==3) {
				bb[2].low = min_z; bb[2].high = max_z;
			}
			return true;
		}
		/** @} */

		/** Users normally don't need to call this. Called by this class or children classes, set m_largestDistanceFromOriginIsUpdated=false, invalidates the kd-tree cache, and such. */
		inline void mark_as_modified() const
		{
			m_largestDistanceFromOriginIsUpdated=false;
			m_boundingBoxIsUpdated = false;
			kdtree_mark_as_outdated();
		}

	protected:
		std::vector<float>     x,y,z;        //!< The point coordinates

		mrpt::obs::CSinCosLookUpTableFor2DScans  m_scans_sincos_cache; //!< Cache of sin/cos values for the latest 2D scan geometries.

		/** Auxiliary variables used in "getLargestDistanceFromOrigin"
		  * \sa getLargestDistanceFromOrigin
		  */
		mutable float	m_largestDistanceFromOrigin;

		/** Auxiliary variables used in "getLargestDistanceFromOrigin"
		  * \sa getLargestDistanceFromOrigin
		  */
		mutable bool	m_largestDistanceFromOriginIsUpdated;

		mutable bool	m_boundingBoxIsUpdated;
		mutable float   m_bb_min_x,m_bb_max_x, m_bb_min_y,m_bb_max_y, m_bb_min_z,m_bb_max_z;

		/** This is a common version of CMetricMap::insertObservation() for point maps (actually, CMetricMap::internal_insertObservation),
		  *   so derived classes don't need to worry implementing that method unless something special is really necesary.
		  * See mrpt::maps::CPointsMap for the enumeration of types of observations which are accepted. */
		bool  internal_insertObservation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose) MRPT_OVERRIDE;

		/** Helper method for ::copyFrom() */
		void  base_copyFrom(const CPointsMap &obj);


		/** @name PLY Import virtual methods to implement in base classes
			@{ */
		/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_face */
		virtual void PLY_import_set_face_count(const size_t N) MRPT_OVERRIDE { MRPT_UNUSED_PARAM(N); }

		/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
		  *  \param pt_color Will be NULL if the loaded file does not provide color info.
		  */
		virtual void PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color = NULL) MRPT_OVERRIDE;
		/** @} */

		/** @name PLY Export virtual methods to implement in base classes
			@{ */
		size_t PLY_export_get_vertex_count() const MRPT_OVERRIDE;
		size_t PLY_export_get_face_count() const MRPT_OVERRIDE { return 0; }
		virtual void PLY_export_get_vertex(const size_t idx,mrpt::math::TPoint3Df &pt,bool &pt_has_color,mrpt::utils::TColorf &pt_color) const MRPT_OVERRIDE;
		/** @} */

		/** The minimum and maximum height for a certain laser scan to be inserted into this map
		   * \sa m_heightfilter_enabled */
		double m_heightfilter_z_min, m_heightfilter_z_max;

		/** Whether or not (default=not) filter the input points by height
		  * \sa m_heightfilter_z_min, m_heightfilter_z_max */
		bool m_heightfilter_enabled;


		// Friend methods:
		template <class Derived> friend struct detail::loadFromRangeImpl;
		template <class Derived> friend struct detail::pointmap_traits;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CPointsMap , CMetricMap, MAPS_IMPEXP )

	} // End of namespace

	namespace global_settings
	{
		/** The size of points when exporting with getAs3DObject() (default=3.0)
		  * Affects to:
		  *		- mrpt::maps::CPointsMap and all its children classes.
		  */
		extern MAPS_IMPEXP float POINTSMAPS_3DOBJECT_POINTSIZE;
	}

	namespace utils
	{
		/** Specialization mrpt::utils::PointCloudAdapter<mrpt::maps::CPointsMap>  \ingroup mrpt_adapters_grp*/
		template <>
		class PointCloudAdapter<mrpt::maps::CPointsMap> : public detail::PointCloudAdapterHelperNoRGB<mrpt::maps::CPointsMap,float>
		{
		private:
			mrpt::maps::CPointsMap &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 0;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const mrpt::maps::CPointsMap &obj) : m_obj(*const_cast<mrpt::maps::CPointsMap*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				m_obj.getPointFast(idx,x,y,z);
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				m_obj.setPointFast(idx,x,y,z);
			}
		}; // end of PointCloudAdapter<mrpt::maps::CPointsMap>

	}

} // End of namespace

#endif
