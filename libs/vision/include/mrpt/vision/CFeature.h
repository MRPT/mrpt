/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFeature_H
#define CFeature_H

#include <mrpt/utils/CImage.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/KDTreeCapable.h>

#include <mrpt/vision/types.h>
#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		class CFeatureList;
		class CMatchedFeatureList;

		enum TListIdx
		{
		    firstList = 0,
		    secondList,
		    bothLists
		};

		/** \defgroup mrptvision_features Feature detection, descriptors and matching 
		  * \ingroup mrpt_vision_grp
		  */

		/** \addtogroup  mrptvision_features
		    @{ */


		/****************************************************
						Class CFEATURE
		*****************************************************/
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CFeature, mrpt::utils::CSerializable, VISION_IMPEXP )

		/** A generic 2D feature from an image, extracted with \a CFeatureExtraction
		  * Each feature may have one or more descriptors (see \a descriptors), in addition to an image patch.
		  * The (Euclidean) distance between descriptors in a pair of features can be computed with  descriptorDistanceTo,
		  *  while the similarity of the patches is given by patchCorrelationTo.
		  *
		  *  \sa CFeatureList, TSimpleFeature, TSimpleFeatureList
		  */
		class VISION_IMPEXP CFeature : public mrpt::utils::CSerializable
		{
			friend class CFeatureList;
			friend class CMatchedFeatureList;

			DEFINE_SERIALIZABLE( CFeature )

		public:
			float				x,y;			//!< Coordinates in the image
			TFeatureID			ID;				//!< ID of the feature
			mrpt::utils::CImage patch;			//!< A patch of the image surrounding the feature
			uint16_t			patchSize;		//!< Size of the patch (patchSize x patchSize) (it must be an odd number)
			TFeatureType		type;			//!< Type of the feature: featNotDefined, featSIFT, featKLT,	featHarris, featSURF, featBeacon
			TFeatureTrackStatus	track_status;	//!< Status of the feature tracking process (old name: KLT_status)
			float				response;		//!< A measure of the "goodness" of the feature (old name: KLT_val)
			float				orientation;	//!< Main orientation of the feature
			float				scale;			//!< Feature scale into the scale space
			uint8_t				user_flags;		//!< A field for any other flags needed by the user (this has not a predefined meaning)
			uint16_t            nTimesSeen;     //!< Number of frames it has been seen in a sequence of images.
			uint16_t            nTimesNotSeen;  //!< Number of frames it has not been seen in a sequence of images.
			uint16_t            nTimesLastSeen; //!< Number of frames since it was seen for the last time.

			double                          depth;              //!< The estimated depth in 3D of this feature wrt the camera in the current frame
			double                          initialDepth;       //!< The estimated depth in 3D of this feature wrt the camera that took its image
			mrpt::math::TPoint3D                        p3D;                //!< The estimated 3D point of this feature wrt its camera
			std::deque<double>                   multiScales;        //!< A set of scales where the multi-resolution descriptor has been computed
			std::deque<std::vector<double> >          multiOrientations;  //!< A vector of main orientations (there is a vector of orientations for each scale)
			std::deque<std::vector<std::vector<int32_t> > >    multiHashCoeffs;    //!< A set of vectors containing the coefficients for a HASH table of descriptors
			bool isPointFeature() const;		                //!< Return false only for Blob detectors (SIFT, SURF)

			/** All the possible descriptors this feature may have */
			struct VISION_IMPEXP TDescriptors
			{
				TDescriptors();  // Initialization

				std::vector<uint8_t>	        SIFT;						//!< SIFT feature descriptor
				std::vector<float>			    SURF;						//!< SURF feature descriptor
				std::vector<float>			    SpinImg;					//!< The 2D histogram as a single row
				uint16_t					    SpinImg_range_rows;			//!< The number of rows (corresponding to range bins in the 2D histogram) of the original matrix from which SpinImg was extracted as a vector.
				mrpt::math::CMatrix			    PolarImg;					//!< A polar image centered at the interest point
				mrpt::math::CMatrix			    LogPolarImg;				//!< A log-polar image centered at the interest point
				bool						    polarImgsNoRotation;		//!< If set to true (manually, default=false) the call to "descriptorDistanceTo" will not consider all the rotations between polar image descriptors (PolarImg, LogPolarImg)
				std::deque<std::vector<std::vector<int32_t> > >    multiSIFTDescriptors;   //!< A set of SIFT-like descriptors for each orientation and scale of the multiResolution feature (there is a vector of descriptors for each scale)
				std::vector<uint8_t>			ORB;						//!< ORB feature descriptor	

				bool hasDescriptorSIFT() const { return !SIFT.empty(); };                       //!< Whether this feature has this kind of descriptor
				bool hasDescriptorSURF() const { return !SURF.empty(); }                        //!< Whether this feature has this kind of descriptor
				bool hasDescriptorSpinImg() const { return !SpinImg.empty(); };                 //!< Whether this feature has this kind of descriptor
				bool hasDescriptorPolarImg() const { return PolarImg.rows()!=0; } ;             //!< Whether this feature has this kind of descriptor
				bool hasDescriptorLogPolarImg() const { return LogPolarImg.rows()!=0; } ;       //!< Whether this feature has this kind of descriptor
				bool hasDescriptorMultiSIFT() const {
                    return (multiSIFTDescriptors.size() > 0 && multiSIFTDescriptors[0].size() > 0); //!< Whether this feature has this kind of descriptor
                }
				bool hasDescriptorORB() const { return !ORB.empty(); }						//!< Whether this feature has this kind of descriptor
			}
			descriptors;

			/** Return the first found descriptor, as a matrix.
			  * \return false on error, i.e. there is no valid descriptor.
			  */
			bool getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat &desc) const;

			/** Computes the normalized cross-correlation between the patches of this and another feature (normalized in the range [0,1], such as 0=best, 1=worst).
			  *  \note If this or the other features does not have patches or they are of different sizes, an exception will be raised.
			  * \sa descriptorDistanceTo
			  */
			float patchCorrelationTo( const CFeature &oFeature) const;

			/** Computes the Euclidean Distance between this feature's and other feature's descriptors, using the given descriptor or the first present one.
			  *  \note If descriptorToUse is not descAny and that descriptor is not present in one of the features, an exception will be raised.
			  * \sa patchCorrelationTo
			  */
			float descriptorDistanceTo( const CFeature &oFeature,  TDescriptorType descriptorToUse = descAny, bool normalize_distances = true ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSIFTDistanceTo( const CFeature &oFeature, bool normalize_distances = true ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSURFDistanceTo( const CFeature &oFeature, bool normalize_distances = true  ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSpinImgDistanceTo( const CFeature &oFeature, bool normalize_distances = true ) const;

			/** Returns the minimum Euclidean Distance between "this" and the "other" polar image descriptor, for the best shift in orientation.
			  * \param oFeature The other feature to compare with.
			  * \param minDistAngle The placeholder for the angle at which the smallest distance is found.
			  * \return The distance for the best orientation (minimum distance).
			  */
			float descriptorPolarImgDistanceTo(
				const CFeature &oFeature,
				float &minDistAngle,
				bool normalize_distances = true ) const;

			/** Returns the minimum Euclidean Distance between "this" and the "other" log-polar image descriptor, for the best shift in orientation.
			  * \param oFeature The other feature to compare with.
			  * \param minDistAngle The placeholder for the angle at which the smallest distance is found.
			  * \return The distance for the best orientation (minimum distance).
			  */
			float descriptorLogPolarImgDistanceTo(
				const CFeature &oFeature,
				float &minDistAngle,
				bool normalize_distances = true ) const;

			/** Computes the Hamming distance "this" and the "other" descriptor ORB descriptor */
			uint8_t descriptorORBDistanceTo( const CFeature &oFeature ) const; 

			/** Save the feature to a text file in this format:
              *    "%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
              *    "%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT [SIFT] HAS_SURF [SURF] HAS_MULTI [MULTI_i] HAS_ORB [ORB]"
              *    "%% |---------------------- feature ------------------| |---------------------- descriptors ------------------------|"
              *    "%% with:\n"
              *    "%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4: SURF, 5: Beacon, 6: FAST, 7: ORB\n"
              *    "%%  HAS_* : 1 if a descriptor of that type is associated to the feature."
              *    "%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1"
              *    "%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1"
              *    "%%  MULTI : Present if HAS_MULTI=1: SCALE ORI N DESC_0 ... DESC_N-1"
			  *	   "%%  ORB   : Present if HAS_ORB=1: DESC_0 ... DESC_31
              *    "%%-----------------------------------------------------------------------------\n");
			*/
            void saveToTextFile( const std::string &filename, bool APPEND = false );

			/** Get the type of the feature
			*/
			TFeatureType get_type() const { return type; }

			/** Dump feature information into a text stream */
			void dumpToTextStream( mrpt::utils::CStream &out) const;

			void dumpToConsole() const;

			/** Constructor
			*/
			CFeature();

			/** Virtual destructor */
			virtual ~CFeature() {}


		protected:

			/** Internal function used by "descriptorLogPolarImgDistanceTo" and "descriptorPolarImgDistanceTo"
			  */
			static float internal_distanceBetweenPolarImages(
				const mrpt::math::CMatrix &desc1,
				const mrpt::math::CMatrix &desc2,
				float &minDistAngle,
				bool normalize_distances,
				bool dont_shift_angle );

		}; // end of class
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CFeature, mrpt::utils::CSerializable, VISION_IMPEXP )


		/****************************************************
						Class CFEATURELIST
		*****************************************************/
		/** A list of visual features, to be used as output by detectors, as input/output by trackers, etc.
		  */
		class VISION_IMPEXP CFeatureList : public mrpt::math::KDTreeCapable<CFeatureList>
		{
		protected:
			typedef std::vector<CFeaturePtr> TInternalFeatList;

			TInternalFeatList  m_feats; //!< The actual container with the list of features

		public:
			/** The type of the first feature in the list */
			inline TFeatureType get_type() const { return empty() ? featNotDefined : (*begin())->get_type(); }

			/** Save feature list to a text file */
			void saveToTextFile( const std::string &fileName, bool APPEND = false );

			/** Save feature list to a text file */
			void loadFromTextFile( const std::string &fileName );

			/** Copies the content of another CFeatureList inside this one. The inner features are also copied. */
			void copyListFrom( const CFeatureList &otherList );

			/** Get the maximum ID into the list */
			TFeatureID getMaxID() const;

			/** Get a reference to a Feature from its ID */
			CFeaturePtr getByID( const TFeatureID &ID ) const;
			CFeaturePtr getByID( const TFeatureID &ID, int &out_idx ) const;

			/** Get a vector of references to a subset of features from their IDs */
			void getByMultiIDs( const std::vector<TFeatureID> &IDs, std::vector<CFeaturePtr> &out, std::vector<int> &outIndex ) const;

			/** Get a reference to the nearest feature to the a given 2D point (version returning distance to closest feature in "max_dist")
			*   \param x [IN] The query point x-coordinate
			*   \param y [IN] The query point y-coordinate
			*   \param max_dist [IN/OUT] At input: The maximum distance to search for. At output: The actual distance to the feature.
			*  \return A reference to the found feature, or a NULL smart pointer if none found.
			*  \note See also all the available KD-tree search methods, listed in mrpt::math::KDTreeCapable
			*/
			CFeaturePtr nearest( const float x, const float y, double &max_dist ) const;

			/** Constructor */
			CFeatureList();

			/** Virtual destructor */
			virtual ~CFeatureList();

			/** Call this when the list of features has been modified so the KD-tree is marked as outdated. */
			inline void mark_kdtree_as_outdated() const { kdtree_mark_as_outdated(); }

			/** @name Method and datatypes to emulate a STL container
			    @{ */
			typedef TInternalFeatList::iterator iterator;
			typedef TInternalFeatList::const_iterator const_iterator;

			typedef TInternalFeatList::reverse_iterator reverse_iterator;
			typedef TInternalFeatList::const_reverse_iterator const_reverse_iterator;

			inline iterator begin() { return m_feats.begin(); }
			inline iterator end() { return m_feats.end(); }
			inline const_iterator begin() const { return m_feats.begin(); }
			inline const_iterator end() const { return m_feats.end(); }

			inline reverse_iterator rbegin() { return m_feats.rbegin(); }
			inline reverse_iterator rend() { return m_feats.rend(); }
			inline const_reverse_iterator rbegin() const { return m_feats.rbegin(); }
			inline const_reverse_iterator rend() const { return m_feats.rend(); }

			inline iterator erase(const iterator &it)  { mark_kdtree_as_outdated(); return m_feats.erase(it); }

			inline bool empty() const  { return m_feats.empty(); }
			inline size_t size() const { return m_feats.size(); }

			inline void clear() { m_feats.clear(); mark_kdtree_as_outdated(); }
			inline void resize(size_t N) { m_feats.resize(N); mark_kdtree_as_outdated(); }

			inline void push_back(const CFeaturePtr &f) { mark_kdtree_as_outdated();  m_feats.push_back(f); }

			inline CFeaturePtr & operator [](const unsigned int index) { return m_feats[index]; }
			inline const CFeaturePtr & operator [](const unsigned int index) const  { return m_feats[index]; }

			/** @} */


			/** @name Methods that MUST be implemented by children classes of KDTreeCapable
				@{ */

			/// Must return the number of data points 
			inline size_t kdtree_get_point_count() const {  return this->size(); }

			/// Returns the dim'th component of the idx'th point in the class:
			inline float kdtree_get_pt(const size_t idx, int dim) const { 
				ASSERTDEB_(dim==0 || dim==1)
				if (dim==0) return m_feats[idx]->x;
				else return m_feats[idx]->y;
			}

			/// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
			inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const
			{
				ASSERTDEB_(size==2)
				MRPT_UNUSED_PARAM(size); // in release mode

				const float d0 = p1[0] - m_feats[idx_p2]->x; 
				const float d1 = p1[1] - m_feats[idx_p2]->y; 
				return d0*d0+d1*d1;
			}

			// Optional bounding-box computation: return false to default to a standard bbox computation loop. 
			//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
			//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
			template <typename BBOX>
			bool kdtree_get_bbox(BBOX &bb) const  {
				MRPT_UNUSED_PARAM(bb);
				return false;
			}

			/** @} */


			/** @name getFeature*() methods for template-based access to feature list
			    @{ */
			inline float getFeatureX(size_t i) const { return m_feats[i]->x; }
			inline float getFeatureY(size_t i) const { return m_feats[i]->y; }
			inline TFeatureID getFeatureID(size_t i) const { return m_feats[i]->ID; }
			inline float getFeatureResponse(size_t i) const { return m_feats[i]->response; }
			inline bool isPointFeature(size_t i) const { return m_feats[i]->isPointFeature(); }
			inline float getScale(size_t i) const { return m_feats[i]->scale; }
			inline TFeatureTrackStatus getTrackStatus(size_t i) { return m_feats[i]->track_status; }

			inline void setFeatureX(size_t i,float x) { m_feats[i]->x=x; }
			inline void setFeatureXf(size_t i,float x) { m_feats[i]->x=x; }
			inline void setFeatureY(size_t i,float y) { m_feats[i]->y=y; }
			inline void setFeatureYf(size_t i,float y) { m_feats[i]->y=y; }

			inline void setFeatureID(size_t i,TFeatureID id) { m_feats[i]->ID=id; }
			inline void setFeatureResponse(size_t i,float r) { m_feats[i]->response=r; }
			inline void setScale(size_t i,float s) { m_feats[i]->scale=s; }
			inline void setTrackStatus(size_t i,TFeatureTrackStatus s) { m_feats[i]->track_status=s; }

			inline void mark_as_outdated() const { kdtree_mark_as_outdated(); }

			/** @} */


		}; // end of class

		/****************************************************
					Class CMATCHEDFEATURELIST
		*****************************************************/
		/** A list of features
		*/
		class VISION_IMPEXP CMatchedFeatureList : public std::deque< std::pair<CFeaturePtr,CFeaturePtr> >
		{
		public:
			/** The type of the first feature in the list */
			inline TFeatureType get_type() const { return empty() ? featNotDefined : (begin()->first)->get_type(); }

			/** Save list of matched features to a text file */
			void saveToTextFile(const std::string &fileName);

            /** Returns the matching features as two separate CFeatureLists */
			void getBothFeatureLists( CFeatureList &list1, CFeatureList &list2 );

			/** Returns a smart pointer to the feature with the provided ID or a empty one if not found */
			CFeaturePtr getByID( const TFeatureID & ID, const TListIdx &idx );

			/** Returns the maximum ID of the features in the list. If the max ID has been already set up, this method just returns it.
			    Otherwise, this method finds, stores and returns it.*/
			void getMaxID( const TListIdx &idx, TFeatureID & firstListID, TFeatureID & secondListID );

			/** Updates the value of the maximum ID of the features in the matched list, i.e. it explicitly searches for the max ID and updates the member variables. */
			void updateMaxID( const TListIdx &idx );

            /** Explicitly set the max IDs values to certain values */
            inline void setLeftMaxID( const TFeatureID &leftID ){ m_leftMaxID = leftID; }
            inline void setRightMaxID( const TFeatureID &rightID ){ m_rightMaxID = rightID; }
			inline void setMaxIDs( const TFeatureID &leftID, const TFeatureID &rightID )
            {
                setLeftMaxID(leftID);
                setRightMaxID(rightID);
            };

			/** Constructor */
			CMatchedFeatureList();

			/** Virtual destructor */
			virtual ~CMatchedFeatureList();
        protected:
            TFeatureID m_leftMaxID, m_rightMaxID;
		}; // end of class


		/** @} */ // End of add to module: mrptvision_features

	} // end of namespace


	namespace utils
	{
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CFeature, mrpt::vision)
	}
} // end of namespace

#endif

