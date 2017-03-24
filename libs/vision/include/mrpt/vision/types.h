/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_types_H
#define mrpt_vision_types_H

#include <mrpt/utils/aligned_containers.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/aligned_containers.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TMatchingPair.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** \addtogroup mrpt_vision_grp
		  *  @{ */
		typedef uint64_t TFeatureID;	//!< Definition of a feature ID

		typedef	uint64_t TLandmarkID;   //!< Unique IDs for landmarks
		typedef uint64_t TCameraPoseID; //!< Unique IDs for camera frames (poses)

		typedef mrpt::aligned_containers<TCameraPoseID,mrpt::poses::CPose3D>::map_t  TFramePosesMap;        //!< A list of camera frames (6D poses) indexed by unique IDs.
		typedef mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t             TFramePosesVec;        //!< A list of camera frames (6D poses), which assumes indexes are unique, consecutive IDs.

		typedef std::map<TLandmarkID,mrpt::math::TPoint3D>   TLandmarkLocationsMap; //!< A list of landmarks (3D points) indexed by unique IDs.
		typedef std::vector<mrpt::math::TPoint3D>            TLandmarkLocationsVec; //!< A list of landmarks (3D points), which assumes indexes are unique, consecutive IDs.


		/** Types of features - This means that the point has been detected with this algorithm, which is independent of additional descriptors a feature may also have
		*/
		enum TFeatureType
		{
			featNotDefined = -1,	//!< Non-defined feature (also used for Occupancy features)
			featKLT = 0,			//!< Kanade-Lucas-Tomasi feature [SHI'94]
			featHarris,				//!< Harris border and corner detector [HARRIS]
			featBCD,				//!< Binary corder detector
			featSIFT,				//!< Scale Invariant Feature Transform [LOWE'04]
			featSURF,				//!< Speeded Up Robust Feature [BAY'06]
			featBeacon,				//!< A especial case: this is not an image feature, but a 2D/3D beacon (used for range-only SLAM from mrpt::maps::CLandmark)
			featFAST,				//!< FAST feature detector, OpenCV's implementation ("Faster and better: A machine learning approach to corner detection", E. Rosten, R. Porter and T. Drummond, PAMI, 2009).
			featFASTER9,			//!< FASTER-9 detector, Edward Rosten's libcvd implementation optimized for SSE2.
			featFASTER10,			//!< FASTER-9 detector, Edward Rosten's libcvd implementation optimized for SSE2.
			featFASTER12,			//!< FASTER-9 detector, Edward Rosten's libcvd implementation optimized for SSE2.
			featORB					//!< ORB detector and descriptor, OpenCV's implementation ("ORB: an efficient alternative to SIFT or SURF", E. Rublee, V. Rabaud, K. Konolige, G. Bradski, ICCV, 2012).
		};

		/** The bitwise OR combination of values of TDescriptorType are used in CFeatureExtraction::computeDescriptors to indicate which descriptors are to be computed for features.
		  */
		enum TDescriptorType
		{
			descAny				= 0,  //!< Used in some methods to mean "any of the present descriptors"
			descSIFT            = 1,  //!< SIFT descriptors
			descSURF			= 2,  //!< SURF descriptors
			descSpinImages      = 4,  //!< Intensity-domain spin image descriptors
			descPolarImages     = 8,  //!< Polar image descriptor
			descLogPolarImages	= 16,  //!< Log-Polar image descriptor
			descORB				= 32  //!< Bit-based feature descriptor
		};

		enum TFeatureTrackStatus
		{
			// Init value
			status_IDLE 	= 0,	//!< Inactive (right after detection, and before being tried to track)

			// Ok:
			status_TRACKED 	= 5,	//!< Feature correctly tracked

			// Bad:
			status_OOB		= 1,	//!< Feature fell Out Of Bounds (out of the image limits, too close to image borders)
			status_LOST 	= 10	//!< Unable to track this feature (mismatch is too high for the given tracking window: lack of texture? oclussion?)
		};


		/** One feature observation entry, used within sequences with TSequenceFeatureObservations */
		struct VISION_IMPEXP TFeatureObservation
		{
			inline TFeatureObservation() { }
			inline TFeatureObservation(const TLandmarkID _id_feature, const TCameraPoseID  _id_frame, const mrpt::utils::TPixelCoordf &_px) : id_feature(_id_feature), id_frame(_id_frame), px(_px) { }

			TLandmarkID    id_feature;  //!< A unique ID of this feature
			TCameraPoseID  id_frame;    //!< A unique ID of a "frame" (camera position) from where the feature was observed.
			mrpt::utils::TPixelCoordf   px;          //!< The pixel coordinates of the observed feature
		};

		/** One relative feature observation entry, used with some relative bundle-adjustment functions.
		  */
		struct TRelativeFeaturePos
		{
			inline TRelativeFeaturePos() { }
			inline TRelativeFeaturePos(const mrpt::vision::TCameraPoseID  _id_frame_base, const mrpt::math::TPoint3D &_pos) : id_frame_base(_id_frame_base), pos(_pos) {  }

			mrpt::vision::TCameraPoseID  id_frame_base;	//!< The ID of the camera frame which is the coordinate reference of \a pos
			mrpt::math::TPoint3D         pos;  //!< The (x,y,z) location of the feature, wrt to the camera frame \a id_frame_base
		};

		/** An index of feature IDs and their relative locations */
		typedef std::map<mrpt::vision::TFeatureID, TRelativeFeaturePos>  TRelativeFeaturePosMap;

		/** A complete sequence of observations of features from different camera frames (poses).
		  *  This structure is the input to some (Bundle-adjustment) methods in mrpt::vision
		  *  \note Pixel coordinates can be either "raw" or "undistorted". Read the doc of functions handling this structure to see what they expect.
		  *  \sa mrpt::vision::bundle_adj_full
		  */
		struct VISION_IMPEXP TSequenceFeatureObservations : public std::vector<TFeatureObservation>
		{
			typedef std::vector<TFeatureObservation> BASE;

			inline TSequenceFeatureObservations() {}
			inline TSequenceFeatureObservations(size_t size) : BASE(size) {}
			inline TSequenceFeatureObservations(const TSequenceFeatureObservations& o) : BASE(o) {}

			/** Saves all entries to a text file, with each line having this format: #FRAME_ID  #FEAT_ID  #PIXEL_X  #PIXEL_Y
			  * The file is self-descripting, since the first line contains a comment line (starting with '%') explaining the format.
			  * Generated files can be loaded from MATLAB.
			  * \sa loadFromTextFile \exception std::exception On I/O error  */
			void saveToTextFile(const std::string &filName, bool skipFirstCommentLine = false) const;

			/** Load from a text file, in the format described in \a saveToTextFile \exception std::exception On I/O or format error */
			void loadFromTextFile(const std::string &filName);

			/** Save the list of observations + the point locations + the camera frame poses to a pair of files in the format
			  *  used by the Sparse Bundle Adjustment (SBA) C++ library.
			  *
			  *  Point file lines: X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...
			  *
			  *  Camera file lines: qr qx qy qz x y z  (Pose as a quaternion)
			  * \return false on any error
			  */
			bool saveAsSBAFiles(
				const TLandmarkLocationsVec &pts,
				const std::string &pts_file,
				const TFramePosesVec        &cams,
				const std::string &cams_file) const;


			/** Remove all those features that don't have a minimum number of observations from different camera frame IDs.
			  * \return the number of erased entries.
			  * \sa After calling this you may want to call \a compressIDs */
			size_t removeFewObservedFeatures(size_t minNumObservations = 3);

			/** Remove all but one out of \a decimate_ratio camera frame IDs from the list (eg: from N camera pose IDs at return there will be just N/decimate_ratio)
			  * The algorithm first builds a sorted list of frame IDs, then keep the lowest ID, remove the next "decimate_ratio-1", and so on.
			  * \sa After calling this you may want to call \a compressIDs */
			void decimateCameraFrames(const size_t decimate_ratio);

			/** Rearrange frame and feature IDs such as they start at 0 and there are no gaps.
			  * \param old2new_camIDs If provided, the mapping from old to new IDs is stored here.
			  * \param old2new_lmIDs If provided, the mapping from old to new IDs is stored here. */
			void compressIDs(
				std::map<TCameraPoseID,TCameraPoseID>  *old2new_camIDs=NULL,
				std::map<TLandmarkID,TLandmarkID>      *old2new_lmIDs=NULL );

		};

		/** Parameters associated to a stereo system
		  */
		struct VISION_IMPEXP TStereoSystemParams : public mrpt::utils::CLoadableOptions
		{
			/** Initilization of default parameters */
			TStereoSystemParams(	);

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** Method for propagating the feature's image coordinate uncertainty into 3D space. Default value: Prop_Linear
			  */
			enum TUnc_Prop_Method
			{
				/** Linear propagation of the uncertainty
				  */
				Prop_Linear = -1,
				/** Uncertainty propagation through the Unscented Transformation
				  */
				Prop_UT,
				/** Uncertainty propagation through the Scaled Unscented Transformation
				  */
				Prop_SUT
			};

			TUnc_Prop_Method uncPropagation;

			/** Stereo Fundamental matrix */
			mrpt::math::CMatrixDouble33 F;

			/** Intrinsic parameters
			  */
			mrpt::math::CMatrixDouble33	K;
			/** Baseline. Default value: baseline = 0.119f;	[Bumblebee]
			  */
			float		baseline;
			/** Standard deviation of the error in feature detection. Default value: stdPixel = 1
			  */
			float		stdPixel;
			/** Standard deviation of the error in disparity computation. Default value: stdDisp = 1
			  */
			float		stdDisp;
			/** Maximum allowed distance. Default value: maxZ = 20.0f
			  */
			float		maxZ;
			/** Maximum allowed distance. Default value: minZ = 0.5f
			  */
			float		minZ;
			/** Maximum allowed height. Default value: maxY = 3.0f
			  */
			float		maxY;
			/** K factor for the UT. Default value: k = 1.5f
			  */
			float		factor_k;
			/** Alpha factor for SUT. Default value: a = 1e-3
			  */
			float		factor_a;
			/** Beta factor for the SUT. Default value: b = 2.0f
			  */
			float		factor_b;

			/** Parameters initialization
			  */
			//TStereoSystemParams();

		}; // End struct TStereoSystemParams

		/** A structure for storing a 3D ROI
		  */
		struct VISION_IMPEXP TROI
		{
			// Constructors
			TROI();
			TROI(float x1, float x2, float y1, float y2, float z1, float z2);

			// Members
			float	xMin;
			float	xMax;
			float	yMin;
			float	yMax;
			float	zMin;
			float	zMax;
		}; // end struct TROI

		/** A structure for defining a ROI within an image
		  */
		struct VISION_IMPEXP TImageROI
		{
			// Constructors
			TImageROI();
			TImageROI( float x1, float x2, float y1, float y2 );

			// Members
			/** X coordinate limits [0,imageWidth)
			  */
			float	xMin, xMax;
			/** Y coordinate limits [0,imageHeight)
			  */
			float	yMin, yMax;
		}; // end struct TImageROI

		/** A structure containing options for the matching
		  */
		struct VISION_IMPEXP TMatchingOptions : public mrpt::utils::CLoadableOptions
		{

			/** Method for propagating the feature's image coordinate uncertainty into 3D space. Default value: Prop_Linear
			  */
			enum TMatchingMethod
			{
				/** Matching by cross correlation of the image patches
				  */
				mmCorrelation = 0,
				/** Matching by Euclidean distance between SIFT descriptors
				  */
				mmDescriptorSIFT,
				/** Matching by Euclidean distance between SURF descriptors
				  */
				mmDescriptorSURF,
				/** Matching by sum of absolute differences of the image patches
				  */
				mmSAD,
				/** Matching by Hamming distance between ORB descriptors
				  */
				mmDescriptorORB
			};

			// For determining
			bool	useEpipolarRestriction;		//!< Whether or not take into account the epipolar restriction for finding correspondences
			bool	hasFundamentalMatrix;		//!< Whether or not there is a fundamental matrix
			bool	parallelOpticalAxis;		//!< Whether or not the stereo rig has the optical axes parallel
			bool	useXRestriction;			//!< Whether or not employ the x-coord restriction for finding correspondences (bumblebee camera, for example)
			bool    addMatches;                 //!< Whether or not to add the matches found into the input matched list (if false the input list will be cleared before being filled with the new matches)
			bool	useDisparityLimits;			//!< Whether or not use limits (min,max) for the disparity, see also 'min_disp, max_disp'
			bool	enable_robust_1to1_match;	//!< Whether or not only permit matches that are consistent from left->right and right->left

			float	min_disp, max_disp;			//!< Disparity limits, see also 'useDisparityLimits'

			mrpt::math::CMatrixDouble33 F;

			// General
			TMatchingMethod	matching_method;	//!< Matching method
			float	epipolar_TH;				//!< Epipolar constraint (rows of pixels)

			// SIFT
			float	maxEDD_TH;					//!< Maximum Euclidean Distance Between SIFT Descriptors
			float	EDD_RATIO;					//!< Boundary Ratio between the two lowest EDD

			// KLT
			float	minCC_TH;					//!< Minimum Value of the Cross Correlation
			float	minDCC_TH;					//!< Minimum Difference Between the Maximum Cross Correlation Values
			float	rCC_TH;						//!< Maximum Ratio Between the two highest CC values

			// SURF
			float	maxEDSD_TH;					//!< Maximum Euclidean Distance Between SURF Descriptors
			float	EDSD_RATIO;					//!< Boundary Ratio between the two lowest SURF EDSD

			// SAD
			double	maxSAD_TH;                  //!< Minimum Euclidean Distance Between Sum of Absolute Differences
			double  SAD_RATIO;                  //!< Boundary Ratio between the two highest SAD

			// ORB
			double	maxORB_dist;				//!< Maximun distance between ORB descriptors

//			// To estimate depth
			bool    estimateDepth;              //!< Whether or not estimate the 3D position of the real features for the matches (only with parallelOpticalAxis by now).
			double  maxDepthThreshold;          //!< The maximum allowed depth for the matching. If its computed depth is larger than this, the match won't be considered.
//            double  fx,cx,cy,baseline;          //!< Intrinsic parameters of the stereo rig

			/** Constructor */
			TMatchingOptions( );

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

#define COPY_MEMBER(_m) this->_m = o._m;
#define CHECK_MEMBER(_m) this->_m == o._m

			bool operator==( const TMatchingOptions & o ) const
			{
				return	
					CHECK_MEMBER(useXRestriction) && 
					CHECK_MEMBER(useDisparityLimits) &&
					CHECK_MEMBER(useEpipolarRestriction) &&
					CHECK_MEMBER(addMatches) &&
					CHECK_MEMBER(EDD_RATIO) &&
					CHECK_MEMBER(EDSD_RATIO) &&
					CHECK_MEMBER(enable_robust_1to1_match) &&
					CHECK_MEMBER(epipolar_TH) &&
					CHECK_MEMBER(estimateDepth) &&
					CHECK_MEMBER(F) &&
					CHECK_MEMBER(hasFundamentalMatrix) &&
					CHECK_MEMBER(matching_method) &&
					CHECK_MEMBER(maxDepthThreshold) &&
					CHECK_MEMBER(maxEDD_TH) &&
					CHECK_MEMBER(maxEDSD_TH) &&
					CHECK_MEMBER(maxORB_dist) &&
					CHECK_MEMBER(maxSAD_TH) &&
					CHECK_MEMBER(max_disp) &&
					CHECK_MEMBER(minCC_TH) &&
					CHECK_MEMBER(minDCC_TH) &&
					CHECK_MEMBER(min_disp) &&
					CHECK_MEMBER(parallelOpticalAxis) &&
					CHECK_MEMBER(rCC_TH) &&
					CHECK_MEMBER(SAD_RATIO);
			}

			void operator=( const TMatchingOptions & o )
			{
				COPY_MEMBER(useXRestriction)
				COPY_MEMBER(useDisparityLimits)
				COPY_MEMBER(useEpipolarRestriction)
				COPY_MEMBER(addMatches)
				COPY_MEMBER(EDD_RATIO)
				COPY_MEMBER(EDSD_RATIO)
				COPY_MEMBER(enable_robust_1to1_match)
				COPY_MEMBER(epipolar_TH)
				COPY_MEMBER(estimateDepth)
				COPY_MEMBER(F)
				COPY_MEMBER(hasFundamentalMatrix)
				COPY_MEMBER(matching_method)
				COPY_MEMBER(maxDepthThreshold)
				COPY_MEMBER(maxEDD_TH)
				COPY_MEMBER(maxEDSD_TH)
				COPY_MEMBER(maxORB_dist)
				COPY_MEMBER(maxSAD_TH)
				COPY_MEMBER(max_disp)
				COPY_MEMBER(minCC_TH)
				COPY_MEMBER(minDCC_TH)
				COPY_MEMBER(min_disp)
				COPY_MEMBER(parallelOpticalAxis)
				COPY_MEMBER(rCC_TH)
				COPY_MEMBER(SAD_RATIO)
			}

		}; // end struct TMatchingOptions

        /** Struct containing the output after matching multi-resolution SIFT-like descriptors
		*/
        struct VISION_IMPEXP TMultiResMatchingOutput
        {
            int                     nMatches;

            std::vector<int>        firstListCorrespondences;    //!< Contains the indexes within the second list corresponding to the first one.
            std::vector<int>        secondListCorrespondences;   //!< Contains the indexes within the first list corresponding to the second one.
            std::vector<int>        firstListFoundScales;        //!< Contains the scales of the first list where the correspondence was found.
            std::vector<double>     firstListDistance;           //!< Contains the distances between the descriptors.

            TMultiResMatchingOutput() : nMatches(0),
                firstListCorrespondences(), secondListCorrespondences(),
                firstListFoundScales(), firstListDistance() {}

        }; // end struct TMultiResMatchingOutput

        /** Struct containing the options when matching multi-resolution SIFT-like descriptors
		*/
		struct VISION_IMPEXP TMultiResDescMatchOptions : public mrpt::utils::CLoadableOptions
		{
			bool      useOriFilter;           //!< Whether or not use the filter based on orientation test
			double    oriThreshold;           //!< The threshold for the orientation test

			bool      useDepthFilter;         //!< Whether or not use the filter based on the depth test

			double    matchingThreshold;      //!< The absolute threshold in descriptor distance for considering a match
			double    matchingRatioThreshold; //!< The ratio between the two lowest distances threshold for considering a match
			uint32_t  lowScl1, lowScl2;       //!< The lowest scales in the two features to be taken into account in the matching process
			uint32_t  highScl1, highScl2;     //!< The highest scales in the two features to be taken into account in the matching process

			uint32_t  searchAreaSize;         //!< Size of the squared area where to search for a match.
			uint32_t  lastSeenThreshold;      //!< The allowed number of frames since a certain feature was seen for the last time.
			uint32_t  timesSeenThreshold;     //!< The minimum number of frames for a certain feature to be considered stable.

			uint32_t  minFeaturesToFind;      //!< The minimum number of features allowed in the system. If current number is below this value, more features will be found.
			uint32_t  minFeaturesToBeLost;    //!< The minimum number of features allowed in the system to not be considered to be lost.

			/** Default constructor
			  */
			TMultiResDescMatchOptions() :
				useOriFilter( true ), oriThreshold( 0.2 ),
				useDepthFilter( true ), matchingThreshold( 1e4 ), matchingRatioThreshold( 0.5 ),
				lowScl1(0), lowScl2(0), highScl1(6), highScl2(6), searchAreaSize(20), lastSeenThreshold(10), timesSeenThreshold(5),
				minFeaturesToFind(30), minFeaturesToBeLost(5) {}

			TMultiResDescMatchOptions(
				const bool &_useOriFilter, const double &_oriThreshold, const bool &_useDepthFilter,
				const double &_th, const double &_th2, const unsigned int &_lwscl1, const unsigned int &_lwscl2,
				const unsigned int &_hwscl1, const unsigned int &_hwscl2, const int &_searchAreaSize, const int &_lsth, const int &_tsth,
				const int &_minFeaturesToFind, const int &_minFeaturesToBeLost ) :
				useOriFilter( _useOriFilter ), oriThreshold( _oriThreshold ), useDepthFilter( _useDepthFilter ),
				matchingThreshold ( _th ), matchingRatioThreshold ( _th2 ), lowScl1( _lwscl1 ), lowScl2( _lwscl2 ),
				highScl1( _hwscl1 ), highScl2( _hwscl2 ), searchAreaSize( _searchAreaSize ), lastSeenThreshold( _lsth ), timesSeenThreshold( _tsth ),
				minFeaturesToFind( _minFeaturesToFind ), minFeaturesToBeLost(_minFeaturesToBeLost)  {}

			void loadFromConfigFile( const mrpt::utils::CConfigFileBase &cfg, const std::string &section ) MRPT_OVERRIDE;
			void saveToConfigFile( mrpt::utils::CConfigFileBase &cfg, const std::string &section ) const MRPT_OVERRIDE;
			void dumpToTextStream( mrpt::utils::CStream &out) const MRPT_OVERRIDE;

		}; // end TMultiResDescMatchOptions

		/** Struct containing the options when computing the multi-resolution SIFT-like descriptors
		*/
		struct VISION_IMPEXP TMultiResDescOptions : public mrpt::utils::CLoadableOptions
		{
			uint32_t        basePSize;          //!< The size of the base patch
			std::vector<double>  scales;             //!< The set of scales relatives to the base patch
			uint32_t        comLScl, comHScl;   //!< The subset of scales for which to compute the descriptors
			double          sg1, sg2, sg3;      //!< The sigmas for the Gaussian kernels
			bool            computeDepth;       //!< Whether or not to compute the depth of the feature
			bool            blurImage;          //!< Whether or not to blur the image previously to compute the descriptors
			double          fx,cx,cy,baseline;  //!< Intrinsic stereo pair parameters for computing the depth of the feature
			bool            computeHashCoeffs;  //!< Whether or not compute the coefficients for mantaining a HASH table of descriptors (for relocalization)

			double          cropValue;          //!< The SIFT-like descriptor is cropped at this value during normalization

			/** Default constructor
			  */
			TMultiResDescOptions() :
				basePSize(23), sg1 (0.5), sg2(7.5), sg3(8.0), computeDepth(true), blurImage(true), fx(0.0), cx(0.0), cy(0.0), baseline(0.0), computeHashCoeffs(false), cropValue(0.2)
			{
				scales.resize(7);
				scales[0] = 0.5;
				scales[1] = 0.8;
				scales[2] = 1.0;
				scales[3] = 1.2;
				scales[4] = 1.5;
				scales[5] = 1.8;
				scales[6] = 2.0;
				comLScl = 0;
				comHScl = 6;
			}

			TMultiResDescOptions( const unsigned int &_basePSize, const std::vector<double> &_scales,
				const unsigned int &_comLScl, const unsigned int &_comHScl,
				const double &_sg1, const double &_sg2, const double &_sg3,
				const bool &_computeDepth, const bool _blurImage, const double &_fx, const double &_cx, const double &_cy, const double &_baseline, const bool &_computeHashCoeffs, const double &_cropValue ):
				basePSize( _basePSize ), comLScl( _comLScl ), comHScl( _comHScl ),
				sg1( _sg1 ), sg2( _sg2 ), sg3( _sg3 ),
				computeDepth( _computeDepth ), blurImage( _blurImage ), fx( _fx ), cx( _cx ), cy( _cy ), baseline( _baseline ), computeHashCoeffs( _computeHashCoeffs), cropValue( _cropValue )
			{
				scales.resize( _scales.size() );
				for(unsigned int k = 0; k < _scales.size(); ++k)
					scales[k] = _scales[k];
			}

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void saveToConfigFile( mrpt::utils::CConfigFileBase &cfg, const std::string &section ) const MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

		}; // end TMultiResDescOptions


	/** @} */ // end of grouping

	}
}


#endif
