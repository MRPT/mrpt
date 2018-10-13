/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/core/aligned_std_map.h>
#include <mrpt/img/CImage.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt::vision
{
/** \addtogroup mrpt_vision_grp
 *  @{ */
/** Definition of a feature ID */
using TFeatureID = uint64_t;

/** Unique IDs for landmarks */
using TLandmarkID = uint64_t;
/** Unique IDs for camera frames (poses) */
using TCameraPoseID = uint64_t;

/** A list of camera frames (6D poses) indexed by unique IDs. */
using TFramePosesMap =
	mrpt::aligned_std_map<TCameraPoseID, mrpt::poses::CPose3D>;
/** A list of camera frames (6D poses), which assumes indexes are unique,
 * consecutive IDs. */
using TFramePosesVec = mrpt::aligned_std_vector<mrpt::poses::CPose3D>;

/** A list of landmarks (3D points) indexed by unique IDs. */
using TLandmarkLocationsMap = std::map<TLandmarkID, mrpt::math::TPoint3D>;
/** A list of landmarks (3D points), which assumes indexes are unique,
 * consecutive IDs. */
using TLandmarkLocationsVec = std::vector<mrpt::math::TPoint3D>;

/** Types of features - This means that the point has been detected with this
 * algorithm, which is independent of additional descriptors a feature may also
 * have
 */
enum TFeatureType
{
	/** Non-defined feature (also used for Occupancy features) */
	featNotDefined = -1,
	/** Kanade-Lucas-Tomasi feature [SHI'94] */
	featKLT = 0,
	/** Harris border and corner detector [HARRIS] */
	featHarris,
	/** Binary corder detector */
	featBCD,
	/** Scale Invariant Feature Transform [LOWE'04] */
	featSIFT,
	/** Speeded Up Robust Feature [BAY'06] */
	featSURF,
	/** A especial case: this is not an image feature, but a 2D/3D beacon (used
	   for range-only SLAM from mrpt::maps::CLandmark) */
	featBeacon,
	/** FAST feature detector, OpenCV's implementation ("Faster and better: A
	   machine learning approach to corner detection", E. Rosten, R. Porter and
	   T. Drummond, PAMI, 2009). */
	featFAST,
	/** FASTER-9 detector, Edward Rosten's libcvd implementation optimized for
	   SSE2. */
	featFASTER9,
	/** FASTER-9 detector, Edward Rosten's libcvd implementation optimized for
	   SSE2. */
	featFASTER10,
	/** FASTER-9 detector, Edward Rosten's libcvd implementation optimized for
	   SSE2. */
	featFASTER12,
	/** ORB detector and descriptor, OpenCV's implementation ("ORB: an efficient
	   alternative to SIFT or SURF", E. Rublee, V. Rabaud, K. Konolige, G.
	   Bradski, ICCV, 2012). */
	featORB,
	// #added by Raghavender Sahdev
	featAKAZE,  //!< AKAZE detector, OpenCV's implementation
	featLSD  //!< LSD detector, OpenCV's implementation
	// Remember: If new values are added, also update MRPT_FILL_ENUM below!

};

/** The bitwise OR combination of values of TDescriptorType are used in
 * CFeatureExtraction::computeDescriptors to indicate which descriptors are to
 * be computed for features.
 */
enum TDescriptorType
{
	/** Used in some methods to mean "any of the present descriptors" */
	descAny = 0,
	/** SIFT descriptors */
	descSIFT = 1,
	/** SURF descriptors */
	descSURF = 2,
	/** Intensity-domain spin image descriptors */
	descSpinImages = 4,
	/** Polar image descriptor */
	descPolarImages = 8,
	/** Log-Polar image descriptor */
	descLogPolarImages = 16,
	/** Bit-based feature descriptor */
	descORB = 32,
	descBLD = 64,  //!< BLD Line descriptor
	descLATCH = 128  //!< LATCH Line descriptor
	// Remember: If new values are added, also update MRPT_FILL_ENUM below!
};

enum TFeatureTrackStatus
{
	// Init value
	/** Inactive (right after detection, and before being tried to track) */
	status_IDLE = 0,

	// Ok:
	/** Feature correctly tracked */
	status_TRACKED = 5,

	// Bad:
	/** Feature fell Out Of Bounds (out of the image limits, too close to image
	   borders) */
	status_OOB = 1,
	/** Unable to track this feature (mismatch is too high for the given
	   tracking window: lack of texture? oclussion?) */
	status_LOST = 10
};

/** One feature observation entry, used within sequences with
 * TSequenceFeatureObservations */
struct TFeatureObservation
{
	inline TFeatureObservation() = default;
	inline TFeatureObservation(
		const TLandmarkID _id_feature, const TCameraPoseID _id_frame,
		const mrpt::img::TPixelCoordf& _px)
		: id_feature(_id_feature), id_frame(_id_frame), px(_px)
	{
	}

	/** A unique ID of this feature */
	TLandmarkID id_feature;
	/** A unique ID of a "frame" (camera position) from where the feature was
	 * observed. */
	TCameraPoseID id_frame;
	/** The pixel coordinates of the observed feature */
	mrpt::img::TPixelCoordf px;
};

/** One relative feature observation entry, used with some relative
 * bundle-adjustment functions.
 */
struct TRelativeFeaturePos
{
	inline TRelativeFeaturePos() = default;
	inline TRelativeFeaturePos(
		const mrpt::vision::TCameraPoseID _id_frame_base,
		const mrpt::math::TPoint3D& _pos)
		: id_frame_base(_id_frame_base), pos(_pos)
	{
	}

	/** The ID of the camera frame which is the coordinate reference of \a pos
	 */
	mrpt::vision::TCameraPoseID id_frame_base;
	/** The (x,y,z) location of the feature, wrt to the camera frame \a
	 * id_frame_base */
	mrpt::math::TPoint3D pos;
};

/** An index of feature IDs and their relative locations */
using TRelativeFeaturePosMap =
	std::map<mrpt::vision::TFeatureID, TRelativeFeaturePos>;

/** A complete sequence of observations of features from different camera frames
 * (poses).
 *  This structure is the input to some (Bundle-adjustment) methods in
 * mrpt::vision
 *  \note Pixel coordinates can be either "raw" or "undistorted". Read the doc
 * of functions handling this structure to see what they expect.
 *  \sa mrpt::vision::bundle_adj_full
 */
struct TSequenceFeatureObservations : public std::vector<TFeatureObservation>
{
	using BASE = std::vector<TFeatureObservation>;

	inline TSequenceFeatureObservations() = default;
	inline TSequenceFeatureObservations(size_t size) : BASE(size) {}
	inline TSequenceFeatureObservations(const TSequenceFeatureObservations& o)

		= default;

	/** Saves all entries to a text file, with each line having this format:
	 * #FRAME_ID  #FEAT_ID  #PIXEL_X  #PIXEL_Y
	 * The file is self-descripting, since the first line contains a comment
	 * line (starting with '%') explaining the format.
	 * Generated files can be loaded from MATLAB.
	 * \sa loadFromTextFile \exception std::exception On I/O error  */
	void saveToTextFile(
		const std::string& filName, bool skipFirstCommentLine = false) const;

	/** Load from a text file, in the format described in \a saveToTextFile
	 * \exception std::exception On I/O or format error */
	void loadFromTextFile(const std::string& filName);

	/** Save the list of observations + the point locations + the camera frame
	 * poses to a pair of files in the format
	 *  used by the Sparse Bundle Adjustment (SBA) C++ library.
	 *
	 *  Point file lines: X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...
	 *
	 *  Camera file lines: qr qx qy qz x y z  (Pose as a quaternion)
	 * \return false on any error
	 */
	bool saveAsSBAFiles(
		const TLandmarkLocationsVec& pts, const std::string& pts_file,
		const TFramePosesVec& cams, const std::string& cams_file) const;

	/** Remove all those features that don't have a minimum number of
	 * observations from different camera frame IDs.
	 * \return the number of erased entries.
	 * \sa After calling this you may want to call \a compressIDs */
	size_t removeFewObservedFeatures(size_t minNumObservations = 3);

	/** Remove all but one out of \a decimate_ratio camera frame IDs from the
	 * list (eg: from N camera pose IDs at return there will be just
	 * N/decimate_ratio)
	 * The algorithm first builds a sorted list of frame IDs, then keep the
	 * lowest ID, remove the next "decimate_ratio-1", and so on.
	 * \sa After calling this you may want to call \a compressIDs */
	void decimateCameraFrames(const size_t decimate_ratio);

	/** Rearrange frame and feature IDs such as they start at 0 and there are no
	 * gaps.
	 * \param old2new_camIDs If provided, the mapping from old to new IDs is
	 * stored here.
	 * \param old2new_lmIDs If provided, the mapping from old to new IDs is
	 * stored here. */
	void compressIDs(
		std::map<TCameraPoseID, TCameraPoseID>* old2new_camIDs = nullptr,
		std::map<TLandmarkID, TLandmarkID>* old2new_lmIDs = nullptr);
};

/** Parameters associated to a stereo system
 */
struct TStereoSystemParams : public mrpt::config::CLoadableOptions
{
	/** Initilization of default parameters */
	TStereoSystemParams();

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;  // See base docs
	void dumpToTextStream(std::ostream& out) const override;  // See base docs

	/** Method for propagating the feature's image coordinate uncertainty into
	 * 3D space. Default value: Prop_Linear
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

	TUnc_Prop_Method uncPropagation{Prop_Linear};

	/** Stereo Fundamental matrix */
	mrpt::math::CMatrixDouble33 F;

	/** Intrinsic parameters
	 */
	mrpt::math::CMatrixDouble33 K;
	/** Baseline. Default value: baseline = 0.119f;	[Bumblebee]
	 */
	float baseline{0.119f};
	/** Standard deviation of the error in feature detection. Default value:
	 * stdPixel = 1
	 */
	float stdPixel{1};
	/** Standard deviation of the error in disparity computation. Default value:
	 * stdDisp = 1
	 */
	float stdDisp{1};
	/** Maximum allowed distance. Default value: maxZ = 20.0f
	 */
	float maxZ{20.0f};
	/** Maximum allowed distance. Default value: minZ = 0.5f
	 */
	float minZ{0.5f};
	/** Maximum allowed height. Default value: maxY = 3.0f
	 */
	float maxY{3.0f};
	/** K factor for the UT. Default value: k = 1.5f
	 */
	float factor_k{1.5f};
	/** Alpha factor for SUT. Default value: a = 1e-3
	 */
	float factor_a{1e-3f};
	/** Beta factor for the SUT. Default value: b = 2.0f
	 */
	float factor_b{2.0f};

	/** Parameters initialization
	 */
	// TStereoSystemParams();

};  // End struct TStereoSystemParams

/** A structure for storing a 3D ROI
 */
struct TROI
{
	// Constructors
	TROI();
	TROI(float x1, float x2, float y1, float y2, float z1, float z2);

	// Members
	float xMin{0};
	float xMax{0};
	float yMin{0};
	float yMax{0};
	float zMin{0};
	float zMax{0};
};  // end struct TROI

/** A structure for defining a ROI within an image
 */
struct TImageROI
{
	// Constructors
	TImageROI();
	TImageROI(float x1, float x2, float y1, float y2);

	// Members
	/** X coordinate limits [0,imageWidth)
	 */
	float xMin{0}, xMax{0};
	/** Y coordinate limits [0,imageHeight)
	 */
	float yMin{0}, yMax{0};
};  // end struct TImageROI

/** A structure containing options for the matching
 */
struct TMatchingOptions : public mrpt::config::CLoadableOptions
{
	/** Method for propagating the feature's image coordinate uncertainty into
	 * 3D space. Default value: Prop_Linear
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
	/** Whether or not take into account the epipolar restriction for finding
	 * correspondences */
	bool useEpipolarRestriction{true};
	/** Whether or not there is a fundamental matrix */
	bool hasFundamentalMatrix{false};
	/** Whether or not the stereo rig has the optical axes parallel */
	bool parallelOpticalAxis{true};
	/** Whether or not employ the x-coord restriction for finding
	 * correspondences (bumblebee camera, for example) */
	bool useXRestriction{true};
	/** Whether or not to add the matches found into the input matched list (if
	 * false the input list will be cleared before being filled with the new
	 * matches) */
	bool addMatches{false};
	/** Whether or not use limits (min,max) for the disparity, see also
	 * 'min_disp, max_disp' */
	bool useDisparityLimits{false};
	/** Whether or not only permit matches that are consistent from left->right
	 * and right->left */
	bool enable_robust_1to1_match;

	/** Disparity limits, see also 'useDisparityLimits' */
	float min_disp{1.0f}, max_disp{1e4f};

	mrpt::math::CMatrixDouble33 F;

	// General
	/** Matching method */
	TMatchingMethod matching_method{mmCorrelation};
	/** Epipolar constraint (rows of pixels) */
	float epipolar_TH{1.5f};

	// SIFT
	/** Maximum Euclidean Distance Between SIFT Descriptors */
	float maxEDD_TH{90.0f};
	/** Boundary Ratio between the two lowest EDD */
	float EDD_RATIO{0.6f};

	// KLT
	/** Minimum Value of the Cross Correlation */
	float minCC_TH{0.95f};
	/** Minimum Difference Between the Maximum Cross Correlation Values */
	float minDCC_TH{0.025f};
	/** Maximum Ratio Between the two highest CC values */
	float rCC_TH{0.92f};

	// SURF
	/** Maximum Euclidean Distance Between SURF Descriptors */
	float maxEDSD_TH{0.15f};
	/** Boundary Ratio between the two lowest SURF EDSD */
	float EDSD_RATIO{0.6f};

	// SAD
	/** Minimum Euclidean Distance Between Sum of Absolute Differences */
	double maxSAD_TH{0.4};
	/** Boundary Ratio between the two highest SAD */
	double SAD_RATIO{0.5};

	// ORB
	/** Maximun distance between ORB descriptors */
	double maxORB_dist;

	//			// To estimate depth
	/** Whether or not estimate the 3D position of the real features for the
	 * matches (only with parallelOpticalAxis by now). */
	bool estimateDepth{false};
	/** The maximum allowed depth for the matching. If its computed depth is
	 * larger than this, the match won't be considered. */
	double maxDepthThreshold{15.0};
	/** Intrinsic parameters of the stereo rig */
	//            double  fx,cx,cy,baseline;

	/** Constructor */
	TMatchingOptions();

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;  // See base docs
	void dumpToTextStream(std::ostream& out) const override;  // See base docs

#define COPY_MEMBER(_m) this->_m = o._m;
#define CHECK_MEMBER(_m) this->_m == o._m

	bool operator==(const TMatchingOptions& o) const
	{
		return CHECK_MEMBER(useXRestriction) &&
			   CHECK_MEMBER(useDisparityLimits) &&
			   CHECK_MEMBER(useEpipolarRestriction) &&
			   CHECK_MEMBER(addMatches) && CHECK_MEMBER(EDD_RATIO) &&
			   CHECK_MEMBER(EDSD_RATIO) &&
			   CHECK_MEMBER(enable_robust_1to1_match) &&
			   CHECK_MEMBER(epipolar_TH) && CHECK_MEMBER(estimateDepth) &&
			   CHECK_MEMBER(F) && CHECK_MEMBER(hasFundamentalMatrix) &&
			   CHECK_MEMBER(matching_method) &&
			   CHECK_MEMBER(maxDepthThreshold) && CHECK_MEMBER(maxEDD_TH) &&
			   CHECK_MEMBER(maxEDSD_TH) && CHECK_MEMBER(maxORB_dist) &&
			   CHECK_MEMBER(maxSAD_TH) && CHECK_MEMBER(max_disp) &&
			   CHECK_MEMBER(minCC_TH) && CHECK_MEMBER(minDCC_TH) &&
			   CHECK_MEMBER(min_disp) && CHECK_MEMBER(parallelOpticalAxis) &&
			   CHECK_MEMBER(rCC_TH) && CHECK_MEMBER(SAD_RATIO);
	}

	void operator=(const TMatchingOptions& o)
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

};  // end struct TMatchingOptions

/** Struct containing the output after matching multi-resolution SIFT-like
 * descriptors
 */
struct TMultiResMatchingOutput
{
	int nMatches{0};

	/** Contains the indexes within the second list corresponding to the first
	 * one. */
	std::vector<int> firstListCorrespondences;
	/** Contains the indexes within the first list corresponding to the second
	 * one. */
	std::vector<int> secondListCorrespondences;
	/** Contains the scales of the first list where the correspondence was
	 * found. */
	std::vector<int> firstListFoundScales;
	/** Contains the distances between the descriptors. */
	std::vector<double> firstListDistance;

	TMultiResMatchingOutput() = default;

};  // end struct TMultiResMatchingOutput

/** Struct containing the options when matching multi-resolution SIFT-like
 * descriptors
 */
struct TMultiResDescMatchOptions : public mrpt::config::CLoadableOptions
{
	/** Whether or not use the filter based on orientation test */
	bool useOriFilter{true};
	/** The threshold for the orientation test */
	double oriThreshold{0.2};

	/** Whether or not use the filter based on the depth test */
	bool useDepthFilter{true};

	/** The absolute threshold in descriptor distance for considering a match */
	double matchingThreshold{1e4};
	/** The ratio between the two lowest distances threshold for considering a
	 * match */
	double matchingRatioThreshold{0.5};
	/** The lowest scales in the two features to be taken into account in the
	 * matching process */
	uint32_t lowScl1{0}, lowScl2{0};
	/** The highest scales in the two features to be taken into account in the
	 * matching process */
	uint32_t highScl1{6}, highScl2{6};

	/** Size of the squared area where to search for a match. */
	uint32_t searchAreaSize{20};
	/** The allowed number of frames since a certain feature was seen for the
	 * last time. */
	uint32_t lastSeenThreshold{10};
	/** The minimum number of frames for a certain feature to be considered
	 * stable. */
	uint32_t timesSeenThreshold{5};

	/** The minimum number of features allowed in the system. If current number
	 * is below this value, more features will be found. */
	uint32_t minFeaturesToFind{30};
	/** The minimum number of features allowed in the system to not be
	 * considered to be lost. */
	uint32_t minFeaturesToBeLost{5};

	/** Default constructor
	 */
	TMultiResDescMatchOptions() = default;

	TMultiResDescMatchOptions(
		const bool& _useOriFilter, const double& _oriThreshold,
		const bool& _useDepthFilter, const double& _th, const double& _th2,
		const unsigned int& _lwscl1, const unsigned int& _lwscl2,
		const unsigned int& _hwscl1, const unsigned int& _hwscl2,
		const int& _searchAreaSize, const int& _lsth, const int& _tsth,
		const int& _minFeaturesToFind, const int& _minFeaturesToBeLost)
		: useOriFilter(_useOriFilter),
		  oriThreshold(_oriThreshold),
		  useDepthFilter(_useDepthFilter),
		  matchingThreshold(_th),
		  matchingRatioThreshold(_th2),
		  lowScl1(_lwscl1),
		  lowScl2(_lwscl2),
		  highScl1(_hwscl1),
		  highScl2(_hwscl2),
		  searchAreaSize(_searchAreaSize),
		  lastSeenThreshold(_lsth),
		  timesSeenThreshold(_tsth),
		  minFeaturesToFind(_minFeaturesToFind),
		  minFeaturesToBeLost(_minFeaturesToBeLost)
	{
	}

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg,
		const std::string& section) override;
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& section) const override;
	void dumpToTextStream(std::ostream& out) const override;

};  // end TMultiResDescMatchOptions

/** Struct containing the options when computing the multi-resolution SIFT-like
 * descriptors
 */
struct TMultiResDescOptions : public mrpt::config::CLoadableOptions
{
	/** The size of the base patch */
	uint32_t basePSize{23};
	/** The set of scales relatives to the base patch */
	std::vector<double> scales;
	/** The subset of scales for which to compute the descriptors */
	uint32_t comLScl, comHScl;
	/** The sigmas for the Gaussian kernels */
	double sg1{0.5}, sg2{7.5}, sg3{8.0};
	/** Whether or not to compute the depth of the feature */
	bool computeDepth{true};
	/** Whether or not to blur the image previously to compute the descriptors
	 */
	bool blurImage{true};
	/** Intrinsic stereo pair parameters for computing the depth of the feature
	 */
	double fx{0.0}, cx{0.0}, cy{0.0}, baseline{0.0};
	/** Whether or not compute the coefficients for mantaining a HASH table of
	 * descriptors (for relocalization) */
	bool computeHashCoeffs{false};

	/** The SIFT-like descriptor is cropped at this value during normalization
	 */
	double cropValue{0.2};

	/** Default constructor
	 */
	TMultiResDescOptions()
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

	TMultiResDescOptions(
		const unsigned int& _basePSize, const std::vector<double>& _scales,
		const unsigned int& _comLScl, const unsigned int& _comHScl,
		const double& _sg1, const double& _sg2, const double& _sg3,
		const bool& _computeDepth, const bool _blurImage, const double& _fx,
		const double& _cx, const double& _cy, const double& _baseline,
		const bool& _computeHashCoeffs, const double& _cropValue)
		: basePSize(_basePSize),
		  comLScl(_comLScl),
		  comHScl(_comHScl),
		  sg1(_sg1),
		  sg2(_sg2),
		  sg3(_sg3),
		  computeDepth(_computeDepth),
		  blurImage(_blurImage),
		  fx(_fx),
		  cx(_cx),
		  cy(_cy),
		  baseline(_baseline),
		  computeHashCoeffs(_computeHashCoeffs),
		  cropValue(_cropValue)
	{
		scales.resize(_scales.size());
		for (unsigned int k = 0; k < _scales.size(); ++k)
			scales[k] = _scales[k];
	}

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;  // See base docs
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& section) const override;  // See base docs
	void dumpToTextStream(std::ostream& out) const override;  // See base docs

};  // end TMultiResDescOptions

/** @} */  // end of grouping
}  // namespace mrpt::vision
MRPT_ENUM_TYPE_BEGIN(mrpt::vision::TFeatureType)
using namespace mrpt::vision;
MRPT_FILL_ENUM(featNotDefined);
MRPT_FILL_ENUM(featKLT);
MRPT_FILL_ENUM(featHarris);
MRPT_FILL_ENUM(featBCD);
MRPT_FILL_ENUM(featSIFT);
MRPT_FILL_ENUM(featSURF);
MRPT_FILL_ENUM(featBeacon);
MRPT_FILL_ENUM(featFAST);
MRPT_FILL_ENUM(featFASTER9);
MRPT_FILL_ENUM(featFASTER10);
MRPT_FILL_ENUM(featFASTER12);
MRPT_FILL_ENUM(featORB);
MRPT_FILL_ENUM(featAKAZE);
MRPT_FILL_ENUM(featLSD);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::vision::TDescriptorType)
using namespace mrpt::vision;
MRPT_FILL_ENUM(descAny);
MRPT_FILL_ENUM(descSIFT);
MRPT_FILL_ENUM(descSURF);
MRPT_FILL_ENUM(descSpinImages);
MRPT_FILL_ENUM(descPolarImages);
MRPT_FILL_ENUM(descLogPolarImages);
MRPT_FILL_ENUM(descORB);
MRPT_FILL_ENUM(descBLD);
MRPT_FILL_ENUM(descLATCH);
MRPT_ENUM_TYPE_END()
