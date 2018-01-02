/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef mrpt_vision_types_H
#define mrpt_vision_types_H

#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/img/CImage.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt
{
namespace vision
{
/** \addtogroup mrpt_vision_grp
  *  @{ */
/** Definition of a feature ID */
typedef uint64_t TFeatureID;

/** Unique IDs for landmarks */
typedef uint64_t TLandmarkID;
/** Unique IDs for camera frames (poses) */
typedef uint64_t TCameraPoseID;

/** A list of camera frames (6D poses) indexed by unique IDs. */
typedef mrpt::aligned_std_map<TCameraPoseID, mrpt::poses::CPose3D>
	TFramePosesMap;
/** A list of camera frames (6D poses), which assumes indexes are unique,
 * consecutive IDs. */
typedef mrpt::aligned_std_vector<mrpt::poses::CPose3D> TFramePosesVec;

/** A list of landmarks (3D points) indexed by unique IDs. */
typedef std::map<TLandmarkID, mrpt::math::TPoint3D> TLandmarkLocationsMap;
/** A list of landmarks (3D points), which assumes indexes are unique,
 * consecutive IDs. */
typedef std::vector<mrpt::math::TPoint3D> TLandmarkLocationsVec;

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
	// Remember: If new values are added, also update TEnumTypeFiller below!

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
	// Remember: If new values are added, also update TEnumTypeFiller below!
	// #added by Raghavender Sahdev
	descBLD = 64,  //!< BLD Line descriptor
	descLATCH = 128  //!< LATCH Line descriptor
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
	inline TFeatureObservation() {}
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
	inline TRelativeFeaturePos() {}
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
typedef std::map<mrpt::vision::TFeatureID, TRelativeFeaturePos>
	TRelativeFeaturePosMap;

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
	typedef std::vector<TFeatureObservation> BASE;

	inline TSequenceFeatureObservations() {}
	inline TSequenceFeatureObservations(size_t size) : BASE(size) {}
	inline TSequenceFeatureObservations(const TSequenceFeatureObservations& o)
		: BASE(o)
	{
	}

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

};  // end TMultiResDescOptions

/** @} */  // end of grouping
}
// Specializations MUST occur at the same namespace:
namespace utils
{
template <>
struct TEnumTypeFiller<mrpt::vision::TFeatureType>
{
	typedef mrpt::vision::TFeatureType enum_t;
	static void fill(bimap<enum_t, std::string>& m_map)
	{
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
	}
};
template <>
struct TEnumTypeFiller<mrpt::vision::TDescriptorType>
{
	typedef mrpt::vision::TDescriptorType enum_t;
	static void fill(bimap<enum_t, std::string>& m_map)
	{
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
	}
};
}
}
#endif
