/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/types.h>
#include <optional>

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

/** A generic 2D feature from an image, extracted with \a CFeatureExtraction
 * Each feature may have one or more descriptors (see \a descriptors), in
 * addition to an image patch.
 * The (Euclidean) distance between descriptors in a pair of features can be
 * computed with  descriptorDistanceTo,
 *  while the similarity of the patches is given by patchCorrelationTo.
 *
 *  \sa CFeatureList, TKeyPoint, TKeyPointList
 */
class CFeature : public mrpt::serialization::CSerializable
{
	friend class CFeatureList;
	friend class CMatchedFeatureList;

	DEFINE_SERIALIZABLE(CFeature, mrpt::vision)

   public:
	CFeature() = default;
	~CFeature() override = default;

	TKeyPointf keypoint;

	/** A patch of the image surrounding the feature */
	std::optional<mrpt::img::CImage> patch;

	/** Size of the patch (patchSize x patchSize) (it must be an odd number) */
	uint16_t patchSize{21};

	/** Keypoint method used to detect this feature */
	TKeyPointMethod type{featNotDefined};

	/** Status of the feature tracking process */
	TFeatureTrackStatus track_status{status_IDLE};

	/** A measure of the "goodness" of the feature */
	float response{0.0};

	float orientation{0.0};  //!< Main orientation of the feature
	// Scale: replaced by keypoint.octave ==> float scale{0};

	/** A field for any other flags needed by the user (this has not a
	 * predefined meaning) */
	uint8_t user_flags{0};

	// # added by Raghavender Sahdev
	float x2[2], y2[2];  //!< Coordinates for a LSD Detector to represent a line

	double depth{
		0};  //!< The estimated depth in 3D of this feature wrt the camera
	//! in the current frame
	//!
	double initialDepth{
		0};  //!< The estimated depth in 3D of this feature wrt the
	//! camera that took its image
	mrpt::math::TPoint3D
		p3D;  //!< The estimated 3D point of this feature wrt its camera

	/** Return false only for Blob detectors (SIFT, SURF) */
	bool isPointFeature() const;

	/** All the possible descriptors this feature may have */
	struct TDescriptors
	{
		TDescriptors() = default;

		/** SIFT feature descriptor */
		std::optional<std::vector<uint8_t>> SIFT;

		/** SURF feature descriptor */
		std::optional<std::vector<float>> SURF;

		/** The 2D histogram as a single row */
		std::optional<std::vector<float>> SpinImg;

		/** The number of rows (corresponding to range bins in the 2D histogram)
		 * of the original matrix from which SpinImg was extracted as a vector.
		 */
		uint16_t SpinImg_range_rows{0};

		/** A polar image centered at the interest point */
		std::optional<mrpt::math::CMatrixF> PolarImg;

		/** A log-polar image centered at the interest point */
		std::optional<mrpt::math::CMatrixF> LogPolarImg;

		/** If set to true (default=false) the call to "descriptorDistanceTo"
		 * will not consider all the rotations between polar image descriptors
		 * (PolarImg, LogPolarImg) */
		bool polarImgsNoRotation{false};

		/** ORB feature descriptor */
		std::optional<std::vector<uint8_t>> ORB;

		// added by Raghavender Sadev
		/** BLD feature descriptor */
		std::optional<std::vector<uint8_t>> BLD;
		/** LATCH feature descriptor */
		std::optional<std::vector<uint8_t>> LATCH;

		bool hasDescriptorSIFT() const { return SIFT.has_value(); }
		bool hasDescriptorSURF() const { return SURF.has_value(); }
		bool hasDescriptorSpinImg() const { return SpinImg.has_value(); }
		bool hasDescriptorPolarImg() const { return PolarImg.has_value(); }
		bool hasDescriptorLogPolarImg() const
		{
			return LogPolarImg.has_value();
		}

		bool hasDescriptorORB() const { return ORB.has_value(); }
		bool hasDescriptorBLD() const { return BLD.has_value(); }
		bool hasDescriptorLATCH() const { return LATCH.has_value(); }
	};

	TDescriptors descriptors;

	/** Return the first found descriptor, as a matrix.
	 * \return false on error, i.e. there is no valid descriptor.
	 */
	bool getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat& desc) const;

	/** Computes the normalized cross-correlation between the patches of this
	 * and another feature (normalized in the range [0,1], such as 0=best,
	 * 1=worst).
	 *  \note If this or the other features does not have patches or they are
	 * of different sizes, an exception will be raised.
	 * \sa descriptorDistanceTo
	 */
	float patchCorrelationTo(const CFeature& oFeature) const;

	/** Computes the Euclidean Distance between this feature's and other
	 * feature's descriptors, using the given descriptor or the first present
	 * one.
	 *  \note If descriptorToUse is not descAny and that descriptor is not
	 * present in one of the features, an exception will be raised.
	 * \sa patchCorrelationTo
	 */
	float descriptorDistanceTo(
		const CFeature& oFeature, TDescriptorType descriptorToUse = descAny,
		bool normalize_distances = true) const;

	/** Computes the Euclidean Distance between "this" and the "other"
	 * descriptors */
	float descriptorSIFTDistanceTo(
		const CFeature& oFeature, bool normalize_distances = true) const;

	/** Computes the Euclidean Distance between "this" and the "other"
	 * descriptors */
	float descriptorSURFDistanceTo(
		const CFeature& oFeature, bool normalize_distances = true) const;

	/** Computes the Euclidean Distance between "this" and the "other"
	 * descriptors */
	float descriptorSpinImgDistanceTo(
		const CFeature& oFeature, bool normalize_distances = true) const;

	/** Returns the minimum Euclidean Distance between "this" and the "other"
	 * polar image descriptor, for the best shift in orientation.
	 * \param oFeature The other feature to compare with.
	 * \param minDistAngle The placeholder for the angle at which the smallest
	 * distance is found.
	 * \return The distance for the best orientation (minimum distance).
	 */
	float descriptorPolarImgDistanceTo(
		const CFeature& oFeature, float& minDistAngle,
		bool normalize_distances = true) const;

	/** Returns the minimum Euclidean Distance between "this" and the "other"
	 * log-polar image descriptor, for the best shift in orientation.
	 * \param oFeature The other feature to compare with.
	 * \param minDistAngle The placeholder for the angle at which the smallest
	 * distance is found.
	 * \return The distance for the best orientation (minimum distance).
	 */
	float descriptorLogPolarImgDistanceTo(
		const CFeature& oFeature, float& minDistAngle,
		bool normalize_distances = true) const;

	/** Computes the Hamming distance "this" and the "other" descriptor ORB
	 * descriptor */
	uint8_t descriptorORBDistanceTo(const CFeature& oFeature) const;

	// # added by Raghavender Sahdev
	/** Computes the Euclidean Distance between "this" and the "other"
	 * descriptors */
	float descriptorBLDDistanceTo(
		const CFeature& oFeature, bool normalize_distances = true) const;
	/** Computes the Euclidean Distance between "this" and the "other"
	 * descriptors */
	float descriptorLATCHDistanceTo(
		const CFeature& oFeature, bool normalize_distances = true) const;

	/** Save the feature to a text file in this format:
	 *    "%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
	 *    "%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT
	 *[SIFT] HAS_SURF [SURF] HAS_MULTI [MULTI_i] HAS_ORB [ORB]"
	 *    "%% |---------------------- feature ------------------|
	 *|---------------------- descriptors ------------------------|"
	 *    "%% with:\n"
	 *    "%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4:
	 *SURF, 5: Beacon, 6: FAST, 7: ORB\n"
	 *    "%%  HAS_* : 1 if a descriptor of that type is associated to the
	 *feature."
	 *    "%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1"
	 *    "%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1"
	 *    "%%  MULTI : Present if HAS_MULTI=1: SCALE ORI N DESC_0 ... DESC_N-1"
	 *	   "%%  ORB   : Present if HAS_ORB=1: DESC_0 ... DESC_31
	 *    "%%-----------------------------------------------------------------------------\n");
	 */
	void saveToTextFile(const std::string& filename, bool APPEND = false);

	/** Get the type of the feature
	 */
	TKeyPointMethod get_type() const { return type; }
	/** Dump feature information into a text stream */
	void dumpToTextStream(std::ostream& out) const;

	void dumpToConsole() const;

   protected:
	/** Internal function used by "descriptorLogPolarImgDistanceTo" and
	 * "descriptorPolarImgDistanceTo"
	 */
	static float internal_distanceBetweenPolarImages(
		const mrpt::math::CMatrixF& desc1, const mrpt::math::CMatrixF& desc2,
		float& minDistAngle, bool normalize_distances, bool dont_shift_angle);

};  // end of class

/** A list of visual features, to be used as output by detectors, as
 * input/output by trackers, etc.
 */
class CFeatureList : public mrpt::math::KDTreeCapable<CFeatureList>
{
   protected:
	using TInternalFeatList = std::vector<CFeature>;

	/** The actual container with the list of features */
	TInternalFeatList m_feats;

   public:
	/** The type of the first feature in the list */
	inline TKeyPointMethod get_type() const
	{
		return empty() ? featNotDefined : (*begin()).get_type();
	}

	/** Save feature list to a text file */
	void saveToTextFile(const std::string& fileName, bool APPEND = false);

	/** Save feature list to a text file */
	void loadFromTextFile(const std::string& fileName);

	/** Copies the content of another CFeatureList inside this one. The inner
	 * features are also copied. */
	void copyListFrom(const CFeatureList& otherList);

	/** Get the maximum ID into the list */
	TFeatureID getMaxID() const;

	/** Get a reference to a Feature from its ID */
	const CFeature* getByID(const TFeatureID& ID) const;
	const CFeature* getByID(const TFeatureID& ID, int& out_idx) const;

	/** Get a reference to the nearest feature to the a given 2D point (version
	 * returning distance to closest feature in "max_dist")
	 *   \param x [IN] The query point x-coordinate
	 *   \param y [IN] The query point y-coordinate
	 *   \param max_dist [IN/OUT] At input: The maximum distance to search for.
	 * At output: The actual distance to the feature.
	 *  \return A pointer to the found feature, or nullptr if not found.
	 *  \note See also all the available KD-tree search methods, listed in
	 * mrpt::math::KDTreeCapable
	 */
	const CFeature* nearest(
		const float x, const float y, double& max_dist) const;

	/** Constructor */
	CFeatureList() = default;

	/** Virtual destructor */
	virtual ~CFeatureList();

	/** Call this when the list of features has been modified so the KD-tree is
	 * marked as outdated. */
	inline void mark_kdtree_as_outdated() const { kdtree_mark_as_outdated(); }
	/** @name Method and datatypes to emulate a STL container
		@{ */
	using iterator = TInternalFeatList::iterator;
	using const_iterator = TInternalFeatList::const_iterator;

	using reverse_iterator = TInternalFeatList::reverse_iterator;
	using const_reverse_iterator = TInternalFeatList::const_reverse_iterator;

	inline iterator begin() { return m_feats.begin(); }
	inline iterator end() { return m_feats.end(); }
	inline const_iterator begin() const { return m_feats.begin(); }
	inline const_iterator end() const { return m_feats.end(); }
	inline reverse_iterator rbegin() { return m_feats.rbegin(); }
	inline reverse_iterator rend() { return m_feats.rend(); }
	inline const_reverse_iterator rbegin() const { return m_feats.rbegin(); }
	inline const_reverse_iterator rend() const { return m_feats.rend(); }
	inline iterator erase(const iterator& it)
	{
		mark_kdtree_as_outdated();
		return m_feats.erase(it);
	}

	inline bool empty() const { return m_feats.empty(); }
	inline size_t size() const { return m_feats.size(); }
	inline void clear()
	{
		m_feats.clear();
		mark_kdtree_as_outdated();
	}
	inline void resize(size_t N)
	{
		m_feats.resize(N);
		mark_kdtree_as_outdated();
	}

	inline void emplace_back(CFeature&& f)
	{
		mark_kdtree_as_outdated();
		m_feats.emplace_back(std::move(f));
	}
	inline void push_back(const CFeature& f)
	{
		mark_kdtree_as_outdated();
		m_feats.push_back(f);
	}

	inline CFeature& operator[](const unsigned int index)
	{
		return m_feats[index];
	}
	inline const CFeature& operator[](const unsigned int index) const
	{
		return m_feats[index];
	}

	/** @} */

	/** @name Methods that MUST be implemented by children classes of
	   KDTreeCapable
		@{ */

	/// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return this->size(); }
	/// Returns the dim'th component of the idx'th point in the class:
	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		ASSERTDEB_(dim == 0 || dim == 1);
		if (dim == 0)
			return m_feats[idx].keypoint.pt.x;
		else
			return m_feats[idx].keypoint.pt.y;
	}

	/// Returns the distance between the vector "p1[0:size-1]" and the data
	/// point with index "idx_p2" stored in the class:
	inline float kdtree_distance(
		const float* p1, const size_t idx_p2, size_t size) const
	{
		ASSERTDEB_(size == 2);
		MRPT_UNUSED_PARAM(size);  // in release mode

		const float d0 = p1[0] - m_feats[idx_p2].keypoint.pt.x;
		const float d1 = p1[1] - m_feats[idx_p2].keypoint.pt.y;
		return d0 * d0 + d1 * d1;
	}

	// Optional bounding-box computation: return false to default to a standard
	// bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned
	//   in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3
	//   for point clouds)
	template <typename BBOX>
	bool kdtree_get_bbox(BBOX& bb) const
	{
		MRPT_UNUSED_PARAM(bb);
		return false;
	}

	/** @} */

	/** @name getFeature*() methods for template-based access to feature list
		@{ */
	inline float getFeatureX(size_t i) const
	{
		return m_feats[i].keypoint.pt.x;
	}
	inline float getFeatureY(size_t i) const
	{
		return m_feats[i].keypoint.pt.y;
	}
	inline TFeatureID getFeatureID(size_t i) const
	{
		return m_feats[i].keypoint.ID;
	}
	inline float getFeatureResponse(size_t i) const
	{
		return m_feats[i].keypoint.response;
	}
	inline bool isPointFeature(size_t i) const
	{
		return m_feats[i].isPointFeature();
	}
	inline float getScale(size_t i) const { return m_feats[i].keypoint.octave; }
	inline TFeatureTrackStatus getTrackStatus(size_t i)
	{
		return m_feats[i].keypoint.track_status;
	}

	inline void setFeatureX(size_t i, float x) { m_feats[i].keypoint.pt.x = x; }
	inline void setFeatureXf(size_t i, float x)
	{
		m_feats[i].keypoint.pt.x = x;
	}
	inline void setFeatureY(size_t i, float y) { m_feats[i].keypoint.pt.y = y; }
	inline void setFeatureYf(size_t i, float y)
	{
		m_feats[i].keypoint.pt.y = y;
	}
	inline void setFeatureID(size_t i, TFeatureID id)
	{
		m_feats[i].keypoint.ID = id;
	}
	inline void setFeatureResponse(size_t i, float r)
	{
		m_feats[i].keypoint.response = r;
	}
	inline void setScale(size_t i, uint8_t s)
	{
		m_feats[i].keypoint.octave = s;
	}
	inline void setTrackStatus(size_t i, TFeatureTrackStatus s)
	{
		m_feats[i].keypoint.track_status = s;
	}

	inline void mark_as_outdated() const { kdtree_mark_as_outdated(); }
	/** @} */

};  // end of class

/****************************************************
			Class CMATCHEDFEATURELIST
*****************************************************/
/** A list of features
 */
class CMatchedFeatureList : public std::deque<std::pair<CFeature, CFeature>>
{
   public:
	/** The type of the first feature in the list */
	inline TKeyPointMethod get_type() const
	{
		return empty() ? featNotDefined : begin()->first.get_type();
	}

	/** Save list of matched features to a text file */
	void saveToTextFile(const std::string& fileName);

	/** Returns the matching features as two separate CFeatureLists */
	void getBothFeatureLists(CFeatureList& list1, CFeatureList& list2);

	/** Returns a smart pointer to the feature with the provided ID or a empty
	 * one if not found */
	const CFeature* getByID(const TFeatureID& ID, const TListIdx& idx);

	/** Returns the maximum ID of the features in the list. If the max ID has
	   been already set up, this method just returns it.
		Otherwise, this method finds, stores and returns it.*/
	void getMaxID(
		const TListIdx& idx, TFeatureID& firstListID, TFeatureID& secondListID);

	/** Updates the value of the maximum ID of the features in the matched list,
	 * i.e. it explicitly searches for the max ID and updates the member
	 * variables. */
	void updateMaxID(const TListIdx& idx);

	/** Explicitly set the max IDs values to certain values */
	inline void setLeftMaxID(const TFeatureID& leftID) { m_leftMaxID = leftID; }
	inline void setRightMaxID(const TFeatureID& rightID)
	{
		m_rightMaxID = rightID;
	}
	inline void setMaxIDs(const TFeatureID& leftID, const TFeatureID& rightID)
	{
		setLeftMaxID(leftID);
		setRightMaxID(rightID);
	}

	CMatchedFeatureList() = default;
	virtual ~CMatchedFeatureList() = default;

   protected:
	TFeatureID m_leftMaxID{0}, m_rightMaxID{0};
};  // end of class

/** @} */  // End of add to module: mrptvision_features

}  // namespace vision

namespace typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CFeature, mrpt::vision)
}  // namespace typemeta
}  // namespace mrpt
