/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/round.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/vision/types.h>
#include <functional>

namespace mrpt::vision
{
/** \addtogroup  mrptvision_features
	@{ */

/** Simple structure for image key points. Descriptors are stored separately in
 * CFeatureList. This is a template to allow using both integers
 * (TKeyPoint) or floats (TKeyPointf) as pixel coordinates. \sa
 * TKeyPoint, TKeyPointf
 */
template <typename PIXEL_COORD_TYPE>
struct TKeyPoint_templ
{
	/** The type of \a pt */
	using pixel_coords_t = PIXEL_COORD_TYPE;
	/** The type of pt.x and pt.y */
	using pixel_coord_t = typename PIXEL_COORD_TYPE::pixel_coord_t;

	/** Coordinates in the image */
	pixel_coords_t pt;

	/** ID of the feature */
	TFeatureID ID{static_cast<TFeatureID>(-1)};

	/** Status of the feature tracking process */
	TFeatureTrackStatus track_status{status_IDLE};

	/** A measure of the "goodness" of the feature (typically, the KLT_response
	 * value) */
	float response{0};
	/** The image octave the image was found in: 0=original image, 1=1/2 image,
	 * 2=1/4 image, etc. */
	uint8_t octave{0};
	/** A field for any other flags needed by the user (this has not a
	 * predefined meaning) */
	uint8_t user_flags{0};

	/** Constructor that only sets the pt.{x,y} values, leaving all other values
	 * to *undefined values*. */
	template <typename COORD_TYPE>
	inline TKeyPoint_templ(const COORD_TYPE x, const COORD_TYPE y) : pt(x, y)
	{
	}

	/** Default constructor, leaves all fields uninitialized */
	inline TKeyPoint_templ() = default;
	template <typename OTHER_TKeyPoint>
	explicit TKeyPoint_templ(const OTHER_TKeyPoint& o)
		: pt(o.pt.x, o.pt.y),
		  ID(o.ID),
		  track_status(o.track_status),
		  response(o.response),
		  octave(o.octave),
		  user_flags(o.user_flags)
	{
	}
};

/** Simple structure for image key points.
 *  \sa TKeyPointf, CFeature, TKeyPointList
 */
using TKeyPoint = TKeyPoint_templ<mrpt::img::TPixelCoord>;

/** A version of TKeyPoint with subpixel precision */
using TKeyPointf = TKeyPoint_templ<mrpt::img::TPixelCoordf>;

template <typename FEATURE>
struct TKeyPointTraits;

template <>
struct TKeyPointTraits<TKeyPoint>
{
	using coord_t = int;

	static inline coord_t f2coord(float f) { return mrpt::round(f); }
};

template <>
struct TKeyPointTraits<TKeyPointf>
{
	using coord_t = float;

	static inline coord_t f2coord(float f) { return f; }
};

/** A list of image features using the structure TKeyPoint for each feature
 * Users normally will use directly: TKeyPointList, TKeyPointfList
 */
template <typename FEATURE>
struct TKeyPointList_templ
{
   public:
	using TFeatureVector = std::vector<FEATURE>;
	using feature_t = FEATURE;

	/** @name Utilities
		@{ */

	/** Returns a const ref to the actual std::vector<> container */
	const TFeatureVector& getVector() const { return m_feats; }
	/** Returns the maximum ID of all features in the list, or 0 if it's empty
	 */
	TFeatureID getMaxID() const
	{
		if (this->empty()) return 0;
		TFeatureID maxID = m_feats[0].ID;
		size_t N = m_feats.size() - 1;
		for (; N; --N) mrpt::keep_max(maxID, m_feats[N].ID);
		return maxID;
	}

	/** Returns a vector with a LUT of the first feature index per row, to
	 * efficiently look for neighbors, etc.
	 *  By default this vector is empty, so if a feature detector is used that
	 * doesn't fill this out, it will remain empty and useless.
	 *  \note FASTER detectors do fill this out. In general, a feature list
	 * that dynamically changes will not use this LUT.
	 */
	const std::vector<size_t>& getFirstIndexPerRowLUT() const
	{
		return m_first_index_per_row;
	}
	/// \overload
	std::vector<size_t>& getFirstIndexPerRowLUT()
	{
		return m_first_index_per_row;
	}

	/** @} */

	/** @name Method and datatypes to emulate a STL container
		@{ */
	using iterator = typename TFeatureVector::iterator;
	using const_iterator = typename TFeatureVector::const_iterator;

	using reverse_iterator = typename TFeatureVector::reverse_iterator;
	using const_reverse_iterator =
		typename TFeatureVector::const_reverse_iterator;

	inline iterator begin() { return m_feats.begin(); }
	inline iterator end() { return m_feats.end(); }
	inline const_iterator begin() const { return m_feats.begin(); }
	inline const_iterator end() const { return m_feats.end(); }
	inline reverse_iterator rbegin() { return m_feats.rbegin(); }
	inline reverse_iterator rend() { return m_feats.rend(); }
	inline const_reverse_iterator rbegin() const { return m_feats.rbegin(); }
	inline const_reverse_iterator rend() const { return m_feats.rend(); }
	inline iterator erase(const iterator& it) { return m_feats.erase(it); }
	inline bool empty() const { return m_feats.empty(); }
	inline size_t size() const { return m_feats.size(); }
	inline void clear()
	{
		m_feats.clear();
		m_first_index_per_row.clear();
	}
	inline void resize(size_t N) { m_feats.resize(N); }
	inline void reserve(size_t N) { m_feats.reserve(N); }
	inline void push_back(const FEATURE& f) { m_feats.push_back(f); }
	inline void push_back_fast(const FEATURE& f) { m_feats.push_back(f); }
	inline void emplace_back(const int x, const int y)
	{
		m_feats.emplace_back(x, y);
	}

	inline FEATURE& operator[](const std::size_t index)
	{
		return m_feats[index];
	}
	inline const FEATURE& operator[](const std::size_t index) const
	{
		return m_feats[index];
	}

	inline FEATURE& back() { return m_feats.back(); }
	inline const FEATURE& back() const { return m_feats.back(); }
	inline FEATURE& front() { return m_feats.front(); }
	inline const FEATURE& front() const { return m_feats.front(); }
	/** @} */

	/** @name getFeature*() methods for template-based access to feature list
		@{ */
	inline typename TKeyPointTraits<FEATURE>::coord_t getFeatureX(
		size_t i) const
	{
		return m_feats[i].pt.x;
	}
	inline typename TKeyPointTraits<FEATURE>::coord_t getFeatureY(
		size_t i) const
	{
		return m_feats[i].pt.y;
	}
	inline TFeatureID getFeatureID(size_t i) const { return m_feats[i].ID; }
	inline float getFeatureResponse(size_t i) const
	{
		return m_feats[i].response;
	}
	inline bool isPointFeature(size_t i) const
	{
		MRPT_UNUSED_PARAM(i);
		return true;
	}
	inline float getScale(size_t i) const
	{
		return d2f(1 << m_feats[i].octave);
	}
	inline TFeatureTrackStatus getTrackStatus(size_t i)
	{
		return m_feats[i].track_status;
	}

	inline void setFeatureX(
		size_t i, typename TKeyPointTraits<FEATURE>::coord_t x)
	{
		m_feats[i].pt.x = x;
	}
	inline void setFeatureY(
		size_t i, typename TKeyPointTraits<FEATURE>::coord_t y)
	{
		m_feats[i].pt.y = y;
	}

	inline void setFeatureXf(size_t i, float x)
	{
		m_feats[i].pt.x = TKeyPointTraits<FEATURE>::f2coord(x);
	}
	inline void setFeatureYf(size_t i, float y)
	{
		m_feats[i].pt.y = TKeyPointTraits<FEATURE>::f2coord(y);
	}

	inline void setFeatureID(size_t i, TFeatureID id) { m_feats[i]->ID = id; }
	inline void setFeatureResponse(size_t i, float r)
	{
		m_feats[i]->response = r;
	}
	inline void setScale(size_t i, float s)
	{
		m_feats[i]->octave = mrpt::round(std::log(s) / std::log(2));
	}
	inline void setTrackStatus(size_t i, TFeatureTrackStatus s)
	{
		m_feats[i].track_status = s;
	}

	inline void mark_as_outdated() const {}
	/** @} */

   private:
	/** The actual container with the list of features */
	TFeatureVector m_feats;
	/** A LUT of the first feature index per row, to efficiently look for
	 * neighbors, etc. */
	std::vector<size_t> m_first_index_per_row;
	mrpt::math::CMatrixBool m_occupied_sections;

};  // end of class

/** A list of image features using the structure TKeyPoint for each feature
 * - capable of KD-tree computations */
using TKeyPointList = TKeyPointList_templ<TKeyPoint>;

/** A list of image features using the structure TKeyPointf for each
 * feature - capable of KD-tree computations */
using TKeyPointfList = TKeyPointList_templ<TKeyPointf>;

/** A helper struct to sort keypoints by their response: It can be used with
 *these types:
 *	  - std::vector<cv::KeyPoint>
 *	  - mrpt::vision::TKeyPointList
 */
template <typename FEATURE_LIST>
struct KeypointResponseSorter : public std::function<bool(size_t, size_t)>
{
	const FEATURE_LIST& m_data;
	KeypointResponseSorter(const FEATURE_LIST& data) : m_data(data) {}
	bool operator()(size_t k1, size_t k2) const
	{
		return (m_data[k1].response > m_data[k2].response);
	}
};

/** Helper class: KD-tree search class for vector<KeyPoint>:
 *  Call mark_as_outdated() to force rebuilding the kd-tree after modifying the
 * linked feature list.
 *  \tparam FEAT Can be cv::KeyPoint or mrpt::vision::TKeyPoint
 */
template <typename FEAT>
class CFeatureListKDTree
	: public mrpt::math::KDTreeCapable<CFeatureListKDTree<FEAT>>
{
   public:
	inline void mark_as_outdated()
	{
		mrpt::math::KDTreeCapable<
			CFeatureListKDTree<FEAT>>::kdtree_mark_as_outdated();
	}

	const std::vector<FEAT>& m_data;
	CFeatureListKDTree(const std::vector<FEAT>& data) : m_data(data) {}
	/** @name Methods that MUST be implemented by children classes of
	   KDTreeCapable
		@{ */

	/// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return m_data.size(); }
	/// Returns the dim'th component of the idx'th point in the class:
	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		ASSERTDEB_(dim == 0 || dim == 1);
		if (dim == 0)
			return m_data[idx].pt.x;
		else
			return m_data[idx].pt.y;
	}

	/// Returns the distance between the vector "p1[0:size-1]" and the data
	/// point with index "idx_p2" stored in the class:
	inline float kdtree_distance(
		const float* p1, const size_t idx_p2, size_t size) const
	{
		MRPT_UNUSED_PARAM(size);  // in release mode
		ASSERTDEB_(size == 2);

		const float d0 = p1[0] - m_data[idx_p2].pt.x;
		const float d1 = p1[1] - m_data[idx_p2].pt.y;
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

};  // end CFeatureListKDTree

/** @} */  // End of add to module: mrptvision_features

}  // namespace mrpt::vision
