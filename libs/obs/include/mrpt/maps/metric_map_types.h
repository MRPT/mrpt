/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
/** Parameters for the determination of matchings between point clouds, etc. \sa
 * CMetricMap::determineMatching2D, CMetricMap::determineMatching3D */
struct TMatchingParams
{
	/** Maximum linear distance between two points to be paired (meters) */
	float maxDistForCorrespondence{0.50f};
	/** Allowed "angular error" (in radians): this permits larger pairing
	 * threshold distances to more distant points. */
	float maxAngularDistForCorrespondence{.0f};
	/** If set to true (default), only the closest correspondence will be
	 * returned. If false all are returned. */
	bool onlyKeepTheClosest{true};
	/** Additional consistency filter: "onlyKeepTheClosest" allows one
	 * correspondence for each "local map" point, but many of them may have as
	 * corresponding pair the same "global point", which this flag avoids. */
	bool onlyUniqueRobust{false};
	/** (Default=1) Only consider 1 out of this number of points from the
	 * "other" map. */
	size_t decimation_other_map_points{1};
	/** Index of the first point in the "other" map to start checking for
	 * correspondences (Default=0) */
	size_t offset_other_map_points{0};
	/** The point used to calculate angular distances: e.g. the coordinates of
	 * the sensor for a 2D laser scanner. */
	mrpt::math::TPoint3D angularDistPivotPoint{0, 0, 0};

	/** Ctor: default values */
	TMatchingParams() = default;
};

/** Additional results from the determination of matchings between point clouds,
 * etc., apart from the pairings themselves \sa CMetricMap::determineMatching2D,
 * CMetricMap::determineMatching3D */
struct TMatchingExtraResults
{
	/** The ratio [0,1] of points in otherMap with at least one correspondence.
	 */
	float correspondencesRatio{0};
	/** The sum of all matched points squared distances.If undesired, set to
	 * nullptr, as default. */
	float sumSqrDist{0};

	TMatchingExtraResults() = default;
};

/** Parameters for CMetricMap::compute3DMatchingRatio() */
struct TMatchingRatioParams
{
	/** (Default: 0.10f) The minimum distance between 2 non-probabilistic map
	 * elements for counting them as a correspondence. */
	float maxDistForCorr{0.10f};
	/** (Default: 2.0f) The minimum Mahalanobis distance between 2 probabilistic
	 * map elements for counting them as a correspondence. */
	float maxMahaDistForCorr{2.0f};

	TMatchingRatioParams() = default;
};

/** Common params to all maps derived from mrpt::maps::CMetricMap  */
class TMapGenericParams : public mrpt::config::CLoadableOptions,
						  public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(TMapGenericParams)
   public:
	/** (Default=true) If false, calling CMetricMap::getAs3DObject() will have
	 * no effects */
	bool enableSaveAs3DObject{true};
	/** (Default=true) Enable computing observation likelihoods with this map */
	bool enableObservationLikelihood{true};
	/** (Default=true) Enable inserting observations in this map  */
	bool enableObservationInsertion{true};

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& sectionNamePrefix) override;  // See base docs
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& target,
		const std::string& section) const override;
};

}  // namespace mrpt::maps
