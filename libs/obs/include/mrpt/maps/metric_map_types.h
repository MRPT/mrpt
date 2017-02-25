/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		/** Parameters for the determination of matchings between point clouds, etc. \sa CMetricMap::determineMatching2D, CMetricMap::determineMatching3D */
		struct OBS_IMPEXP TMatchingParams
		{
			float  maxDistForCorrespondence;          //!< Maximum linear distance between two points to be paired (meters)
			float  maxAngularDistForCorrespondence;   //!< Allowed "angular error" (in radians): this permits larger pairing threshold distances to more distant points.
			bool   onlyKeepTheClosest;  //!< If set to true (default), only the closest correspondence will be returned. If false all are returned.
			bool   onlyUniqueRobust;    //!< Additional consistency filter: "onlyKeepTheClosest" allows one correspondence for each "local map" point, but many of them may have as corresponding pair the same "global point", which this flag avoids.
			size_t decimation_other_map_points; //!< (Default=1) Only consider 1 out of this number of points from the "other" map.
			size_t offset_other_map_points;  //!< Index of the first point in the "other" map to start checking for correspondences (Default=0)
			mrpt::math::TPoint3D angularDistPivotPoint; //!< The point used to calculate angular distances: e.g. the coordinates of the sensor for a 2D laser scanner.

			/** Ctor: default values */
			TMatchingParams() :
				maxDistForCorrespondence(0.50f),
				maxAngularDistForCorrespondence(.0f),
				onlyKeepTheClosest(true),
				onlyUniqueRobust(false),
				decimation_other_map_points(1),
				offset_other_map_points(0),
				angularDistPivotPoint(0,0,0)
			{}
		};

		/** Additional results from the determination of matchings between point clouds, etc., apart from the pairings themselves \sa CMetricMap::determineMatching2D, CMetricMap::determineMatching3D */
		struct OBS_IMPEXP TMatchingExtraResults
		{
			float correspondencesRatio; //!< The ratio [0,1] of points in otherMap with at least one correspondence.
			float sumSqrDist;           //!< The sum of all matched points squared distances.If undesired, set to NULL, as default.

			TMatchingExtraResults() : correspondencesRatio(0),sumSqrDist(0)
			{}
		};

		/** Parameters for CMetricMap::compute3DMatchingRatio() */
		struct OBS_IMPEXP TMatchingRatioParams
		{
			float maxDistForCorr;     //!< (Default: 0.10f) The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
			float maxMahaDistForCorr; //!< (Default: 2.0f) The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.

			TMatchingRatioParams() : maxDistForCorr(0.10f), maxMahaDistForCorr(2.0f)
			{}
		};

		DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE ( TMapGenericParams, OBS_IMPEXP )
		/** Common params to all maps derived from mrpt::maps::CMetricMap  */
		class OBS_IMPEXP TMapGenericParams : public mrpt::utils::CLoadableOptions, public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( TMapGenericParams )
		public:
			bool  enableSaveAs3DObject;        //!< (Default=true) If false, calling CMetricMap::getAs3DObject() will have no effects
			bool  enableObservationLikelihood; //!< (Default=true) Enable computing observation likelihoods with this map
			bool  enableObservationInsertion;  //!< (Default=true) Enable inserting observations in this map 

			TMapGenericParams();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &sectionNamePrefix) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_LINKAGE ( TMapGenericParams, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

