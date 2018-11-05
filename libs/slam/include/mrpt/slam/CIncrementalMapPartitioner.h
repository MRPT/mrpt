/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/typemeta/TEnumType.h>
#include <functional>
#include <limits>

namespace mrpt::slam
{
/** For use in CIncrementalMapPartitioner
 * \ingroup mrpt_slam_grp  */
enum similarity_method_t : uint8_t
{
	smMETRIC_MAP_MATCHING = 0,
	smOBSERVATION_OVERLAP,
	smCUSTOM_FUNCTION
};

/** Map keyframe, comprising raw observations and they as a metric map.
 * For use in CIncrementalMapPartitioner
 * \ingroup mrpt_slam_grp */
struct map_keyframe_t
{
	uint32_t kf_id{0};
	mrpt::maps::CMultiMetricMap::Ptr metric_map;
	mrpt::obs::CSensoryFrame::Ptr raw_observations;
};

/** Type of similarity evaluator for map keyframes.
 * For use in CIncrementalMapPartitioner
 * \ingroup mrpt_slam_grp  */
using similarity_func_t = std::function<double(
	const map_keyframe_t& kf1, const map_keyframe_t& kf2,
	const mrpt::poses::CPose3D& relPose2wrt1)>;

/** Finds partitions in metric maps based on N-cut graph partition theory.
 * \ingroup mrpt_slam_grp
 */
class CIncrementalMapPartitioner : public mrpt::system::COutputLogger,
								   public mrpt::serialization::CSerializable
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE(CIncrementalMapPartitioner)

   public:
	/** ctor */
	CIncrementalMapPartitioner() : COutputLogger("CIncrementalMapPartitioner")
	{
	}

	/** Configuration parameters */
	struct TOptions : public mrpt::config::CLoadableOptions
	{
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& target,
			const std::string& section) const override;

		/**!< N-cut partition threshold [0,2] (default=1) */
		double partitionThreshold{1.0};

		/** These parameters are loaded/saved to config files
		 * with the prefix "mrp.{param_name}" */
		mrpt::maps::TMatchingRatioParams mrp;

		/* Force bisection (true) or automatically determine
		 * number of partitions (false=default).
		 */
		bool forceBisectionOnly{false};

		/** Defines the method for determining the adjacency matrix values.
		 * \sa CIncrementalMapPartitioner::setSimilarityMethod()
		 */
		similarity_method_t simil_method{smMETRIC_MAP_MATCHING};

		/** If a partition leads to a cluster with less elements than this, it
		 * will be rejected even if had a good Ncut (default=1).
		 */
		uint64_t minimumNumberElementsEachCluster{1};

		/** Type and parameters of metric map(s) to build for each keyframe.
		 * Parameters can be loaded from a config file from sections
		 * with the prefix of this "TOptions" section + ".metricmap".
		 * Default: a CSimplePointsMap */
		mrpt::maps::TSetOfMetricMapInitializers metricmap;

		/** Maximum distance, in KF identifier numbers, to check for similarity.
		 * Default=Infinite. Can be used to constraint the wrong detection of
		 * clusters after loop closures but before correcting global poses. */
		uint64_t maxKeyFrameDistanceToEval{
			std::numeric_limits<uint64_t>::max()};

		TOptions();
	};

	/** \name Main map partition API
	 * @{ */

	/** Algorithm parameters */
	TOptions options;

	/* Reset the internal state to an empty map */
	void clear();

	/**\brief Insert a new keyframe to the graph.
	 *
	 * Call this method each time a new observation is added to the map/graph.
	 * Afterwards, call updatePartitions() to get the updated partitions.
	 *
	 * \param frame The sensed data
	 * \param robotPose An estimation of the robot global pose.
	 *
	 * \return The index of the new pose in the graph, which can be used to
	 * refer to this pose in the future.
	 *
	 * \sa updatePartitions
	 */
	uint32_t addMapFrame(
		const mrpt::obs::CSensoryFrame& frame,
		const mrpt::poses::CPose3DPDF& robotPose3D);

	/** Recalculate the map/graph partitions. \sa addMapFrame() */
	void updatePartitions(std::vector<std::vector<uint32_t>>& partitions);

	/**Get the total node count currently in the internal map/graph. */
	size_t getNodesCount();

	/** Remove a list of keyframes, with indices as returned by addMapFrame()
	 * \param changeCoordsRef If true, coordinates are changed to leave the
	 * first node at (0,0,0).
	 */
	void removeSetOfNodes(
		std::vector<uint32_t> indexesToRemove, bool changeCoordsRef = true);

	/**\name Change Coordinates System
	 * \brief Change the coordinate origin of all stored poses
	 *
	 * Used for consistency with future new poses to enter in the system.
	 */
	void changeCoordinatesOrigin(const mrpt::poses::CPose3D& newOrigin);
	/**\brief  The new origin is given by the index of the pose that is to
	 * become the new origin.
	 */
	void changeCoordinatesOriginPoseIndex(unsigned int newOriginPose);

	/** Select the similarity method to use for newly inserted keyframes */
	void setSimilarityMethod(similarity_method_t method)
	{
		options.simil_method = method;
	}

	/** Sets a custom function for the similarity of new keyframes */
	void setSimilarityMethod(similarity_func_t func)
	{
		options.simil_method = smCUSTOM_FUNCTION;
		m_sim_func = func;
	}
	/** @} */

	/** \name Access API to internal graph data
	 * @{ */
	/**Return a 3D representation of the graph: poses & links between them.
	 * The previous contents of "objs" will be discarded
	 */
	void getAs3DScene(
		mrpt::opengl::CSetOfObjects::Ptr& objs,
		const std::map<uint32_t, int64_t>* renameIndexes = nullptr) const;

	/** Return a copy of the adjacency matrix.  */
	template <class MATRIX>
	void getAdjacencyMatrix(MATRIX& outMatrix) const
	{
		outMatrix = m_A;
	}

	/** Return a const ref to the internal adjacency matrix.  */
	const mrpt::math::CMatrixDouble& getAdjacencyMatrix() const { return m_A; }
	/** Read-only access to the sequence of Sensory Frames */
	const mrpt::maps::CSimpleMap* getSequenceOfFrames() const
	{
		return &m_individualFrames;
	}

	/** Access to the sequence of Sensory Frames */
	mrpt::maps::CSimpleMap* getSequenceOfFrames()
	{
		return &m_individualFrames;
	}
	/** @} */

   private:
	mrpt::maps::CSimpleMap m_individualFrames;
	std::deque<mrpt::maps::CMultiMetricMap::Ptr> m_individualMaps;

	/** Adjacency matrix */
	mrpt::math::CMatrixD m_A{0, 0};

	/** The last partition */
	std::vector<std::vector<uint32_t>> m_last_partition;

	/** This will be true after adding new observations, and before an
	 * "updatePartitions" is invoked. */
	bool m_last_last_partition_are_new_ones{false};

	similarity_func_t m_sim_func;

};  // End of class def.
}  // namespace mrpt::slam
MRPT_ENUM_TYPE_BEGIN(mrpt::slam::similarity_method_t)
MRPT_FILL_ENUM_MEMBER(mrpt::slam, smMETRIC_MAP_MATCHING);
MRPT_FILL_ENUM_MEMBER(mrpt::slam, smOBSERVATION_OVERLAP);
MRPT_FILL_ENUM_MEMBER(mrpt::slam, smCUSTOM_FUNCTION);
MRPT_ENUM_TYPE_END()
