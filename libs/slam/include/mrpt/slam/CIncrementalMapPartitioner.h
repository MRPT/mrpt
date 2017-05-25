/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CINCREMENTALMAPPARTITIONER_H
#define CINCREMENTALMAPPARTITIONER_H

#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/poses/poses_frwds.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt { namespace slam {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CIncrementalMapPartitioner, mrpt::utils::CSerializable, SLAM_IMPEXP)

	/** This class can be used to make partitions on a map/graph build from
	  *   observations taken at some poses/nodes.
	  * \ingroup mrpt_slam_grp
	  */
	class SLAM_IMPEXP  CIncrementalMapPartitioner :
		public mrpt::utils::COutputLogger,
		public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE(CIncrementalMapPartitioner)

	public:
		CIncrementalMapPartitioner(); //!< ctor
		virtual ~CIncrementalMapPartitioner(); //!< dtor

		/*\brief  Initialization: Start of a new map, new internal matrices,...
		  */
		void clear();

		/** Configuration of the algorithm:
		  */
		struct SLAM_IMPEXP TOptions : public utils::CLoadableOptions
		{
			/*\brief  Sets default values at object creation
			  */
			TOptions();

			void loadFromConfigFile(
					const mrpt::utils::CConfigFileBase &source,
					const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/**\brief The partition threshold for bisection in range [0,2], default=1.0
			 *
			 */
			float	partitionThreshold;

			/*\brief  For the occupancy grid maps of each node, default=0.10
			 *
			 */
			float	gridResolution;

			/*\brief  Used in the computation of weights, default=0.20
			 *
			 */
			float	minDistForCorrespondence;

			/*\brief  Used in the computation of weights, default=2.0
			 *
			 */
			float	minMahaDistForCorrespondence;

			/*\brief  If set to true (default), 1 or 2 clusters will be returned.
			 * Default=false -> Autodetermine the number of partitions.
			 */
			bool forceBisectionOnly;

			/** If set to true (default), adjacency matrix is computed from maps
			 * matching; otherwise, the method CObservation::likelihoodWith will be
			 * called directly from the SFs.
			 */
			bool useMapMatching;

			/** If a partition leads to a cluster with less elements than this, it
			 * will be rejected even if had a good Ncut (default=1).
			 */
			int minimumNumberElementsEachCluster;

		} options;

		/**\name Add to the Map Frames
		 * \brief Add a new frame to the current graph
		 *
		 * Call this method each time a new observation is added to the map/graph,
		 * and whenever you want to update the partitions, call "updatePartitions"
		 *
		 * \param frame The sensed data
		 * \param robotPose*D An estimation of the robot global *D pose.
		 *
		 * \return { The index of the new pose in the internal list, which will be
		 * used to refer to the pose in the future. }
		 *
		 * \sa updatePartitions
		 */
		/**\{*/
		unsigned int addMapFrame(
				const mrpt::obs::CSensoryFramePtr &frame,
				const mrpt::poses::CPosePDFPtr &robotPose2D);
		unsigned int addMapFrame(
				const mrpt::obs::CSensoryFramePtr &frame,
				const mrpt::poses::CPose3DPDFPtr &robotPose3D);
		unsigned int addMapFrame(
				const mrpt::obs::CSensoryFrame &frame,
				const mrpt::poses::CPose3DPDF &robotPose3D);
		/**\}*/

		/**\brief This method executed only the neccesary part of the partition to
		 * take into account the lastest added frames.
		 *
		 * \sa addMapFrame
		 */
		void updatePartitions(std::vector<vector_uint> &partitions);

		/**\brief Get the total node count currently in the internal map/graph.
		 *
		 */
		unsigned int getNodesCount();

		/**\brief Remove the stated nodes (0-based indexes) from the internal
		 * lists.
		 *
		 * If changeCoordsRef is true, coordinates are changed to leave the first
		 * node at (0,0,0).
		 */
		void  removeSetOfNodes(vector_uint indexesToRemove, bool changeCoordsRef=true);

		/**\brief Return a copy of the internal adjacency matrix.  */
		template <class MATRIX>
		void  getAdjacencyMatrix(MATRIX &outMatrix) const { outMatrix = m_A; }

		/**\brief Return a const ref to the internal adjacency matrix.  */
		const mrpt::math::CMatrixDouble& getAdjacencyMatrix() const { return m_A; }

		/**\brief Read-only access to the sequence of Sensory Frames
		 *
		 */
		const mrpt::maps::CSimpleMap* getSequenceOfFrames() const {
			return &m_individualFrames;
		}

		/** Access to the sequence of Sensory Frames
		 *
		 */
		mrpt::maps::CSimpleMap* getSequenceOfFrames()
		{
			return &m_individualFrames;
		}

		/** Mark all nodes for reconsideration in the next call to
		 * "updatePartitions", instead of considering just those affected by
		 * aditions of new arcs.
		 */
		void markAllNodesForReconsideration();
		/**\name Change Coordinates System 
		 * \brief Change the coordinate origin of all stored poses
		 *
		 * Used for consistency with future new poses to enter in the system.
		 */
		/**\{*/
		void changeCoordinatesOrigin(const mrpt::poses::CPose3D  &newOrigin);
		/**\brief  The new origin is given by the index of the pose that is to
		 * become the new origin.
		 */
		void changeCoordinatesOriginPoseIndex(const unsigned &newOriginPose);
		/**\}*/

		/**\brief Return a 3D representation of the current state: poses & links
		 * between them.
		 *
		 * The previous contents of "objs" will be discarded
		 */
		void getAs3DScene(
			mrpt::opengl::CSetOfObjectsPtr &objs,
			const std::map< uint32_t, int64_t > *renameIndexes = NULL
			) const;

	private:
		mrpt::maps::CSimpleMap m_individualFrames;
		std::deque<mrpt::maps::CMultiMetricMap>	m_individualMaps;

		/** Adjacency matrix */
		mrpt::math::CMatrixD m_A;

		/** The last partition */
		std::vector<vector_uint> m_last_partition;

		/** This will be true after adding new observations, and before an
		 * "updatePartitions" is invoked. */
		bool m_last_last_partition_are_new_ones;

		/** The list of keyframes to consider in the next update */
		std::vector<uint8_t> m_modified_nodes;

	};	// End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CIncrementalMapPartitioner, mrpt::utils::CSerializable, SLAM_IMPEXP)


} } // End of namespace

#endif
