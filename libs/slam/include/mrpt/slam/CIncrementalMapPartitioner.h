/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef CINCREMENTALMAPPARTITIONER_H
#define CINCREMENTALMAPPARTITIONER_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/utils/stl_extensions.h>

#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/CMultiMetricMap.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace poses { class CPose3DPDF; }
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CIncrementalMapPartitioner, mrpt::utils::CSerializable, SLAM_IMPEXP )

	/** This class can be used to make partitions on a map/graph build from
	  *   observations taken at some poses/nodes.
	  * \ingroup mrpt_slam_grp
	  */
	class SLAM_IMPEXP  CIncrementalMapPartitioner : public utils::CDebugOutputCapable, public CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CIncrementalMapPartitioner )

	public:
		/** Constructor:
		  */
		CIncrementalMapPartitioner( );

		/** Destructor:
		  */
		virtual ~CIncrementalMapPartitioner();

		/** Initialization: Start of a new map, new internal matrices,...
		  */
		void clear();

		/** Configuration of the algorithm:
		  */
		struct SLAM_IMPEXP TOptions : public utils::CLoadableOptions
		{
			/** Sets default values at object creation
			  */
			TOptions();

			/** Load parameters from configuration source
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
			  */
			void  dumpToTextStream(CStream	&out) const;

			/** The partition threshold for bisection in range [0,2], default=1.0
			  */
			float	partitionThreshold;

			/** For the occupancy grid maps of each node, default=0.10
			  */
			float	gridResolution;

			/** Used in the computation of weights, default=0.20
			  */
			float	minDistForCorrespondence;

			/** Used in the computation of weights, default=2.0
			  */
			float	minMahaDistForCorrespondence;

			/** If set to true (default), 1 or 2 clusters will be returned. Default=false -> Autodetermine the number of partitions.
			  */
			bool	forceBisectionOnly;

			/** If set to true (default), adjacency matrix is computed from maps matching; otherwise, the method CObservation::likelihoodWith will be called directly from the SFs.
			  */
			bool   useMapMatching;

			/** If a partition leads to a cluster with less elements than this, it will be rejected even if had a good Ncut (default=1). */
			int    minimumNumberElementsEachCluster;

		} options;

		/** Add a new frame to the current graph: call this method each time a new observation
		  *   is added to the map/graph, and whenever you want to update the partitions, call "updatePartitions"
		  * \param frame The sensed data
		  * \param robotPose An estimation of the robot global 2D pose.
		  * \return The index of the new pose in the internal list, which will be used to refer to the pose in the future.
		  * \sa updatePartitions
		  */
		unsigned int addMapFrame( const CSensoryFramePtr &frame, const CPosePDFPtr &robotPose2D );

		/** Add a new frame to the current graph: call this method each time a new observation
		  *   is added to the map/graph, and whenever you want to update the partitions, call "updatePartitions"
		  * \param frame The sensed data
		  * \param robotPose An estimation of the robot global 2D pose.
		  * \return The index of the new pose in the internal list, which will be used to refer to the pose in the future.
		  * \sa updatePartitions
		  */
		unsigned int addMapFrame( const CSensoryFramePtr &frame, const CPose3DPDFPtr &robotPose3D );

		/** Add a new frame to the current graph: call this method each time a new observation
		  *   is added to the map/graph, and whenever you want to update the partitions, call "updatePartitions"
		  * \param frame The sensed data
		  * \param robotPose An estimation of the robot global 2D pose.
		  * \return The index of the new pose in the internal list, which will be used to refer to the pose in the future.
		  * \sa updatePartitions
		  */
		unsigned int addMapFrame( const CSensoryFrame &frame, const CPose3DPDF &robotPose3D );

		/** This method executed only the neccesary part of the partition to take
		  *   into account the lastest added frames.
		  * \sa addMapFrame
		  */
		void updatePartitions( std::vector<vector_uint> &partitions );

		/** It returns the nodes count currently in the internal map/graph.
		  */
		unsigned int getNodesCount();

		/** Remove the stated nodes (0-based indexes) from the internal lists.
		  *  If changeCoordsRef is true, coordinates are changed to leave the first node at (0,0,0).
		  */
		void  removeSetOfNodes(vector_uint	indexesToRemove, bool changeCoordsRef = true);

		/** Returns a copy of the internal adjacency matrix.  */
		template <class MATRIX>
		void  getAdjacencyMatrix( MATRIX &outMatrix ) const { outMatrix = m_A; }

		/** Returns a const ref to the internal adjacency matrix.  */
		const CMatrixDouble & getAdjacencyMatrix( ) const { return m_A; }

		/** Read-only access to the sequence of Sensory Frames
		  */
		const CSimpleMap* getSequenceOfFrames( ) const
		{
			return &m_individualFrames;
		}

		/** Access to the sequence of Sensory Frames
		  */
		CSimpleMap* getSequenceOfFrames( )
		{
			return &m_individualFrames;
		}

		/** Mark all nodes for reconsideration in the next call to "updatePartitions", instead of considering just those affected by aditions of new arcs.
		  */
		void markAllNodesForReconsideration();

		/** Change the coordinate origin of all stored poses, for consistency with future new poses to enter in the system. */
		void changeCoordinatesOrigin( const CPose3D  &newOrigin );

		/** Change the coordinate origin of all stored poses, for consistency with future new poses to enter in the system; the new origin is given by the index of the pose to become the new origin. */
		void changeCoordinatesOriginPoseIndex( const unsigned &newOriginPose );

		/** Returns a 3D representation of the current state: poses & links between them.
		  *  The previous contents of "objs" will be discarded
		  */
		void getAs3DScene(
			mrpt::opengl::CSetOfObjectsPtr &objs,
			const std::map< uint32_t, int64_t >  *renameIndexes = NULL
			) const;

	private:
		/** El conjunto de los scans se guardan en un array:
		  */
		CSimpleMap					m_individualFrames;
		std::deque<mrpt::slam::CMultiMetricMap>	m_individualMaps;


		/** Adjacency matrix
		  */
		CMatrixD	m_A;

		/** The last partition
		  */
		std::vector<vector_uint>			m_last_partition;

		/** This will be true after adding new observations, and before an "updatePartitions" is invoked.
		  */
		bool						m_last_last_partition_are_new_ones;

		/** La lista de nodos que hay que tener en cuenta en la proxima actualizacion:
		  */
		std::vector<uint8_t>	m_modified_nodes;

	};	// End of class def.


	} // End of namespace
} // End of namespace

#endif
