/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHierarchicalMapMHPartition_H
#define CHierarchicalMapMHPartition_H

#include <mrpt/hmtslam/CLocalMetricHypothesis.h>
#include <mrpt/hmtslam/CHMHMapArc.h>
#include <mrpt/hmtslam/CHMHMapNode.h>

#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <map>

namespace mrpt
{
	namespace poses  { class CPose3DPDFParticles; }

	namespace hmtslam
	{
		/** Represents a set of nodes and arcs, posibly only a part of the whole hierarchical, multi-hypothesis map.
		 *  A usar will never create an instance of this class, rather it will employ CHierarchicalMHMap.
		 * \sa CHierarchicalMHMap, CHMHMapArc, CHMHMapNode
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHierarchicalMapMHPartition : public mrpt::utils::COutputLogger
		{
		protected:
			/** The internal list of nodes and arcs in the whole hierarchical model.
			  *  The objects must be deleted only in the CHierarchicalMap class, not in partitions only objects.
			  */
			TNodeList  m_nodes;
			TArcList   m_arcs;

		public:

			typedef TNodeList::iterator iterator;
			typedef TNodeList::const_iterator const_iterator;

			/** Returns an iterator to the first node in the graph. */
			const_iterator begin() const { return m_nodes.begin(); }

			/** Returns an iterator to the first node in the graph. */
			iterator begin() { return m_nodes.begin(); }

			/** Returns an iterator to the end of the list of nodes in the graph. */
			const_iterator end() const { return m_nodes.end(); }

			/** Returns an iterator to the end of the list of nodes in the graph. */
			iterator end() { return m_nodes.end(); }


			CHierarchicalMapMHPartition() : m_nodes(), m_arcs()
			{ }

			/** A type that reprensents a sequence of node IDs
			  */
			typedef std::vector<CHMHMapNode::TNodeID> TNodeIDsList;

			/** Returns the number of nodes in the partition:
			  */
			size_t nodeCount() const;

			/** Returns the number of arcs in the partition:
			  */
			size_t arcCount() const;

			/** Returns the first node in the graph, or NULL if it does not exist.
			  * \return A pointer to the object. DO NOT DELETE this object, if you want to modify it in someway, first obtain a copy by invoking "CSerializable::duplicate"
			  */
			CHMHMapNodePtr  getFirstNode();

			/** Returns the node with the given ID, or NULL if it does not exist.
			  * \return A pointer to the object. DO NOT DELETE this object, if you want to modify it in someway, first obtain a copy by invoking "CSerializable::duplicate"
			  */
			CHMHMapNodePtr  getNodeByID(CHMHMapNode::TNodeID	id);

			/** Returns the node with the given ID, or NULL if it does not exist.
			  * \return A pointer to the object. DO NOT DELETE this object, if you want to modify it in someway, first obtain a copy by invoking "CSerializable::duplicate"
			  */
			const CHMHMapNodePtr  getNodeByID(CHMHMapNode::TNodeID	id) const;

			/** Returns the node with the given label (case insensitive) for some given hypothesis ID, or NULL if it does not exist.
			  * \return A pointer to the object. DO NOT DELETE this object, if you want to modify it in someway, first obtain a copy by invoking "CSerializable::duplicate"
			  */
			CHMHMapNodePtr  getNodeByLabel(const std::string &label, const THypothesisID &hypothesisID );

			/** Returns the node with the given label (case insensitive) for some given hypothesis ID, or NULL if it does not exist.
			  * \return A pointer to the object. DO NOT DELETE this object, if you want to modify it in someway, first obtain a copy by invoking "CSerializable::duplicate"
			  */
			const CHMHMapNodePtr  getNodeByLabel(const std::string &label, const THypothesisID &hypothesisID) const;

			/** Returns a partition of this graph only with nodes at a given level in the hierarchy (0=ground level,1=parent level,etc)
			   *	- The partition may be empty if no node fulfills the condition.
			   *	- All arcs STARTING at each node from the partition will be added to the partition as well.
			   *	- Levels in the hierarchy here stands for arcs of type "arcType_Belongs" only.
			   * \sa CHMHMapArc
			   */
			//CHierarchicalMapMHPartition	 getPartitionByHiearchyLevel( unsigned int level );

			 /** Saves a MATLAB script that represents graphically the nodes with <i>type</i>="Area" in this hierarchical-map(partition), using the stated node as global coordinates reference.
			   *  ADDITIONAL NOTES:
			   *	- Coordinates are computed simply as the mean value of the first arc with an annotation "RelativePose", added to the pose of the original node.
			   *	- If the coordinates of any node can not be computed (no arcs,...), an exception will be raised.
			   */
			void  saveAreasDiagramForMATLAB(
				const std::string			&filName,
				const CHMHMapNode::TNodeID	&idReferenceNode,
				const THypothesisID &hypothesisID) const;

			 /** Saves a MATLAB script that represents graphically the nodes with <i>type</i>="Area" in this hierarchical-map(partition), using the stated node as global coordinates reference, and drawing the ellipses of the localization uncertainty for each node.
			   *  ADDITIONAL NOTES:
			   *	- Coordinates are computed simply as the mean value of the first arc with an annotation "RelativePose", added to the pose of the original node.
			   *	- If the coordinates of any node can not be computed (no arcs,...), an exception will be raised.
			   */
			 void  saveAreasDiagramWithEllipsedForMATLAB(
				const std::string				&filName,
				const CHMHMapNode::TNodeID		&idReferenceNode,
				const THypothesisID					&hypothesisID,
				float							uncertaintyExagerationFactor = 1.0f,
				bool							drawArcs = false,
				unsigned int					numberOfIterationsForOptimalGlobalPoses = 4
				 ) const;

			 /** Saves a MATLAB script that represents graphically the reconstructed "global map"
			   *  ADDITIONAL NOTES:
			   *	- Coordinates are computed simply as the mean value of the first arc with an annotation "RelativePose", added to the pose of the original node.
			   *	- If the coordinates of any node can not be computed (no arcs,...), an exception will be raised.
			   */
			 void  saveGlobalMapForMATLAB(
				const std::string			&filName,
				const THypothesisID				&hypothesisID,
				const CHMHMapNode::TNodeID	&idReferenceNode ) const;


			 /** The Dijkstra algorithm for finding the shortest path between a pair of nodes.
			   * \return The sequence of arcs connecting the nodes.It will be empty if no path is found or when the starting and ending node coincide.
			   */
			 void findPathBetweenNodes(
				 const CHMHMapNode::TNodeID		&nodeFrom,
				 const CHMHMapNode::TNodeID		&nodeTo,
				 const THypothesisID			&hypothesisID,
				 TArcList						&out_path,
				 bool							direction=false) const;


			 /** Draw a number of samples according to the PDF of the coordinates transformation between a pair of "Area"'s nodes.
			   * \exception std::exception If there is not enought information in arcs to compute the PDF
			   * \sa computeGloballyConsistentNodeCoordinates
			   */
			 void  computeCoordinatesTransformationBetweenNodes(
				const CHMHMapNode::TNodeID	&nodeFrom,
				const CHMHMapNode::TNodeID	&nodeTo,
				mrpt::poses::CPose3DPDFParticles			&posePDF,
				const THypothesisID				&hypothesisID,
				unsigned int				particlesCount = 100,
				float						additionalNoiseXYratio = 0.02,
				float						additionalNoisePhiRad = mrpt::utils::DEG2RAD(0.1)
				) const;

			/** Computes the probability [0,1] of two areas' gridmaps to "match" (loop closure), according to the grid maps and pose uncertainty from information in arcs (uses a Monte Carlo aproximation)
			   *  If there is not enough information or a robust estimation cannot be found, there will not be particles in "estimatedRelativePose".
			   */
			float computeMatchProbabilityBetweenNodes(
				const CHMHMapNode::TNodeID	&nodeFrom,
				const CHMHMapNode::TNodeID	&nodeTo,
				float						&maxMatchProb,
				mrpt::poses::CPose3DPDFSOG				&estimatedRelativePose,
				const THypothesisID				&hypothesisID,
				unsigned int				monteCarloSamplesPose = 300
				);

			 /** Returns all the arcs between a pair of nodes:
			   */
			 void findArcsBetweenNodes(
				const CHMHMapNode::TNodeID	&node1,
				const CHMHMapNode::TNodeID	&node2,
				const THypothesisID				&hypothesisID,
				TArcList					&out_listArcs ) const;

			 /** Returns the arcs between a pair of nodes of a given type.
			   */
			void findArcsOfTypeBetweenNodes(
				const CHMHMapNode::TNodeID	&node1id,
				const CHMHMapNode::TNodeID	&node2id,
				const THypothesisID			&hypothesisID,
				const std::string			&arcType,
				TArcList					&ret) const;

			 /** Returns the first arc between a pair of nodes of a given type, and if it is in the opposite direction.
			   * \return The arc, or NULL if not found.
			   */
			CHMHMapArcPtr findArcOfTypeBetweenNodes(
				const CHMHMapNode::TNodeID	&node1id,
				const CHMHMapNode::TNodeID	&node2id,
				const THypothesisID			&hypothesisID,
				const std::string			&arcType,
				bool						&isInverted ) const;


			 /** Returns whether two nodes are "neightbour", i.e. have a direct arc between them  */
			bool areNodesNeightbour(
				const CHMHMapNode::TNodeID	&node1,
				const CHMHMapNode::TNodeID	&node2,
				const THypothesisID				&hypothesisID,
				const char					*requiredAnnotation=NULL ) const;

			 /** This methods implements a Lu&Milios-like globally optimal estimation for the global coordinates of all the nodes in the graph according to all available arcs with relative pose information.
			   * Global coordinates will be computed relative to the node "idReferenceNode".
			   * \exception std::exception If there is any node without a pose arc, invalid (non invertible) matrixes, etc...
			   * \sa computeCoordinatesTransformationBetweenNodes
			   */
			void  computeGloballyConsistentNodeCoordinates(
				std::map<CHMHMapNode::TNodeID,mrpt::poses::CPose3DPDFGaussian, std::less<CHMHMapNode::TNodeID>, Eigen::aligned_allocator<std::pair<const CHMHMapNode::TNodeID,mrpt::poses::CPose3DPDFGaussian> > >		&nodePoses,
				const CHMHMapNode::TNodeID							&idReferenceNode,
				const THypothesisID										&hypothesisID,
				const unsigned int									&numberOfIterations = 2) const;

			 /** Returns a 3D scene reconstruction of the hierarchical map.
			   *  See "computeGloballyConsistentNodeCoordinates" for the meaning of "numberOfIterationsForOptimalGlobalPoses"
			   */
			 void  getAs3DScene(
				mrpt::opengl::COpenGLScene				&outScene,
				const CHMHMapNode::TNodeID	&idReferenceNode,
				const THypothesisID				&hypothesisID,
				const unsigned int			&numberOfIterationsForOptimalGlobalPoses = 5,
				const bool 					&showRobotPoseIDs = true
				) const;

			 /** Return a textual description of the whole graph */
			 void  dumpAsText(utils::CStringList &s) const;



			/** Computes the probability [0,1] of two areas' gridmaps to overlap, via a Monte Carlo aproximation.
			  * \exception std::exception If there is not enought information in arcs, etc...
			  * \param margin_to_substract In meters, the area of each gridmap is "eroded" this amount to compensate the area in excess usually found in gridmaps.
			  */
			 double computeOverlapProbabilityBetweenNodes(
				const CHMHMapNode::TNodeID	&nodeFrom,
				const CHMHMapNode::TNodeID	&nodeTo,
				const THypothesisID			&hypothesisID,
				const size_t &monteCarloSamples = 100,
				const float margin_to_substract = 6
				) const ;

		protected:

		}; // End of class def.
	}
}

#endif
