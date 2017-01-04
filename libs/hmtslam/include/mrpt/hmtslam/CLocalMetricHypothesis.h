/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CLocalMetricHypothesis_H
#define CLocalMetricHypothesis_H

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/bayes/CParticleFilterCapable.h>

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/hmtslam/CHMHMapNode.h>

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <list>

namespace mrpt
{
	namespace poses { class CPose3DPDFParticles; }

	namespace hmtslam
	{
		typedef mrpt::aligned_containers<TPoseID,mrpt::poses::CPose3D>::map_t TMapPoseID2Pose3D;

		class HMTSLAM_IMPEXP CHMTSLAM;
		class HMTSLAM_IMPEXP CLSLAM_RBPF_2DLASER;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLSLAMParticleData, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLocalMetricHypothesis, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** Auxiliary class used in mrpt::slam::CLocalMetricHypothesis for HMT-SLAM; this class keeps the data relative to each local metric particle ("a robot metric path hypothesis" and its associated metric map).
		  * \ingroup mrpt_hmtslam_grp
		  */
		class HMTSLAM_IMPEXP CLSLAMParticleData : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CLSLAMParticleData )

		public:
			CLSLAMParticleData( const mrpt::maps::TSetOfMetricMapInitializers *mapsInitializers = NULL ) :
			  metricMaps( mapsInitializers ),
			  robotPoses()
			{
			}

			mrpt::maps::CMultiMetricMap            metricMaps;
			TMapPoseID2Pose3D  robotPoses;
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLSLAMParticleData, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


		/** This class is used in HMT-SLAM to represent each of the Local Metric Hypotheses (LMHs).
		 *   It has a set of particles representing the robot path in nearby poses.
		 * \sa CHMTSLAM, CLSLAM_RBPF_2DLASER
		 */
		class HMTSLAM_IMPEXP CLocalMetricHypothesis :
			public mrpt::bayes::CParticleFilterData<CLSLAMParticleData>,
			public mrpt::bayes::CParticleFilterDataImpl<CLocalMetricHypothesis,mrpt::bayes::CParticleFilterData<CLSLAMParticleData>::CParticleList>,
			public mrpt::utils::CSerializable
		{
			friend class HMTSLAM_IMPEXP CLSLAM_RBPF_2DLASER;

			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CLocalMetricHypothesis )

		public:
			/** Constructor (Default param only used from STL classes)
			  */
			CLocalMetricHypothesis( CHMTSLAM  * parent = NULL );

			/** Destructor
			  */
			~CLocalMetricHypothesis();

			synch::CCriticalSection				m_lock;					//!< Critical section for threads signaling they are working with the LMH.
			THypothesisID						m_ID;					//!< The unique ID of the hypothesis (Used for accessing mrpt::slam::CHierarchicalMHMap).
			mrpt::utils::safe_ptr<CHMTSLAM>					m_parent;				//!< For quick access to our parent object.
			TPoseID								m_currentRobotPose;		//!< The current robot pose (its global unique ID) for this hypothesis.
			//TNodeIDList							m_neighbors;			//!< The list of all areas sourronding the current one (this includes the current area itself).
			TNodeIDSet							m_neighbors;			//!< The list of all areas sourronding the current one (this includes the current area itself).
			std::map<TPoseID,CHMHMapNode::TNodeID> m_nodeIDmemberships; //!< The hybrid map node membership for each robot pose.
			std::map<TPoseID,mrpt::obs::CSensoryFrame> 	m_SFs; 					//!< The SF gathered at each robot pose.
			TPoseIDList							m_posesPendingAddPartitioner; //!< The list of poseIDs waiting to be added to the graph partitioner, what happens in the LSLAM thread main loop.
			TNodeIDList							m_areasPendingTBI;		//!< The list of area IDs waiting to be processed by the TBI (topological bayesian inference) engines to search for potential loop-closures. Set in CHMTSLAM::LSLAM_process_message_from_AA, read in

			double								m_log_w;				//!< Log-weight of this hypothesis.
			std::vector<std::map<TPoseID,double> >	m_log_w_metric_history;		//!< The historic log-weights of the metric observations inserted in this LMH, for each particle.
			//std::map<TPoseID,double>	m_log_w_topol_history;		//!< The historic log-weights of the topological observations inserted in this LMH.

			mrpt::obs::CActionRobotMovement2D				m_accumRobotMovement;	//!< Used in CLSLAM_RBPF_2DLASER
			bool								m_accumRobotMovementIsValid;	//!< Used in CLSLAM_RBPF_2DLASER

			/** Used by AA thread */
			struct TRobotPosesPartitioning
			{
				synch::CCriticalSection		lock;  //!< CS to access the entire struct.
				mrpt::slam::CIncrementalMapPartitioner			partitioner;
				std::map<uint32_t,TPoseID> 		idx2pose;   //!< For the poses in "partitioner".

				unsigned int pose2idx(const TPoseID &id) const;  //!< Uses idx2pose to perform inverse searches.

			} m_robotPosesGraph;

			/** Returns a 3D representation of the the current robot pose, all the poses in the auxiliary graph, and each of the areas they belong to.
			  *  The metric maps are *not* included here for convenience, call m_metricMaps.getAs3DScene().
			  *  The previous contents of "objs" will be discarded
			  */
			void getAs3DScene( mrpt::opengl::CSetOfObjectsPtr &objs ) const;

			/** Returns the mean of each robot pose in this LMH, as computed from the set of particles.
			  * \sa getPathParticles, getRelativePose
			  */
			void getMeans( TMapPoseID2Pose3D &outList ) const;

			/** Returns the mean and covariance of each robot pose in this LMH, as computed from the set of particles.
			  * \sa getMeans, getPoseParticles
			  */
			void getPathParticles( std::map< TPoseID, mrpt::poses::CPose3DPDFParticles > &outList ) const;

			/** Returns the mean and covariance of each robot pose in this LMH, as computed from the set of particles.
			  * \sa getMeans, getPathParticles
			  */
			void getPoseParticles( const TPoseID &poseID, mrpt::poses::CPose3DPDFParticles &outPDF ) const;

			/** Returns the pose PDF of some pose relative to some other pose ID (both must be part of the the LMH).
			  * \sa getMeans, getPoseParticles
			  */
			void getRelativePose(
				const  TPoseID &reference,
				const  TPoseID &pose,
				mrpt::poses::CPose3DPDFParticles &outPDF ) const;

			/** Describes the LMH in text.
			  */
			void  dumpAsText(utils::CStringList &st) const;

			/** Change all coordinates to set a given robot pose as the new coordinate origin, and rebuild metric maps and change coords in the partitioning subsystem as well.
			  */
			void changeCoordinateOrigin( const TPoseID &newOrigin );

			/** Rebuild the metric maps of all particles from the observations and their estimated poses. */
			void rebuildMetricMaps();

			/** Rebuild the auxiliary metric maps in "m_robotPosesGraph" from the observations "m_SFs" and their estimated poses. */
			//void rebuildSSOMatrix();

			/** Sets the number of particles to the initial number according to the PF options, and initialize them with no robot poses & empty metric maps.
			  */
			void clearRobotPoses();

			/** Returns the i'th particle hypothesis for the current robot pose.  */
			const mrpt::poses::CPose3D * getCurrentPose(const size_t &particleIdx) const;

			/** Returns the i'th particle hypothesis for the current robot pose.  */
			mrpt::poses::CPose3D * getCurrentPose(const size_t &particleIdx);

			/** Removes a given area from the LMH:
			  *	- The corresponding node in the HMT map is updated with the robot poses & SFs in the LMH.
			  *	- Robot poses belonging to that area are removed from:
			  *		- the particles.
			  *		- the graph partitioner.
			  *		- the list of SFs.
			  *		- the list m_nodeIDmemberships.
			  *	- m_neighbors is updated.
			  * - The weights of all particles are changed to remove the effects of the removed metric observations.
			  *	- After calling this the metric maps should be updated.
			  * - This method internally calls updateAreaFromLMH
			  */
			void removeAreaFromLMH( const CHMHMapNode::TNodeID areaID );

			/** The corresponding node in the HMT map is updated with the robot poses & SFs in the LMH: the poses are referenced to the area's reference poseID, such as that reference is at the origin.
			  *  If eraseSFsFromLMH=true, the sensoryframes are moved rather than copied to the area, and removed from the LMH.
			  * \note The critical section m_map_cs is locked internally, unlock it before calling this.
			  */
			void updateAreaFromLMH(
				const CHMHMapNode::TNodeID areaID,
				bool  eraseSFsFromLMH = false );


		protected:

			/** @name Virtual methods for Particle Filter implementation (just a wrapper interface, actually implemented in CHMTSLAM::m_LSLAM_method)
				@{
			 */

			/** The PF algorithm implementation.
			  */
			void  prediction_and_update_pfAuxiliaryPFOptimal(
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;

			/** The PF algorithm implementation.  */
			void  prediction_and_update_pfOptimalProposal(
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;
			/** @}
			 */


			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			  */
			mutable mrpt::math::CVectorDouble				m_pfAuxiliaryPFOptimal_estimatedProb;

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			  */
			mutable std::vector<double>				m_maxLikelihood;

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm. */
			mutable mrpt::poses::StdVector_CPose2D m_movementDraws;

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm. */
			mutable unsigned int				m_movementDrawsIdx;

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm. */
			mutable mrpt::poses::StdVector_CPose2D		m_movementDrawMaximumLikelihood;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLocalMetricHypothesis, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

	} // End of namespace
} // End of namespace

#endif
