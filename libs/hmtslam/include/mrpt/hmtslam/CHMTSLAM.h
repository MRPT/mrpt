/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHMTSLAM_H
#define CHMTSLAM_H

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CMessageQueue.h>
#include <mrpt/system/threads.h>

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/hmtslam/CLocalMetricHypothesis.h>
#include <mrpt/hmtslam/CHierarchicalMHMap.h>
#include <mrpt/hmtslam/CTopLCDetector_GridMatching.h>
#include <mrpt/hmtslam/CTopLCDetector_FabMap.h>
#include <mrpt/hmtslam/link_pragmas.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/slam/TKLDParams.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <queue>

namespace mrpt
{
	/** Classes related to the implementation of Hybrid Metric Topological (HMT) SLAM. \ingroup mrpt_hmtslam_grp */
	namespace hmtslam
	{
		class CHMTSLAM;
		class CLSLAMAlgorithmBase;
		class CLSLAM_RBPF_2DLASER;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHMTSLAM, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** An implementation of Hybrid Metric Topological SLAM (HMT-SLAM).
		 *
		 *   The main entry points for a user are pushAction() and pushObservations(). Several parameters can be modified
		 *   through m_options.
		 *
		 *  The mathematical models of this approach have been reported in:
		 *		- Blanco J.L., Fernandez-Madrigal J.A., and Gonzalez J., "Towards a Unified Bayesian Approach to Hybrid Metric-Topological SLAM", in  IEEE Transactions on Robotics (TRO), Vol. 24, No. 2, April 2008.
		 *		- ...
		 *
		 *  More information in the wiki page: http://www.mrpt.org/HMT-SLAM . A complete working application can be found in "MRPT/apps/hmt-slam".
		 *
		 *  The complete state of the SLAM framework is serializable, so it can be saved and restore to/from a binary dump. This class implements mrpt::utils::CSerializable, so
		 *    it can be saved with "stream << slam_object;" and restored with "stream >> slam_object;". Alternatively, the methods CHMTSLAM::saveState and CHMTSLAM::loadState
		 *    can be invoked, which in turn call internally to the CSerializable interface.
		 *
		 * \sa CHierarchicalMHMap
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHMTSLAM : public mrpt::utils::COutputLogger, public mrpt::utils::CSerializable
		{
			friend class CLocalMetricHypothesis;
			friend class CLSLAM_RBPF_2DLASER;
			friend class CTopLCDetector_GridMatching;
			friend class CTopLCDetector_FabMap;

			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CHMTSLAM )

		protected:


			/** @name Inter-thread communication queues:
				@{ */
			/** Message definition:
				- From: LSLAM
				- To: AA
				- Meaning: Reconsider the graph partition for the given LMH. Compute SSO for the new node(s) "newPoseIDs".
				*/
			/*struct TMessageAA
			{
				CLocalMetricHypothesis	*LMH;
				TPoseIDList				newPoseIDs;
			};*/

			/** Message definition:
				- From: AA
				- To: LSLAM
				- Meaning: The partitioning of robot poses.
				*/
			struct TMessageLSLAMfromAA
			{
				THypothesisID				hypothesisID; //!< The hypothesis ID (LMH) for these results.
				std::vector< TPoseIDList >	partitions;

				void dumpToConsole( ) const;  //!< for debugging only
			};
			typedef stlplus::smart_ptr<TMessageLSLAMfromAA> TMessageLSLAMfromAAPtr;

			/** Message definition:
				- From: LSLAM
				- To: TBI
				- Meaning: One or more areas are to be considered by the TBI engines.
				*/
			struct TMessageLSLAMtoTBI
			{
				CLocalMetricHypothesis	*LMH;		//!< The LMH
				TNodeIDList				 areaIDs;	//!< The areas to consider.
			};
			typedef stlplus::smart_ptr<TMessageLSLAMtoTBI> TMessageLSLAMtoTBIPtr;

			/** Message definition:
				- From: TBI
				- To: LSLAM
				- Meaning:
				*/
			struct TMessageLSLAMfromTBI
			{
				THypothesisID				hypothesisID; //!< The hypothesis ID (LMH) for these results.

				CHMHMapNode::TNodeID		cur_area; //!< The area for who the loop-closure has been computed.

				struct TBI_info
				{
					TBI_info() : log_lik(0),delta_new_cur(0)
					{}

					double		log_lik;  //!< Log likelihood for this loop-closure.

					/** Depending on the loop-closure engine, an guess of the relative pose of "area_new" relative to "cur_area" is given here.
					  *  If the SOG contains 0 modes, then the engine does not provide this information.
					  */
					mrpt::poses::CPose3DPDFSOG	delta_new_cur;
				};

				/** The meat is here: only feasible loop closures from "cur_area" are included here, with associated data.
				  */
				std::map< CHMHMapNode::TNodeID, TBI_info >	loopClosureData;

				//MRPT_MAKE_ALIGNED_OPERATOR_NEW
			};
			typedef stlplus::smart_ptr<TMessageLSLAMfromTBI>	TMessageLSLAMfromTBIPtr;


			utils::CMessageQueue	m_LSLAM_queue;  //!< LSLAM thread input queue, messages of type CHMTSLAM::TMessageLSLAMfromAA

			/** @} */

			/** The Area Abstraction (AA) method, invoked from LSLAM.
			  * \param LMH (IN) The LMH which to this query applies.
			  * \param newPoseIDs (IN) The new poseIDs to be added to the graph partitioner.
			  * \return A structure with all return data. Memory to be freed by user.
			  * \note The critical section for LMH must be locked BEFORE calling this method (it does NOT lock any critical section).
			  */
			static TMessageLSLAMfromAAPtr areaAbstraction(
				CLocalMetricHypothesis	*LMH,
				const TPoseIDList		&newPoseIDs );


			/** The entry point for Topological Bayesian Inference (TBI) engines, invoked from LSLAM.
			  * \param LMH (IN) The LMH which to this query applies.
			  * \param areaID (IN) The area ID to consider for potential loop-closures.
			  * \note The critical section for LMH must be locked BEFORE calling this method (it does NOT lock any critical section).
			  */
			static TMessageLSLAMfromTBIPtr TBI_main_method(
				CLocalMetricHypothesis	*LMH,
				const CHMHMapNode::TNodeID &areaID
				);

			/** @name Related to the input queue:
				@{ */
		public:
			/** Empty the input queue. */
			void  clearInputQueue();

			/** Returns true if the input queue is empty (Note that the queue must not be empty to the user to enqueue more actions/observaitions)
			  * \sa pushAction,pushObservations, inputQueueSize
			  */
			bool  isInputQueueEmpty();

			/** Returns the number of objects waiting for processing in the input queue.
			  * \sa pushAction,pushObservations, isInputQueueEmpty
			  */
			size_t inputQueueSize();

			/** Here the user can enter an action into the system (will go to the SLAM process).
			  *  This class will delete the passed object when required, so DO NOT DELETE the passed object after calling this.
			  * \sa pushObservations,pushObservation
			  */
			void  pushAction( const mrpt::obs::CActionCollectionPtr &acts );

			/** Here the user can enter observations into the system (will go to the SLAM process).
			  *  This class will delete the passed object when required, so DO NOT DELETE the passed object after calling this.
			  * \sa pushAction,pushObservation
			  */
			void  pushObservations( const mrpt::obs::CSensoryFramePtr &sf );

			/** Here the user can enter an observation into the system (will go to the SLAM process).
			  *  This class will delete the passed object when required, so DO NOT DELETE the passed object after calling this.
			  * \sa pushAction,pushObservation
			  */
			void  pushObservation( const mrpt::obs::CObservationPtr &obs );

			enum TLSlamMethod
			{
				lsmRBPF_2DLASER = 1
			};

		protected:
			/** Used from the LSLAM thread to retrieve the next object from the queue.
			  * \return The object, or NULL if empty.
			  */
			mrpt::utils::CSerializablePtr getNextObjectFromInputQueue();

			/** The queue of pending actions/observations supplied by the user waiting for being processed. */
			std::queue<mrpt::utils::CSerializablePtr>		m_inputQueue;

			/** Critical section for accessing  m_inputQueue */
			synch::CCriticalSection	m_inputQueue_cs;

			/** Critical section for accessing m_map */
			synch::CCriticalSection	m_map_cs;

			synch::CCriticalSection	m_LMHs_cs; //!< Critical section for accessing m_LMHs

			/** @} */


			/** @name Threads stuff
				@{ */

			/** The function for the "Local SLAM" thread. */
			void thread_LSLAM( );

			/** The function for the "TBI" thread. */
			void thread_TBI( );

			/** The function for the "3D viewer" thread. */
			void thread_3D_viewer( );
			/** Threads handles */
			mrpt::system::TThreadHandle m_hThread_LSLAM, m_hThread_TBI, m_hThread_3D_viewer;
			/** @} */


			/** @name HMT-SLAM sub-processes.
				@{ */
			void LSLAM_process_message( const mrpt::utils::CMessage &msg ); //!< Auxiliary method within thread_LSLAM

			/** No critical section locks are assumed at the entrance of this method.
			  */
			void LSLAM_process_message_from_AA( const TMessageLSLAMfromAA &myMsg );

			/** No critical section locks are assumed at the entrance of this method.
			  */
			void LSLAM_process_message_from_TBI( const TMessageLSLAMfromTBI &myMsg );

			/** Topological Loop Closure: Performs all the required operations
				to close a loop between two areas which have been determined
				to be the same.
			 */
			void perform_TLC(
				CLocalMetricHypothesis					&LMH,
				const CHMHMapNode::TNodeID				areaInLMH,
				const CHMHMapNode::TNodeID				areaLoopClosure,
				const mrpt::poses::CPose3DPDFGaussian	&pose1wrt2
				);


			/** @} */

			/** @name The different SLAM algorithms that can be invoked from the LSLAM thread.
				@{ */

			/** An instance of a local SLAM method, to be applied to each LMH - initialized by "initializeEmptyMap" or "loadState".
			  */
			CLSLAMAlgorithmBase		*m_LSLAM_method;

			/** @} */

			/** @name The different Loop-Closure modules that are to be executed in the TBI thread.
				@{ */
		protected:

			typedef CTopLCDetectorBase*		(*TLopLCDetectorFactory)(CHMTSLAM*);

			std::map<std::string,TLopLCDetectorFactory>   m_registeredLCDetectors;

			/** The list of LC modules in operation - initialized by "initializeEmptyMap" or "loadState".  */
			std::deque<CTopLCDetectorBase*>	m_topLCdets;

			/** The critical section for accessing m_topLCdets */
			synch::CCriticalSection	m_topLCdets_cs;
		public:

			/** Must be invoked before calling  initializeEmptyMap, so LC objects can be created. */
			void registerLoopClosureDetector(
				const std::string   	&name,
				CTopLCDetectorBase*		(*ptrCreateObject)(CHMTSLAM*)
				);

			/** The class factory for topological loop closure detectors.
			  *  Possible values are enumerated in TOptions::TLC_detectors
			  *
			  * \exception std::exception On unknown name.
			  */
			CTopLCDetectorBase* loopClosureDetector_factory(const std::string  &name);


			/** @} */
		protected:


			/** Termination flag for signaling all threads to terminate */
			bool					m_terminateThreads;

			/** Threads termination flags:
			  */
			bool					m_terminationFlag_LSLAM,
									m_terminationFlag_TBI,
									m_terminationFlag_3D_viewer;

			/** Generates a new and unique area textual label (currently this generates "0","1",...) */
			static std::string		generateUniqueAreaLabel();

			/** Generates a new and unique pose ID */
			static TPoseID 			generatePoseID();

			/** Generates a new and unique hypothesis ID */
			static THypothesisID 	generateHypothesisID();

			static int64_t			m_nextAreaLabel;
			static TPoseID			m_nextPoseID;
			static THypothesisID	m_nextHypID;


		public:
			/** Default constructor
			  *  \param debug_out_stream If debug output messages should be redirected to any other stream apart from std::cout
			  */
			CHMTSLAM( );

			CHMTSLAM(const CHMTSLAM &) : mrpt::utils::COutputLogger() { THROW_EXCEPTION("This object cannot be copied."); }
			const CHMTSLAM& operator =(const CHMTSLAM &) { THROW_EXCEPTION("This object cannot be copied."); }

			/** Destructor
			  */
			virtual ~CHMTSLAM();

			/** Return true if an exception has been caught in any thread leading to the end of the mapping application: no more actions/observations will be processed from now on.
			  */
			bool abortedDueToErrors();


			/** @name High-level map management
				@{ */

			/** Loads the options from a config file. */
			void loadOptions( const std::string &configFile );
			/** Loads the options from a config source */
			void loadOptions( const mrpt::utils::CConfigFileBase &cfgSource );

			/** Initializes the whole HMT-SLAM framework, reseting to an empty map (It also clears the logs directory) - this must be called AFTER loading the options with CHMTSLAM::loadOptions. */
			void  initializeEmptyMap();

			/** Save the state of the whole HMT-SLAM framework to some binary stream (e.g. a file).
			  * \return true if everything goes OK.
			  * \sa loadState
			  */
			bool saveState( mrpt::utils::CStream &out ) const;

			/** Load the state of the whole HMT-SLAM framework from some binary stream (e.g. a file).
			  * \return true if everything goes OK.
			  * \sa saveState
			  */
			bool loadState( mrpt::utils::CStream &in );
			/** @} */

			/** @name The important data.
				@{ */
			CHierarchicalMHMap									m_map;	//!< The hiearchical, multi-hypothesis graph-based map.
			aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t m_LMHs;	//!< The list of LMHs at each instant.
			/** @} */

			/** Called from LSLAM thread when log files must be created.
			  */
			void generateLogFiles(unsigned int nIteration);


			/** Gets a 3D representation of the current state of the whole mapping framework.
			  */
			void  getAs3DScene( mrpt::opengl::COpenGLScene	&outScene );

		protected:
			/** A variety of options and configuration params (private, use loadOptions).
			  */
			struct TOptions : public utils::CLoadableOptions
			{
				/** Initialization of default params
				  */
				TOptions();

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs


				std::string	LOG_OUTPUT_DIR;		//!< [LOGGING] If it is not an empty string (""), a directory with that name will be created and log files save there.
				int			LOG_FREQUENCY;		//!< [LOGGING] One SLAM iteration out of "LOGGING_logFrequency", a log file will be generated.

				/** [LSLAM] The method to use for local SLAM
				  */
				TLSlamMethod   SLAM_METHOD;

				/** [LSLAM] Minimum distance (and heading) difference between observations inserted in the map.
				*/
				float		SLAM_MIN_DIST_BETWEEN_OBS, SLAM_MIN_HEADING_BETWEEN_OBS;

				/** [LSLAM] Minimum uncertainty (1 sigma, meters) in x and y for odometry increments (Default=0) */
				float		MIN_ODOMETRY_STD_XY;

				/** [LSLAM] Minimum uncertainty (1 sigma, rads) in phi for odometry increments (Default=0) */
				float		MIN_ODOMETRY_STD_PHI;

				/** [VIEW3D] The height of the areas' spheres.
				  */
				float		VIEW3D_AREA_SPHERES_HEIGHT;

				/** [VIEW3D] The radius of the areas' spheres.
				  */
				float		VIEW3D_AREA_SPHERES_RADIUS;

				/** A 3-length vector with the std. deviation of the transition model in (x,y,phi) used only when there is no odometry (if there is odo, its uncertainty values will be used instead); x y: In meters, phi: radians (but in degrees when loading from a configuration ini-file!)
				  */
				mrpt::math::CVectorFloat  stds_Q_no_odo;

				/** [AA] The options for the partitioning algorithm
				*/
				mrpt::slam::CIncrementalMapPartitioner::TOptions	AA_options;

				mrpt::maps::TSetOfMetricMapInitializers		defaultMapsInitializers;  //!< The default set of maps to be created in each particle
				bayes::CParticleFilter::TParticleFilterOptions pf_options;	//!< These params are used from every LMH object.
				mrpt::slam::TKLDParams						KLD_params;

				int  random_seed;	//!< 0 means randomize, use any other value to have repetitive experiments.

				/** A list of topological loop-closure detectors to use: can be one or more from this list:
				  *  'gridmaps': Occupancy Grid matching.
				  *  'fabmap': Mark Cummins' image matching framework.
				  */
				vector_string				TLC_detectors;

				CTopLCDetector_GridMatching::TOptions	TLC_grid_options;	//!< Options passed to this TLC constructor
				CTopLCDetector_FabMap::TOptions	TLC_fabmap_options;	//!< Options passed to this TLC constructor

			} m_options;

		}; // End of class CHMTSLAM.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHMTSLAM, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


		/** Virtual base for local SLAM methods, used in mrpt::slam::CHMTSLAM.
		  */
		class HMTSLAM_IMPEXP CLSLAMAlgorithmBase
		{
			friend class CLocalMetricHypothesis;
		protected:
			mrpt::utils::safe_ptr<CHMTSLAM>	m_parent;

		public:
			/** Constructor
			  */
			CLSLAMAlgorithmBase( CHMTSLAM *parent ) : m_parent(parent) { }

			/** Destructor
			  */
			virtual ~CLSLAMAlgorithmBase() {}

			/** Main entry point from HMT-SLAM: process some actions & observations.
			  *  The passed action/observation will be deleted, so a copy must be made if necessary.
			  *  This method must be in charge of updating the robot pose estimates and also to update the
			  *   map when required.
			  *
			  * \param LMH   The local metric hypothesis which must be updated by this SLAM algorithm.
			  * \param act   The action to process (or NULL).
			  * \param sf    The observations to process (or NULL).
			  */
			virtual void processOneLMH(
				CLocalMetricHypothesis	*LMH,
				const mrpt::obs::CActionCollectionPtr 	&act,
				const mrpt::obs::CSensoryFramePtr		&sf ) = 0;


			/** The PF algorithm implementation.
			  */
			virtual void  prediction_and_update_pfAuxiliaryPFOptimal(
				CLocalMetricHypothesis			* LMH,
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) = 0;

			/** The PF algorithm implementation.  */
			virtual void  prediction_and_update_pfOptimalProposal(
				CLocalMetricHypothesis			*LMH,
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) = 0;

		}; // end of class CLSLAMAlgorithmBase


		/** Implements a 2D local SLAM method based on a RBPF over an occupancy grid map.
		  *  This class is used internally in mrpt::slam::CHMTSLAM
		  */
		class HMTSLAM_IMPEXP CLSLAM_RBPF_2DLASER : public CLSLAMAlgorithmBase
		{
			friend class CLocalMetricHypothesis;

		public:
			/** Constructor
			  */
			CLSLAM_RBPF_2DLASER( CHMTSLAM *parent );

			/** Destructor
			  */
			virtual ~CLSLAM_RBPF_2DLASER();

			/** Main entry point from HMT-SLAM: process some actions & observations.
			  *  The passed action/observation will be deleted, so a copy must be made if necessary.
			  *  This method must be in charge of updating the robot pose estimates and also to update the
			  *   map when required.
			  *
			  * \param LMH   The local metric hypothesis which must be updated by this SLAM algorithm.
			  * \param act   The action to process (or NULL).
			  * \param sf    The observations to process (or NULL).
			  */
			void processOneLMH(
				CLocalMetricHypothesis	*LMH,
				const mrpt::obs::CActionCollectionPtr	&act,
				const mrpt::obs::CSensoryFramePtr		&sf );

			/** The PF algorithm implementation.  */
			void  prediction_and_update_pfAuxiliaryPFOptimal(
				CLocalMetricHypothesis			*LMH,
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

			/** The PF algorithm implementation.  */
			void  prediction_and_update_pfOptimalProposal(
				CLocalMetricHypothesis			*LMH,
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

		protected:
			bool					m_insertNewRobotPose; //!<  For use within PF callback methods

			/** Auxiliary structure
			  */
			struct TPathBin
			{
				TPathBin() : x(),y(),phi()
				{}

				vector_int		x,y,phi;

				/** For debugging purposes!
				  */
				void  dumpToStdOut() const;
			};


			/** Fills out a "TPathBin" variable, given a path hypotesis and (if not set to NULL) a new pose appended at the end, using the KLD params in "options".
				*/
			void  loadTPathBinFromPath(
				TPathBin	&outBin,
				TMapPoseID2Pose3D *path = NULL,
				mrpt::poses::CPose2D  *newPose = NULL );

			/** Checks if a given "TPathBin" element is already into a set of them, and return its index (first one is 0), or -1 if not found.
				*/
			int  findTPathBinIntoSet(
				TPathBin						&desiredBin,
				std::deque<TPathBin>			&theSet
				);

			/** Auxiliary function used in "prediction_and_update_pfAuxiliaryPFOptimal"
				*/
			static double  particlesEvaluator_AuxPFOptimal(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const mrpt::bayes::CParticleFilterCapable	*obj,
				size_t					index,
				const void				*action,
				const void				*observation );

			/** Auxiliary function that evaluates the likelihood of an observation, given a robot pose, and according to the options in "CPosePDFParticles::options".
			  */
			static double  auxiliarComputeObservationLikelihood(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const mrpt::bayes::CParticleFilterCapable		*obj,
				size_t						particleIndexForMap,
				const mrpt::obs::CSensoryFrame			*observation,
				const mrpt::poses::CPose2D				*x );

		}; // end class CLSLAM_RBPF_2DLASER

	} // End of namespace
} // End of namespace

#endif
