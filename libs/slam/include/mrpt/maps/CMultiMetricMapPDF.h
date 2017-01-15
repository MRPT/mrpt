/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMultiMetricMapPDF_H
#define CMultiMetricMapPDF_H

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

#include <mrpt/poses/CPoseRandomSampler.h>

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/slam/CICP.h>

#include <mrpt/slam/PF_implementations_data.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam { class CMetricMapBuilderRBPF; }
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRBPFParticleData, mrpt::utils::CSerializable, SLAM_IMPEXP )

	/** Auxiliary class used in mrpt::maps::CMultiMetricMapPDF
	 * \ingroup mrpt_slam_grp
	  */
	class SLAM_IMPEXP CRBPFParticleData : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CRBPFParticleData )
	public:
		CRBPFParticleData( const TSetOfMetricMapInitializers *mapsInitializers = NULL ) :
		  mapTillNow( mapsInitializers ),
		  robotPath()
		{
		}

		CMultiMetricMap                 mapTillNow;
		std::deque<mrpt::math::TPose3D> robotPath;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRBPFParticleData, mrpt::utils::CSerializable, SLAM_IMPEXP )


	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CMultiMetricMapPDF, mrpt::utils::CSerializable, SLAM_IMPEXP )

	/** Declares a class that represents a Rao-Blackwellized set of particles for solving the SLAM problem (This class is the base of RBPF-SLAM applications).
	 *   This class is used internally by the map building algorithm in "mrpt::slam::CMetricMapBuilderRBPF"
	 *
	 * \sa mrpt::slam::CMetricMapBuilderRBPF
	 * \ingroup metric_slam_grp
	 */
	class SLAM_IMPEXP CMultiMetricMapPDF :
		public mrpt::utils::CSerializable,
		public mrpt::bayes::CParticleFilterData<CRBPFParticleData>,
		public mrpt::bayes::CParticleFilterDataImpl<CMultiMetricMapPDF,mrpt::bayes::CParticleFilterData<CRBPFParticleData>::CParticleList>,
		public mrpt::slam::PF_implementation<CRBPFParticleData,CMultiMetricMapPDF>
	{
		friend class mrpt::slam::CMetricMapBuilderRBPF;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CMultiMetricMapPDF )

	protected:
		// PF algorithm implementations:
		void  prediction_and_update_pfStandardProposal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;
		void  prediction_and_update_pfOptimalProposal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;
		void  prediction_and_update_pfAuxiliaryPFOptimal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;
		void  prediction_and_update_pfAuxiliaryPFStandard(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;


	private:
		/** Internal buffer for the averaged map. */
		mrpt::maps::CMultiMetricMap			averageMap;
		bool					averageMapIsUpdated;

		mrpt::maps::CSimpleMap  SFs; //!< The SFs and their corresponding pose estimations
		std::vector<uint32_t>   SF2robotPath; //!< A mapping between indexes in the SFs to indexes in the robot paths from particles.

	public:

		/** The struct for passing extra simulation parameters to the prediction/update stage
		 *    when running a particle filter.
		 * \sa prediction_and_update
		 */
		struct SLAM_IMPEXP TPredictionParams : public utils::CLoadableOptions
		{
			/** Default settings method */
			TPredictionParams();

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** [pf optimal proposal only]  Only for PF algorithm=2 (Exact "pfOptimalProposal")
			 *   Select the map on which to calculate the optimal
			 *    proposal distribution. Values:
			 *   0: Gridmap   -> Uses Scan matching-based approximation (based on Stachniss' work)
			 *   1: Landmarks -> Uses matching to approximate optimal
			 *   2: Beacons   -> Used for exact optimal proposal in RO-SLAM
			 *   3: Pointsmap -> Uses Scan matching-based approximation with a map of points (based on Stachniss' work)
			 *  Default = 0
			 */
			int 		pfOptimalProposal_mapSelection;


			/** [prediction stage][pf optimal proposal only] If useICPGlobalAlign_withGrid=true, this is the minimum quality ratio [0,1] of the alignment such as it will be accepted. Otherwise, raw odometry is used for those bad cases (default=0.7).
			  */
			float		ICPGlobalAlign_MinQuality;

			/** [update stage] If the likelihood is computed through the occupancy grid map, then this structure is passed to the map when updating the particles weights in the update stage.
			  */
			COccupancyGridMap2D::TLikelihoodOptions		update_gridMapLikelihoodOptions;

			mrpt::slam::TKLDParams	KLD_params;

			mrpt::slam::CICP::TConfigParams		icp_params; //!< ICP parameters, used only when "PF_algorithm=2" in the particle filter.

		} options;

		/** Constructor
		  */
		CMultiMetricMapPDF(
			const bayes::CParticleFilter::TParticleFilterOptions    &opts = bayes::CParticleFilter::TParticleFilterOptions(),
			const mrpt::maps::TSetOfMetricMapInitializers		    *mapsInitializers = NULL,
			const TPredictionParams						    *predictionOptions = NULL );

		/** Clear all elements of the maps, and restore all paths to a single starting pose */
		void  clear( const mrpt::poses::CPose2D &initialPose );
		void  clear( const mrpt::poses::CPose3D &initialPose ); //!< \overload

		 /** Returns the estimate of the robot pose as a particles PDF for the instant of time "timeStep", from 0 to N-1.
		  * \sa getEstimatedPosePDF
		  */
		void  getEstimatedPosePDFAtTime(
			size_t				timeStep,
			mrpt::poses::CPose3DPDFParticles &out_estimation ) const;

		 /** Returns the current estimate of the robot pose, as a particles PDF.
		  * \sa getEstimatedPosePDFAtTime
		  */
		void  getEstimatedPosePDF( mrpt::poses::CPose3DPDFParticles	&out_estimation ) const;

		/** Returns the weighted averaged map based on the current best estimation. If you need a persistent copy of this object, please use "CSerializable::duplicate" and use the copy.
		  * \sa Almost 100% sure you would prefer the best current map, given by getCurrentMostLikelyMetricMap()
		  */
		const CMultiMetricMap * getAveragedMetricMapEstimation();

		/** Returns a pointer to the current most likely map (associated to the most likely particle) */
		const CMultiMetricMap  * getCurrentMostLikelyMetricMap() const;

		/** Get the number of CSensoryFrame inserted into the internal member SFs */
		size_t  getNumberOfObservationsInSimplemap() const { return SFs.size(); }

		/** Insert an observation to the map, at each particle's pose and to each particle's metric map.
		  * \param sf The SF to be inserted
		  * \return true if any may was updated, false otherwise
		  */
		bool  insertObservation(mrpt::obs::CSensoryFrame	&sf);

		/** Return the path (in absolute coordinate poses) for the i'th particle.
		  * \exception On index out of bounds
		  */
		void  getPath(size_t i, std::deque<math::TPose3D> &out_path) const;

		/** Returns the current entropy of paths, computed as the average entropy of poses along the path, where entropy of each pose estimation is computed as the entropy of the gaussian approximation covariance.
		  */
		double  getCurrentEntropyOfPaths();

		/** Returns the joint entropy estimation over paths and maps, acording to "Information Gain-based Exploration Using" by C. Stachniss, G. Grissetti and W.Burgard.
		  */
		double  getCurrentJointEntropy();

		/** Update the poses estimation of the member "SFs" according to the current path belief.
		  */
		void  updateSensoryFrameSequence();

		/** A logging utility: saves the current path estimation for each particle in a text file (a row per particle, each 3-column-entry is a set [x,y,phi], respectively).
		  */
		void  saveCurrentPathEstimationToTextFile( const std::string  &fil );

	 private:
		/** Rebuild the "expected" grid map. Used internally, do not call  */
		void  rebuildAverageMap();

		float newInfoIndex; //!< An index [0,1] measuring how much information an observation aports to the map (Typ. threshold=0.07)

	public:
			/** \name Virtual methods that the PF_implementations assume exist.
			    @{ */

			/** Return a pointer to the last robot pose in the i'th particle (or NULL if it's a path and it's empty). */
			const mrpt::math::TPose3D * getLastPose(const size_t i) const MRPT_OVERRIDE;

			void PF_SLAM_implementation_custom_update_particle_with_new_pose(
				CParticleDataContent *particleData,
				const mrpt::math::TPose3D &newPose) const MRPT_OVERRIDE;

			// The base implementation is fine for this class.
			// void PF_SLAM_implementation_replaceByNewParticleSet( ...

			bool PF_SLAM_implementation_doWeHaveValidObservations(
				const CParticleList	&particles,
				const mrpt::obs::CSensoryFrame *sf) const MRPT_OVERRIDE;

			bool PF_SLAM_implementation_skipRobotMovement() const MRPT_OVERRIDE;

			/** Evaluate the observation likelihood for one particle at a given location */
			double PF_SLAM_computeObservationLikelihoodForParticle(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions	&PF_options,
				const size_t			particleIndexForMap,
				const mrpt::obs::CSensoryFrame		&observation,
				const mrpt::poses::CPose3D			&x ) const MRPT_OVERRIDE;
			/** @} */


	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CMultiMetricMapPDF, mrpt::utils::CSerializable, SLAM_IMPEXP )

	} // End of namespace
} // End of namespace

#endif
