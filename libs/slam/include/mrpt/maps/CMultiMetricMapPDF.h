/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

#include <mrpt/poses/CPoseRandomSampler.h>

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/slam/CICP.h>

#include <mrpt/slam/PF_implementations_data.h>

namespace mrpt
{
namespace slam
{
class CMetricMapBuilderRBPF;
}
namespace maps
{
/** Auxiliary class used in mrpt::maps::CMultiMetricMapPDF
 * \ingroup mrpt_slam_grp
 */
class CRBPFParticleData : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CRBPFParticleData)
   public:
	CRBPFParticleData(
		const TSetOfMetricMapInitializers* mapsInitializers = nullptr)
		: mapTillNow(mapsInitializers), robotPath()
	{
	}

	CMultiMetricMap mapTillNow;
	std::deque<mrpt::math::TPose3D> robotPath;
};

/** Declares a class that represents a Rao-Blackwellized set of particles for
 * solving the SLAM problem (This class is the base of RBPF-SLAM applications).
 *   This class is used internally by the map building algorithm in
 * "mrpt::slam::CMetricMapBuilderRBPF"
 *
 * \sa mrpt::slam::CMetricMapBuilderRBPF
 * \ingroup metric_slam_grp
 */
class CMultiMetricMapPDF
	: public mrpt::serialization::CSerializable,
	  public mrpt::bayes::CParticleFilterData<CRBPFParticleData>,
	  public mrpt::bayes::CParticleFilterDataImpl<
		  CMultiMetricMapPDF,
		  mrpt::bayes::CParticleFilterData<CRBPFParticleData>::CParticleList>,
	  public mrpt::slam::PF_implementation<
		  CRBPFParticleData, CMultiMetricMapPDF,
		  mrpt::bayes::particle_storage_mode::POINTER>
{
	friend class mrpt::slam::CMetricMapBuilderRBPF;

	DEFINE_SERIALIZABLE(CMultiMetricMapPDF)

   protected:
	// PF algorithm implementations:
	void prediction_and_update_pfStandardProposal(
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
		override;
	void prediction_and_update_pfOptimalProposal(
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
		override;
	void prediction_and_update_pfAuxiliaryPFOptimal(
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
		override;
	void prediction_and_update_pfAuxiliaryPFStandard(
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
		override;

   private:
	/** Internal buffer for the averaged map. */
	mrpt::maps::CMultiMetricMap averageMap;
	bool averageMapIsUpdated{false};

	/** The SFs and their corresponding pose estimations */
	mrpt::maps::CSimpleMap SFs;
	/** A mapping between indexes in the SFs to indexes in the robot paths from
	 * particles. */
	std::vector<uint32_t> SF2robotPath;

   public:
	/** The struct for passing extra simulation parameters to the
	 * prediction/update stage
	 *    when running a particle filter.
	 * \sa prediction_and_update
	 */
	struct TPredictionParams : public mrpt::config::CLoadableOptions
	{
		/** Default settings method */
		TPredictionParams();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** [pf optimal proposal only]  Only for PF algorithm=2 (Exact
		 * "pfOptimalProposal")
		 *   Select the map on which to calculate the optimal
		 *    proposal distribution. Values:
		 *   0: Gridmap   -> Uses Scan matching-based approximation (based on
		 * Stachniss' work)
		 *   1: Landmarks -> Uses matching to approximate optimal
		 *   2: Beacons   -> Used for exact optimal proposal in RO-SLAM
		 *   3: Pointsmap -> Uses Scan matching-based approximation with a map
		 * of points (based on Stachniss' work)
		 *  Default = 0
		 */
		int pfOptimalProposal_mapSelection{0};

		/** [prediction stage][pf optimal proposal only] If
		 * useICPGlobalAlign_withGrid=true, this is the minimum quality ratio
		 * [0,1] of the alignment such as it will be accepted. Otherwise, raw
		 * odometry is used for those bad cases (default=0.7).
		 */
		float ICPGlobalAlign_MinQuality{0.70f};

		/** [update stage] If the likelihood is computed through the occupancy
		 * grid map, then this structure is passed to the map when updating the
		 * particles weights in the update stage.
		 */
		COccupancyGridMap2D::TLikelihoodOptions update_gridMapLikelihoodOptions;

		mrpt::slam::TKLDParams KLD_params;

		/** ICP parameters, used only when "PF_algorithm=2" in the particle
		 * filter. */
		mrpt::slam::CICP::TConfigParams icp_params;

	} options;

	/** Constructor
	 */
	CMultiMetricMapPDF(
		const bayes::CParticleFilter::TParticleFilterOptions& opts =
			bayes::CParticleFilter::TParticleFilterOptions(),
		const mrpt::maps::TSetOfMetricMapInitializers* mapsInitializers =
			nullptr,
		const TPredictionParams* predictionOptions = nullptr);

	/** Clear all elements of the maps, and restore all paths to a single
	 * starting pose */
	void clear(const mrpt::poses::CPose2D& initialPose);
	/** \overload */
	void clear(const mrpt::poses::CPose3D& initialPose);
	/** Resets the map by loading an already-mapped map for past poses.
	 * Current robot pose should be normally set to the last keyframe
	 * in the simplemap. */
	void clear(
		const mrpt::maps::CSimpleMap& prevMap,
		const mrpt::poses::CPose3D& currentPose);

	/** Returns the estimate of the robot pose as a particles PDF for the
	 * instant of time "timeStep", from 0 to N-1.
	 * \sa getEstimatedPosePDF
	 */
	void getEstimatedPosePDFAtTime(
		size_t timeStep,
		mrpt::poses::CPose3DPDFParticles& out_estimation) const;

	/** Returns the current estimate of the robot pose, as a particles PDF.
	 * \sa getEstimatedPosePDFAtTime
	 */
	void getEstimatedPosePDF(
		mrpt::poses::CPose3DPDFParticles& out_estimation) const;

	/** Returns the weighted averaged map based on the current best estimation.
	 * If you need a persistent copy of this object, please use
	 * "CSerializable::duplicate" and use the copy.
	 * \sa Almost 100% sure you would prefer the best current map, given by
	 * getCurrentMostLikelyMetricMap()
	 */
	const CMultiMetricMap* getAveragedMetricMapEstimation();

	/** Returns a pointer to the current most likely map (associated to the most
	 * likely particle) */
	const CMultiMetricMap* getCurrentMostLikelyMetricMap() const;

	/** Get the number of CSensoryFrame inserted into the internal member SFs */
	size_t getNumberOfObservationsInSimplemap() const { return SFs.size(); }
	/** Insert an observation to the map, at each particle's pose and to each
	 * particle's metric map.
	 * \param sf The SF to be inserted
	 * \return true if any may was updated, false otherwise
	 */
	bool insertObservation(mrpt::obs::CSensoryFrame& sf);

	/** Return the path (in absolute coordinate poses) for the i'th particle.
	 * \exception On index out of bounds
	 */
	void getPath(size_t i, std::deque<math::TPose3D>& out_path) const;

	/** Returns the current entropy of paths, computed as the average entropy of
	 * poses along the path, where entropy of each pose estimation is computed
	 * as the entropy of the gaussian approximation covariance.
	 */
	double getCurrentEntropyOfPaths();

	/** Returns the joint entropy estimation over paths and maps, acording to
	 * "Information Gain-based Exploration Using" by C. Stachniss, G. Grissetti
	 * and W.Burgard.
	 */
	double getCurrentJointEntropy();

	/** Update the poses estimation of the member "SFs" according to the current
	 * path belief.
	 */
	void updateSensoryFrameSequence();

	/** A logging utility: saves the current path estimation for each particle
	 * in a text file (a row per particle, each 3-column-entry is a set
	 * [x,y,phi], respectively).
	 */
	void saveCurrentPathEstimationToTextFile(const std::string& fil);

   private:
	/** Rebuild the "expected" grid map. Used internally, do not call  */
	void rebuildAverageMap();

	/** An index [0,1] measuring how much information an observation aports to
	 * the map (Typ. threshold=0.07) */
	float newInfoIndex{0};

   public:
	/** \name Virtual methods that the PF_implementations assume exist.
		@{ */

	// see docs in base
	mrpt::math::TPose3D getLastPose(
		const size_t i, bool& pose_is_valid) const override;

	void PF_SLAM_implementation_custom_update_particle_with_new_pose(
		CParticleDataContent* particleData,
		const mrpt::math::TPose3D& newPose) const override;

	// The base implementation is fine for this class.
	// void PF_SLAM_implementation_replaceByNewParticleSet( ...

	bool PF_SLAM_implementation_doWeHaveValidObservations(
		const CParticleList& particles,
		const mrpt::obs::CSensoryFrame* sf) const override;

	bool PF_SLAM_implementation_skipRobotMovement() const override;

	/** Evaluate the observation likelihood for one particle at a given location
	 */
	double PF_SLAM_computeObservationLikelihoodForParticle(
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const size_t particleIndexForMap,
		const mrpt::obs::CSensoryFrame& observation,
		const mrpt::poses::CPose3D& x) const override;
	/** @} */

};  // End of class def.

}  // namespace maps
}  // namespace mrpt
