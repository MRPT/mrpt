/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CMultiMetricMapPDF_H
#define CMultiMetricMapPDF_H

#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CSimpleMap.h>
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
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRBPFParticleData, mrpt::utils::CSerializable, SLAM_IMPEXP )

	/** Auxiliary class used in mrpt::slam::CMultiMetricMapPDF
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

		CMultiMetricMap			mapTillNow;
		std::deque<TPose3D>		robotPath;
	};


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
		public mrpt::bayes::CParticleFilterCapable,
		public mrpt::slam::PF_implementation<CRBPFParticleData,CMultiMetricMapPDF>
	{
		friend class CMetricMapBuilderRBPF;
		//template <class PARTICLE_TYPE, class MYSELF> friend class PF_implementation;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CMultiMetricMapPDF )

		// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
		IMPLEMENT_PARTICLE_FILTER_CAPABLE(CRBPFParticleData);

	protected:
		/** The PF algorithm implementation.
		  */
		void  prediction_and_update_pfStandardProposal(
			const mrpt::slam::CActionCollection	* action,
			const mrpt::slam::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

		/** The PF algorithm implementation.
		  */
		void  prediction_and_update_pfOptimalProposal(
			const mrpt::slam::CActionCollection	* action,
			const mrpt::slam::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

		/** The PF algorithm implementation.
		  */
		void  prediction_and_update_pfAuxiliaryPFOptimal(
			const mrpt::slam::CActionCollection	* action,
			const mrpt::slam::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );


	private:
		/** Internal buffer for the averaged map.
		  */
		CMultiMetricMap			averageMap;
		bool					averageMapIsUpdated;

		/** The SFs and their corresponding pose estimations:
		 */
		CSimpleMap	SFs;

		/** A mapping between indexes in the SFs to indexes in the robot paths from particles.
		  */
		std::vector<uint32_t>	SF2robotPath;


		/** Entropy aux. function
		  */
		float  H(float p);

	public:

		/** The struct for passing extra simulation parameters to the prediction/update stage
		 *    when running a particle filter.
		 * \sa prediction_and_update
		 */
		struct SLAM_IMPEXP TPredictionParams : public utils::CLoadableOptions
		{
			/** Default settings method.
			  */
			TPredictionParams();

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

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

			TKLDParams		KLD_params;

			CICP::TConfigParams		icp_params; //!< ICP parameters, used only when "PF_algorithm=2" in the particle filter.

		} options;

		/** Constructor
		  */
		CMultiMetricMapPDF(
			const bayes::CParticleFilter::TParticleFilterOptions    &opts = bayes::CParticleFilter::TParticleFilterOptions(),
			const mrpt::slam::TSetOfMetricMapInitializers		    *mapsInitializers = NULL,
			const TPredictionParams						    *predictionOptions = NULL );

		/** Destructor
		 */
		virtual ~CMultiMetricMapPDF();

		/** Clear all elements of the maps, and restore all paths to a single starting pose */
		void  clear( const CPose2D &initialPose );

		/** Clear all elements of the maps, and restore all paths to a single starting pose */
		void  clear( const CPose3D &initialPose );

		 /** Returns the estimate of the robot pose as a particles PDF for the instant of time "timeStep", from 0 to N-1.
		  * \sa getEstimatedPosePDF
		  */
		void  getEstimatedPosePDFAtTime(
			size_t				timeStep,
			CPose3DPDFParticles &out_estimation ) const;

		 /** Returns the current estimate of the robot pose, as a particles PDF.
		  * \sa getEstimatedPosePDFAtTime
		  */
		void  getEstimatedPosePDF( CPose3DPDFParticles	&out_estimation ) const;

		/** Returns the weighted averaged map based on the current best estimation. If you need a persistent copy of this object, please use "CSerializable::duplicate" and use the copy.
		  */
		CMultiMetricMap * getCurrentMetricMapEstimation( );

		/** Returns a pointer to the current most likely map (associated to the most likely particle).
		  */
		CMultiMetricMap  * getCurrentMostLikelyMetricMap( );

		/** Get the number of CSensoryFrame inserted into the internal member SFs */
		size_t  getNumberOfObservationsInSimplemap() const { return SFs.size(); }

		/** Insert an observation to the map, at each particle's pose and to each particle's metric map.
		  * \param sf The SF to be inserted
		  */
		void  insertObservation(CSensoryFrame	&sf);

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

		/** An index [0,1] measuring how much information an observation aports to the map (Typ. threshold=0.07)
		  */
		float						newInfoIndex;

	 private:
		/** Rebuild the "expected" grid map. Used internally, do not call
		  */
		void  rebuildAverageMap();



	//protected:
	public:
			/** \name Virtual methods that the PF_implementations assume exist.
			    @{ */

			/** Return a pointer to the last robot pose in the i'th particle (or NULL if it's a path and it's empty). */
			const TPose3D * getLastPose(const size_t i) const;

			void PF_SLAM_implementation_custom_update_particle_with_new_pose(
				CParticleDataContent *particleData,
				const TPose3D &newPose) const;

			// The base implementation is fine for this class.
			// void PF_SLAM_implementation_replaceByNewParticleSet( ...

			bool PF_SLAM_implementation_doWeHaveValidObservations(
				const CParticleList	&particles,
				const CSensoryFrame *sf) const;

			bool PF_SLAM_implementation_skipRobotMovement() const;

			/** Evaluate the observation likelihood for one particle at a given location */
			double PF_SLAM_computeObservationLikelihoodForParticle(
				const CParticleFilter::TParticleFilterOptions	&PF_options,
				const size_t			particleIndexForMap,
				const CSensoryFrame		&observation,
				const CPose3D			&x ) const;
			/** @} */


	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
