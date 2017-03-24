/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMetricMapBuilderRBPF_H
#define CMetricMapBuilderRBPF_H

#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/CMultiMetricMap.h>

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	/** This class implements a Rao-Blackwelized Particle Filter (RBPF) approach to map building (SLAM).
	 *   Internally, the list of particles, each containing a hypothesis for the robot path plus its associated
	 *    metric map, is stored in an object of class CMultiMetricMapPDF.
	 *
	 *  This class processes robot actions and observations sequentially (through the method CMetricMapBuilderRBPF::processActionObservation)
	 *   and exploits the generic design of metric map classes in MRPT to deal with any number and combination of maps simultaneously: the likelihood
	 *   of observations is the product of the likelihood in the different maps, etc.
	 *
	 *   A number of particle filter methods are implemented as well, by selecting the appropriate values in TConstructionOptions::PF_options.
	 *   Not all the PF algorithms are implemented for all kinds of maps.
	 *
     *  For an example of usage, check the application "rbpf-slam", in "apps/RBPF-SLAM". See also the <a href="http://www.mrpt.org/Application:RBPF-SLAM" >wiki page</a>.
	 *
	 *  \note Since MRPT 0.7.2, the new variables "localizeLinDistance,localizeAngDistance" are introduced to provide a way to update the robot pose at a different rate than the map is updated.
	 *  \note Since MRPT 0.7.1 the semantics of the parameters "insertionLinDistance" and "insertionAngDistance" changes: the entire RBFP is now NOT updated unless odometry increments surpass the threshold (previously, only the map was NOT updated). This is done to gain efficiency.
	 *  \note Since MRPT 0.6.2 this class implements full 6D SLAM. Previous versions worked in 2D + heading only.
	 *
	 * \sa CMetricMap   \ingroup metric_slam_grp
	 */
	class SLAM_IMPEXP CMetricMapBuilderRBPF : public mrpt::slam::CMetricMapBuilder
	{
	public:
		/** The map PDF: It includes a path and associated map for each particle. */
		mrpt::maps::CMultiMetricMapPDF			mapPDF;

	protected:
		/** The configuration of the particle filter */
		bayes::CParticleFilter::TParticleFilterOptions  m_PF_options;

		/** Distances (linear and angular) for inserting a new observation into the map. */
		float	insertionLinDistance,insertionAngDistance;

		/** Distances (linear and angular) for updating the robot pose estimate (and particles weighs, if applicable). */
		float	localizeLinDistance,localizeAngDistance;


		mrpt::poses::CPose3DPDFGaussian	odoIncrementSinceLastLocalization;	//!< Traveled distance since last localization update
		mrpt::poses::CPose3D			odoIncrementSinceLastMapUpdate;		//!< Traveled distance since last map update

	public:

		/** Options for building a CMetricMapBuilderRBPF object, passed to the constructor.
		  */
		struct SLAM_IMPEXP TConstructionOptions : public utils::CLoadableOptions
		{
		public:
			/** Constructor */
			TConstructionOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			float	insertionLinDistance;
			float	insertionAngDistance;

			float 	localizeLinDistance;
			float 	localizeAngDistance;

			bayes::CParticleFilter::TParticleFilterOptions  PF_options;

			mrpt::maps::TSetOfMetricMapInitializers					mapsInitializers;
			mrpt::maps::CMultiMetricMapPDF::TPredictionParams		predictionOptions;
			mrpt::utils::VerbosityLevel  verbosity_level;
		};

		/** Constructor. */
		CMetricMapBuilderRBPF( const TConstructionOptions &initializationOptions );
		
		/** This second constructor is created for the situation where a class member needs to be
of type CMetricMapBuilderRBPF  */
		CMetricMapBuilderRBPF();
		
		/** Copy Operator. */
		CMetricMapBuilderRBPF & operator =(const CMetricMapBuilderRBPF &src);

		/** Destructor. */
		virtual ~CMetricMapBuilderRBPF( );

		/** Initialize the method, starting with a known location PDF "x0"(if supplied, set to NULL to left unmodified) and a given fixed, past map. */
		void  initialize(
			const mrpt::maps::CSimpleMap		&initialMap  = mrpt::maps::CSimpleMap(),
			mrpt::poses::CPosePDF					*x0 = NULL
			);

		/** Clear all elements of the maps.
		  */
		void  clear();

		/** Returns a copy of the current best pose estimation as a pose PDF.
		  */
		mrpt::poses::CPose3DPDFPtr  getCurrentPoseEstimation() const;

		/** Returns the current most-likely path estimation (the path associated to the most likely particle).
		  */
		void  getCurrentMostLikelyPath( std::deque<mrpt::math::TPose3D> &outPath ) const;

		/** Appends a new action and observations to update this map: See the description of the class at the top of this page to see a more complete description.
		 *  \param action The incremental 2D pose change in the robot pose. This value is deterministic.
		 *	\param observations The set of observations that robot senses at the new pose.
		 *  Statistics will be saved to statsLastIteration
		 */
		void  processActionObservation(
			mrpt::obs::CActionCollection	&action,
			mrpt::obs::CSensoryFrame		&observations );

		/** Fills "out_map" with the set of "poses"-"sensory-frames", thus the so far built map.
		  */
		void  getCurrentlyBuiltMap(mrpt::maps::CSimpleMap &out_map) const;

		/** Returns the map built so far. NOTE that for efficiency a pointer to the internal object is passed, DO NOT delete nor modify the object in any way, if desired, make a copy of ir with "duplicate()".
		  */
		const mrpt::maps::CMultiMetricMap* getCurrentlyBuiltMetricMap() const;

		/** Returns just how many sensory-frames are stored in the currently build map.
		  */
		unsigned int  getCurrentlyBuiltMapSize();

		/** A useful method for debugging: the current map (and/or poses) estimation is dumped to an image file.
		  * \param file The output file name
		  * \param formatEMF_BMP Output format = true:EMF, false:BMP
		  */
		void  saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true);

		/** A useful method for debugging: draws the current map and path hypotheses to a CCanvas  */
		void  drawCurrentEstimationToImage( utils::CCanvas *img );

		/** A logging utility: saves the current path estimation for each particle in a text file (a row per particle, each 3-column-entry is a set [x,y,phi], respectively).
		  */
		void  saveCurrentPathEstimationToTextFile( const std::string  &fil );

		double  getCurrentJointEntropy();

		/** This structure will hold stats after each execution of processActionObservation
		  */
		struct SLAM_IMPEXP TStats
		{
			TStats() :
				observationsInserted(false)
			{ }

			/** Whether the SF has been inserted in the metric maps. */
			bool	observationsInserted;

		};

		/** This structure will hold stats after each execution of processActionObservation */
		TStats 		m_statsLastIteration;

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
