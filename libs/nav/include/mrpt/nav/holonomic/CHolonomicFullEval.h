/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include "CAbstractHolonomicReactiveMethod.h"
#include <mrpt/utils/CLoadableOptions.h>

namespace mrpt
{
  namespace nav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CLogFileRecord_FullEval, CHolonomicLogFileRecord, NAV_IMPEXP)
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CHolonomicFullEval, CAbstractHolonomicReactiveMethod, NAV_IMPEXP )

	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */
	
	/** Full evaluation of all possible directions within the discrete set of input directions.
	 *
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicFullEval::initialize() or directly in \a CHolonomicFullEval::options
	 *
	 * \code
	 * # Section name can be changed via setConfigFileSectionName()
	 * [FULL_EVAL_CONFIG]
	 * factorWeights        = 1.0 1.0 1.0 0.05 1.0
	 * factorNormalizeOrNot =   0   0   0    0   1
	 * // 0: Clearness in direction
	 * // 1: Closest approach to target along straight line (Euclidean)
	 * // 2: Distance of end collision-free point to target (Euclidean)
	 * // 3: Hysteresis
	 * // 4: Clearness to nearest obstacle along path
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.20   // Start to reduce speed when closer than this to target.
	 * TOO_CLOSE_OBSTACLE               = 0.02   // Directions with collision-free distances below this threshold are not elegible.
	 * HYSTERESIS_SECTOR_COUNT          = 5      // Range of "sectors" (directions) for hysteresis over succesive timesteps
	 * PHASE1_FACTORS   = 0 1 2                  // Indices of the factors above to be considered in phase 1
	 * PHASE2_FACTORS   = 3 4                    // Indices of the factors above to be considered in phase 2
	 * PHASE1_THRESHOLD = 0.75                   // Phase1 scores must be above this relative range threshold [0,1] to be considered in phase 2 (Default:`0.75`)
	 * \endcode
	 *
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 *  \ingroup 
	 */
	class NAV_IMPEXP CHolonomicFullEval : public CAbstractHolonomicReactiveMethod
	{
		DEFINE_SERIALIZABLE( CHolonomicFullEval )
	public:
		/**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL */
		CHolonomicFullEval( const mrpt::utils::CConfigFileBase *INI_FILE = NULL );

		// See base class docs
		void  navigate(
			const mrpt::math::TPoint2D &target,
			const std::vector<double>	&obstacles,
			double			maxRobotSpeed,
			double			&desiredDirection,
			double			&desiredSpeed,
			CHolonomicLogFileRecordPtr &logRecord,
			const double    max_obstacle_dist,
			const mrpt::nav::ClearanceDiagram *clearance = NULL) MRPT_OVERRIDE;

		void initialize(const mrpt::utils::CConfigFileBase &INI_FILE) MRPT_OVERRIDE; // See base class docs

		/** Algorithm options */
		struct NAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TOO_CLOSE_OBSTACLE;  //!< Directions with collision-free distances below this threshold are not elegible.
			double TARGET_SLOW_APPROACHING_DISTANCE; //!< Start to reduce speed when closer than this to target.
			double OBSTACLE_SLOW_DOWN_DISTANCE;      //!< Start to reduce speed when clearance is below this value ([0,1] ratio wrt obstacle reference/max distance)
			double HYSTERESIS_SECTOR_COUNT; //!< Range of "sectors" (directions) for hysteresis over succesive timesteps
			std::vector<double>   factorWeights;  //!< See docs above
			std::vector<int32_t>  factorNormalizeOrNot; //!< 0/1 to normalize factors.
			std::vector<int32_t>  PHASE1_FACTORS, PHASE2_FACTORS; //!< Factor indices [0,4] for the factors to consider in each phase of the movement decision (Defaults: `PHASE1_FACTORS=0 1 2`, `PHASE2_FACTORS=`3 4`)
			double                PHASE1_THRESHOLD;   //!< Phase1 scores must be above this relative range threshold [0,1] to be considered in phase 2 (Default:`0.75`)

			TOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg ,const std::string &section) const MRPT_OVERRIDE; // See base docs
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicFullEval::initialize or options.loadFromConfigFile(), etc.)

	private:
		unsigned int m_last_selected_sector;
		unsigned int direction2sector(const double a, const unsigned int N);
		mrpt::math::CMatrixD m_dirs_scores; //!< Individual scores for each direction: (i,j), i (row) are directions, j (cols) are scores. Not all directions may have evaluations, in which case a "-1" value will be found.

		virtual void postProcessDirectionEvaluations(std::vector<double> &dir_evals); // If desired, override in a derived class to manipulate the final evaluations of each directions

	}; // end of CHolonomicFullEval
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHolonomicFullEval, CAbstractHolonomicReactiveMethod, NAV_IMPEXP )

	/** A class for storing extra information about the execution of CHolonomicFullEval navigation.
	 * \sa CHolonomicFullEval, CHolonomicLogFileRecord
	 */
	class CLogFileRecord_FullEval : public CHolonomicLogFileRecord
	{
		DEFINE_SERIALIZABLE( CLogFileRecord_FullEval )
	public:
		 /** Member data */
		std::vector<double>  dirs_eval; //!< Evaluation of each direction, in the same order of TP-Obstacles.
		int32_t              selectedSector;
		double               evaluation;
		mrpt::math::CMatrixD dirs_scores; //!< Individual scores for each direction: (i,j), i (row) are directions, j (cols) are scores. Not all directions may have evaluations, in which case a "-1" value will be found.

		const mrpt::math::CMatrixD * getDirectionScores() const MRPT_OVERRIDE { return &dirs_scores; }
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CLogFileRecord_FullEval, CHolonomicLogFileRecord, NAV_IMPEXP)

	  /** @} */
	} // end namespace
}

