/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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

	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */
	
	/** Full evaluation of all possible directions within the discrete set of input directions.
	 *
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicFullEval::initialize() or directly in \a CHolonomicFullEval::options
	 *
	 * \code
	 * [FULL_EVAL_CONFIG]
	 * factorWeights=0.5 3.0 0.1 0.01 2.0
	 * // 1: Clearness in direction
	 * // 2: Closest approach to target along straight line (Euclidean)
	 * // 3: Distance of end colission-free point to target (Euclidean)
	 * // 4: Hysteresis
	 * // 5: Clearness to nearest obstacle along path
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.60  // For stopping gradually
	 * TOO_CLOSE_OBSTACLE               = 0.02  // In meters
	 * HYSTERESIS_SECTOR_COUNT          = 5     // Range of "sectors" (directions) for hysteresis over succesive timesteps
	 * \endcode
	 *
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 *  \ingroup 
	 */
	class NAV_IMPEXP CHolonomicFullEval : public CAbstractHolonomicReactiveMethod
	{
	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	public:
		/**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL */
		CHolonomicFullEval( const mrpt::utils::CConfigFileBase *INI_FILE = NULL );

		// See base class docs
		void  navigate(	const mrpt::math::TPoint2D &target,
							const std::vector<double>	&obstacles,
							double			maxRobotSpeed,
							double			&desiredDirection,
							double			&desiredSpeed,
							CHolonomicLogFileRecordPtr &logRecord,
			const double    max_obstacle_dist );

		/** Initialize the parameters of the navigator */
		void initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
		{
			options.loadFromConfigFile(INI_FILE, std::string("FULL_EVAL_CONFIG"));
		}

		/** Algorithm options */
		struct NAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TOO_CLOSE_OBSTACLE;
			double TARGET_SLOW_APPROACHING_DISTANCE;
			double HYSTERESIS_SECTOR_COUNT;
			std::vector<double> factorWeights;  //!< See docs above

			TOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg ,const std::string &section) const MRPT_OVERRIDE; // See base docs
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicFullEval::initialize or options.loadFromConfigFile(), etc.)

	private:
		unsigned int m_last_selected_sector;
		unsigned int direction2sector(const double a, const unsigned int N);
	}; // end of CHolonomicFullEval

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

