/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHolonomicND_H
#define CHolonomicND_H

#include "CAbstractHolonomicReactiveMethod.h"
#include <mrpt/utils/CLoadableOptions.h>

namespace mrpt
{
  namespace nav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CLogFileRecord_ND, CHolonomicLogFileRecord, NAV_IMPEXP)
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CHolonomicND, CAbstractHolonomicReactiveMethod, NAV_IMPEXP )

	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */
	
	/** An implementation of the holonomic reactive navigation method "Nearness-Diagram".
	 *   The algorithm "Nearness-Diagram" was proposed in:
	 *
	 *  Nearness diagram (ND) navigation: collision avoidance in troublesome scenarios, IEEE Transactions on
	 *   Robotics and Automation, Minguez, J. and Montano, L., vol. 20, no. 1, pp. 45-59, 2004.
	 *
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicND::initialize() or directly in \a CHolonomicND::options
	 *
	 * \code
	 * # Section name can be changed via setConfigFileSectionName()
	 * [ND_CONFIG]
	 * factorWeights=1.0 0.5 2.0 0.4
	 * // 1: Free space
	 * // 2: Dist. in sectors
	 * // 3: Closer to target (euclidean)
	 * // 4: Hysteresis
	 * WIDE_GAP_SIZE_PERCENT            = 0.25
	 * MAX_SECTOR_DIST_FOR_D2_PERCENT   = 0.25
	 * RISK_EVALUATION_SECTORS_PERCENT  = 0.25
	 * RISK_EVALUATION_DISTANCE         = 0.15  // In normalized ps-meters [0,1]
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.60  // For stopping gradually
	 * TOO_CLOSE_OBSTACLE               = 0.02  // In normalized ps-meters
	 * \endcode
	 *
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 *  \ingroup 
	 */
	class NAV_IMPEXP CHolonomicND : public CAbstractHolonomicReactiveMethod
	{
		DEFINE_SERIALIZABLE( CHolonomicND )
	public:
		 /**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL */
		CHolonomicND( const mrpt::utils::CConfigFileBase *INI_FILE = NULL );

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

		/** The structure used to store a detected gap in obstacles. */
		struct TGap
		{
			unsigned int  ini;
			unsigned int  end;
			double        maxDistance;
			double		  minDistance;
			unsigned int  representative_sector;
		};

		typedef std::vector<TGap> TGapArray;

		/** The set of posible situations for each trajectory. (mrpt::utils::TEnumType works with this enum) */
		enum TSituations
		{
			SITUATION_TARGET_DIRECTLY = 1,
			SITUATION_SMALL_GAP,
			SITUATION_WIDE_GAP,
			SITUATION_NO_WAY_FOUND
		};

		/**  Initialize the parameters of the navigator. */
		void  initialize(const mrpt::utils::CConfigFileBase &INI_FILE) MRPT_OVERRIDE;

		/** Algorithm options */
		struct NAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TOO_CLOSE_OBSTACLE,WIDE_GAP_SIZE_PERCENT,RISK_EVALUATION_SECTORS_PERCENT;
			double RISK_EVALUATION_DISTANCE,MAX_SECTOR_DIST_FOR_D2_PERCENT;
			double TARGET_SLOW_APPROACHING_DISTANCE;
			std::vector<double> factorWeights;  //!< Vector of 4 weights: [0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis


			TOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg ,const std::string &section) const MRPT_OVERRIDE; // See base docs
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicND::initialize or options.loadFromConfigFile(), etc.)

	private:
		unsigned int m_last_selected_sector;

		unsigned int direction2sector(const double a, const unsigned int N);

		/**  Find gaps in the obtacles.
		  */
		void  gapsEstimator(
			const std::vector<double>         & obstacles,
			const mrpt::math::TPoint2D  & in_target,
			TGapArray                   & gaps );


		/** Search the best gap.
		  */
		void  searchBestGap(
			const std::vector<double>         & in_obstacles,
			const double                  in_maxObsRange,
			const TGapArray             & in_gaps,
			const mrpt::math::TPoint2D  & in_target,
			unsigned int                & out_selDirection,
			double                      & out_selEvaluation,
			TSituations                 & out_situation,
			double                      & out_riskEvaluation,
			CLogFileRecord_NDPtr	      log);

		/** Fills in the representative sector field in the gap structure:
		  */
		void  calcRepresentativeSectorForGap(
			TGap                        & gap,
			const mrpt::math::TPoint2D  & target,
			const std::vector<double>         & obstacles);

		/** Evaluate each gap:
		  */
		void  evaluateGaps(
			const std::vector<double> & in_obstacles,
			const double          in_maxObsRange,
			const TGapArray     & in_gaps,
			const unsigned int	  TargetSector,
			const float          TargetDist,
			std::vector<double>       & out_gaps_evaluation );

	}; // end of CHolonomicND
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHolonomicND, CAbstractHolonomicReactiveMethod, NAV_IMPEXP )

	/** A class for storing extra information about the execution of
	 *    CHolonomicND navigation.
	 * \sa CHolonomicND, CHolonomicLogFileRecord
	 */
	class CLogFileRecord_ND : public CHolonomicLogFileRecord
	{
		DEFINE_SERIALIZABLE( CLogFileRecord_ND )

	public:
		 /** Member data.
		   */
		vector_int				gaps_ini,gaps_end;
		std::vector<double>			gaps_eval;
		int32_t                 selectedSector;
		double                   evaluation;
		double					riskEvaluation;
		CHolonomicND::TSituations      situation;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CLogFileRecord_ND, CHolonomicLogFileRecord, NAV_IMPEXP)

	  /** @} */
	} // end namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<nav::CHolonomicND::TSituations>
		{
			typedef nav::CHolonomicND::TSituations enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(nav::CHolonomicND::SITUATION_TARGET_DIRECTLY, "SITUATION_TARGET_DIRECTLY");
				m_map.insert(nav::CHolonomicND::SITUATION_SMALL_GAP,       "SITUATION_SMALL_GAP");
				m_map.insert(nav::CHolonomicND::SITUATION_WIDE_GAP,        "SITUATION_WIDE_GAP");
				m_map.insert(nav::CHolonomicND::SITUATION_NO_WAY_FOUND,    "SITUATION_NO_WAY_FOUND");
			}
		};
	} // End of namespace
}


#endif



