/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CHolonomicND_H
#define CHolonomicND_H

#include "CAbstractHolonomicReactiveMethod.h"
#include <mrpt/config/CLoadableOptions.h>

namespace mrpt
{
namespace nav
{
class CLogFileRecord_ND;
/** \addtogroup nav_holo Holonomic navigation methods
 * \ingroup mrpt_nav_grp
 * @{ */

/** An implementation of the holonomic reactive navigation method
 * "Nearness-Diagram".
 *   The algorithm "Nearness-Diagram" was proposed in:
 *
 *  Nearness diagram (ND) navigation: collision avoidance in troublesome
 * scenarios, IEEE Transactions on
 *   Robotics and Automation, Minguez, J. and Montano, L., vol. 20, no. 1, pp.
 * 45-59, 2004.
 *
 * These are the optional parameters of the method which can be set by means of
 * a configuration file passed to the constructor or to
 * CHolonomicND::initialize() or directly in \a CHolonomicND::options
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
 */
class CHolonomicND : public CAbstractHolonomicReactiveMethod
{
	DEFINE_SERIALIZABLE(CHolonomicND)
   public:
	/**  Initialize the parameters of the navigator, from some configuration
	 * file, or default values if set to nullptr */
	CHolonomicND(const mrpt::config::CConfigFileBase* INI_FILE = nullptr);

	// See base class docs
	void navigate(const NavInput& ni, NavOutput& no) override;

	/** The structure used to store a detected gap in obstacles. */
	struct TGap
	{
		unsigned int ini;
		unsigned int end;
		double maxDistance;
		double minDistance;
		unsigned int representative_sector;
	};

	typedef std::vector<TGap> TGapArray;

	/** The set of posible situations for each trajectory.
	 * (mrpt::typemeta::TEnumType works with this enum) */
	enum TSituations
	{
		SITUATION_TARGET_DIRECTLY = 1,
		SITUATION_SMALL_GAP,
		SITUATION_WIDE_GAP,
		SITUATION_NO_WAY_FOUND
	};

	/**  Initialize the parameters of the navigator. */
	void initialize(const mrpt::config::CConfigFileBase& INI_FILE) override;
	virtual void saveConfigFile(mrpt::config::CConfigFileBase& c)
		const override;  // See base class docs

	/** Algorithm options */
	struct TOptions : public mrpt::config::CLoadableOptions
	{
		double TOO_CLOSE_OBSTACLE, WIDE_GAP_SIZE_PERCENT,
			RISK_EVALUATION_SECTORS_PERCENT;
		double RISK_EVALUATION_DISTANCE, MAX_SECTOR_DIST_FOR_D2_PERCENT;
		double TARGET_SLOW_APPROACHING_DISTANCE;
		/** Vector of 4 weights: [0]=Free space, [1]=Dist. in sectors,
		 * [2]=Closer to target (Euclidean), [3]=Hysteresis */
		std::vector<double> factorWeights;

		TOptions();
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& cfg,
			const std::string& section) const override;  // See base docs
	};

	/** Parameters of the algorithm (can be set manually or loaded from
	 * CHolonomicND::initialize or options.loadFromConfigFile(), etc.) */
	TOptions options;

	double getTargetApproachSlowDownDistance() const override
	{
		return options.TARGET_SLOW_APPROACHING_DISTANCE;
	}
	void setTargetApproachSlowDownDistance(const double dist) override
	{
		options.TARGET_SLOW_APPROACHING_DISTANCE = dist;
	}

   private:
	unsigned int m_last_selected_sector;

	unsigned int direction2sector(const double a, const unsigned int N);

	/**  Find gaps in the obtacles.
	 */
	void gapsEstimator(
		const std::vector<double>& obstacles,
		const mrpt::math::TPoint2D& in_target, TGapArray& gaps);

	/** Search the best gap.
	 */
	void searchBestGap(
		const std::vector<double>& in_obstacles, const double in_maxObsRange,
		const TGapArray& in_gaps, const mrpt::math::TPoint2D& in_target,
		unsigned int& out_selDirection, double& out_selEvaluation,
		TSituations& out_situation, double& out_riskEvaluation,
		CLogFileRecord_ND& log);

	/** Fills in the representative sector field in the gap structure:
	 */
	void calcRepresentativeSectorForGap(
		TGap& gap, const mrpt::math::TPoint2D& target,
		const std::vector<double>& obstacles);

	/** Evaluate each gap:
	 */
	void evaluateGaps(
		const std::vector<double>& in_obstacles, const double in_maxObsRange,
		const TGapArray& in_gaps, const unsigned int TargetSector,
		const float TargetDist, std::vector<double>& out_gaps_evaluation);

};  // end of CHolonomicND

/** A class for storing extra information about the execution of
 *    CHolonomicND navigation.
 * \sa CHolonomicND, CHolonomicLogFileRecord
 */
class CLogFileRecord_ND : public CHolonomicLogFileRecord
{
	DEFINE_SERIALIZABLE(CLogFileRecord_ND)

   public:
	/** Member data.
	 */
	std::vector<int> gaps_ini, gaps_end;
	std::vector<double> gaps_eval;
	int32_t selectedSector;
	double evaluation;
	double riskEvaluation;
	CHolonomicND::TSituations situation;
};

/** @} */
}  // namespace nav

// Specializations MUST occur at the same namespace:
namespace typemeta
{
template <>
struct TEnumTypeFiller<nav::CHolonomicND::TSituations>
{
	typedef nav::CHolonomicND::TSituations enum_t;
	static void fill(internal::bimap<enum_t, std::string>& m_map)
	{
		m_map.insert(
			nav::CHolonomicND::SITUATION_TARGET_DIRECTLY,
			"SITUATION_TARGET_DIRECTLY");
		m_map.insert(
			nav::CHolonomicND::SITUATION_SMALL_GAP, "SITUATION_SMALL_GAP");
		m_map.insert(
			nav::CHolonomicND::SITUATION_WIDE_GAP, "SITUATION_WIDE_GAP");
		m_map.insert(
			nav::CHolonomicND::SITUATION_NO_WAY_FOUND,
			"SITUATION_NO_WAY_FOUND");
	}
};
}  // namespace typemeta
}  // namespace mrpt

#endif
