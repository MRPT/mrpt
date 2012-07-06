/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CHolonomicND_H
#define CHolonomicND_H

#include "CAbstractHolonomicReactiveMethod.h"

namespace mrpt
{
  namespace reactivenav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CLogFileRecord_ND, CHolonomicLogFileRecord, REACTIVENAV_IMPEXP)

	/** An implementation of the holonomic reactive navigation method "Nearness-Diagram".
	 *   The algorithm "Nearness-Diagram" was proposed in:
	 *
	 *  Nearness diagram (ND) navigation: collision avoidance in troublesome scenarios, IEEE Transactions on
	 *   Robotics and Automation, Minguez, J. and Montano, L., vol. 20, no. 1, pp. 45-59, 2004.
	 *
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicND::initialize() or directly in \a CHolonomicND::options
	 *
	 * \code
	 * [ND_CONFIG]
	 * factorWeights=1.0 0.5 2.0 0.4
	 * // 1: Free space
	 * // 2: Dist. in sectors
	 * // 3: Closer to target (euclidean)
	 * // 4: Hysteresis
	 * WIDE_GAP_SIZE_PERCENT            = 0.50
	 * MAX_SECTOR_DIST_FOR_D2_PERCENT   = 0.25
	 * RISK_EVALUATION_SECTORS_PERCENT  = 0.25
	 * RISK_EVALUATION_DISTANCE         = 0.15  // In normalized ps-meters [0,1]
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.60  // For stopping gradually
	 * TOO_CLOSE_OBSTACLE               = 0.02  // In normalized ps-meters
	 * \endcode
	 *
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CHolonomicND : public CAbstractHolonomicReactiveMethod
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		 /**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL.
		   */
		 CHolonomicND( const mrpt::utils::CConfigFileBase *INI_FILE = NULL );

		 /** This method performs the holonomic navigation itself.
		   *  \param target [IN] The relative location (x,y) of target point.
		   *  \param obstacles [IN] Distance to obstacles from robot location (0,0). First index refers to -PI direction, and last one to +PI direction. Distances can be dealed as "meters", although they are "pseudometers", see note below.
		   *  \param maxRobotSpeed [IN] Maximum robot speed, in "pseudometers/sec". See note below.
		   *  \param desiredDirection [OUT] The desired motion direction, in the range [-PI,PI]
		   *  \param desiredSpeed [OUT] The desired motion speed in that direction, in "pseudometers"/sec. (See note below)
		   *  \param logRecord [IN/OUT] A placeholder for a pointer to a log record with extra info about the execution. Set to NULL if not required. User <b>must free memory</b> using "delete logRecord" after using it.
		   *
		   *  NOTE: With "pseudometers" we refer to the distance unit in TP-Space, thus:
		   *     <br><center><code>pseudometer<sup>2</sup>= meter<sup>2</sup> + (rad · r)<sup>2</sup></code><br></center>
		   */
		 void  navigate(	const mrpt::math::TPoint2D &target,
							const vector_double	&obstacles,
							double			maxRobotSpeed,
							double			&desiredDirection,
							double			&desiredSpeed,
							CHolonomicLogFileRecordPtr &logRecord );

		 /** The structure used to store a detected gap in obstacles.
		   */
		struct TGap
		{
			unsigned int  ini;
			unsigned int  end;
			double        entranceDistance;
			double        maxDistance;
			unsigned int  representative_sector;
		};

		typedef std::vector<TGap> TGapArray;

		/** The set of posible situations for each trajectory. (mrpt::utils::TEnumType works with this enum)
		  */
		enum TSituations
		{
			SITUATION_TARGET_DIRECTLY = 1,
			SITUATION_SMALL_GAP,
			SITUATION_WIDE_GAP,
			SITUATION_NO_WAY_FOUND
		};

		/**  Initialize the parameters of the navigator.
		  */
		void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
		{
			options.loadFromConfigFile(INI_FILE, std::string("ND_CONFIG"));
		}

		/** Algorithm options */
		struct REACTIVENAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TOO_CLOSE_OBSTACLE,WIDE_GAP_SIZE_PERCENT,RISK_EVALUATION_SECTORS_PERCENT;
			double RISK_EVALUATION_DISTANCE,MAX_SECTOR_DIST_FOR_D2_PERCENT;
			double TARGET_SLOW_APPROACHING_DISTANCE;
			vector_double factorWeights;  //!< Vector of 4 weights: [0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis


			TOptions();
			virtual void saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const;
			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section);
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicND::initialize or options.loadFromConfigFile(), etc.)

	private:
		unsigned int m_last_selected_sector;

		unsigned int direction2sector(const double a, const unsigned int N);

		/**  Find gaps in the obtacles.
		  */
		void  gapsEstimator(
			const vector_double         & obstacles,
			const mrpt::math::TPoint2D  & in_target,
			TGapArray                   & gaps );

		/** Search the best gap.
		  */
		void  searchBestGap(
			const vector_double         & in_obstacles,
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
			const vector_double         & obstacles);

		/** Evaluate each gap:
		  */
		void  evaluateGaps(
			const vector_double & in_obstacles,
			const double          in_maxObsRange,
			const TGapArray     & in_gaps,
			const unsigned int	  TargetSector,
			const double          TargetDist,
			vector_double       & out_gaps_evaluation );

	}; // end of CHolonomicND

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
		vector_double			gaps_eval;
		int32_t                 selectedSector;
		double                   evaluation;
		double					riskEvaluation;
		CHolonomicND::TSituations      situation;
	};

	} // end namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<reactivenav::CHolonomicND::TSituations>
		{
			typedef reactivenav::CHolonomicND::TSituations enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(reactivenav::CHolonomicND::SITUATION_TARGET_DIRECTLY, "SITUATION_TARGET_DIRECTLY");
				m_map.insert(reactivenav::CHolonomicND::SITUATION_SMALL_GAP,       "SITUATION_SMALL_GAP");
				m_map.insert(reactivenav::CHolonomicND::SITUATION_WIDE_GAP,        "SITUATION_WIDE_GAP");
				m_map.insert(reactivenav::CHolonomicND::SITUATION_NO_WAY_FOUND,    "SITUATION_NO_WAY_FOUND");
			}
		};
	} // End of namespace
}


#endif



