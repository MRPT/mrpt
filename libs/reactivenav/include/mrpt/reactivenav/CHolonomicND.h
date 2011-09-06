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
		 void  navigate(	poses::CPoint2D	&target,
							vector_double	&obstacles,
							double			maxRobotSpeed,
							double			&desiredDirection,
							double			&desiredSpeed,
							CHolonomicLogFileRecordPtr &logRecord );

		 /** The structure used to store a detected gap in obstacles.
		   */
        struct TGap
		{
                int		ini;
                int		end;
                double	entranceDistance;
                double	maxDistance;
                int		representative_sector;
        };

		typedef std::vector<TGap> TGapArray;

		/** The set of posible situations for each trajectory.
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
		 void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE );



	 private:
		 int	last_selected_sector;

		 int  direction2sector(double a, int N);

		/** Configuration:
		  */
		double TOO_CLOSE_OBSTACLE,WIDE_GAP_SIZE_PERCENT,RISK_EVALUATION_SECTORS_PERCENT;
		double RISK_EVALUATION_DISTANCE,MAX_SECTOR_DIST_FOR_D2_PERCENT;
		double TARGET_SLOW_APPROACHING_DISTANCE;

		vector_double factorWeights;

		/**  Find gaps in the obtacles.
		  */
        void  gapsEstimator(
					vector_double		&obstacles,
					poses::CPoint2D		&in_target,
					TGapArray			&gaps );

		/** Search the best gap.
		  */
        void  searchBestGap(
					vector_double		&in_obstacles,
					double				in_maxObsRange,
					TGapArray			&in_gaps,
					poses::CPoint2D		&in_target,
					int					&out_selDirection,
					double				&out_selEvaluation,
					TSituations			&out_situation,
					double				&out_riskEvaluation,
					CLogFileRecord_NDPtr	log);

		/** Fills in the representative sector field in the gap structure:
		  */
        void  calcRepresentativeSectorForGap(
					TGap					&gap,
					const poses::CPoint2D	&target,
					const vector_double		&obstacles);

		/** Evaluate each gap:
		  */
		void  evaluateGaps(
                    const vector_double	&in_obstacles,
					const double			in_maxObsRange,
					const TGapArray		&in_gaps,
                    const int			TargetSector,
                    const double			TargetDist,
                    vector_double		&out_gaps_evaluation );
	};

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

  }
}


#endif



