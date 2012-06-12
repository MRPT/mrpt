/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::reactivenav;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_ND, CHolonomicLogFileRecord,mrpt::reactivenav )



/**  Initialize the parameters of the navigator, from some
*    configuration file, or default values if filename is set to NULL.
*/
CHolonomicND::CHolonomicND(const mrpt::utils::CConfigFileBase *INI_FILE ) :
	m_last_selected_sector ( std::numeric_limits<unsigned int>::max() ),
	// Default values:
	WIDE_GAP_SIZE_PERCENT              ( 0.50 ),
	MAX_SECTOR_DIST_FOR_D2_PERCENT     ( 0.25 ),
	RISK_EVALUATION_SECTORS_PERCENT    ( 0.10 ),
	RISK_EVALUATION_DISTANCE           ( 0.4  ),
	TOO_CLOSE_OBSTACLE                 ( 0.15 ),
	TARGET_SLOW_APPROACHING_DISTANCE   ( 0.60 )
{
	if (INI_FILE!=NULL)
		initialize( *INI_FILE );
}

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
void  CHolonomicND::initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
{
	MRPT_START

	const std::string section("ND_CONFIG");

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(WIDE_GAP_SIZE_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(MAX_SECTOR_DIST_FOR_D2_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_SECTORS_PERCENT,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_DISTANCE,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  INI_FILE,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  INI_FILE,section );

	INI_FILE.read_vector(section,"factorWeights", vector_double(0), factorWeights, true );
	ASSERT_(factorWeights.size()==4);

	MRPT_END
}


/*---------------------------------------------------------------
						Navigate
  ---------------------------------------------------------------*/
void  CHolonomicND::navigate(
	const mrpt::math::TPoint2D &target,
	const vector_double	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord)
{
	TGapArray			gaps;
	TSituations			situation;
	unsigned int		selectedSector;
	double				riskEvaluation;
	CLogFileRecord_NDPtr log;
	double				evaluation;

	// Create a log record for returning data.
	if (!logRecord.present())
	{
		log = CLogFileRecord_ND::Create();
		logRecord = log;
	}


	// Search gaps:
	gaps.clear();
	gapsEstimator(	obstacles,
					target,
					gaps );


	// Select best gap:
	searchBestGap(	obstacles,
					1.0,
					gaps,
					target,
					selectedSector,
					evaluation,
					situation,
					riskEvaluation,
					log);

	if (situation == SITUATION_NO_WAY_FOUND)
	{
		// No way found!
		desiredDirection = 0;
		desiredSpeed = 0;
	}
	else
	{
		// A valid movement:
		desiredDirection = (double)(M_PI*(-1 + 2*(0.5f+selectedSector)/((double)obstacles.size())));

		// Speed control: Reduction factors
		// ---------------------------------------------
		double		targetNearnessFactor = max(0.20, min(1.0, 1.0-exp(-(target.norm()+0.01)/TARGET_SLOW_APPROACHING_DISTANCE)));
		//printf(" TARGET NEARNESS = %f\n",targetNearnessFactor);
		double		riskFactor = min(1.0, riskEvaluation / RISK_EVALUATION_DISTANCE );

		desiredSpeed = maxRobotSpeed * min(riskFactor,targetNearnessFactor);
	}

	m_last_selected_sector = selectedSector;

	// LOG --------------------------
	if (log)
	{
		// gaps:
		if (situation != SITUATION_TARGET_DIRECTLY )
		{
			int	i,n = gaps.size();
			log->gaps_ini.resize(n);
			log->gaps_end.resize(n);
			for (i=0;i<n;i++)
			{
				log->gaps_ini[i]  = gaps[i].ini;
				log->gaps_end[i]  = gaps[i].end;
			}
		}
		// Selection:
		log->selectedSector = selectedSector;
		log->evaluation = evaluation;
		log->situation = situation;
		log->riskEvaluation = riskEvaluation;
	}
}


/*---------------------------------------------------------------
						Find gaps in the obtacles.
  ---------------------------------------------------------------*/
void  CHolonomicND::gapsEstimator(
	const vector_double         & obstacles,
	const mrpt::math::TPoint2D  & target,
	TGapArray                   & gaps_out)
{
	const size_t n = obstacles.size();

	// ===== Parameters =====
	const double GAPS_MIN_RELATIVE_DEPTH = 0.1; // Minimum ratio gap_entrance_depth/max_depth_of_all_gaps for being considered as a gap.
	const double GAPS_REMOVE_INNER_IF_INNER_WIDTH_RATIO_BELOW = 0.05;
	const double GAPS_REMOVE_INNER_IF_OUTER_WIDTH_RATIO_BELOW = 0.25;
	const double GAPS_MINIMUM_DEPTH_DIFFERENCES_RATIO = 0.1; // The ratio of overall_maximum_depth/overall_minimum_depth which will be the minimum significative difference in depth of two gaps for considering both the outer and the inner gap.
// ======================

	// Find the maximum distances to obstacles:
	// ----------------------------------------------------------
	double overall_max_dist = std::numeric_limits<double>::min(), overall_min_dist = std::numeric_limits<double>::max();
	for (size_t i=1;i<(n-1);i++)
	{
		mrpt::utils::keep_max(overall_max_dist, obstacles[i]);
		mrpt::utils::keep_min(overall_min_dist, obstacles[i]);
	}

	//  Build list of "GAPS":
	// --------------------------------------------------------
	TGapArray gaps_temp;
	gaps_temp.reserve( 150 );

	for (double threshold_ratio = 0.975;threshold_ratio>=0.04;threshold_ratio-=0.05)
	{
			const double  dist_threshold = threshold_ratio* overall_max_dist + (1.0f-threshold_ratio)*overall_min_dist;
			
			bool    is_inside = false;
			size_t  sec_ini=0, sec_end=0;
			double  maxDist=0;

			for (size_t i=0;i<n;i++)
			{
				if ( !is_inside && (!i || obstacles[i]>=dist_threshold) )
				{
					sec_ini = i;
					maxDist = obstacles[i];
					is_inside = true;
				}
				else if (is_inside && (i==(n-1) || obstacles[i]<dist_threshold ))
				{
					sec_end = i;
					is_inside = false;

					if ( (sec_end-sec_ini) > 2 )
					{
						// Add new gap:
						gaps_temp.resize( gaps_temp.size() + 1 );
						TGap	& newGap = *gaps_temp.rbegin();

						newGap.ini				= sec_ini;
						newGap.end				= sec_end;
						newGap.entranceDistance = min( obstacles[sec_ini], obstacles[sec_end] );
						newGap.maxDistance		= maxDist;
					}
				}

				if (is_inside) maxDist = std::max( maxDist, obstacles[i] );
			}
	}

	// Remove redundant gaps:
	// -------------------------------------------------------------
	const size_t nTempGaps = gaps_temp.size();

	std::vector<bool> delete_gaps;
	delete_gaps.assign( nTempGaps, false);

	// Remove gaps with a tiny depth
	double max_depth = 0;
	for (size_t i=0;i<nTempGaps;i++)
	{
		const double depth = gaps_temp[i].maxDistance - gaps_temp[i].entranceDistance;
		mrpt::utils::keep_max(max_depth, depth);
	}

	for (size_t i=0;i<nTempGaps;i++)
	{
		const double depth = gaps_temp[i].maxDistance - gaps_temp[i].entranceDistance;
		if ( depth< max_depth * GAPS_MIN_RELATIVE_DEPTH )
				delete_gaps[i]=true;
	}

	// If a gap is very narrow but it's inside another gap (a bit wider), then delete the former:
	for (size_t i=0;i<nTempGaps;i++)
	{
		const unsigned int ini_i = gaps_temp[i].ini;
		const unsigned int end_i = gaps_temp[i].end;
		const unsigned int width_i = end_i - ini_i;

		if ( !delete_gaps[i] )
		{
			for (unsigned int j=0;j<nTempGaps;j++)
			{
				if (i==j || delete_gaps[j]) continue;

				const unsigned int ini_j = gaps_temp[j].ini;
				const unsigned int end_j = gaps_temp[j].end;
				const unsigned int width_j = end_j - ini_j;

				// "j" is inside "i" and only SLIGHTLY larger -> delete
				if (ini_j>=ini_i && end_j<=end_i &&
				    width_i < (GAPS_REMOVE_INNER_IF_INNER_WIDTH_RATIO_BELOW*n) && 
				    width_j < (GAPS_REMOVE_INNER_IF_OUTER_WIDTH_RATIO_BELOW*n) )
				{
					delete_gaps[i] = true;
					break;
				}
			}
		}
	}

	// Delete all gaps that have more than one inner gaps (and leave those inner ones!)
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i]) continue;

		const unsigned int ini_i = gaps_temp[i].ini;
		const unsigned int end_i = gaps_temp[i].end;
		unsigned int inner_gap_count = 0;

		for (unsigned int j=0;j<nTempGaps;j++)
		{
			if (i==j || delete_gaps[j]) continue;

			const unsigned int ini_j = gaps_temp[j].ini;
			const unsigned int end_j = gaps_temp[j].end;

			// j is_inside de i:
			if (ini_j>=ini_i && end_j<=end_i ) 
				if (++inner_gap_count>1) 
					break;
		}
		if (inner_gap_count>1) delete_gaps[i] = true;
	}

	// If there're still 
	const double MIN_GAPS_ENTR_DIST = (overall_max_dist-overall_min_dist) * GAPS_MINIMUM_DEPTH_DIFFERENCES_RATIO;
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i]) continue;

		const double	   ent_i = gaps_temp[i].entranceDistance;
		const unsigned int ini_i = gaps_temp[i].ini;
		const unsigned int end_i = gaps_temp[i].end;

		for (unsigned int j=0;j<nTempGaps;j++)
		{
			if (i==j || delete_gaps[j]) continue;

			const double	ent_j = gaps_temp[j].entranceDistance;
			const unsigned int ini_j = gaps_temp[j].ini;
			const unsigned int end_j = gaps_temp[j].end;

			// "j" is inside "i" and have almost the same entrance depths: -> delete "i"
			if (ini_j>=ini_i && end_j<=end_i &&
			    std::abs(ent_i-ent_j)< MIN_GAPS_ENTR_DIST )
			{
				delete_gaps[i]=true;
				break;
			}
		}
	}

	// Copy as result only those gaps not marked for deletion:
	// --------------------------------------------------------
	gaps_out.clear();
	gaps_out.reserve( nTempGaps/2 );
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i]) continue;

		// Compute the representative direction ("sector") for this gap:
		calcRepresentativeSectorForGap( gaps_temp[i], target, obstacles);

		gaps_out.push_back( gaps_temp[i] );
	}

}

/*---------------------------------------------------------------
						Search the best gap.
  ---------------------------------------------------------------*/
void  CHolonomicND::searchBestGap(
	const vector_double         & obstacles,
	const double                  maxObsRange,
	const TGapArray             & in_gaps,
	const mrpt::math::TPoint2D  & target,
	unsigned int                & out_selDirection,
	double                      & out_selEvaluation,
	TSituations                 & out_situation,
	double                      & out_riskEvaluation,
	CLogFileRecord_NDPtr	      log)
{
	// For evaluating the "risk":
	unsigned int min_risk_eval_sector = 0;
	unsigned int max_risk_eval_sector = obstacles.size()-1;
	const unsigned int target_sector  = direction2sector(atan2(target.y,target.x),obstacles.size());
	const double target_dist          = std::max(0.01,target.norm());
	// (Risk is evaluated at the end, for all the situations)

	// D1 : Straight path?
	// --------------------------------------------------------
	const int freeSectorsNearTarget = 10;  // 3
	bool theyAreFree = true, caseD1 = false;
	if (target_sector>static_cast<unsigned int>(freeSectorsNearTarget) &&
		target_sector<static_cast<unsigned int>(obstacles.size()-freeSectorsNearTarget) )
	{
		for (int j=-freeSectorsNearTarget;j<=freeSectorsNearTarget;j++)
				if (obstacles[ target_sector + j ]<0.95*target_dist)
						theyAreFree = false;
		caseD1 = theyAreFree;
	}

	if (caseD1)
	{
		// S1: Move straight towards target:
		out_selDirection	= target_sector;

		// In case of several paths, the shortest:
		out_selEvaluation   =	1.0 + std::max( 0.0, (maxObsRange - target_dist) / maxObsRange );
		out_situation		=	SITUATION_TARGET_DIRECTLY;
	}
	else
	{
		// Evaluate all gaps (if any):
		vector_double  gaps_evaluation;
		int            selected_gap		=-1;
		double         selected_gap_eval	= -100;

		evaluateGaps(
				obstacles,
				maxObsRange,
				in_gaps,
				target_sector,
				target_dist,
				gaps_evaluation );

		if (log) log->gaps_eval = gaps_evaluation;

		// D2: is there any gap "beyond" the target (and not too far away)?
		// -----------------------------------------------------------------
		for ( unsigned int i=0;i<in_gaps.size();i++ )
			if ( in_gaps[i].maxDistance >= target_dist &&
				 abs((int)(in_gaps[i].representative_sector-(int)target_sector)) <= (int)floor(MAX_SECTOR_DIST_FOR_D2_PERCENT * obstacles.size()) )
					if ( gaps_evaluation[i]>selected_gap_eval )
					{
						selected_gap_eval = gaps_evaluation[i];
						selected_gap = i;
					}


		// Keep the best gaps (if none was picked up to this point)
		if ( selected_gap==-1 )
			for ( unsigned int i=0;i<in_gaps.size();i++ )
				if ( gaps_evaluation[i]>selected_gap_eval )
				{
					selected_gap_eval = gaps_evaluation[i];
					selected_gap = i;
				}
		//  D3: Wasn't a good enough gap (or there were none)?
		// ------------------------------------------------------------
		if ( selected_gap_eval <= 0 )
		{
			// S2: No way found
			// ------------------------------------------------------
			out_selDirection	= 0;
			out_selEvaluation	= 0.0; // Worst case
			out_situation		= SITUATION_NO_WAY_FOUND;
		}
		else
		{
			// The selected gap:
			const TGap & gap = in_gaps[selected_gap];

			const unsigned int sectors_to_be_wide = round( WIDE_GAP_SIZE_PERCENT * obstacles.size() );

			out_selDirection	= in_gaps[selected_gap].representative_sector;
			out_selEvaluation	= selected_gap_eval;

			// D4: Is it a WIDE gap?
			// -----------------------------------------------------
			if ( (gap.end-gap.ini) < sectors_to_be_wide )
			{
				// S3: Narrow gap
				// -------------------------------------------
				out_situation	= SITUATION_SMALL_GAP;
			}
			else
			{
				// S4: Wide gap
				// -------------------------------------------
				out_situation	= SITUATION_WIDE_GAP;
			}

			// Evaluate the risk only within the gap:
			min_risk_eval_sector = gap.ini;
			max_risk_eval_sector = gap.end;
		}
	}

	// Evaluate short-term minimum distance to obstacles, in a small interval around the selected direction:
	const unsigned int risk_eval_nsectors = round( RISK_EVALUATION_SECTORS_PERCENT * obstacles.size() );
	const unsigned int sec_ini = std::max(min_risk_eval_sector, risk_eval_nsectors<out_selDirection ? out_selDirection-risk_eval_nsectors : 0 );
	const unsigned int sec_fin = std::min(max_risk_eval_sector, out_selDirection + risk_eval_nsectors );

	out_riskEvaluation = 0.0;
	for (unsigned int i=sec_ini;i<=sec_fin;i++) out_riskEvaluation+= obstacles[ i ];
	out_riskEvaluation /= (sec_fin - sec_ini + 1 );
}


/*---------------------------------------------------------------
	Fills in the representative sector
		field in the gap structure:
  ---------------------------------------------------------------*/
void  CHolonomicND::calcRepresentativeSectorForGap(
	TGap                        & gap,
	const mrpt::math::TPoint2D  & target,
	const vector_double         & obstacles)
{
	int sector;
	const unsigned int sectors_to_be_wide = round( WIDE_GAP_SIZE_PERCENT * obstacles.size());
	const unsigned int target_sector = direction2sector( atan2(target.y,target.x), obstacles.size() );

	if ( (gap.end-gap.ini) < sectors_to_be_wide )
	{
#if	1
		sector = round(0.5f*gap.ini+0.5f*gap.end);
#else
		double	min_dist_obs_near_ini=1, min_dist_obs_near_end=1;
		int		i;
		for ( i= gap.ini;i>=max(0,gap.ini-2);i--)
			min_dist_obs_near_ini = min(min_dist_obs_near_ini, obstacles[i]);
		for ( i= gap.end;i<=min((int)obstacles.size()-1,gap.end+2);i++)
			min_dist_obs_near_end = min(min_dist_obs_near_end, obstacles[i]);
		sector = round((min_dist_obs_near_ini*gap.ini+min_dist_obs_near_end*gap.end)/(min_dist_obs_near_ini+min_dist_obs_near_end));
#endif
	}
	else
	{
		const unsigned int dist_ini = mrpt::utils::abs_diff(target_sector, gap.ini );
		const unsigned int dist_end = mrpt::utils::abs_diff(target_sector, gap.end );
		int dir;
		if (dist_ini<dist_end) {
				sector = gap.ini;
				dir = +1; }
		else {
				sector = gap.end;
				dir = -1; }
		sector = sector + dir * sectors_to_be_wide/2 ;
	}

	keep_max(sector, 0);
	keep_min(sector, static_cast<int>(obstacles.size())-1 );

	gap.representative_sector = sector;
}



/*---------------------------------------------------------------
						Evaluate each gap
  ---------------------------------------------------------------*/
void  CHolonomicND::evaluateGaps(
	const vector_double	&obstacles,
	const double		maxObsRange,
	const TGapArray		&gaps,
	const unsigned int	target_sector,
	const double		target_dist,
	vector_double		&out_gaps_evaluation )
{
	out_gaps_evaluation.resize( gaps.size());

	double	targetAng = M_PI*(-1 + 2*(0.5+target_sector)/double(obstacles.size()));
	double	target_x =  target_dist*cos(targetAng);
	double	target_y =  target_dist*sin(targetAng);

	for (unsigned int i=0;i<gaps.size();i++)
	{
		// Short cut:
		const TGap *gap = &gaps[i];

		const double d = min3( 
			obstacles[ gap->representative_sector ],
			maxObsRange,
			0.95*target_dist );

		// The TP-Space representative coordinates for this gap:
		const double	phi = M_PI*(-1 + 2*(0.5+gap->representative_sector)/double(obstacles.size()));
		const double	x =  d*cos(phi);
		const double	y =  d*sin(phi);

		// Factor #1: Maximum reachable distance with this PTG:
		// -----------------------------------------------------
		// Calcular la distancia media a donde llego por este gap:
		double	meanDist = 0;
		for (unsigned int j=gap->ini;j<=gap->end;j++)
			meanDist+= obstacles[j];
		meanDist/= ( gap->end - gap->ini + 1);

		double factor1;
		if (mrpt::utils::abs_diff(gap->representative_sector,target_sector)<=1 && target_dist<1)
		      factor1 = std::min(target_dist,meanDist) / target_dist;
		else  factor1 = meanDist;

		// Factor #2: Distance to target in "sectors"
		// -------------------------------------------
		unsigned int dif = mrpt::utils::abs_diff(target_sector, gap->representative_sector );
		// Handle the -PI,PI circular topology:
		if (dif> (obstacles.size()/2) && (target_sector- obstacles.size()/2)*(gap->representative_sector-obstacles.size()/2)<0 )
			dif = obstacles.size() - dif;

		const double factor2= exp(-square( dif / (obstacles.size()*0.25))) ;

		// Factor #4: Stabilizing factor (hysteresis) to avoid quick switch among very similar paths:
		// -------------------------------------------
		double factor_AntiCab;
		if (m_last_selected_sector != std::numeric_limits<unsigned int>::max() )
		{
			unsigned int dist = mrpt::utils::abs_diff(m_last_selected_sector, gap->representative_sector);
			if (dist> (obstacles.size()/2) )
				dist = obstacles.size() - dist;

			factor_AntiCab = (dist > 0.10*obstacles.size()) ? 0.0:1.0;
		}
		else
		{
			factor_AntiCab = 0;
		}

		// Factor #3: Punish paths that take us far away wrt the target:
		// -----------------------------------------------------
		double	closestX,closestY;
		double dist_eucl = math::minimumDistanceFromPointToSegment(
			target_x, target_y, // Point
			0,0,  x,y,          // Segment
			closestX,closestY   // Out
			);

		const double factor3=  ( maxObsRange - std::min(maxObsRange ,dist_eucl) ) / maxObsRange;

		ASSERT_(factorWeights.size()==4);

		if ( obstacles[gap->representative_sector] < TOO_CLOSE_OBSTACLE ) // Too close to obstacles
				out_gaps_evaluation[i] = 0;
		else	out_gaps_evaluation[i] = (
				  factorWeights[0] * factor1 +
				  factorWeights[1] * factor2 +
				  factorWeights[2] * factor3 +
				  factorWeights[3] * factor_AntiCab ) / (math::sum(factorWeights)) ;
	} // for each gap
}

unsigned int CHolonomicND::direction2sector(const double a, const unsigned int N)
{
	const int idx = round(0.5*(N*(1+ mrpt::math::wrapToPi(a)/M_PI) - 1));
	if (idx<0) return 0;
	else return static_cast<unsigned int>(idx);
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLogFileRecord_ND::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << gaps_ini << gaps_end << gaps_eval;
		out << selectedSector << evaluation << riskEvaluation << (uint32_t) situation;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLogFileRecord_ND::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			int32_t	n;

			in >> n;
			gaps_ini.resize(n);
			gaps_end.resize(n);
			in.ReadBuffer( &(*gaps_ini.begin()), sizeof(gaps_ini[0]) * n );
			in.ReadBuffer( &(*gaps_end.begin()), sizeof(gaps_end[0]) * n );

			in >> n;
			gaps_eval.resize(n);
			in.ReadBuffer( &(*gaps_eval.begin()), sizeof(gaps_eval[0]) * n );

			in >> selectedSector >> evaluation >> riskEvaluation >> n;

			situation = (CHolonomicND::TSituations) n;
		} break;
	case 1:
		{
			uint32_t    n;
			in >> gaps_ini >> gaps_end >> gaps_eval;
			in >> selectedSector >> evaluation >> riskEvaluation >> n;
			situation  = (CHolonomicND::TSituations) n;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
