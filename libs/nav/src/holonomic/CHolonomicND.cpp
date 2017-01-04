/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_ND, CHolonomicLogFileRecord,mrpt::nav )
IMPLEMENTS_SERIALIZABLE( CHolonomicND, CAbstractHolonomicReactiveMethod,mrpt::nav)

/**  Initialize the parameters of the navigator, from some
*    configuration file, or default values if filename is set to NULL.
*/
CHolonomicND::CHolonomicND(const mrpt::utils::CConfigFileBase *INI_FILE ) :
	CAbstractHolonomicReactiveMethod("ND_CONFIG"),
	m_last_selected_sector ( std::numeric_limits<unsigned int>::max() )
{
	if (INI_FILE!=NULL)
		initialize( *INI_FILE );
}

void CHolonomicND::initialize(const mrpt::utils::CConfigFileBase &INI_FILE)
{
	options.loadFromConfigFile(INI_FILE, getConfigFileSectionName());
}

/*---------------------------------------------------------------
						Navigate
  ---------------------------------------------------------------*/
void  CHolonomicND::navigate(
	const mrpt::math::TPoint2D &target,
	const std::vector<double>	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord,
	const double    max_obstacle_dist,
	const mrpt::nav::ClearanceDiagram *clearance)
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
	gapsEstimator( obstacles, target, gaps);


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
		const double targetNearnessFactor = m_enableApproachTargetSlowDown ? 
			std::min(1.0, target.norm() / (options.TARGET_SLOW_APPROACHING_DISTANCE))
			:
			1.0;

		const double riskFactor = std::min(1.0, riskEvaluation / options.RISK_EVALUATION_DISTANCE );
		desiredSpeed = maxRobotSpeed * std::min(riskFactor,targetNearnessFactor);
	}

	m_last_selected_sector = selectedSector;

	// LOG --------------------------
	if (log)
	{
		// gaps:
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
				Find gaps in the obtacles (Beta version)
  ---------------------------------------------------------------*/
void  CHolonomicND::gapsEstimator(
	const std::vector<double>         & obstacles,
	const mrpt::math::TPoint2D  & target,
	TGapArray                   & gaps_out)
{
	const size_t n = obstacles.size();
	ASSERT_(n>2);

	// ================ Parameters ================
	const int     GAPS_MIN_WIDTH = ceil(n*0.01); // was: 3
	const double  GAPS_MIN_DEPTH_CONSIDERED = 0.6;
	const double  GAPS_MAX_RELATIVE_DEPTH = 0.5;
	// ============================================

	// Find the maximum distances to obstacles:
	// ----------------------------------------------------------
	float overall_max_dist = std::numeric_limits<float>::min(), overall_min_dist = std::numeric_limits<float>::max();
	for (size_t i=1;i<(n-1);i++)
	{
		mrpt::utils::keep_max(overall_max_dist, obstacles[i]);
		mrpt::utils::keep_min(overall_min_dist, obstacles[i]);
	}
	double max_depth = overall_max_dist - overall_min_dist;

	//  Build list of "GAPS":
	// --------------------------------------------------------
	TGapArray gaps_temp;
	gaps_temp.reserve( 150 );

	for (double threshold_ratio = 0.95;threshold_ratio>=0.05;threshold_ratio-=0.05)
	{
			const double  dist_threshold = threshold_ratio* overall_max_dist + (1.0f-threshold_ratio)*min(target.norm(), GAPS_MIN_DEPTH_CONSIDERED);

			bool    is_inside = false;
			size_t  sec_ini=0, sec_end=0;
			double  maxDist=0.;

			for (size_t i=0;i<n;i++)
			{
				if ( !is_inside && ( obstacles[i]>=dist_threshold) )	//A gap begins
				{
					sec_ini = i;
					maxDist = obstacles[i];
					is_inside = true;
				}
				else if (is_inside && (i==(n-1) || obstacles[i]<dist_threshold ))	//A gap ends
				{
					if (obstacles[i]<dist_threshold)
						sec_end = i-1;
					else
						sec_end = i;

					is_inside = false;

					if ( (sec_end-sec_ini) >= (size_t)GAPS_MIN_WIDTH )
					{
						// Add new gap:
						gaps_temp.resize( gaps_temp.size() + 1 );
						TGap	& newGap = *gaps_temp.rbegin();

						newGap.ini				= sec_ini;
						newGap.end				= sec_end;
						newGap.minDistance		= min( obstacles[sec_ini], obstacles[sec_end] );
						newGap.maxDistance		= maxDist;
					}
				}

				if (is_inside)
					maxDist = std::max( maxDist, obstacles[i] );
			}
	}

	//Start to filter the gap list
	//--------------------------------------------------------------

	const size_t nTempGaps = gaps_temp.size();

	std::vector<bool> delete_gaps;
	delete_gaps.assign( nTempGaps, false);

	// First, remove redundant gaps
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i] == 1)
			continue;

		for (size_t j=i+1;j<nTempGaps;j++)
		{
			if (gaps_temp[i].ini == gaps_temp[j].ini || gaps_temp[i].end == gaps_temp[j].end)
				delete_gaps[j] = 1;
		}
	}

	// Remove gaps with a big depth
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i] == 1)
			continue;

		if ((gaps_temp[i].maxDistance - gaps_temp[i].minDistance) > max_depth*GAPS_MAX_RELATIVE_DEPTH)
			delete_gaps[i] = 1;
	}

	//Delete gaps which contain more than one other gaps
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i])
			continue;

		unsigned int inner_gap_count = 0;

		for (unsigned int j=0;j<nTempGaps;j++)
		{
			if (i==j || delete_gaps[j])
				continue;

			// j is inside of i?
			if (gaps_temp[j].ini >= gaps_temp[i].ini && gaps_temp[j].end <= gaps_temp[i].end )
				if (++inner_gap_count>1)
				{
					delete_gaps[i] = 1;
					break;
				}
		}
	}

	//Delete gaps included in other gaps
	for (size_t i=0;i<nTempGaps;i++)
	{
		if (delete_gaps[i])
			continue;

		for (unsigned int j=0;j<nTempGaps;j++)
		{
			if (i==j || delete_gaps[j])
				continue;
			if (gaps_temp[i].ini <= gaps_temp[j].ini && gaps_temp[i].end >= gaps_temp[j].end)
				delete_gaps[j] = 1;
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
	const std::vector<double>         & obstacles,
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
	const int freeSectorsNearTarget = ceil(0.02*obstacles.size());
	bool theyAreFree = true, caseD1 = false;
	if (target_sector>static_cast<unsigned int>(freeSectorsNearTarget) &&
		target_sector<static_cast<unsigned int>(obstacles.size()-freeSectorsNearTarget) )
	{
		const double min_free_dist = std::min(1.05*target_dist, 0.95*maxObsRange);
		for (int j=-freeSectorsNearTarget;theyAreFree && j<=freeSectorsNearTarget;j++)
			if (obstacles[ (int(target_sector) + j) % obstacles.size()]<min_free_dist)
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
		std::vector<double>  gaps_evaluation;
		int            selected_gap			=-1;
		double         selected_gap_eval	= -100;

		evaluateGaps(
				obstacles,
				maxObsRange,
				in_gaps,
				target_sector,
				target_dist,
				gaps_evaluation );

		if (log) log->gaps_eval = gaps_evaluation;

		// D2: is there any gap "beyond" the target (and not too far away)?   (Not used)
		// ----------------------------------------------------------------

		//unsigned int dist;
		//for ( unsigned int i=0;i<in_gaps.size();i++ )
		//{
		//	dist = mrpt::utils::abs_diff(target_sector, in_gaps[i].representative_sector );
		//	if (dist > 0.5*obstacles.size())
		//		dist = obstacles.size() - dist;
		//
		//	if ( in_gaps[i].maxDistance >= target_dist && dist <= (int)floor(options.MAX_SECTOR_DIST_FOR_D2_PERCENT * obstacles.size()) )
		//
		//		if ( gaps_evaluation[i]>selected_gap_eval )
		//		{
		//			selected_gap_eval = gaps_evaluation[i];
		//			selected_gap = i;
		//		}
		//}


		// Keep the best gaps (if none was picked up to this point)
		if ( selected_gap==-1 )
			for ( unsigned int i=0;i<in_gaps.size();i++ )
				if ( gaps_evaluation[i]>selected_gap_eval )
				{
					selected_gap_eval = gaps_evaluation[i];
					selected_gap = i;
				}

		//  D3: Wasn't a good enough gap (or there were none)?
		// ----------------------------------------------------------
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

			const unsigned int sectors_to_be_wide = round( options.WIDE_GAP_SIZE_PERCENT * obstacles.size() );

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
	const unsigned int risk_eval_nsectors = round( options.RISK_EVALUATION_SECTORS_PERCENT * obstacles.size() );
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
	const std::vector<double>         & obstacles)
{
	int sector;
	const unsigned int sectors_to_be_wide = round( options.WIDE_GAP_SIZE_PERCENT * obstacles.size());
	const unsigned int target_sector = direction2sector( atan2(target.y,target.x), obstacles.size() );

	if ( (gap.end-gap.ini) < sectors_to_be_wide )	//Select the intermediate sector
	{
#if	1
		sector = round(0.5f*gap.ini+0.5f*gap.end);
#else
		float	min_dist_obs_near_ini=1, min_dist_obs_near_end=1;
		int		i;
		for ( i= gap.ini;i>=max(0,gap.ini-2);i--)
			min_dist_obs_near_ini = min(min_dist_obs_near_ini, obstacles[i]);
		for ( i= gap.end;i<=min((int)obstacles.size()-1,gap.end+2);i++)
			min_dist_obs_near_end = min(min_dist_obs_near_end, obstacles[i]);
		sector = round((min_dist_obs_near_ini*gap.ini+min_dist_obs_near_end*gap.end)/(min_dist_obs_near_ini+min_dist_obs_near_end));
#endif
	}
	else	//Select a sector close to the target but spaced "sectors_to_be_wide/2" from it
	{
		unsigned int dist_ini = mrpt::utils::abs_diff(target_sector, gap.ini );
		unsigned int dist_end = mrpt::utils::abs_diff(target_sector, gap.end );

		if (dist_ini > 0.5*obstacles.size())
			dist_ini = obstacles.size() - dist_ini;
		if (dist_end > 0.5*obstacles.size())
			dist_end = obstacles.size() - dist_end;

		int dir;
		if (dist_ini<dist_end) {
				sector = gap.ini;
				dir = +1; }
		else {
				sector = gap.end;
				dir = -1; }

		sector = sector + dir*static_cast<int>(sectors_to_be_wide)/2;
	}

	keep_max(sector, 0);
	keep_min(sector, static_cast<int>(obstacles.size())-1 );

	gap.representative_sector = sector;
}



/*---------------------------------------------------------------
						Evaluate each gap
  ---------------------------------------------------------------*/
void  CHolonomicND::evaluateGaps(
	const std::vector<double>	&obstacles,
	const double maxObsRange,
	const TGapArray		&gaps,
	const unsigned int	target_sector,
	const float		target_dist,
	std::vector<double>		&out_gaps_evaluation )
{
	out_gaps_evaluation.resize( gaps.size());

	double	targetAng = M_PI*(-1 + 2*(0.5+target_sector)/double(obstacles.size()));
	double	target_x =  target_dist*cos(targetAng);
	double	target_y =  target_dist*sin(targetAng);

	for (unsigned int i=0;i<gaps.size();i++)
	{
		// Short cut:
		const TGap *gap = &gaps[i];

		const float d = min3(
			obstacles[ gap->representative_sector ],
			maxObsRange,
			0.95*target_dist );

		// The TP-Space representative coordinates for this gap:
		const double	phi = M_PI*(-1 + 2*(0.5+gap->representative_sector)/double(obstacles.size()));
		const double	x =  d*cos(phi);
		const double	y =  d*sin(phi);


		// Factor #1: Maximum reachable distance with this PTG:
		// -----------------------------------------------------
		// It computes the average free distance of the gap:
		float	meanDist = 0.f;
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
		if (dif> 0.5*obstacles.size())
			dif = obstacles.size() - dif;

		const double factor2= exp(-square( dif / (obstacles.size()*0.25))) ;



		// Factor #3: Punish paths that take us far away wrt the target:  **** I don't understand it *********
		// -----------------------------------------------------
		double	closestX,closestY;
		double dist_eucl = math::minimumDistanceFromPointToSegment(
			target_x, target_y, // Point
			0,0,  x,y,          // Segment
			closestX,closestY   // Out
			);

		const float factor3 = ( maxObsRange - std::min(maxObsRange ,dist_eucl) ) / maxObsRange;



		// Factor #4: Stabilizing factor (hysteresis) to avoid quick switch among very similar paths:
		// ------------------------------------------------------------------------------------------
		double factor_AntiCab;


		if (m_last_selected_sector != std::numeric_limits<unsigned int>::max() )
		{
			unsigned int dist = mrpt::utils::abs_diff(m_last_selected_sector, gap->representative_sector);

			if (dist > unsigned(0.1*obstacles.size()))
				factor_AntiCab = 0.0;
			else
				factor_AntiCab = 1.0;
		}
		else
		{
			factor_AntiCab = 0;
		}


		ASSERT_(options.factorWeights.size()==4);

		if ( obstacles[gap->representative_sector] < options.TOO_CLOSE_OBSTACLE ) // Too close to obstacles
				out_gaps_evaluation[i] = 0;
		else	out_gaps_evaluation[i] = (
				  options.factorWeights[0] * factor1 +
				  options.factorWeights[1] * factor2 +
				  options.factorWeights[2] * factor3 +
				  options.factorWeights[3] * factor_AntiCab ) / (math::sum(options.factorWeights)) ;
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
void  CLogFileRecord_ND::writeToStream(mrpt::utils::CStream &out,int *version) const
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
void  CLogFileRecord_ND::readFromStream(mrpt::utils::CStream &in,int version)
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

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHolonomicND::TOptions::TOptions() :
	// Default values:
	TOO_CLOSE_OBSTACLE                 ( 0.15 ),
	WIDE_GAP_SIZE_PERCENT              ( 0.25 ),
	RISK_EVALUATION_SECTORS_PERCENT    ( 0.10 ),
	RISK_EVALUATION_DISTANCE           ( 0.4  ),
	MAX_SECTOR_DIST_FOR_D2_PERCENT     ( 0.25 ),
	TARGET_SLOW_APPROACHING_DISTANCE   ( 0.60 )
{
	factorWeights.resize(4);
	factorWeights[0]=1.0;
	factorWeights[1]=0.5;
	factorWeights[2]=2.0;
	factorWeights[3]=0.4;
}

void CHolonomicND::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(WIDE_GAP_SIZE_PERCENT,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(MAX_SECTOR_DIST_FOR_D2_PERCENT,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_SECTORS_PERCENT,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_DISTANCE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  source,section );

	source.read_vector(section,"factorWeights", std::vector<double>(), factorWeights, true );
	ASSERT_(factorWeights.size()==4);

	MRPT_END
}

void CHolonomicND::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg , const std::string &section) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	cfg.write(section,"WIDE_GAP_SIZE_PERCENT",WIDE_GAP_SIZE_PERCENT,   WN,WV, "");
	cfg.write(section,"MAX_SECTOR_DIST_FOR_D2_PERCENT",MAX_SECTOR_DIST_FOR_D2_PERCENT,   WN,WV, "");
	cfg.write(section,"RISK_EVALUATION_SECTORS_PERCENT",RISK_EVALUATION_SECTORS_PERCENT,   WN,WV, "");
	cfg.write(section,"RISK_EVALUATION_DISTANCE",RISK_EVALUATION_DISTANCE,   WN,WV, "In normalized ps-meters [0,1]");
	cfg.write(section,"TOO_CLOSE_OBSTACLE",TOO_CLOSE_OBSTACLE,   WN,WV, "For stopping gradually");
	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "In normalized ps-meters");

	ASSERT_EQUAL_(factorWeights.size(),4)
	cfg.write(section,"factorWeights",mrpt::format("%.2f %.2f %.2f %.2f",factorWeights[0],factorWeights[1],factorWeights[2],factorWeights[3]),   WN,WV, "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis");

	MRPT_END
}

void  CHolonomicND::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// Params:
		out << options.factorWeights << options.MAX_SECTOR_DIST_FOR_D2_PERCENT << 
			options.RISK_EVALUATION_DISTANCE << options.RISK_EVALUATION_SECTORS_PERCENT <<
			options.TARGET_SLOW_APPROACHING_DISTANCE << options.TOO_CLOSE_OBSTACLE << options.WIDE_GAP_SIZE_PERCENT;
		// State:
		out << m_last_selected_sector;
	}
}
void  CHolonomicND::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
		// Params:
		in >> options.factorWeights >> options.MAX_SECTOR_DIST_FOR_D2_PERCENT >> 
			options.RISK_EVALUATION_DISTANCE >> options.RISK_EVALUATION_SECTORS_PERCENT >>
			options.TARGET_SLOW_APPROACHING_DISTANCE >> options.TOO_CLOSE_OBSTACLE >> options.WIDE_GAP_SIZE_PERCENT;
		// State:
		in >> m_last_selected_sector;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

