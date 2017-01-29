/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/utils.h>  // make_vector()
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/stl_serialization.h>
#include <cmath>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_FullEval, CHolonomicLogFileRecord,mrpt::nav )
IMPLEMENTS_SERIALIZABLE( CHolonomicFullEval, CAbstractHolonomicReactiveMethod,mrpt::nav)

CHolonomicFullEval::CHolonomicFullEval(const mrpt::utils::CConfigFileBase *INI_FILE ) :
	CAbstractHolonomicReactiveMethod("FULL_EVAL_CONFIG"),
	m_last_selected_sector ( std::numeric_limits<unsigned int>::max() )
{
	if (INI_FILE!=NULL)
		initialize( *INI_FILE );
}

void CHolonomicFullEval::initialize(const mrpt::utils::CConfigFileBase &INI_FILE)
{
	options.loadFromConfigFile(INI_FILE, getConfigFileSectionName());
}

void CHolonomicFullEval::navigate(const NavInput & ni, NavOutput &no)
{
	using mrpt::utils::square;

	ASSERT_(ni.clearance!=nullptr);

	// Create a log record for returning data.
	CLogFileRecord_FullEvalPtr log = CLogFileRecord_FullEval::Create();
	no.logRecord = log;

	const size_t nDirs = ni.obstacles.size();
	const double target_dir = ::atan2(ni.target.y, ni.target.x);
	const unsigned int target_sector = mrpt::utils::round( -0.5 + (target_dir/M_PI + 1)*nDirs*0.5 );
	const double target_dist = ni.target.norm();

	m_dirs_scores.resize(nDirs, options.factorWeights.size() + 2 );

	// TP-Obstacles in 2D:
	std::vector<mrpt::math::TPoint2D> obstacles_2d(nDirs);
	std::vector<double> k2dir(nDirs);

	for (unsigned int i=0;i<nDirs;i++)
	{
		k2dir[i] = CParameterizedTrajectoryGenerator::index2alpha(i, nDirs);
		obstacles_2d[i].x = ni.obstacles[i] * cos(k2dir[i]);
		obstacles_2d[i].y = ni.obstacles[i] * sin(k2dir[i]);
	}

	const int NUM_FACTORS = 5;

	ASSERT_(options.factorWeights.size()==NUM_FACTORS);

	for (unsigned int i=0;i<nDirs;i++)
	{
		double scores[NUM_FACTORS];  // scores for each criterion

		if (ni.obstacles[i] < options.TOO_CLOSE_OBSTACLE && !(i==target_sector &&ni.obstacles[i]>1.02*target_dist) ) // Too close to obstacles? (unless target is in between obstacles and the robot)
		{
			for (int l=0;l<NUM_FACTORS;l++) m_dirs_scores(i,l)= .0;
			continue;
		}

		const double d = std::min(ni.obstacles[i], 0.95*target_dist );

		// The TP-Space representative coordinates for this direction:
		const double x = d*cos(k2dir[i]);
		const double y = d*sin(k2dir[i]);

		// Factor #1: clearance
		// -----------------------------------------------------
		if (mrpt::utils::abs_diff(i, target_sector) <= 1 && target_dist < 1.0 - options.TOO_CLOSE_OBSTACLE &&ni.obstacles[i]>1.05*target_dist)
		{
			// Don't count obstacles ahead of the target.
			scores[0] = std::max(target_dist,ni.obstacles[i]) / (target_dist*1.05);
		}
		else
		{
			scores[0] = std::max(0.0,ni.obstacles[i] - options.TOO_CLOSE_OBSTACLE);
		}

		// Discount "circular loop aparent free distance" here, but don't count it for clearance, since those are not real obstacle points.
		if (getAssociatedPTG()) {
			const double max_real_freespace = getAssociatedPTG()->getActualUnloopedPathLength(i);
			const double max_real_freespace_norm = max_real_freespace / getAssociatedPTG()->getRefDistance();

			mrpt::utils::keep_min(scores[0], max_real_freespace_norm);
		}

		// Factor #2: Closest approach to target along straight line (Euclidean)
		// -------------------------------------------
		mrpt::math::TSegment2D sg;
		sg.point1.x = 0;
		sg.point1.y = 0;
		sg.point2.x = x;
		sg.point2.y = y;

		// Range of attainable values: 0=passes thru target. 2=opposite direction
		double min_dist_target_along_path = sg.distance(ni.target);

		// Idea: if this segment is taking us *away* from target, don't make the segment to start at (0,0), since all 
		// paths "running away" will then have identical minimum distances to target. Use the middle of the segment instead:
		const double endpt_dist_to_target = (ni.target - TPoint2D(x, y)).norm();
		const double endpt_dist_to_target_norm = std::min(1.0, endpt_dist_to_target);

		if (endpt_dist_to_target_norm > target_dist && endpt_dist_to_target_norm >= 0.95 * target_dist) {
			// path takes us away:
			sg.point1.x = x*0.5;
			sg.point1.y = y*0.5;
			min_dist_target_along_path = sg.distance(ni.target);
		}

		scores[1] = 1.0 / (1.0 + square(min_dist_target_along_path) );

		// Factor #3: Distance of end collision-free point to target (Euclidean)
		// -----------------------------------------------------
		{
			scores[2] = std::sqrt(1.01 - endpt_dist_to_target_norm); // the 1.01 instead of 1.0 is to be 100% sure we don't get a domain error in sqrt()
		}

		// Factor #4: Stabilizing factor (hysteresis) to avoid quick switch among very similar paths:
		// ------------------------------------------------------------------------------------------
		if (m_last_selected_sector != std::numeric_limits<unsigned int>::max() )
		{
			const unsigned int hist_dist = mrpt::utils::abs_diff(m_last_selected_sector, i);  // It's fine here to consider that -PI is far from +PI.

			if (hist_dist >= options.HYSTERESIS_SECTOR_COUNT)
			     scores[3] = square( 1.0-(hist_dist-options.HYSTERESIS_SECTOR_COUNT)/double(nDirs) );
			else scores[3] = 1.0;
		}
		else {
			scores[3] = 1.0;
		}

		// Factor #5: clearance to nearest obstacle along path
		// ------------------------------------------------------------------------------------------
		{
			double avr_path_clearance = ni.clearance->getClearance(i /*path index*/, std::min(0.99, target_dist*0.95));
			scores[4] = avr_path_clearance;
		}

		// Save stats for debugging:
		for (int l=0;l<NUM_FACTORS;l++) m_dirs_scores(i,l)= scores[l];
	}

	// Normalize factors?
	ASSERT_(options.factorNormalizeOrNot.size() == NUM_FACTORS);
	for (int l = 0; l < NUM_FACTORS; l++)
	{
		if (!options.factorNormalizeOrNot[l]) continue;

		const double mmax = m_dirs_scores.col(l).maxCoeff();
		const double mmin = m_dirs_scores.col(l).minCoeff();
		const double span = mmax - mmin;
		if (span <= .0) continue;

		m_dirs_scores.col(l).array() -= mmin;
		m_dirs_scores.col(l).array() /= span;
	}

	// Phase 1: average of PHASE1_FACTORS and thresholding:
	// ----------------------------------------------------------------------
	const unsigned int NUM_PHASES = options.PHASE_FACTORS.size();
	ASSERT_(NUM_PHASES>=1);

	std::vector<double> weights_sum_phase(NUM_PHASES, .0), weights_sum_phase_inv(NUM_PHASES);
	for (unsigned int i = 0; i < NUM_PHASES; i++) 
	{
		for (unsigned int l : options.PHASE_FACTORS[i]) weights_sum_phase[i] += options.factorWeights[l];
		ASSERT_(weights_sum_phase[i]>.0);
		weights_sum_phase_inv[i] = 1.0 / weights_sum_phase[i];
	}

	std::vector<std::vector<double> > phase_scores(NUM_PHASES, std::vector<double>(nDirs,.0) );
	double last_phase_threshold = -1.0; // don't threshold for the first phase

	for (unsigned int phase_idx = 0; phase_idx < NUM_PHASES; phase_idx++)
	{
		double phase_min = std::numeric_limits<double>::max(), phase_max = .0;

		for (unsigned int i = 0; i < nDirs; i++)
		{
			double this_dir_eval = 0;

			if (ni.obstacles[i] < options.TOO_CLOSE_OBSTACLE ||  // Too close to obstacles ?
				(phase_idx>0 && phase_scores[phase_idx-1][i]<last_phase_threshold)  // thresholding of the previous phase
				)
			{
				this_dir_eval = .0;
			}
			else
			{
				// Weighted avrg of factors:
				for (unsigned int l : options.PHASE_FACTORS[phase_idx])
					this_dir_eval += options.factorWeights[l] * std::log( std::max(1e-6, m_dirs_scores(i, l) ));

				this_dir_eval *= weights_sum_phase_inv[phase_idx];
				this_dir_eval = std::exp(this_dir_eval);

				// For the last phase, boost score of directions that: 
				if (phase_idx == (NUM_PHASES - 1) &&
					target_sector == i &&                // take us straight to the target, and 
					ni.obstacles[i] >= 1.05*target_dist &&  // are safe (no collision), and
					m_dirs_scores(i, 4) > options.TOO_CLOSE_OBSTACLE
					)
				{
					const double extra_score = (m_dirs_scores(i, 4) + 1.0) * std::max(0.0, 1.0 - target_dist);
					this_dir_eval += std::max(0.0, extra_score);
				}

			}
			phase_scores[phase_idx][i] = this_dir_eval;

			mrpt::utils::keep_max(phase_max, phase_scores[phase_idx][i]);
			mrpt::utils::keep_min(phase_min, phase_scores[phase_idx][i]);

		} // for each direction

		if (phase_idx < (NUM_PHASES - 1))
		{
			ASSERT_(options.PHASE_THRESHOLDS.size() >= (NUM_PHASES - 1));
			ASSERT_(options.PHASE_THRESHOLDS[phase_idx] > .0 && options.PHASE_THRESHOLDS[phase_idx] < 1.0);

			last_phase_threshold = options.PHASE_THRESHOLDS[phase_idx] * phase_max + (1.0 - options.PHASE_THRESHOLDS[phase_idx]) * phase_min;
		}
	} // end for each phase
	
	// Give a chance for a derived class to manipulate the final evaluations:
	auto & dirs_eval = *phase_scores.rbegin();

	postProcessDirectionEvaluations(dirs_eval);

	// Search for best direction:
	unsigned int best_dir  = std::numeric_limits<unsigned int>::max();
	double       best_eval = .0;
	for (unsigned int i=0;i<nDirs;i++) {
		if (dirs_eval[i]>best_eval) {
			best_eval = dirs_eval[i];
			best_dir = i;
		}
	}

	if (best_eval==.0)
	{
		// No way found!
		no.desiredDirection = 0;
		no.desiredSpeed = 0;
	}
	else
	{
		// A valid movement:
		no.desiredDirection = CParameterizedTrajectoryGenerator::index2alpha(best_dir, ni.obstacles.size());

		// Speed control: Reduction factors
		// ---------------------------------------------
		const double targetNearnessFactor = m_enableApproachTargetSlowDown ?
			std::min(1.0, ni.target.norm() / (options.TARGET_SLOW_APPROACHING_DISTANCE))
			:
			1.0;


		const double obs_clearance = m_dirs_scores(best_dir, 4);
		const double obs_dist = std::min(ni.obstacles[best_dir], obs_clearance);
		const double obs_dist_th = std::max(options.TOO_CLOSE_OBSTACLE, options.OBSTACLE_SLOW_DOWN_DISTANCE*ni.maxObstacleDist);
		double riskFactor = 1.0;
		if (obs_dist <= options.TOO_CLOSE_OBSTACLE) {
			riskFactor = 0.0;
		}
		else if (obs_dist< obs_dist_th && obs_dist_th>options.TOO_CLOSE_OBSTACLE)
		{
			riskFactor = (obs_dist - options.TOO_CLOSE_OBSTACLE) / (obs_dist_th - options.TOO_CLOSE_OBSTACLE);
		}
		no.desiredSpeed = ni.maxRobotSpeed * std::min(riskFactor,targetNearnessFactor);
	}

	m_last_selected_sector = best_dir;

	// LOG --------------------------
	if (log)
	{
		log->selectedSector = best_dir;
		log->evaluation = best_eval;

		log->dirs_eval = phase_scores;

		if (options.LOG_SCORE_MATRIX) {
			log->dirs_scores = m_dirs_scores;
		}
	}
}


unsigned int CHolonomicFullEval::direction2sector(const double a, const unsigned int N)
{
	const int idx = round(0.5*(N*(1+ mrpt::math::wrapToPi(a)/M_PI) - 1));
	if (idx<0) return 0;
	else return static_cast<unsigned int>(idx);
}

void  CLogFileRecord_FullEval::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		out << CHolonomicLogFileRecord::dirs_eval << dirs_scores << selectedSector << evaluation;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLogFileRecord_FullEval::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
		if (version >= 2)
		{
			in >> CHolonomicLogFileRecord::dirs_eval;
		}
		else
		{
			CHolonomicLogFileRecord::dirs_eval.resize(2);
			in >> CHolonomicLogFileRecord::dirs_eval[0];
			if (version >= 1) {
				in >> CHolonomicLogFileRecord::dirs_eval[1];
			}
		}
		in >> dirs_scores >> selectedSector >> evaluation;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHolonomicFullEval::TOptions::TOptions() :
	// Default values:
	TOO_CLOSE_OBSTACLE                 ( 0.15 ),
	TARGET_SLOW_APPROACHING_DISTANCE   ( 0.60 ),
	OBSTACLE_SLOW_DOWN_DISTANCE        ( 0.15 ),
	HYSTERESIS_SECTOR_COUNT            ( 5 ),
	LOG_SCORE_MATRIX(false)
{
	factorWeights = mrpt::math::make_vector<5,double>(1.0, 1.0, 1.0, 0.1, 1.0);
	factorNormalizeOrNot = mrpt::math::make_vector<5, int>(0, 0, 0, 0, 1);

	PHASE_THRESHOLDS = mrpt::math::make_vector<2, double>(0.6);

	PHASE_FACTORS.resize(3);
	PHASE_FACTORS[0] = mrpt::math::make_vector<1, int>(0);
	PHASE_FACTORS[1] = mrpt::math::make_vector<1, int>(1);
	PHASE_FACTORS[2] = mrpt::math::make_vector<1, int>(4);
}

void CHolonomicFullEval::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &c,const std::string &s)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  c,s );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE, double, c, s);
	MRPT_LOAD_CONFIG_VAR(OBSTACLE_SLOW_DOWN_DISTANCE,double,  c,s );
	MRPT_LOAD_CONFIG_VAR(HYSTERESIS_SECTOR_COUNT,double,  c,s );
	MRPT_LOAD_CONFIG_VAR(LOG_SCORE_MATRIX,bool,  c,s );

	c.read_vector(s,"factorWeights", std::vector<double>(), factorWeights, true );
	ASSERT_(factorWeights.size()==5);

	c.read_vector(s, "factorNormalizeOrNot", factorNormalizeOrNot, factorNormalizeOrNot);
	ASSERT_(factorNormalizeOrNot.size() == factorWeights.size());

	// Phases:
	int PHASE_COUNT = 0;
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(PHASE_COUNT,int, c,s);

	PHASE_FACTORS.resize(PHASE_COUNT);
	PHASE_THRESHOLDS.resize(PHASE_COUNT-1);
	for (int i = 0; i < PHASE_COUNT; i++)
	{
		c.read_vector(s, mrpt::format("PHASE%i_FACTORS",i + 1), PHASE_FACTORS[i], PHASE_FACTORS[i], true);
		ASSERT_(!PHASE_FACTORS[i].empty());
		
		if (i != (PHASE_COUNT - 1)) // last stage does not need threshold, since its evaluation is the final output.
		{
			PHASE_THRESHOLDS[i] = c.read_double(s, mrpt::format("PHASE%i_THRESHOLD", i + 1),.0, true);
			ASSERT_(PHASE_THRESHOLDS[i]>=.0 && PHASE_THRESHOLDS[i]<=1.0);
		}
	}

	MRPT_END
}

void CHolonomicFullEval::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg , const std::string &section) const
{
	MRPT_START
	const int WN = 25, WV = 30;

	cfg.write(section,"TOO_CLOSE_OBSTACLE",TOO_CLOSE_OBSTACLE,   WN,WV, "Directions with collision-free distances below this threshold are not elegible.");
	cfg.write(section, "TARGET_SLOW_APPROACHING_DISTANCE", TARGET_SLOW_APPROACHING_DISTANCE, WN, WV, "Start to reduce speed when closer than this to target.");
	cfg.write(section,"OBSTACLE_SLOW_DOWN_DISTANCE", OBSTACLE_SLOW_DOWN_DISTANCE,   WN,WV, "Start to reduce speed when clearance is below this value ([0,1] ratio wrt obstacle reference/max distance)");
	cfg.write(section,"HYSTERESIS_SECTOR_COUNT",HYSTERESIS_SECTOR_COUNT,   WN,WV, "Range of `sectors` (directions) for hysteresis over succesive timesteps");
	cfg.write(section,"LOG_SCORE_MATRIX", LOG_SCORE_MATRIX,   WN,WV, "Log entire score matrix");

	ASSERT_EQUAL_(factorWeights.size(),5)
	cfg.write(section,"factorWeights", mrpt::system::sprintf_container("%.2f ",factorWeights),   WN,WV, "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis, [4]=clearance along path");
	cfg.write(section,"factorNormalizeOrNot", mrpt::system::sprintf_container("%u ", factorNormalizeOrNot), WN, WV, "Normalize factors or not (1/0)");

	
	cfg.write(section, "PHASE_COUNT", PHASE_FACTORS.size(), WN, WV, "Number of evaluation phases to run (params for each phase below)");

	for (unsigned int i = 0; i < PHASE_FACTORS.size(); i++)
	{
		cfg.write(section, mrpt::format("PHASE%u_THRESHOLD",i+1), PHASE_THRESHOLDS[i], WN, WV, "Phase scores must be above this relative range threshold [0,1] to be considered in next phase (Default:`0.75`)");
		cfg.write(section, mrpt::format("PHASE%u_FACTORS", i + 1), mrpt::system::sprintf_container("%d ", PHASE_FACTORS[i]), WN, WV, "Indices of the factors above to be considered in this phase");
	}

	MRPT_END
}

void  CHolonomicFullEval::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 3;
	else
	{
		// Params:
		out << options.factorWeights << options.HYSTERESIS_SECTOR_COUNT <<
			options.PHASE_FACTORS << // v3
			options.TARGET_SLOW_APPROACHING_DISTANCE << options.TOO_CLOSE_OBSTACLE << options.PHASE_THRESHOLDS // v3
			<< options.OBSTACLE_SLOW_DOWN_DISTANCE // v1
			<< options.factorNormalizeOrNot; // v2

		// State:
		out << m_last_selected_sector;
	}
}
void  CHolonomicFullEval::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
		// Params:
		in >> options.factorWeights >> options.HYSTERESIS_SECTOR_COUNT;
		
		if (version>=3) {
			in >> options.PHASE_FACTORS;
		} 
		else {
			options.PHASE_THRESHOLDS.resize(2);
			in >> options.PHASE_FACTORS[0] >> options.PHASE_FACTORS[1];
		}
		in >> options.TARGET_SLOW_APPROACHING_DISTANCE >> options.TOO_CLOSE_OBSTACLE;
		
		if (version >= 3) {
			in >> options.PHASE_THRESHOLDS;
		}
		else
		{
			options.PHASE_THRESHOLDS.resize(1);
			in >> options.PHASE_THRESHOLDS[0];
		}

		if (version >= 1)
			in >> options.OBSTACLE_SLOW_DOWN_DISTANCE;
		if (version >= 2)
			in >> options.factorNormalizeOrNot;

		// State:
		in >> m_last_selected_sector;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CHolonomicFullEval::postProcessDirectionEvaluations(std::vector<double> &dir_evals)
{
	// Default: do nothing
}
