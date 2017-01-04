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

void  CHolonomicFullEval::navigate(
	const mrpt::math::TPoint2D &target,
	const std::vector<double>	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord,
	const double    max_obstacle_dist,
	const mrpt::nav::ClearanceDiagram *clearance)
{
	using mrpt::utils::square;

	ASSERT_(clearance!=nullptr);

	CLogFileRecord_FullEvalPtr log;

	// Create a log record for returning data.
	if (!logRecord.present())
	{
		log = CLogFileRecord_FullEval::Create();
		logRecord = log;
	}

	const size_t nDirs = obstacles.size();
	const double target_dir = ::atan2(target.y,target.x);
	const unsigned int target_sector = mrpt::utils::round( -0.5 + (target_dir/M_PI + 1)*nDirs*0.5 );
	const double target_dist = target.norm();

	m_dirs_scores.resize(nDirs, options.factorWeights.size() + 2 );

	// TP-Obstacles in 2D:
	std::vector<mrpt::math::TPoint2D> obstacles_2d(nDirs);
	std::vector<double> k2dir(nDirs);

	for (unsigned int i=0;i<nDirs;i++)
	{
		k2dir[i] = M_PI*(-1 + 2*(0.5+i)/nDirs);
		obstacles_2d[i].x = obstacles[i] * cos(k2dir[i]);
		obstacles_2d[i].y = obstacles[i] * sin(k2dir[i]);
	}

	std::vector<double> dirs_eval(nDirs, .0);  // Evaluation of each possible direction
	const int NUM_FACTORS = 5;

	ASSERT_(options.factorWeights.size()==NUM_FACTORS);

	mrpt::math::TSegment2D sg;
	sg.point1.x = 0;
	sg.point1.y = 0;

	for (unsigned int i=0;i<nDirs;i++)
	{
		double scores[NUM_FACTORS];  // scores for each criterion

		if ( obstacles[i] < options.TOO_CLOSE_OBSTACLE ) // Too close to obstacles ?
		{
			for (int l=0;l<NUM_FACTORS;l++) m_dirs_scores(i,l)= .0;
			continue;
		}

		const double d = std::min(obstacles[i], 0.95*target_dist );

		// The TP-Space representative coordinates for this direction:
		const double x = d*cos(k2dir[i]);
		const double y = d*sin(k2dir[i]);

		// Factor #1: clearance
		// -----------------------------------------------------
		if (mrpt::utils::abs_diff(i,target_sector)<=1 && target_dist<1.0-options.TOO_CLOSE_OBSTACLE)
		scores[0] = std::min(target_dist,obstacles[i]) / (target_dist+options.TOO_CLOSE_OBSTACLE);
		else  scores[0] = std::max(0.0, obstacles[i]-options.TOO_CLOSE_OBSTACLE);

		// Factor #2: Closest approach to target along straight line (Euclidean)
		// -------------------------------------------
		{
			sg.point2.x = x;
			sg.point2.y = y;
			// Range of attainable values: 0=passes thru target. 2=opposite direction
			const double min_dist_target_along_path = sg.distance(target);

			scores[1] = 1.0 / (1.0 + square(min_dist_target_along_path) );
		}

		// Factor #3: Distance of end collision-free point to target (Euclidean)
		// -----------------------------------------------------
		{
			const double endpt_dist_to_target = (target - TPoint2D(x,y)).norm();
			const double endpt_dist_to_target_norm = std::min(1.0, endpt_dist_to_target );

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
#if 1
			double avr_path_clearance = clearance->getClearance(i /*path index*/, std::min(0.99, target_dist*0.95));
			scores[4] = avr_path_clearance;
#else
			size_t num_avrs = 0;
			for (
				auto it = ++clearance->raw_clearances[i /*path index*/].begin();
				it != clearance->raw_clearances[i].end() && it->first <= target_dist*1.5;
				++it, ++num_avrs)
			{
				const double clearance = it->second;
				avr_path_clearance += clearance;
			}
			scores[4] = num_avrs != 0 ? (avr_path_clearance / num_avrs) : 0.0;
#endif
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
	const unsigned int PHASE1_NUM_FACTORS = options.PHASE1_FACTORS.size(), PHASE2_NUM_FACTORS = options.PHASE2_FACTORS.size();
	ASSERT_(PHASE1_NUM_FACTORS>0);
	ASSERT_(PHASE2_NUM_FACTORS>0);

	double weights_sum_phase1=.0;
	for (unsigned int l : options.PHASE1_FACTORS) weights_sum_phase1+=options.factorWeights[l];
	ASSERT_(weights_sum_phase1>.0);
	const double weights_sum_phase1_inv = 1.0/weights_sum_phase1;

	double weights_sum_phase2=.0;
	for (unsigned int l : options.PHASE2_FACTORS) weights_sum_phase2+=options.factorWeights[l];
	ASSERT_(weights_sum_phase2>.0);
	const double weights_sum_phase2_inv = 1.0/weights_sum_phase2;

	std::vector<double> phase1_score(nDirs,.0);
	double phase1_min = std::numeric_limits<double>::max(), phase1_max=.0;


	for (unsigned int i=0;i<nDirs;i++)
	{
		// Sum up:
		for (unsigned int l : options.PHASE1_FACTORS)
			phase1_score[i] += options.factorWeights[l] * m_dirs_scores(i,l);

		phase1_score[i]*=weights_sum_phase1_inv;

		mrpt::utils::keep_max(phase1_max, phase1_score[i]);
		mrpt::utils::keep_min(phase1_min, phase1_score[i]);

		m_dirs_scores(i,NUM_FACTORS+0)= phase1_score[i];  // for logging.
	}

	// Phase 2:
	// ----------------------------------------------------------------------
	ASSERT_(options.PHASE1_THRESHOLD>.0 && options.PHASE1_THRESHOLD<1.0);

	const double p1_threshold = options.PHASE1_THRESHOLD * phase1_max + (1.0-options.PHASE1_THRESHOLD) * phase1_min;

	for (unsigned int i=0;i<nDirs;i++)
	{
		double this_dir_eval;

		if ( obstacles[i] < options.TOO_CLOSE_OBSTACLE ||  // Too close to obstacles ?
			phase1_score[i]<p1_threshold  // thresholding
			)
		{
			this_dir_eval = .0;
		}
		else
		{
			this_dir_eval = .0;
			for (unsigned int l : options.PHASE2_FACTORS) this_dir_eval += options.factorWeights[l] * std::log(m_dirs_scores(i, l)+1e-6);
			this_dir_eval = std::exp(this_dir_eval*weights_sum_phase2_inv);

			// Boost score of directions that: 
			if (
				target_sector==i &&                // take us straight to the target, and 
				obstacles[i]>=1.05*target_dist &&  // are safe (no collision), and
				m_dirs_scores(i, 4)> options.TOO_CLOSE_OBSTACLE*3.0
				)
				this_dir_eval+=std::max(0.0, 1.0-target_dist);
		}

		dirs_eval[i] = this_dir_eval;

		m_dirs_scores(i,NUM_FACTORS+1)= this_dir_eval; // for logging:
	} // for each direction

	// Give a chance for a derived class to manipulate these evaluations:
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
		desiredDirection = 0;
		desiredSpeed = 0;
	}
	else
	{
		// A valid movement:
		desiredDirection = (double)(M_PI*(-1 + 2*(0.5+best_dir)/((double)obstacles.size())));

		// Speed control: Reduction factors
		// ---------------------------------------------
		const double targetNearnessFactor = m_enableApproachTargetSlowDown ?
			std::min(1.0, target.norm() / (options.TARGET_SLOW_APPROACHING_DISTANCE))
			:
			1.0;


		const double obs_clearance = m_dirs_scores(best_dir, 4);
		const double obs_dist = std::min(obstacles[best_dir], obs_clearance);
		const double obs_dist_th = std::max(options.TOO_CLOSE_OBSTACLE, options.OBSTACLE_SLOW_DOWN_DISTANCE*max_obstacle_dist);
		double riskFactor = 1.0;
		if (obs_dist <= options.TOO_CLOSE_OBSTACLE) {
			riskFactor = 0.0;
		}
		else if (obs_dist< obs_dist_th && obs_dist_th>options.TOO_CLOSE_OBSTACLE)
		{
			riskFactor = (obs_dist - options.TOO_CLOSE_OBSTACLE) / (obs_dist_th - options.TOO_CLOSE_OBSTACLE);
		}
		desiredSpeed = maxRobotSpeed * std::min(riskFactor,targetNearnessFactor);
	}

	m_last_selected_sector = best_dir;

	// LOG --------------------------
	if (log)
	{
		log->selectedSector = best_dir;
		log->evaluation = best_eval;
		log->dirs_eval  = dirs_eval;
		log->dirs_scores = m_dirs_scores;
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
		*version = 0;
	else
	{
		out << dirs_eval << dirs_scores << selectedSector << evaluation;
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
		{
			in >> dirs_eval >> dirs_scores >> selectedSector >> evaluation;
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
	PHASE1_THRESHOLD( 0.75 )
{
	factorWeights = mrpt::math::make_vector<5,double>(1.0, 1.0, 1.0, 0.1, 1.0);
	factorNormalizeOrNot = mrpt::math::make_vector<5, int>(0, 0, 0, 0, 1);

	PHASE1_FACTORS = mrpt::math::make_vector<3,int>(0,1,2);
	PHASE2_FACTORS = mrpt::math::make_vector<2,int>(3,4);
}

void CHolonomicFullEval::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE, double, source, section);
	MRPT_LOAD_CONFIG_VAR(OBSTACLE_SLOW_DOWN_DISTANCE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(HYSTERESIS_SECTOR_COUNT,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(PHASE1_THRESHOLD,double,  source,section );

	source.read_vector(section,"factorWeights", std::vector<double>(), factorWeights, true );
	ASSERT_(factorWeights.size()==5);

	source.read_vector(section, "factorNormalizeOrNot", factorNormalizeOrNot, factorNormalizeOrNot);
	ASSERT_(factorNormalizeOrNot.size() == factorWeights.size());

	source.read_vector(section,"PHASE1_FACTORS", PHASE1_FACTORS, PHASE1_FACTORS );
	ASSERT_(PHASE1_FACTORS.size()>0);

	source.read_vector(section,"PHASE2_FACTORS", PHASE2_FACTORS, PHASE2_FACTORS );
	ASSERT_(PHASE2_FACTORS.size()>0);

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
	cfg.write(section,"PHASE1_THRESHOLD",PHASE1_THRESHOLD,   WN,WV, "Phase1 scores must be above this relative range threshold [0,1] to be considered in phase 2 (Default:`0.75`)");

	ASSERT_EQUAL_(factorWeights.size(),5)
	cfg.write(section,"factorWeights", mrpt::system::sprintf_container("%.2f ",factorWeights),   WN,WV, "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis, [4]=clearance along path");
	cfg.write(section,"factorNormalizeOrNot", mrpt::system::sprintf_container("%u ", factorNormalizeOrNot), WN, WV, "Normalize factors or not (1/0)");

	cfg.write(section,"PHASE1_FACTORS", mrpt::system::sprintf_container("%d ",PHASE1_FACTORS),   WN,WV, "Indices of the factors above to be considered in phase 1");
	cfg.write(section,"PHASE2_FACTORS", mrpt::system::sprintf_container("%d ",PHASE2_FACTORS),   WN,WV, "Indices of the factors above to be considered in phase 2");

	MRPT_END
}

void  CHolonomicFullEval::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		// Params:
		out << options.factorWeights << options.HYSTERESIS_SECTOR_COUNT <<
			options.PHASE1_FACTORS << options.PHASE2_FACTORS <<
			options.TARGET_SLOW_APPROACHING_DISTANCE << options.TOO_CLOSE_OBSTACLE << options.PHASE1_THRESHOLD
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
		{
		// Params:
		in >> options.factorWeights >> options.HYSTERESIS_SECTOR_COUNT >>
			options.PHASE1_FACTORS >> options.PHASE2_FACTORS >>
			options.TARGET_SLOW_APPROACHING_DISTANCE >> options.TOO_CLOSE_OBSTACLE >> options.PHASE1_THRESHOLD;
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
