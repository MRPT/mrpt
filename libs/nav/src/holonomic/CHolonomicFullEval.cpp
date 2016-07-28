/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <cmath>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_FullEval, CHolonomicLogFileRecord,mrpt::nav )


CHolonomicFullEval::CHolonomicFullEval(const mrpt::utils::CConfigFileBase *INI_FILE ) :
	m_last_selected_sector ( std::numeric_limits<unsigned int>::max() )
{
	if (INI_FILE!=NULL)
		initialize( *INI_FILE );
}


void  CHolonomicFullEval::navigate(
	const mrpt::math::TPoint2D &target,
	const std::vector<double>	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord,
	const double    max_obstacle_dist)
{
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

	log->dirs_scores.resize(nDirs, options.factorWeights.size() + 2 );

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
	double score_min[NUM_FACTORS] = { 100.0, 100.0, 100.0, 100.0, 100.0 };
	double score_max[NUM_FACTORS] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

	ASSERT_(options.factorWeights.size()==NUM_FACTORS);
	const double weights_sum = mrpt::math::sum(options.factorWeights);
	ASSERT_(weights_sum!=.0);
	const double weights_sum_inv = 1.0/weights_sum;

	mrpt::math::TSegment2D sg;
	sg.point1.x = 0;
	sg.point1.y = 0;

	for (unsigned int i=0;i<nDirs;i++)
	{
		double scores[NUM_FACTORS];  // scores for each criterion

		if ( obstacles[i] < options.TOO_CLOSE_OBSTACLE ) // Too close to obstacles ?
		{
			for (int l=0;l<NUM_FACTORS;l++) log->dirs_scores(i,l)= .0;
			continue;
		}

		const double d = std::min(obstacles[i], 0.95*target_dist );

		// The TP-Space representative coordinates for this direction:
		const double x = d*cos(k2dir[i]);
		const double y = d*sin(k2dir[i]);

		// Factor #1: Clearness
		// -----------------------------------------------------
		if (mrpt::utils::abs_diff(i,target_sector)<=1 && target_dist<1)
		      scores[0] = std::min(target_dist,obstacles[i]) / target_dist;
		else  scores[0] = obstacles[i];

		// Factor #2: Closest approach to target along straight line (Euclidean)
		// -------------------------------------------
		{
			sg.point2.x = x;
			sg.point2.y = y;
			// Range of attainable values: 0=passes thru target. 2=opposite direction
			const double min_dist_target_along_path = sg.distance(target);
			const double min_dist_target_along_path_norm = std::min(1.0, min_dist_target_along_path*0.5); // Now it is normalized [0,1]

			scores[1] = std::sqrt(1.01 - min_dist_target_along_path_norm); // the 1.01 instead of 1.0 is to be 100% sure we don't get a domain error in sqrt()
		}

		// Factor #3: Distance of end colission-free point to target (Euclidean)
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

		// Factor #5: Clearness to nearest obstacle along path
		// ------------------------------------------------------------------------------------------
		MRPT_TODO("Revise after refactoring all holo nav interface and impl. PTG nearness output?");
		{
			// "Temporary" (?) approximation:
			double avr_path_clearness = 1.0;
			int i0 = i-nDirs/4;
			int i1 = i+nDirs/4;
			for (int ki=i0;ki<=i1;ki++)
			{
				const int k = ((ki<0) ? (ki+nDirs) : ki) % nDirs;
				if (obstacles[k]<0.99*max_obstacle_dist)
					mrpt::utils::keep_min(avr_path_clearness, sg.distance(obstacles_2d[k]) );
			}
			scores[4] = avr_path_clearness;
		}

		// Keep min/max:
		for (int l=0;l<NUM_FACTORS;l++) 
		{
			mrpt::utils::keep_max(score_max[l], scores[l]);
			mrpt::utils::keep_min(score_min[l], scores[l]);
		}

		// Save stats for debugging:
		for (int l=0;l<NUM_FACTORS;l++) log->dirs_scores(i,l)= scores[l];
	}

	// Prepare normalization constants:
	double score_norm_f[NUM_FACTORS];
	for (int l=0;l<NUM_FACTORS;l++) {
		if (std::abs(score_max[l]-score_min[l])<1e-5)
		     score_norm_f[l] = 1.0;
		else score_norm_f[l] = 1.0/(score_max[l]-score_min[l]);
	}

	// Phase 1: average of normalized factors 1,2 & 3 and thresholding:
	// ----------------------------------------------------------------------
	const double weights_sum123 = options.factorWeights[0]+options.factorWeights[1]+options.factorWeights[2];
	ASSERT_(weights_sum123>.0);

	std::vector<double> phase1_score(nDirs,.0);
	double phase1_min = 1e6, phase1_max=.0;

	for (unsigned int i=0;i<nDirs;i++)
	{
		// Normalize:
		double scores_norm[3];
		for (int l=0;l<3;l++) 
			scores_norm[l] = (log->dirs_scores(i,l) - score_min[l]) * score_norm_f[l];

		// Sum up:
		for (int l=0;l<3;l++) phase1_score[i] += options.factorWeights[l] * scores_norm[l];
		phase1_score[i]/=weights_sum123;

		mrpt::utils::keep_max(phase1_max, phase1_score[i]);
		mrpt::utils::keep_min(phase1_min, phase1_score[i]);

		log->dirs_scores(i,NUM_FACTORS+0)= phase1_score[i];
	}

	// Phase 2: 
	// ----------------------------------------------------------------------
	const double F123_THRESHOLD_RATIO = 0.75;
	const double p1_threshold = F123_THRESHOLD_RATIO * phase1_max + (1.0-F123_THRESHOLD_RATIO) * phase1_min;
	const double weights_sum34 = options.factorWeights[3]+options.factorWeights[4];
	ASSERT_(weights_sum34>.0);
	const double weights_sum34_inv = 1.0/weights_sum34;

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
			this_dir_eval = (options.factorWeights[3] * log->dirs_scores(i,3) + options.factorWeights[4] * log->dirs_scores(i,4))*weights_sum34_inv;
		}

		dirs_eval[i] = this_dir_eval;

		// save for logging:
		log->dirs_scores(i,NUM_FACTORS+1)= this_dir_eval;
	} // for each direction

	// Search for best direction:
	unsigned int best_dir  = std::numeric_limits<unsigned int>::max();
	double       best_eval = .0;
	for (unsigned int i=0;i<nDirs;i++) {
		if (dirs_eval[i]>best_eval) {
			best_eval = dirs_eval[i];
			best_dir = i;
		}
	}

	if (best_eval=.0)
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
		const double targetNearnessFactor = std::min( 1.0, target.norm()/(options.TARGET_SLOW_APPROACHING_DISTANCE));
		const double riskFactor = 1.0;
		desiredSpeed = maxRobotSpeed * std::min(riskFactor,targetNearnessFactor);
	}

	m_last_selected_sector = best_dir;

	// LOG --------------------------
	if (log)
	{
		log->selectedSector = best_dir;
		log->evaluation = best_eval;
		log->dirs_eval  = dirs_eval;
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
	HYSTERESIS_SECTOR_COUNT            ( 5 )
{
	factorWeights.resize(5);
	factorWeights[0]=1.0;
	factorWeights[1]=0.5;
	factorWeights[2]=2.0;
	factorWeights[3]=0.1;
	factorWeights[4]=1.0;
}

void CHolonomicFullEval::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(HYSTERESIS_SECTOR_COUNT,double,  source,section );

	source.read_vector(section,"factorWeights", std::vector<double>(), factorWeights, true );
	ASSERT_(factorWeights.size()==5);

	MRPT_END
}

void CHolonomicFullEval::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg , const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"TOO_CLOSE_OBSTACLE",TOO_CLOSE_OBSTACLE,   WN,WV, "For stopping gradually");
	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "In normalized ps-meters");
	cfg.write(section,"HYSTERESIS_SECTOR_COUNT",HYSTERESIS_SECTOR_COUNT,   WN,WV, "Range of `sectors` (directions) for hysteresis over succesive timesteps");
	
	ASSERT_EQUAL_(factorWeights.size(),5)
	cfg.write(section,"factorWeights",mrpt::format("%.2f %.2f %.2f %.2f %.2f",factorWeights[0],factorWeights[1],factorWeights[2],factorWeights[3],factorWeights[4]),   WN,WV, "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis, [4]=Clearness along path");

	MRPT_END
}
