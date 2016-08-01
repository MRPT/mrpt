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
#include <mrpt/math/utils.h>  // make_vector()
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
			for (int l=0;l<NUM_FACTORS;l++) m_dirs_scores(i,l)= .0;
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

		// Save stats for debugging:
		for (int l=0;l<NUM_FACTORS;l++) m_dirs_scores(i,l)= scores[l];
	}

	// Phase 1: average of normalized factors 1,2 & 3 and thresholding:
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

		MRPT_TODO("Save only if logging")
		m_dirs_scores(i,NUM_FACTORS+0)= phase1_score[i];
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
			for (unsigned int l : options.PHASE2_FACTORS) this_dir_eval+=options.factorWeights[l] * m_dirs_scores(i,l);
			this_dir_eval*=weights_sum_phase2_inv;

			// Boost score of directions that take us straight to the target:
			if (target_sector==i && obstacles[i]>=0.99*target_dist)
				this_dir_eval+=std::max(0.0, 1.0-target_dist);
		}

		dirs_eval[i] = this_dir_eval;

		// save for logging:
		m_dirs_scores(i,NUM_FACTORS+1)= this_dir_eval;
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
	HYSTERESIS_SECTOR_COUNT            ( 5 ),
	PHASE1_THRESHOLD( 0.75 )
{
	factorWeights = mrpt::math::make_vector<5,double>(1.0, 1.0, 1.0, 0.1, 1.0);

	PHASE1_FACTORS = mrpt::math::make_vector<3,int>(0,1,2);
	PHASE2_FACTORS = mrpt::math::make_vector<2,int>(3,4);
}

void CHolonomicFullEval::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(HYSTERESIS_SECTOR_COUNT,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(PHASE1_THRESHOLD,double,  source,section );

	source.read_vector(section,"factorWeights", std::vector<double>(), factorWeights, true );
	ASSERT_(factorWeights.size()==5);

	source.read_vector(section,"PHASE1_FACTORS", PHASE1_FACTORS, PHASE1_FACTORS );
	ASSERT_(PHASE1_FACTORS.size()>0);
	
	source.read_vector(section,"PHASE2_FACTORS", PHASE2_FACTORS, PHASE2_FACTORS );
	ASSERT_(PHASE2_FACTORS.size()>0);

	MRPT_END
}

void CHolonomicFullEval::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg , const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"TOO_CLOSE_OBSTACLE",TOO_CLOSE_OBSTACLE,   WN,WV, "Directions with collision-free distances below this threshold are not elegible.");
	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "Start to reduce speed when closer than this to target.");
	cfg.write(section,"HYSTERESIS_SECTOR_COUNT",HYSTERESIS_SECTOR_COUNT,   WN,WV, "Range of `sectors` (directions) for hysteresis over succesive timesteps");
	cfg.write(section,"PHASE1_THRESHOLD",PHASE1_THRESHOLD,   WN,WV, "Phase1 scores must be above this relative range threshold [0,1] to be considered in phase 2 (Default:`0.75`)");
	
	ASSERT_EQUAL_(factorWeights.size(),5)
	cfg.write(section,"factorWeights", mrpt::system::sprintf_container("%.2f ",factorWeights),   WN,WV, "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis, [4]=Clearness along path");

	cfg.write(section,"PHASE1_FACTORS", mrpt::system::sprintf_container("%d ",PHASE1_FACTORS),   WN,WV, "Indices of the factors above to be considered in phase 1");
	cfg.write(section,"PHASE2_FACTORS", mrpt::system::sprintf_container("%d ",PHASE2_FACTORS),   WN,WV, "Indices of the factors above to be considered in phase 2");

	MRPT_END
}
