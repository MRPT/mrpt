/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "nav-precomp.h" // Precomp header

#include <cmath>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/math/utils.h> // make_vector()
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/stl_serialization.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CLogFileRecord_FullEval, CHolonomicLogFileRecord,
						mrpt::nav)
IMPLEMENTS_SERIALIZABLE(CHolonomicFullEval, CAbstractHolonomicReactiveMethod,
						mrpt::nav)

const unsigned int INVALID_K = std::numeric_limits<unsigned int>::max();

const unsigned NUM_FACTORS = 7U;

CHolonomicFullEval::CHolonomicFullEval(
	const mrpt::utils::CConfigFileBase *INI_FILE)
	: CAbstractHolonomicReactiveMethod("CHolonomicFullEval"),
	  m_last_selected_sector(std::numeric_limits<unsigned int>::max()) {
  if (INI_FILE != NULL)
	initialize(*INI_FILE);
}

void CHolonomicFullEval::saveConfigFile(mrpt::utils::CConfigFileBase &c) const {
  options.saveToConfigFile(c, getConfigFileSectionName());
}

void CHolonomicFullEval::initialize(const mrpt::utils::CConfigFileBase &c) {
  options.loadFromConfigFile(c, getConfigFileSectionName());
}

struct TGap {
  int k_from, k_to;
  double min_eval, max_eval;
  bool contains_target_k;
  int k_best_eval; //!< Direction with the best evaluation inside the gap.

  TGap()
	  : k_from(-1), k_to(-1), min_eval(std::numeric_limits<double>::max()),
		max_eval(-std::numeric_limits<double>::max()), contains_target_k(false),
		k_best_eval(-1) {}
};

void CHolonomicFullEval::evalSingleTarget(unsigned int target_idx,
										  const NavInput &ni, EvalOutput &eo) {
  ASSERT_(target_idx < ni.targets.size());
  const auto target = ni.targets[target_idx];

  using mrpt::utils::square;

  eo = EvalOutput();

  const auto ptg = getAssociatedPTG();
  const size_t nDirs = ni.obstacles.size();

  const double target_dir = ::atan2(target.y, target.x);
  const unsigned int target_k =
	  CParameterizedTrajectoryGenerator::alpha2index(target_dir, nDirs);
  const double target_dist = target.norm();

  m_dirs_scores.resize(nDirs, options.factorWeights.size() + 2);

  // TP-Obstacles in 2D:
  std::vector<mrpt::math::TPoint2D> obstacles_2d(nDirs);

  mrpt::obs::T2DScanProperties sp;
  sp.aperture = 2.0 * M_PI;
  sp.nRays = nDirs;
  sp.rightToLeft = true;
  const auto &sc_lut = m_sincos_lut.getSinCosForScan(sp);

  for (unsigned int i = 0; i < nDirs; i++) {
	obstacles_2d[i].x = ni.obstacles[i] * sc_lut.ccos[i];
	obstacles_2d[i].y = ni.obstacles[i] * sc_lut.csin[i];
  }

  // Sanity checks:
  ASSERT_EQUAL_(options.factorWeights.size(), NUM_FACTORS);
  ASSERT_ABOVE_(nDirs, 3);

  for (unsigned int i = 0; i < nDirs; i++) {
	double scores[NUM_FACTORS]; // scores for each criterion

	// Too close to obstacles? (unless target is in between obstacles and
	// the robot)
	if (ni.obstacles[i] < options.TOO_CLOSE_OBSTACLE &&
		!(i == target_k && ni.obstacles[i] > 1.02 * target_dist)) {
	  for (size_t l = 0; l < NUM_FACTORS; l++)
		m_dirs_scores(i, l) = .0;
	  continue;
	}

	const double d = std::min(ni.obstacles[i], 0.95 * target_dist);

	// The TP-Space representative coordinates for this direction:
	const double x = d * sc_lut.ccos[i];
	const double y = d * sc_lut.csin[i];

	// Factor [0]: collision-free distance
	// -----------------------------------------------------
	if (mrpt::utils::abs_diff(i, target_k) <= 1 &&
		target_dist < 1.0 - options.TOO_CLOSE_OBSTACLE &&
		ni.obstacles[i] > 1.05 * target_dist) {
	  // Don't count obstacles ahead of the target.
	  scores[0] = std::max(target_dist, ni.obstacles[i]) / (target_dist * 1.05);
	} else {
	  scores[0] = std::max(0.0, ni.obstacles[i] - options.TOO_CLOSE_OBSTACLE);
	}

	// Discount "circular loop aparent free distance" here, but don't count
	// it for clearance, since those are not real obstacle points.
	if (ptg != nullptr) {
	  const double max_real_freespace = ptg->getActualUnloopedPathLength(i);
	  const double max_real_freespace_norm =
		  max_real_freespace / ptg->getRefDistance();

	  mrpt::utils::keep_min(scores[0], max_real_freespace_norm);
	}

	// Factor [1]: Closest approach to target along straight line
	// (Euclidean)
	// -------------------------------------------
	mrpt::math::TSegment2D sg;
	sg.point1.x = 0;
	sg.point1.y = 0;
	sg.point2.x = x;
	sg.point2.y = y;

	// Range of attainable values: 0=passes thru target. 2=opposite
	// direction
	double min_dist_target_along_path = sg.distance(target);

	// Idea: if this segment is taking us *away* from target, don't make
	// the segment to start at (0,0), since all paths "running away"
	// will then have identical minimum distances to target. Use the
	// middle of the segment instead:
	const double endpt_dist_to_target = (target - TPoint2D(x, y)).norm();
	const double endpt_dist_to_target_norm =
		std::min(1.0, endpt_dist_to_target);

	if ((endpt_dist_to_target_norm > target_dist &&
		 endpt_dist_to_target_norm >= 0.95 * target_dist) &&
		/* the path does not get any closer to trg */
		min_dist_target_along_path >
			1.05 * std::min(target_dist, endpt_dist_to_target_norm)) {
	  // path takes us away or way blocked:
	  sg.point1.x = x * 0.5;
	  sg.point1.y = y * 0.5;
	  min_dist_target_along_path = sg.distance(target);
	}

	scores[1] = 1.0 / (1.0 + square(min_dist_target_along_path));

	// Factor [2]: Distance of end collision-free point to target
	// (Euclidean)
	// Factor [5]: idem (except: no decimation afterwards)
	// -----------------------------------------------------
	scores[2] = std::sqrt(1.01 - endpt_dist_to_target_norm);
	scores[5] = scores[2];
	// the 1.01 instead of 1.0 is to be 100% sure we don't get a domain
	// error in sqrt()

	// Factor [3]: Stabilizing factor (hysteresis) to avoid quick switch
	// among very similar paths:
	// ------------------------------------------------------------------------------------------
	if (m_last_selected_sector != std::numeric_limits<unsigned int>::max()) {
	  // It's fine here to consider that -PI is far from +PI.
	  const unsigned int hist_dist =
		  mrpt::utils::abs_diff(m_last_selected_sector, i);

	  if (hist_dist >= options.HYSTERESIS_SECTOR_COUNT)
		scores[3] = square(1.0 - (hist_dist - options.HYSTERESIS_SECTOR_COUNT) /
									 double(nDirs));
	  else
		scores[3] = 1.0;
	} else {
	  scores[3] = 1.0;
	}

	// Factor [4]: clearance to nearest obstacle along path
	// Use TP-obstacles instead of real obstacles in Workspace since
	// it's way faster, despite being an approximation:
	// -------------------------------------------------------------------
	{
	  sg.point1.x = 0;
	  sg.point1.y = 0;
	  sg.point2.x = x;
	  sg.point2.y = y;

	  double &closest_obs = scores[4];
	  closest_obs = 1.0;

	  // eval obstacles within a certain region of this "i" direction only
	  const int W = std::max(1, mrpt::utils::round(nDirs * 0.1));
	  const int i_min = std::max(0, static_cast<int>(i) - W);
	  const int i_max =
		  std::min(static_cast<int>(nDirs) - 1, static_cast<int>(i) + W);
	  for (int oi = i_min; oi <= i_max; oi++) {
		// "no obstacle" (norm_dist=1.0) doesn't count as a real obs:
		if (ni.obstacles[oi] >= 0.99)
		  continue;
		mrpt::utils::keep_min(closest_obs, sg.distance(obstacles_2d[oi]));
	  }
	}

	// Factor [6]: Direct distance in "sectors":
	// -------------------------------------------------------------------
	scores[6] =
		1.0 / (1.0 + mrpt::utils::square((4.0 / nDirs) *
										 mrpt::utils::abs_diff(i, target_k)));

	// If target is not directly reachable for this i-th direction, decimate
	// its scorings:
	if (target_dist < 1.0 - options.TOO_CLOSE_OBSTACLE &&
		ni.obstacles[i] < 1.01 * target_dist) {
	  // this direction cannot reach target, so assign a low score:
	  scores[1] *= 0.1;
	  scores[2] *= 0.1;
	  scores[6] *= 0.1;
	}

	// Save stats for debugging:
	for (size_t l = 0; l < NUM_FACTORS; l++)
	  m_dirs_scores(i, l) = scores[l];

  } // end for each direction "i"

  // Normalize factors?
  ASSERT_(options.factorNormalizeOrNot.size() == NUM_FACTORS);
  for (size_t l = 0; l < NUM_FACTORS; l++) {
	if (!options.factorNormalizeOrNot[l])
	  continue;

	const double mmax = m_dirs_scores.col(l).maxCoeff();
	const double mmin = m_dirs_scores.col(l).minCoeff();
	const double span = mmax - mmin;
	if (span <= .0)
	  continue;

	m_dirs_scores.col(l).array() -= mmin;
	m_dirs_scores.col(l).array() /= span;
  }

  // Phase 1: average of PHASE1_FACTORS and thresholding:
  // ----------------------------------------------------------------------
  const unsigned int NUM_PHASES = options.PHASE_FACTORS.size();
  ASSERT_(NUM_PHASES >= 1);

  std::vector<double> weights_sum_phase(NUM_PHASES, .0),
	  weights_sum_phase_inv(NUM_PHASES);
  for (unsigned int i = 0; i < NUM_PHASES; i++) {
	for (unsigned int l : options.PHASE_FACTORS[i])
	  weights_sum_phase[i] += options.factorWeights.at(l);
	ASSERT_(weights_sum_phase[i] > .0);
	weights_sum_phase_inv[i] = 1.0 / weights_sum_phase[i];
  }

  eo.phase_scores = std::vector<std::vector<double>>(
	  NUM_PHASES, std::vector<double>(nDirs, .0));
  auto &phase_scores = eo.phase_scores; // shortcut
  double last_phase_threshold = -1.0;   // don't threshold for the first phase

  for (unsigned int phase_idx = 0; phase_idx < NUM_PHASES; phase_idx++) {
	double phase_min = std::numeric_limits<double>::max(), phase_max = .0;

	for (unsigned int i = 0; i < nDirs; i++) {
	  double this_dir_eval = 0;

	  if (ni.obstacles[i] <
			  options.TOO_CLOSE_OBSTACLE || // Too close to obstacles ?
		  (phase_idx > 0 &&
		   phase_scores[phase_idx - 1][i] <
			   last_phase_threshold) // thresholding of the previous
									 // phase
	  ) {
		this_dir_eval = .0;
	  } else {
		// Weighted avrg of factors:
		for (unsigned int l : options.PHASE_FACTORS[phase_idx])
		  this_dir_eval += options.factorWeights.at(l) *
						   std::log(std::max(1e-6, m_dirs_scores(i, l)));

		this_dir_eval *= weights_sum_phase_inv[phase_idx];
		this_dir_eval = std::exp(this_dir_eval);
	  }
	  phase_scores[phase_idx][i] = this_dir_eval;

	  mrpt::utils::keep_max(phase_max, phase_scores[phase_idx][i]);
	  mrpt::utils::keep_min(phase_min, phase_scores[phase_idx][i]);

	} // for each direction

	ASSERT_(options.PHASE_THRESHOLDS.size() == NUM_PHASES);
	ASSERT_(options.PHASE_THRESHOLDS[phase_idx] > .0 &&
			options.PHASE_THRESHOLDS[phase_idx] < 1.0);

	last_phase_threshold =
		options.PHASE_THRESHOLDS[phase_idx] * phase_max +
		(1.0 - options.PHASE_THRESHOLDS[phase_idx]) * phase_min;
  } // end for each phase

  // Give a chance for a derived class to manipulate the final evaluations:
  auto &dirs_eval = phase_scores.back();

  postProcessDirectionEvaluations(dirs_eval, ni, target_idx);

  // Recalculate the threshold just in case the postProcess function above
  // changed things:
  {
	double phase_min = std::numeric_limits<double>::max(), phase_max = .0;
	for (unsigned int i = 0; i < nDirs; i++) {
	  mrpt::utils::keep_max(phase_max, dirs_eval[i]);
	  mrpt::utils::keep_min(phase_min, dirs_eval[i]);
	}
	last_phase_threshold = options.PHASE_THRESHOLDS.back() * phase_max +
						   (1.0 - options.PHASE_THRESHOLDS.back()) * phase_min;
  }

  // Thresholding:
  for (unsigned int i = 0; i < nDirs; i++) {
	double &val = dirs_eval[i];
	if (val < last_phase_threshold)
	  val = .0;
  }
}

void CHolonomicFullEval::navigate(const NavInput &ni, NavOutput &no) {
  using mrpt::math::square;

  ASSERT_(ni.clearance != nullptr);
  ASSERT_(!ni.targets.empty());

  // Create a log record for returning data.
  auto log = CLogFileRecord_FullEval::Create();
  no.logRecord = log;

  // Evaluate for each target:
  const size_t numTrgs = ni.targets.size();
  std::vector<EvalOutput> evals(numTrgs);
  for (unsigned int trg_idx = 0; trg_idx < numTrgs; trg_idx++) {
	evalSingleTarget(trg_idx, ni, evals[trg_idx]);
  }

  ASSERT_(!evals.empty());
  const auto nDirs = evals.front().phase_scores.back().size();
  ASSERT_EQUAL_(nDirs, ni.obstacles.size());

  // Now, sum all weights for the last stage for each target into an "overall"
  // score vector, one score per direction of motion:
  std::vector<double> overall_scores;
  overall_scores.assign(nDirs, .0);
  for (const auto &e : evals) {
	for (unsigned int i = 0; i < nDirs; i++)
	  overall_scores[i] += e.phase_scores.back()[i];
  }
  // Normalize:
  for (unsigned int i = 0; i < nDirs; i++)
	overall_scores[i] *= (1.0 / numTrgs);

  // Search for best direction in the "overall score" vector:

  // Keep the GAP with the largest maximum value within;
  // then pick the MIDDLE point as the final selection.
  std::vector<TGap> gaps;
  std::size_t best_gap_idx = std::string::npos;
  {
	bool inside_gap = false;
	for (unsigned int i = 0; i < nDirs; i++) {
	  const double val = overall_scores[i];
	  if (val < 0.01) {
		// This direction didn't pass the cut threshold for the "last
		// phase":
		if (inside_gap) {
		  // We just ended a gap:
		  auto &active_gap = *gaps.rbegin();
		  active_gap.k_to = i - 1;
		  inside_gap = false;
		}
	  } else {
		// higher or EQUAL to the treshold (equal is important just in case we
		// have a "flat" diagram...)
		if (!inside_gap) {
		  // We just started a gap:
		  TGap new_gap;
		  new_gap.k_from = i;
		  gaps.emplace_back(new_gap);
		  inside_gap = true;
		}
	  }

	  if (inside_gap) {
		auto &active_gap = *gaps.rbegin();
		if (val >= active_gap.max_eval) {
		  active_gap.k_best_eval = i;
		}
		mrpt::utils::keep_max(active_gap.max_eval, val);
		mrpt::utils::keep_min(active_gap.min_eval, val);

		if (best_gap_idx == std::string::npos ||
			val > gaps[best_gap_idx].max_eval) {
		  best_gap_idx = gaps.size() - 1;
		}
	  }
	} // end for i

	// Handle the case where we end with an open, active gap:
	if (inside_gap) {
	  auto &active_gap = *gaps.rbegin();
	  active_gap.k_to = nDirs - 1;
	}
  }

  // Not heading to target: go thru the "middle" of the gap to maximize
  // clearance
  int best_dir_k = -1;
  double best_dir_eval = 0;

  // We may have no gaps if all paths are blocked by obstacles, for example:
  if (!gaps.empty()) {
	ASSERT_(best_gap_idx < gaps.size());
	const TGap &best_gap = gaps[best_gap_idx];
	best_dir_k = best_gap.k_best_eval;
	best_dir_eval = overall_scores.at(best_dir_k);
  }

  // Prepare NavigationOutput data:
  if (best_dir_eval == .0) {
	// No way found!
	no.desiredDirection = 0;
	no.desiredSpeed = 0;
  } else {
	// A valid movement:
	const auto ptg = getAssociatedPTG();
	const double ptg_ref_dist = ptg ? ptg->getRefDistance() : 1.0;

	no.desiredDirection =
		CParameterizedTrajectoryGenerator::index2alpha(best_dir_k, nDirs);

	// Speed control: Reduction factors
	// ---------------------------------------------
	const double targetNearnessFactor =
		m_enableApproachTargetSlowDown
			? std::min(1.0, ni.targets.front().norm() /
								(options.TARGET_SLOW_APPROACHING_DISTANCE /
								 ptg_ref_dist))
			: 1.0;

	const double obs_dist = ni.obstacles[best_dir_k];
	// Was: min with obs_clearance too.
	const double obs_dist_th =
		std::max(options.TOO_CLOSE_OBSTACLE,
				 (options.OBSTACLE_SLOW_DOWN_DISTANCE / ptg_ref_dist) *
					 ni.maxObstacleDist);
	double riskFactor = 1.0;
	if (obs_dist <= options.TOO_CLOSE_OBSTACLE) {
	  riskFactor = 0.0;
	} else if (obs_dist < obs_dist_th &&
			   obs_dist_th > options.TOO_CLOSE_OBSTACLE) {
	  riskFactor = (obs_dist - options.TOO_CLOSE_OBSTACLE) /
				   (obs_dist_th - options.TOO_CLOSE_OBSTACLE);
	}
	no.desiredSpeed =
		ni.maxRobotSpeed * std::min(riskFactor, targetNearnessFactor);
  }

  m_last_selected_sector = best_dir_k;

  // LOG --------------------------
  if (log) {
	log->selectedTarget = 0; // was: best_trg_idx
	log->selectedSector = best_dir_k;
	log->evaluation = best_dir_eval;
	// Copy the evaluation of first phases for (arbitrarily) the first
	// target, then overwrite the scores of its last phase with the OVERALL
	// phase scores:
	log->dirs_eval = evals.front().phase_scores;
	log->dirs_eval.back() = overall_scores;

	if (options.LOG_SCORE_MATRIX) {
	  log->dirs_scores = m_dirs_scores;
	}
  }
}

unsigned int CHolonomicFullEval::direction2sector(const double a,
												  const unsigned int N) {
  const int idx = round(0.5 * (N * (1 + mrpt::math::wrapToPi(a) / M_PI) - 1));
  if (idx < 0)
	return 0;
  else
	return static_cast<unsigned int>(idx);
}

CLogFileRecord_FullEval::CLogFileRecord_FullEval()
	: selectedSector(0), evaluation(.0), dirs_scores(), selectedTarget(0) {}

void CLogFileRecord_FullEval::writeToStream(mrpt::utils::CStream &out,
											int *version) const {
  if (version)
	*version = 3;
  else {
	out << CHolonomicLogFileRecord::dirs_eval << dirs_scores << selectedSector
		<< evaluation << selectedTarget /*v3*/;
  }
}

/*---------------------------------------------------------------
																																																																																																																																																																																																																																																																																																																																readFromStream
  ---------------------------------------------------------------*/
void CLogFileRecord_FullEval::readFromStream(mrpt::utils::CStream &in,
											 int version) {
  switch (version) {
  case 0:
  case 1:
  case 2:
  case 3: {
	if (version >= 2) {
	  in >> CHolonomicLogFileRecord::dirs_eval;
	} else {
	  CHolonomicLogFileRecord::dirs_eval.resize(2);
	  in >> CHolonomicLogFileRecord::dirs_eval[0];
	  if (version >= 1) {
		in >> CHolonomicLogFileRecord::dirs_eval[1];
	  }
	}
	in >> dirs_scores >> selectedSector >> evaluation;
	if (version >= 3) {
	  in >> selectedTarget;
	} else {
	  selectedTarget = 0;
	}
  } break;
  default:
	MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
  };
}

/*---------------------------------------------------------------
																																																																																																																																																																																																																																																																																																																																																																																																TOptions
  ---------------------------------------------------------------*/
CHolonomicFullEval::TOptions::TOptions()
	: // Default values:
	  TOO_CLOSE_OBSTACLE(0.15), TARGET_SLOW_APPROACHING_DISTANCE(0.60),
	  OBSTACLE_SLOW_DOWN_DISTANCE(0.15), HYSTERESIS_SECTOR_COUNT(5),
	  LOG_SCORE_MATRIX(false), clearance_threshold_ratio(0.05),
	  gap_width_ratio_threshold(0.25) {
  factorWeights = mrpt::math::make_vector<5, double>(0.1, 0.5, 0.5, 0.01, 1);
  factorNormalizeOrNot = mrpt::math::make_vector<5, int>(0, 0, 0, 0, 1);

  PHASE_FACTORS.resize(3);
  PHASE_FACTORS[0] = mrpt::math::make_vector<2, int>(1, 2);
  PHASE_FACTORS[1] = mrpt::math::make_vector<1, int>(4);
  PHASE_FACTORS[2] = mrpt::math::make_vector<1, int>(0, 2);

  PHASE_THRESHOLDS = mrpt::math::make_vector<3, double>(0.5, 0.6, 0.7);
}

void CHolonomicFullEval::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase &c, const std::string &s) {
  MRPT_START

  // Load from config text:
  MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE, double, c, s);
  MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE, double, c, s);
  MRPT_LOAD_CONFIG_VAR(OBSTACLE_SLOW_DOWN_DISTANCE, double, c, s);
  MRPT_LOAD_CONFIG_VAR(HYSTERESIS_SECTOR_COUNT, double, c, s);
  MRPT_LOAD_CONFIG_VAR(LOG_SCORE_MATRIX, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(clearance_threshold_ratio, double, c, s);
  MRPT_LOAD_CONFIG_VAR(gap_width_ratio_threshold, double, c, s);

  c.read_vector(s, "factorWeights", std::vector<double>(), factorWeights, true);
  ASSERT_EQUAL_(factorWeights.size(), NUM_FACTORS);

  c.read_vector(s, "factorNormalizeOrNot", factorNormalizeOrNot,
				factorNormalizeOrNot);
  ASSERT_EQUAL_(factorNormalizeOrNot.size(), factorWeights.size());

  // Phases:
  int PHASE_COUNT = 0;
  MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(PHASE_COUNT, int, c, s);

  PHASE_FACTORS.resize(PHASE_COUNT);
  PHASE_THRESHOLDS.resize(PHASE_COUNT);
  for (int i = 0; i < PHASE_COUNT; i++) {
	c.read_vector(s, mrpt::format("PHASE%i_FACTORS", i + 1), PHASE_FACTORS[i],
				  PHASE_FACTORS[i], true);
	ASSERT_(!PHASE_FACTORS[i].empty());

	PHASE_THRESHOLDS[i] =
		c.read_double(s, mrpt::format("PHASE%i_THRESHOLD", i + 1), .0, true);
	ASSERT_(PHASE_THRESHOLDS[i] >= .0 && PHASE_THRESHOLDS[i] <= 1.0);
  }

  MRPT_END
}

void CHolonomicFullEval::TOptions::saveToConfigFile(
	mrpt::utils::CConfigFileBase &c, const std::string &s) const {
  MRPT_START;

  const int WN = mrpt::utils::MRPT_SAVE_NAME_PADDING,
			WV = mrpt::utils::MRPT_SAVE_VALUE_PADDING;

  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  TOO_CLOSE_OBSTACLE,
	  "Directions with collision-free distances below this threshold are not "
	  "elegible.");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  TARGET_SLOW_APPROACHING_DISTANCE,
	  "Start to reduce speed when closer than this to target.");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  OBSTACLE_SLOW_DOWN_DISTANCE,
	  "Start to reduce speed when clearance is below this value ([0,1] ratio "
	  "wrt obstacle reference/max distance)");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  HYSTERESIS_SECTOR_COUNT,
	  "Range of `sectors` (directions) for hysteresis over successive "
	  "timesteps");
  MRPT_SAVE_CONFIG_VAR_COMMENT(LOG_SCORE_MATRIX,
							   "Save the entire score matrix in log files");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  clearance_threshold_ratio,
	  "Ratio [0,1], times path_count, gives the minimum number of paths at "
	  "each side of a target direction to be accepted as desired direction");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
	  gap_width_ratio_threshold,
	  "Ratio [0,1], times path_count, gives the minimum gap width to accept "
	  "a direct motion towards target.");

  ASSERT_EQUAL_(factorWeights.size(), NUM_FACTORS);
  c.write(s, "factorWeights",
		  mrpt::system::sprintf_container("%.2f ", factorWeights), WN, WV,
		  "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target "
		  "(Euclidean), [3]=Hysteresis, [4]=clearance along path, [5]=Like [2] "
		  "without decimation if path obstructed");
  c.write(s, "factorNormalizeOrNot",
		  mrpt::system::sprintf_container("%u ", factorNormalizeOrNot), WN, WV,
		  "Normalize factors or not (1/0)");

  c.write(s, "PHASE_COUNT", PHASE_FACTORS.size(), WN, WV,
		  "Number of evaluation phases to run (params for each phase below)");

  for (unsigned int i = 0; i < PHASE_FACTORS.size(); i++) {
	c.write(s, mrpt::format("PHASE%u_THRESHOLD", i + 1), PHASE_THRESHOLDS[i],
			WN, WV,
			"Phase scores must be above this relative range threshold [0,1] to "
			"be considered in next phase (Default:`0.75`)");
	c.write(s, mrpt::format("PHASE%u_FACTORS", i + 1),
			mrpt::system::sprintf_container("%d ", PHASE_FACTORS[i]), WN, WV,
			"Indices of the factors above to be considered in this phase");
  }

  MRPT_END;
}

void CHolonomicFullEval::writeToStream(mrpt::utils::CStream &out,
									   int *version) const {
  if (version)
	*version = 4;
  else {
	// Params:
	out << options.factorWeights << options.HYSTERESIS_SECTOR_COUNT
		<< options.PHASE_FACTORS << // v3
		options.TARGET_SLOW_APPROACHING_DISTANCE << options.TOO_CLOSE_OBSTACLE
		<< options.PHASE_THRESHOLDS            // v3
		<< options.OBSTACLE_SLOW_DOWN_DISTANCE // v1
		<< options.factorNormalizeOrNot        // v2
		<< options.clearance_threshold_ratio
		<< options.gap_width_ratio_threshold // v4:
		;
	// State:
	out << m_last_selected_sector;
  }
}
void CHolonomicFullEval::readFromStream(mrpt::utils::CStream &in, int version) {
  switch (version) {
  case 0:
  case 1:
  case 2:
  case 3:
  case 4: {
	// Params:
	in >> options.factorWeights >> options.HYSTERESIS_SECTOR_COUNT;

	if (version >= 3) {
	  in >> options.PHASE_FACTORS;
	} else {
	  options.PHASE_THRESHOLDS.resize(2);
	  in >> options.PHASE_FACTORS[0] >> options.PHASE_FACTORS[1];
	}
	in >> options.TARGET_SLOW_APPROACHING_DISTANCE >>
		options.TOO_CLOSE_OBSTACLE;

	if (version >= 3) {
	  in >> options.PHASE_THRESHOLDS;
	} else {
	  options.PHASE_THRESHOLDS.resize(1);
	  in >> options.PHASE_THRESHOLDS[0];
	}

	if (version >= 1)
	  in >> options.OBSTACLE_SLOW_DOWN_DISTANCE;
	if (version >= 2)
	  in >> options.factorNormalizeOrNot;

	if (version >= 4) {
	  in >> options.clearance_threshold_ratio >>
		  options.gap_width_ratio_threshold;
	}

	// State:
	in >> m_last_selected_sector;
  } break;
  default:
	MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
  };
}

void CHolonomicFullEval::postProcessDirectionEvaluations(
	std::vector<double> &dir_evals, const NavInput &ni, unsigned int trg_idx) {
  // Default: do nothing
}
