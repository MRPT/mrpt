/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/round.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CLogFileRecord_ND, CHolonomicLogFileRecord, mrpt::nav)
IMPLEMENTS_SERIALIZABLE(CHolonomicND, CAbstractHolonomicReactiveMethod, mrpt::nav)

/**  Initialize the parameters of the navigator, from some
 *    configuration file, or default values if filename is set to NULL.
 */
CHolonomicND::CHolonomicND(const mrpt::config::CConfigFileBase* INI_FILE) :
    CAbstractHolonomicReactiveMethod("CHolonomicND"),
    m_last_selected_sector(std::numeric_limits<unsigned int>::max())
{
  if (INI_FILE != nullptr) initialize(*INI_FILE);
}

void CHolonomicND::initialize(const mrpt::config::CConfigFileBase& INI_FILE)
{
  options.loadFromConfigFile(INI_FILE, getConfigFileSectionName());
}
void CHolonomicND::saveConfigFile(mrpt::config::CConfigFileBase& c) const
{
  options.saveToConfigFile(c, getConfigFileSectionName());
}

/*---------------------------------------------------------------
            Navigate
  ---------------------------------------------------------------*/
CHolonomicND::NavOutput CHolonomicND::navigate(const NavInput& ni)
{
  NavOutput no;
  const auto ptg = getAssociatedPTG();
  const double ptg_ref_dist = ptg ? ptg->getRefDistance() : 1.0;

  TGapArray gaps;
  TSituations situation;
  unsigned int selectedSector;
  double riskEvaluation;
  double evaluation;

  // Create a log record for returning data.
  CLogFileRecord_ND::Ptr log = std::make_shared<CLogFileRecord_ND>();
  no.logRecord = log;

  // Search gaps:
  gaps.clear();
  ASSERT_(!ni.targets.empty());
  const auto trg = *ni.targets.rbegin();

  gapsEstimator(ni.obstacles, trg, gaps);

  // Select best gap:
  searchBestGap(
      ni.obstacles, 1.0 /* max obs range*/, gaps, trg, selectedSector, evaluation, situation,
      riskEvaluation, *log);

  if (situation == CHolonomicND::TSituations::NO_WAY_FOUND)
  {
    // No way found!
    no.desiredDirection = 0;
    no.desiredSpeed = 0;
  }
  else
  {
    // A valid movement:
    no.desiredDirection =
        CParameterizedTrajectoryGenerator::Index2alpha(selectedSector, ni.obstacles.size());

    // Speed control: Reduction factors
    // ---------------------------------------------
    const double targetNearnessFactor =
        m_enableApproachTargetSlowDown
            ? std::min(1.0, trg.norm() / (options.TARGET_SLOW_APPROACHING_DISTANCE / ptg_ref_dist))
            : 1.0;

    const double riskFactor = std::min(1.0, riskEvaluation / options.RISK_EVALUATION_DISTANCE);
    no.desiredSpeed = ni.maxRobotSpeed * std::min(riskFactor, targetNearnessFactor);
  }

  m_last_selected_sector = selectedSector;

  // LOG --------------------------
  if (log)
  {
    // gaps:
    {
      int i, n = gaps.size();
      log->gaps_ini.resize(n);
      log->gaps_end.resize(n);
      for (i = 0; i < n; i++)
      {
        log->gaps_ini[i] = gaps[i].ini;
        log->gaps_end[i] = gaps[i].end;
      }
    }
    // Selection:
    log->selectedSector = selectedSector;
    log->evaluation = evaluation;
    log->situation = situation;
    log->riskEvaluation = riskEvaluation;
  }

  return no;
}

/*---------------------------------------------------------------
        Find gaps in the obtacles (Beta version)
  ---------------------------------------------------------------*/
// Build the raw list of candidate gaps by sweeping threshold ratios.
static CHolonomicND::TGapArray buildRawGaps(
    const std::vector<double>& obstacles,
    const mrpt::math::TPose2D& target,
    double overall_max_dist,
    int GAPS_MIN_WIDTH)
{
  const size_t n = obstacles.size();
  constexpr double GAPS_MIN_DEPTH_CONSIDERED = 0.6;

  CHolonomicND::TGapArray gaps;
  gaps.reserve(150);

  for (double threshold_ratio = 0.95; threshold_ratio >= 0.05; threshold_ratio -= 0.05)
  {
    const double dist_threshold =
        threshold_ratio * overall_max_dist +
        (1.0 - threshold_ratio) * std::min(target.norm(), GAPS_MIN_DEPTH_CONSIDERED);

    bool is_inside = false;
    size_t sec_ini = 0, sec_end = 0;
    double maxDist = 0.;

    for (size_t i = 0; i < n; i++)
    {
      if (!is_inside && obstacles[i] >= dist_threshold)
      {
        sec_ini = i;
        maxDist = obstacles[i];
        is_inside = true;
      }
      else if (is_inside && (i == (n - 1) || obstacles[i] < dist_threshold))
      {
        sec_end = (obstacles[i] < dist_threshold) ? i - 1 : i;
        is_inside = false;

        if ((sec_end - sec_ini) >= static_cast<size_t>(GAPS_MIN_WIDTH))
        {
          gaps.resize(gaps.size() + 1);
          CHolonomicND::TGap& g = gaps.back();
          g.ini = sec_ini;
          g.end = sec_end;
          g.minDistance = std::min(obstacles[sec_ini], obstacles[sec_end]);
          g.maxDistance = maxDist;
        }
      }
      if (is_inside) maxDist = std::max(maxDist, obstacles[i]);
    }
  }
  return gaps;
}

// Remove redundant/deep/contained gaps from `gaps`, returning a filtered copy.
static CHolonomicND::TGapArray filterGaps(
    const CHolonomicND::TGapArray& gaps, double max_depth, double GAPS_MAX_RELATIVE_DEPTH)
{
  const size_t N = gaps.size();
  std::vector<bool> del(N, false);

  // Remove redundant gaps (same ini or end)
  for (size_t i = 0; i < N; i++)
  {
    if (del[i]) continue;
    for (size_t j = i + 1; j < N; j++)
    {
      if (gaps[i].ini == gaps[j].ini || gaps[i].end == gaps[j].end) del[j] = true;
    }
  }

  // Remove gaps with excessive depth variation
  for (size_t i = 0; i < N; i++)
  {
    if (!del[i] &&
        (gaps[i].maxDistance - gaps[i].minDistance) > max_depth * GAPS_MAX_RELATIVE_DEPTH)
      del[i] = true;
  }

  // Remove gaps containing more than one sub-gap
  for (size_t i = 0; i < N; i++)
  {
    if (del[i]) continue;
    unsigned int inner = 0;
    for (size_t j = 0; j < N; j++)
    {
      if (i == j || del[j]) continue;
      if (gaps[j].ini >= gaps[i].ini && gaps[j].end <= gaps[i].end)
        if (++inner > 1)
        {
          del[i] = true;
          break;
        }
    }
  }

  // Remove gaps fully contained within another gap
  for (size_t i = 0; i < N; i++)
  {
    if (del[i]) continue;
    for (size_t j = 0; j < N; j++)
    {
      if (i == j || del[j]) continue;
      if (gaps[i].ini <= gaps[j].ini && gaps[i].end >= gaps[j].end) del[j] = true;
    }
  }

  CHolonomicND::TGapArray result;
  result.reserve(N / 2 + 1);
  for (size_t i = 0; i < N; i++)
    if (!del[i]) result.push_back(gaps[i]);
  return result;
}

void CHolonomicND::gapsEstimator(
    const std::vector<double>& obstacles, const mrpt::math::TPose2D& target, TGapArray& gaps_out)
{
  const size_t n = obstacles.size();
  ASSERT_(n > 2);

  const int GAPS_MIN_WIDTH = static_cast<int>(std::ceil(n * 0.01));
  constexpr double GAPS_MAX_RELATIVE_DEPTH = 0.5;

  // Find obstacle distance range:
  double overall_max_dist = std::numeric_limits<float>::min();
  double overall_min_dist = std::numeric_limits<float>::max();
  for (size_t i = 1; i < (n - 1); i++)
  {
    mrpt::keep_max(overall_max_dist, obstacles[i]);
    mrpt::keep_min(overall_min_dist, obstacles[i]);
  }
  const double max_depth = overall_max_dist - overall_min_dist;

  // Build and filter gap list:
  TGapArray gaps_temp = buildRawGaps(obstacles, target, overall_max_dist, GAPS_MIN_WIDTH);
  TGapArray filtered = filterGaps(gaps_temp, max_depth, GAPS_MAX_RELATIVE_DEPTH);

  // Compute representative sector for each surviving gap:
  gaps_out.clear();
  gaps_out.reserve(filtered.size());
  for (auto& g : filtered)
  {
    calcRepresentativeSectorForGap(g, target, obstacles);
    gaps_out.push_back(g);
  }
}

/*---------------------------------------------------------------
            Search the best gap.
  ---------------------------------------------------------------*/
void CHolonomicND::searchBestGap(
    const std::vector<double>& obstacles,
    const double maxObsRange,
    const TGapArray& in_gaps,
    const mrpt::math::TPose2D& target,
    unsigned int& out_selDirection,
    double& out_selEvaluation,
    TSituations& out_situation,
    double& out_riskEvaluation,
    CLogFileRecord_ND& log)
{
  // For evaluating the "risk":
  unsigned int min_risk_eval_sector = 0;
  unsigned int max_risk_eval_sector = obstacles.size() - 1;
  const unsigned int target_sector = direction2sector(atan2(target.y, target.x), obstacles.size());
  // Minimum target distance to avoid division by zero in ratio computations:
  constexpr double MIN_TARGET_DIST = 0.01;  // [m or normalized units]
  const double target_dist = std::max(MIN_TARGET_DIST, target.norm());
  // (Risk is evaluated at the end, for all the situations)

  // D1 : Straight path?
  // --------------------------------------------------------
  // Fraction of total obstacle sectors examined each side of the target
  // direction to verify the direct path is free.
  constexpr double DIRECT_PATH_SECTOR_FRACTION = 0.02;
  // Slight extra margin above target distance to accept a sector as free
  // (5% clearance factor).
  constexpr double DIRECT_PATH_DIST_MARGIN = 1.05;
  // Safety cap below maxObsRange so the robot doesn't rely on the last
  // sample right at the sensor limit (95% of max range).
  constexpr double DIRECT_PATH_RANGE_FRACTION = 0.95;

  const int freeSectorsNearTarget =
      static_cast<int>(std::ceil(DIRECT_PATH_SECTOR_FRACTION * obstacles.size()));
  bool theyAreFree = true, caseD1 = false;
  if (target_sector > static_cast<unsigned int>(freeSectorsNearTarget) &&
      target_sector < static_cast<unsigned int>(obstacles.size() - freeSectorsNearTarget))
  {
    const double min_free_dist =
        std::min(DIRECT_PATH_DIST_MARGIN * target_dist, DIRECT_PATH_RANGE_FRACTION * maxObsRange);
    for (int j = -freeSectorsNearTarget; theyAreFree && j <= freeSectorsNearTarget; j++)
      if (obstacles[(int(target_sector) + j) % obstacles.size()] < min_free_dist)
        theyAreFree = false;
    caseD1 = theyAreFree;
  }

  if (caseD1)
  {
    // S1: Move straight towards target:
    out_selDirection = target_sector;

    // In case of several paths, the shortest:
    out_selEvaluation = 1.0 + std::max(0.0, (maxObsRange - target_dist) / maxObsRange);
    out_situation = CHolonomicND::TSituations::TARGET_DIRECTLY;
  }
  else
  {
    // Evaluate all gaps (if any):
    std::vector<double> gaps_evaluation;
    int selected_gap = -1;
    double selected_gap_eval = -100;

    evaluateGaps(obstacles, maxObsRange, in_gaps, target_sector, target_dist, gaps_evaluation);

    log.gaps_eval = gaps_evaluation;

    // D2: is there any gap "beyond" the target (and not too far away)?
    // (Not used)
    // ----------------------------------------------------------------

    // unsigned int dist;
    // for ( unsigned int i=0;i<in_gaps.size();i++ )
    //{
    //	dist = mrpt::abs_diff(target_sector,
    // in_gaps[i].representative_sector );
    //	if (dist > 0.5*obstacles.size())
    //		dist = obstacles.size() - dist;
    //
    //	if ( in_gaps[i].maxDistance >= target_dist && dist <=
    //(int)floor(options.MAX_SECTOR_DIST_FOR_D2_PERCENT * obstacles.size())
    //)
    //
    //		if ( gaps_evaluation[i]>selected_gap_eval )
    //		{
    //			selected_gap_eval = gaps_evaluation[i];
    //			selected_gap = i;
    //		}
    //}

    // Keep the best gaps (if none was picked up to this point)
    if (selected_gap == -1)
      for (unsigned int i = 0; i < in_gaps.size(); i++)
        if (gaps_evaluation[i] > selected_gap_eval)
        {
          selected_gap_eval = gaps_evaluation[i];
          selected_gap = i;
        }

    //  D3: Wasn't a good enough gap (or there were none)?
    // ----------------------------------------------------------
    if (selected_gap_eval <= 0)
    {
      // S2: No way found
      // ------------------------------------------------------
      out_selDirection = 0;
      out_selEvaluation = 0.0;  // Worst case
      out_situation = CHolonomicND::TSituations::NO_WAY_FOUND;
    }
    else
    {
      // The selected gap:
      const TGap& gap = in_gaps[selected_gap];

      const unsigned int sectors_to_be_wide =
          round(options.WIDE_GAP_SIZE_PERCENT * obstacles.size());

      out_selDirection = in_gaps[selected_gap].representative_sector;
      out_selEvaluation = selected_gap_eval;

      // D4: Is it a WIDE gap?
      // -----------------------------------------------------
      if ((gap.end - gap.ini) < sectors_to_be_wide)
      {
        // S3: Narrow gap
        // -------------------------------------------
        out_situation = CHolonomicND::TSituations::SMALL_GAP;
      }
      else
      {
        // S4: Wide gap
        // -------------------------------------------
        out_situation = CHolonomicND::TSituations::WIDE_GAP;
      }

      // Evaluate the risk only within the gap:
      min_risk_eval_sector = gap.ini;
      max_risk_eval_sector = gap.end;
    }
  }

  // Evaluate short-term minimum distance to obstacles, in a small interval
  // around the selected direction:
  const unsigned int risk_eval_nsectors =
      round(options.RISK_EVALUATION_SECTORS_PERCENT * obstacles.size());
  const unsigned int sec_ini = std::max(
      min_risk_eval_sector,
      risk_eval_nsectors < out_selDirection ? out_selDirection - risk_eval_nsectors : 0);
  const unsigned int sec_fin =
      std::min(max_risk_eval_sector, out_selDirection + risk_eval_nsectors);

  out_riskEvaluation = 0.0;
  for (unsigned int i = sec_ini; i <= sec_fin; i++) out_riskEvaluation += obstacles[i];
  out_riskEvaluation /= (sec_fin - sec_ini + 1);
}

/*---------------------------------------------------------------
  Fills in the representative sector
    field in the gap structure:
  ---------------------------------------------------------------*/
void CHolonomicND::calcRepresentativeSectorForGap(
    TGap& gap, const mrpt::math::TPose2D& target, const std::vector<double>& obstacles)
{
  int sector;
  const unsigned int sectors_to_be_wide = round(options.WIDE_GAP_SIZE_PERCENT * obstacles.size());
  const unsigned int target_sector = direction2sector(atan2(target.y, target.x), obstacles.size());

  if ((gap.end - gap.ini) < sectors_to_be_wide)  // Select the intermediate sector
  {
#if 1
    sector = round(0.5f * gap.ini + 0.5f * gap.end);
#else
    float min_dist_obs_near_ini = 1, min_dist_obs_near_end = 1;
    int i;
    for (i = gap.ini; i >= max(0, gap.ini - 2); i--)
      min_dist_obs_near_ini = min(min_dist_obs_near_ini, obstacles[i]);
    for (i = gap.end; i <= min(static_cast<int>(obstacles.size()) - 1, gap.end + 2); i++)
      min_dist_obs_near_end = min(min_dist_obs_near_end, obstacles[i]);
    sector = round(
        (min_dist_obs_near_ini * gap.ini + min_dist_obs_near_end * gap.end) /
        (min_dist_obs_near_ini + min_dist_obs_near_end));
#endif
  }
  else  // Select a sector close to the target but spaced
  // "sectors_to_be_wide/2" from it
  {
    unsigned int dist_ini = mrpt::abs_diff(target_sector, gap.ini);
    unsigned int dist_end = mrpt::abs_diff(target_sector, gap.end);

    if (dist_ini > 0.5 * obstacles.size()) dist_ini = obstacles.size() - dist_ini;
    if (dist_end > 0.5 * obstacles.size()) dist_end = obstacles.size() - dist_end;

    int dir;
    if (dist_ini < dist_end)
    {
      sector = gap.ini;
      dir = +1;
    }
    else
    {
      sector = gap.end;
      dir = -1;
    }

    sector = sector + dir * static_cast<int>(sectors_to_be_wide) / 2;
  }

  keep_max(sector, 0);
  keep_min(sector, static_cast<int>(obstacles.size()) - 1);

  gap.representative_sector = sector;
}

/*---------------------------------------------------------------
            Evaluate each gap
  ---------------------------------------------------------------*/
void CHolonomicND::evaluateGaps(
    const std::vector<double>& obstacles,
    const double maxObsRange,
    const TGapArray& gaps,
    const unsigned int target_sector,
    const float target_dist,
    std::vector<double>& out_gaps_evaluation)
{
  out_gaps_evaluation.resize(gaps.size());

  const double targetAng =
      CParameterizedTrajectoryGenerator::Index2alpha(target_sector, obstacles.size());
  const double target_x = target_dist * cos(targetAng);
  const double target_y = target_dist * sin(targetAng);

  for (unsigned int i = 0; i < gaps.size(); i++)
  {
    // Short cut:
    const TGap* gap = &gaps[i];

    const float d = min3(obstacles[gap->representative_sector], maxObsRange, 0.95 * target_dist);

    // The TP-Space representative coordinates for this gap:
    const double phi = CParameterizedTrajectoryGenerator::Index2alpha(
        gap->representative_sector, obstacles.size());
    const double x = d * cos(phi);
    const double y = d * sin(phi);

    // Factor #1: Maximum reachable distance with this PTG:
    // -----------------------------------------------------
    // It computes the average free distance of the gap:
    float meanDist = 0.f;
    for (unsigned int j = gap->ini; j <= gap->end; j++) meanDist += obstacles[j];
    meanDist /= (gap->end - gap->ini + 1);

    double factor1;
    if (mrpt::abs_diff(gap->representative_sector, target_sector) <= 1 && target_dist < 1)
      factor1 = std::min(target_dist, meanDist) / target_dist;
    else
      factor1 = meanDist;

    // Factor #2: Distance to target in "sectors"
    // -------------------------------------------
    unsigned int dif = mrpt::abs_diff(target_sector, gap->representative_sector);

    // Handle the -PI,PI circular topology:
    if (dif > 0.5 * obstacles.size()) dif = obstacles.size() - dif;

    const double factor2 = exp(-square(dif / (obstacles.size() * 0.25)));

    // Factor #3: Punish paths that take us far away wrt the target:  **** I
    // don't understand it *********
    // -----------------------------------------------------
    double closestX, closestY;
    double dist_eucl = math::minimumDistanceFromPointToSegment(
        target_x, target_y,  // Point
        0, 0, x, y,          // Segment
        closestX, closestY   // Out
    );

    const float factor3 = (maxObsRange - std::min(maxObsRange, dist_eucl)) / maxObsRange;

    // Factor #4: Stabilizing factor (hysteresis) to avoid quick switch
    // among very similar paths:
    // ------------------------------------------------------------------------------------------
    double factor_AntiCab;

    if (m_last_selected_sector != std::numeric_limits<unsigned int>::max())
    {
      unsigned int dist = mrpt::abs_diff(m_last_selected_sector, gap->representative_sector);

      if (dist > unsigned(0.1 * obstacles.size()))
        factor_AntiCab = 0.0;
      else
        factor_AntiCab = 1.0;
    }
    else
    {
      factor_AntiCab = 0;
    }

    ASSERT_(options.factorWeights.size() == 4);

    if (obstacles[gap->representative_sector] <
        options.TOO_CLOSE_OBSTACLE)  // Too close to obstacles
      out_gaps_evaluation[i] = 0;
    else
      out_gaps_evaluation[i] =
          (options.factorWeights[0] * factor1 + options.factorWeights[1] * factor2 +
           options.factorWeights[2] * factor3 + options.factorWeights[3] * factor_AntiCab) /
          (math::sum(options.factorWeights));
  }  // for each gap
}

unsigned int CHolonomicND::direction2sector(const double a, const unsigned int N)
{
  const int idx = round(0.5 * (N * (1 + mrpt::math::wrapToPi(a) / M_PI) - 1));
  if (idx < 0)
    return 0;
  else
    return static_cast<unsigned int>(idx);
}

uint8_t CLogFileRecord_ND::serializeGetVersion() const { return 1; }
void CLogFileRecord_ND::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << gaps_ini << gaps_end << gaps_eval;
  out << selectedSector << evaluation << riskEvaluation << static_cast<uint32_t>(situation);
}

void CLogFileRecord_ND::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      int32_t n;

      in >> n;
      gaps_ini.resize(n);
      gaps_end.resize(n);
      in.ReadBuffer(&(*gaps_ini.begin()), sizeof(gaps_ini[0]) * n);
      in.ReadBuffer(&(*gaps_end.begin()), sizeof(gaps_end[0]) * n);

      in >> n;
      gaps_eval.resize(n);
      in.ReadBuffer(&(*gaps_eval.begin()), sizeof(gaps_eval[0]) * n);

      in >> selectedSector >> evaluation >> riskEvaluation >> n;

      situation = (CHolonomicND::TSituations)n;
    }
    break;
    case 1:
    {
      uint32_t n;
      in >> gaps_ini >> gaps_end >> gaps_eval;
      in >> selectedSector >> evaluation >> riskEvaluation >> n;
      situation = (CHolonomicND::TSituations)n;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

/*---------------------------------------------------------------
            TOptions
  ---------------------------------------------------------------*/
CHolonomicND::TOptions::TOptions() : factorWeights{1.0, 0.5, 2.0, 0.4} {}
void CHolonomicND::TOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& section)
{
  MRPT_START

  // Load from config text:
  MRPT_LOAD_CONFIG_VAR(WIDE_GAP_SIZE_PERCENT, double, source, section);
  MRPT_LOAD_CONFIG_VAR(MAX_SECTOR_DIST_FOR_D2_PERCENT, double, source, section);
  MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_SECTORS_PERCENT, double, source, section);
  MRPT_LOAD_CONFIG_VAR(RISK_EVALUATION_DISTANCE, double, source, section);
  MRPT_LOAD_CONFIG_VAR(TOO_CLOSE_OBSTACLE, double, source, section);
  MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE, double, source, section);

  source.read_vector(section, "factorWeights", std::vector<double>(), factorWeights, true);
  ASSERT_(factorWeights.size() == 4);

  MRPT_END
}

void CHolonomicND::TOptions::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  MRPT_START
  const int WN = mrpt::config::MRPT_SAVE_NAME_PADDING(),
            WV = mrpt::config::MRPT_SAVE_VALUE_PADDING();

  MRPT_SAVE_CONFIG_VAR_COMMENT(WIDE_GAP_SIZE_PERCENT, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(MAX_SECTOR_DIST_FOR_D2_PERCENT, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(RISK_EVALUATION_SECTORS_PERCENT, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(RISK_EVALUATION_DISTANCE, "In normalized ps-meters [0,1]");
  MRPT_SAVE_CONFIG_VAR_COMMENT(TOO_CLOSE_OBSTACLE, "For stopping gradually");
  MRPT_SAVE_CONFIG_VAR_COMMENT(TARGET_SLOW_APPROACHING_DISTANCE, "In normalized ps-meters");

  ASSERT_EQUAL_(factorWeights.size(), 4);
  c.write(
      s, "factorWeights",
      mrpt::format(
          "%.2f %.2f %.2f %.2f", factorWeights[0], factorWeights[1], factorWeights[2],
          factorWeights[3]),
      WN, WV,
      "[0]=Free space, [1]=Dist. in sectors, [2]=Closer to target "
      "(Euclidean), [3]=Hysteresis");

  MRPT_END
}

uint8_t CHolonomicND::serializeGetVersion() const { return 0; }
void CHolonomicND::serializeTo(mrpt::serialization::CArchive& out) const
{
  // Params:
  out << options.factorWeights << options.MAX_SECTOR_DIST_FOR_D2_PERCENT
      << options.RISK_EVALUATION_DISTANCE << options.RISK_EVALUATION_SECTORS_PERCENT
      << options.TARGET_SLOW_APPROACHING_DISTANCE << options.TOO_CLOSE_OBSTACLE
      << options.WIDE_GAP_SIZE_PERCENT;
  // State:
  out << m_last_selected_sector;
}
void CHolonomicND::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      // Params:
      in >> options.factorWeights >> options.MAX_SECTOR_DIST_FOR_D2_PERCENT >>
          options.RISK_EVALUATION_DISTANCE >> options.RISK_EVALUATION_SECTORS_PERCENT >>
          options.TARGET_SLOW_APPROACHING_DISTANCE >> options.TOO_CLOSE_OBSTACLE >>
          options.WIDE_GAP_SIZE_PERCENT;
      // State:
      in >> m_last_selected_sector;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}
