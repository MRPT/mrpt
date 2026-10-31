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

#include "SlamDemoSimulator.h"

#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/round.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <cmath>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::slam;
using namespace mrpt::random;

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------
void SlamDemoSimulator::TSimulationOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& f, const std::string& c)
{
  MRPT_LOAD_CONFIG_VAR(random_seed, int, f, c)
  MRPT_LOAD_CONFIG_VAR(map_generator, string, f, c)
  MRPT_LOAD_CONFIG_VAR(randomMap_nLMs, int, f, c)

  sensorOnTheRobot.x(f.read_double(c, "sensorOnTheRobot.x", sensorOnTheRobot.x()));
  sensorOnTheRobot.y(f.read_double(c, "sensorOnTheRobot.y", sensorOnTheRobot.y()));
  sensorOnTheRobot.phi(
      DEG2RAD(f.read_double(c, "sensorOnTheRobot.phi", RAD2DEG(sensorOnTheRobot.phi()))));

  MRPT_LOAD_CONFIG_VAR(sensor_max_range, double, f, c)
  MRPT_LOAD_CONFIG_VAR(sensor_min_range, double, f, c)
  MRPT_LOAD_CONFIG_VAR_DEGREES(sensor_fov, f, c)

  MRPT_LOAD_CONFIG_VAR(sensorDistingishesLandmarks, bool, f, c)

  MRPT_LOAD_CONFIG_VAR(path_square_len, double, f, c)
  MRPT_LOAD_CONFIG_VAR(robot_step_length, double, f, c)

  MRPT_LOAD_CONFIG_VAR(odometry_noise_std_xy, double, f, c)
  MRPT_LOAD_CONFIG_VAR_DEGREES(odometry_noise_std_phi, f, c)

  MRPT_LOAD_CONFIG_VAR(uncert_overestim_odom, double, f, c)
  MRPT_LOAD_CONFIG_VAR(uncert_overestim_sensor, double, f, c)

  MRPT_LOAD_CONFIG_VAR(show_map_real_correspondences, bool, f, c)

  MRPT_LOAD_CONFIG_VAR(spurious_count_mean, double, f, c)
  MRPT_LOAD_CONFIG_VAR(spurious_count_std, double, f, c)
}

void SlamDemoSimulator::TSimulationOptions::saveToConfigFile(
    mrpt::config::CConfigFileBase& f, const std::string& c) const
{
  MRPT_SAVE_CONFIG_VAR(random_seed, f, c)
  MRPT_SAVE_CONFIG_VAR(map_generator, f, c)
  MRPT_SAVE_CONFIG_VAR(randomMap_nLMs, f, c)

  f.write(c, "sensorOnTheRobot.x", sensorOnTheRobot.x());
  f.write(c, "sensorOnTheRobot.y", sensorOnTheRobot.y());
  f.write(c, "sensorOnTheRobot.phi", RAD2DEG(sensorOnTheRobot.phi()));

  MRPT_SAVE_CONFIG_VAR(sensor_max_range, f, c)
  MRPT_SAVE_CONFIG_VAR(sensor_min_range, f, c)
  MRPT_SAVE_CONFIG_VAR_DEGREES(sensor_fov, f, c)

  MRPT_SAVE_CONFIG_VAR(sensorDistingishesLandmarks, f, c)

  MRPT_SAVE_CONFIG_VAR(path_square_len, f, c)
  MRPT_SAVE_CONFIG_VAR(robot_step_length, f, c)

  MRPT_SAVE_CONFIG_VAR(odometry_noise_std_xy, f, c)
  MRPT_SAVE_CONFIG_VAR_DEGREES(odometry_noise_std_phi, f, c)

  MRPT_SAVE_CONFIG_VAR(uncert_overestim_odom, f, c)
  MRPT_SAVE_CONFIG_VAR(uncert_overestim_sensor, f, c)

  MRPT_SAVE_CONFIG_VAR(show_map_real_correspondences, f, c)

  MRPT_SAVE_CONFIG_VAR(spurious_count_mean, f, c)
  MRPT_SAVE_CONFIG_VAR(spurious_count_std, f, c)
}

void SlamDemoSimulator::loadConfig(const std::string& cfgFile)
{
  ASSERT_FILE_EXISTS_(cfgFile);
  mrpt::config::CConfigFile cf(cfgFile);
  options.loadFromConfigFile(cf, "");
  slam.options.loadFromConfigFile(cf, "");
  slam.KF_options.loadFromConfigFile(cf, "");
}

void SlamDemoSimulator::saveConfig(const std::string& cfgFile) const
{
  mrpt::config::CConfigFile cf(cfgFile);
  options.saveToConfigFile(cf, "");
  slam.options.saveToConfigFile(cf, "");
  slam.KF_options.saveToConfigFile(cf, "");
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
void SlamDemoSimulator::reset()
{
  if (options.random_seed >= 0)
  {
    getRandomGenerator().randomize(static_cast<unsigned int>(options.random_seed));
  }
  else
  {
    getRandomGenerator().randomize();
  }

  slam.reset();
  historic.clear();
  estimatedIDX2realIDX.clear();
  m_realIDX_already_mapped.clear();

  lastObservation.sensedData.clear();
  lastObservationGTIndices.clear();

  gtPose = CPose2D(0, 0, 0);

  // The map:
  gtMap.clear();
  const auto& mapType = options.map_generator;

  if (mapType == "1")
  {
    // Default map: two rows of landmarks at each side of the square path.
    size_t ID = 0;
    CLandmark newLM;
    newLM.pose_mean.z = 0;
    const auto addLM = [&](double x, double y)
    {
      newLM.pose_mean.x = x;
      newLM.pose_mean.y = y;
      newLM.ID = ID++;
      gtMap.landmarks.push_back(newLM);
    };
    const double L = options.path_square_len;

    for (int i = 0; i <= mrpt::round(L / 2.0); i++)
    {
      const double di = static_cast<double>(i);
      // Bottom & top corridors:
      addLM(1 + 2 * di, -0.35);
      addLM(1 + 2 * di, -0.35 + L);
      addLM(1 + 2 * di, -0.6);
      addLM(1 + 2 * di, -0.6 + L);
      addLM(2 * (1 + di), 0.35);
      addLM(2 * di, 0.35 + L);
      addLM(2 * (1 + di), 0.6);
      addLM(2 * di, 0.6 + L);
      // Left & right corridors:
      addLM(-0.35, 2 * di);
      addLM(-0.35 + L, 2 * di);
      addLM(-0.6, 2 * di);
      addLM(-0.6 + L, 2 * di);
      addLM(0.35, 2 * di - 1);
      addLM(0.35 + L, 2 * di + 1);
      addLM(0.6, 2 * di - 1);
      addLM(0.6 + L, 2 * di + 1);
    }
  }
  else if (mapType == "2")
  {
    const double extra_width = 5;
    for (size_t i = 0; i < options.randomMap_nLMs; i++)
    {
      CLandmark newLM;
      newLM.pose_mean.x =
          getRandomGenerator().drawUniform(-extra_width, options.path_square_len + extra_width);
      newLM.pose_mean.y =
          getRandomGenerator().drawUniform(-extra_width, options.path_square_len + extra_width);
      newLM.pose_mean.z = 0;
      newLM.ID = i;
      gtMap.landmarks.push_back(newLM);
    }
  }
  else
  {
    // A text file with one landmark per row: "x y"
    ASSERT_FILE_EXISTS_(mapType);
    CMatrixDouble M;
    M.loadFromTextFile(mapType);
    ASSERTMSG_(M.cols() >= 2, "The map file must have (at least) 2 columns: x y");
    for (int i = 0; i < M.rows(); i++)
    {
      CLandmark newLM;
      newLM.pose_mean.x = M(i, 0);
      newLM.pose_mean.y = M(i, 1);
      newLM.pose_mean.z = 0;
      newLM.ID = static_cast<CLandmark::TLandmarkID>(i);
      gtMap.landmarks.push_back(newLM);
    }
  }
}

void SlamDemoSimulator::step()
{
  // Move the robot (not in the first step) ------------------------
  CPose2D poseIncr(0, 0, 0);

  const int turnSteps = 10;
  const double Aphi = DEG2RAD(90.0 / turnSteps);

  if (!historic.empty())
  {
    const double L = options.path_square_len;

    if (std::abs(std::fmod(gtPose.phi(), 90.0_deg)) < 1e-2)
    {
      const int dir = mrpt::round(gtPose.phi() / Aphi);

      // Continue in a straight line, unless we reach a corner:
      if ((gtPose.x() > L && gtPose.y() < L && dir == 0) ||
          (gtPose.x() > L && gtPose.y() > L && dir == turnSteps) ||
          (gtPose.x() <= 0 && std::abs(dir) == 2 * turnSteps) ||
          (gtPose.y() <= 0 && dir == -turnSteps))
      {
        poseIncr = CPose2D(0, 0, Aphi);  // Turn
      }
      else
      {
        poseIncr = CPose2D(options.robot_step_length, 0, 0);  // Straight
      }
    }
    else
    {
      poseIncr = CPose2D(0, 0, Aphi);  // Continue turning
    }
  }

  gtPose = gtPose + poseIncr;
  // Round phi so the paths are always perfect squares:
  gtPose.phi(mrpt::round(gtPose.phi() / Aphi) * Aphi);

  // Simulate observation ------------------------
  {
    const CPose3D robotPose = CPose3D(gtPose);
    const CPose3D sensorOnRobot = CPose3D(options.sensorOnTheRobot);

    lastObservation.fieldOfView_yaw = static_cast<float>(options.sensor_fov);
    lastObservation.maxSensorDistance = static_cast<float>(options.sensor_max_range);
    lastObservation.minSensorDistance = static_cast<float>(options.sensor_min_range);

    gtMap.simulateRangeBearingReadings(
        robotPose, sensorOnRobot, lastObservation, options.sensorDistingishesLandmarks,
        slam.options.std_sensor_range / options.uncert_overestim_sensor,
        slam.options.std_sensor_yaw / options.uncert_overestim_sensor,
        0,  // sigma_pitch: we are in 2D
        &lastObservationGTIndices, options.spurious_count_mean, options.spurious_count_std);
  }

  // Before processing the observation:
  const std::set<size_t> old_realIDX_already_mapped = m_realIDX_already_mapped;

  // SLAM ------------------------
  double executionTime = 0;
  {
    auto act = CActionCollection::Create();
    CActionRobotMovement2D actmov;
    CActionRobotMovement2D::TMotionModelOptions odo_opts;
    odo_opts.modelSelection = CActionRobotMovement2D::mmGaussian;

    // Model as a constant noise in X,Y,PHI:
    odo_opts.gaussianModel.a1 = 0;
    odo_opts.gaussianModel.a2 = 0;
    odo_opts.gaussianModel.a3 = 0;
    odo_opts.gaussianModel.a4 = 0;
    odo_opts.gaussianModel.minStdXY = options.odometry_noise_std_xy;
    odo_opts.gaussianModel.minStdPHI = options.odometry_noise_std_phi;

    // Add noise:
    CPose2D noisyPoseIncr = poseIncr;
    noisyPoseIncr.x_incr(getRandomGenerator().drawGaussian1D(
        0, options.odometry_noise_std_xy / options.uncert_overestim_odom));
    noisyPoseIncr.y_incr(getRandomGenerator().drawGaussian1D(
        0, options.odometry_noise_std_xy / options.uncert_overestim_odom));
    noisyPoseIncr.phi_incr(getRandomGenerator().drawGaussian1D(
        0, options.odometry_noise_std_phi / options.uncert_overestim_odom));

    actmov.computeFromOdometry(noisyPoseIncr, odo_opts);
    actmov.timestamp = mrpt::Clock::now();
    act->insert(actmov);

    CSensoryFrame::Ptr sf = CSensoryFrame::Create();
    lastObservation.timestamp = mrpt::Clock::now();
    lastObservation.sensorLabel = "SIMUL_2D_RB";

    sf->insert(CObservationBearingRange::Create(lastObservation));

    mrpt::system::CTicTac tictac;
    slam.processActionObservation(act, sf);
    executionTime = tictac.Tac();

    if (m_rawlogOut.fileOpenCorrectly())
    {
      mrpt::serialization::archiveFrom(m_rawlogOut) << act << sf;
    }
  }

  // Correspondences REAL_MAP <-> ESTIMATED_MAP, to evaluate D.A.:
  const CRangeBearingKFSLAM2D::TDataAssocInfo& da = slam.getLastDataAssociation();

  for (const auto& [obs_idx, est_map_idx] : da.newly_inserted_landmarks)
  {
    const size_t real_map_idx = lastObservationGTIndices[obs_idx];
    estimatedIDX2realIDX[est_map_idx] = real_map_idx;
    // Do NOT remember spurious readings, since they all look alike:
    if (real_map_idx != std::string::npos)
    {
      m_realIDX_already_mapped.insert(real_map_idx);
    }
  }

  // Save historic data ------------------------
  THistoric& hist = historic.emplace_back();

  hist.GT_robot_pose = gtPose;
  slam.getCurrentRobotPose(hist.estimate_robot_pose);
  hist.run_time = executionTime;
  hist.jcbb_iters = da.results.nNodesExploredInJCBB;

  // D.A. statistics:
  if (!options.sensorDistingishesLandmarks)
  {
    ASSERT_EQUAL_(lastObservation.sensedData.size(), lastObservationGTIndices.size());

    for (unsigned int o = 0; o < lastObservation.sensedData.size(); o++)
    {
      const bool o_has_assoc = da.results.associations.find(o) != da.results.associations.end();
      // Note: npos for spurious readings
      const size_t o_realmap_idx = lastObservationGTIndices[o];
      const bool o_was_mapped = o_realmap_idx != std::string::npos &&
                                old_realIDX_already_mapped.count(o_realmap_idx) != 0;
      const bool o_has_been_just_inserted =
          da.newly_inserted_landmarks.find(o) != da.newly_inserted_landmarks.end();

      if (o_was_mapped)
      {
        // Already observed in the past
        if (o_has_assoc)
        {
          if (estimatedIDX2realIDX[da.results.associations.find(o)->second] == o_realmap_idx)
          {
            hist.da_true_pos++;  // recognized as the known landmark
          }
          else
          {
            hist.da_false_pos++;
          }
        }
        else
        {
          ASSERT_(o_has_been_just_inserted);
          hist.da_false_neg++;  // a known landmark taken as new
        }
      }
      else
      {
        // Observed for the first time
        if (o_has_assoc)
        {
          hist.da_false_pos++;
        }
        else
        {
          ASSERT_(o_has_been_just_inserted);
          hist.da_true_neg++;  // a new landmark, inserted as such
        }
      }
    }
  }
}

size_t SlamDemoSimulator::defaultBatchSteps() const
{
  return static_cast<size_t>(std::round(6 * options.path_square_len / options.robot_step_length));
}

SlamDemoSimulator::TRunStats SlamDemoSimulator::computeStats(double totalSeconds) const
{
  TRunStats s;
  const size_t N = historic.size();
  s.steps = N;
  if (N == 0)
  {
    return s;
  }
  for (const auto& h : historic)
  {
    s.meanFP += h.da_false_pos;
    s.meanFN += h.da_false_neg;
    s.meanJCBBIters += static_cast<double>(h.jcbb_iters);
    s.meanErrXY += h.GT_robot_pose.distanceTo(h.estimate_robot_pose.getMeanVal());

    const CPosePDFGaussian p(h.GT_robot_pose, CMatrixDouble33() /* cov= zero */);
    double dm2 = 1000;
    try
    {
      CPosePDFGaussian est = h.estimate_robot_pose;
      dm2 = est.mahalanobisDistanceTo(p);
    }
    catch (...)
    {
    }
    s.meanMahalanobis2 += dm2;
    s.meanErrPhi +=
        std::abs(wrapToPi(h.GT_robot_pose.phi() - h.estimate_robot_pose.getMeanVal().phi()));
  }
  const auto dN = static_cast<double>(N);
  s.meanFP /= dN;
  s.meanFN /= dN;
  s.meanJCBBIters /= dN;
  s.meanErrXY /= dN;
  s.meanErrPhi /= dN;
  s.meanMahalanobis2 /= dN;
  s.meanStepTime = totalSeconds / dN;
  return s;
}

void SlamDemoSimulator::setRawlogOutput(const std::string& file)
{
  if (m_rawlogOut.fileOpenCorrectly())
  {
    m_rawlogOut.close();
  }
  if (file.empty())
  {
    return;
  }
  if (!m_rawlogOut.open(file))
  {
    THROW_EXCEPTION_FMT("Cannot open output file: '%s'", file.c_str());
  }
  // A first "observation" with information about the dataset:
  CObservationComment obs;
  obs.text = std::string("Rawlog generated by 2d-slam-demo\n MRPT version: ") +
             mrpt::system::MRPT_getVersion() +
             "\n Creation date: " + mrpt::system::dateTimeLocalToString(mrpt::Clock::now()) + "\n";
  mrpt::serialization::archiveFrom(m_rawlogOut) << obs;
}

void SlamDemoSimulator::saveFilterState(const std::string& dir) const
{
  CPosePDFGaussian estRobotPose;
  std::vector<TPoint2D> LMs;
  std::map<unsigned int, CLandmark::TLandmarkID> landmarkIDs;
  CVectorDouble Xkk;
  CMatrixDouble Pkk;
  slam.getCurrentState(estRobotPose, LMs, landmarkIDs, Xkk, Pkk);

  Xkk.saveToTextFile(dir + "/kf_mean.txt");
  Pkk.saveToTextFile(dir + "/kf_cov.txt");

  auto obj3D = mrpt::viz::CSetOfObjects::Create();
  slam.getAs3DObject(obj3D);
  mrpt::viz::Scene scene;
  scene.insert(obj3D);
  scene.saveToFile(dir + "/slam.3Dscene");
}

void SlamDemoSimulator::saveLastDataAssociation(const std::string& dir) const
{
  const auto& da = slam.getLastDataAssociation();
  (void)mrpt::io::vectorToTextFile(da.predictions_IDs, dir + "/prediction_IDs.txt");
  da.Y_pred_means.saveToTextFile(dir + "/prediction_means.txt");
  da.Y_pred_covs.saveToTextFile(dir + "/prediction_cov.txt");
}
