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
#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>

#include <map>
#include <set>
#include <string>
#include <vector>

/** The range-bearing EKF-SLAM simulation of 2d-slam-demo, independent of any
 * GUI, so it can run in batch mode without a display.
 */
class SlamDemoSimulator
{
 public:
  struct TSimulationOptions : public mrpt::config::CLoadableOptions
  {
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source, const std::string& section) override;
    void saveToConfigFile(
        mrpt::config::CConfigFileBase& source, const std::string& section) const override;

    /** -1: random, other, use as seed */
    int random_seed{-1};

    /** Ground truth map: "1" (square corridor), "2" (random) or the name of
     * a text file with a Nx2 matrix of landmark coordinates. */
    std::string map_generator{"1"};

    /** # of landmarks in the random map */
    uint32_t randomMap_nLMs{70};

    mrpt::poses::CPose2D sensorOnTheRobot{0, 0, 0};

    double sensor_max_range{5};
    double sensor_min_range{0.50};
    double sensor_fov{mrpt::DEG2RAD(140.)};
    /** If false (default), data association must be done */
    bool sensorDistingishesLandmarks{false};

    /** Side of the square robot path (m) */
    double path_square_len{8};
    /** The length (in meters) of each robot forward step. */
    double robot_step_length{0.3};

    /** sigma of the odometry errors in X/Y */
    double odometry_noise_std_xy{0.02};
    /** sigma of the odometry errors in PHI */
    double odometry_noise_std_phi{mrpt::DEG2RAD(0.2)};

    double uncert_overestim_odom{1.2};
    double uncert_overestim_sensor{1.2};

    bool show_map_real_correspondences{false};

    /** Mean and std of spurious readings per "sensor observation". */
    double spurious_count_mean{0};
    double spurious_count_std{0};
  };

  /** Historic data of one step */
  struct THistoric
  {
    mrpt::poses::CPose2D GT_robot_pose;
    mrpt::poses::CPosePDFGaussian estimate_robot_pose;
    double run_time{0};
    uint16_t da_true_pos{0};
    uint16_t da_true_neg{0};
    uint16_t da_false_pos{0};
    uint16_t da_false_neg{0};
    size_t jcbb_iters{0};
  };

  /** Summary of a run, as computed by batch experiments. */
  struct TRunStats
  {
    size_t steps = 0;
    double meanStepTime = 0;  //!< seconds
    double meanErrXY = 0;
    double meanErrPhi = 0;  //!< rad
    double meanMahalanobis2 = 0;
    double meanJCBBIters = 0;
    double meanFP = 0;
    double meanFN = 0;
  };

  SlamDemoSimulator() = default;

  TSimulationOptions options;

  /** The main object which handles all 2D-EKF SLAM */
  mrpt::slam::CRangeBearingKFSLAM2D slam;

  /** The ground truth map, used to simulate observations */
  mrpt::maps::CLandmarksMap gtMap;

  /** The ground truth robot pose */
  mrpt::poses::CPose2D gtPose;

  mrpt::obs::CObservationBearingRange lastObservation;
  /** Ground truth indices in gtMap of the sensed landmarks (npos for
   * spurious readings) */
  std::vector<size_t> lastObservationGTIndices;

  /** Estimated map index -> ground truth map index, to evaluate data
   * association. */
  std::map<size_t, size_t> estimatedIDX2realIDX;

  std::vector<THistoric> historic;

  /** Loads the simulation and SLAM parameters from a config file (section
   * ""), as used by the batch experiments. */
  void loadConfig(const std::string& cfgFile);
  void saveConfig(const std::string& cfgFile) const;

  /** Resets the simulation and regenerates the ground truth map. */
  void reset();

  /** Simulates one step (robot motion, observation, SLAM update). */
  void step();

  /** Number of steps of a batch run: some laps of the square path. */
  [[nodiscard]] size_t defaultBatchSteps() const;

  /** Statistics of the steps run so far. */
  [[nodiscard]] TRunStats computeStats(double totalSeconds) const;

  /** Starts (or stops, with an empty name) saving the simulated data to a
   * rawlog file. */
  void setRawlogOutput(const std::string& file);
  [[nodiscard]] bool isSavingRawlog() const { return m_rawlogOut.fileOpenCorrectly(); }

  /** Saves the KF state (kf_mean.txt, kf_cov.txt) and its 3D view
   * (slam.3Dscene) into a directory. */
  void saveFilterState(const std::string& dir) const;

  /** Saves the last data association (predicted IDs, means, covariances)
   * into a directory. */
  void saveLastDataAssociation(const std::string& dir) const;

 private:
  std::set<size_t> m_realIDX_already_mapped;
  mrpt::io::CCompressedOutputStream m_rawlogOut;
};
