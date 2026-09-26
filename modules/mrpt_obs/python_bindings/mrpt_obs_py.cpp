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

#include <mrpt/core/Clock.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T3DPointsProjectionParams.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::obs — sensor observations and actions";

  // -------------------------------------------------------------------------
  // CObservation — abstract base class for all sensor observations
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservation, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::obs::CObservation>>(m, "CObservation")
      .def_readwrite("timestamp", &mrpt::obs::CObservation::timestamp)
      .def_readwrite("sensorLabel", &mrpt::obs::CObservation::sensorLabel)
      .def(
          "getSensorPose", [](const mrpt::obs::CObservation& o) { return o.getSensorPose(); },
          "Returns the sensor pose (6D) relative to the robot")
      .def("getTimeStamp", &mrpt::obs::CObservation::getTimeStamp)
      .def(
          "load", [](const mrpt::obs::CObservation& o) { o.load(); },
          "Loads externally-stored data (e.g. images), if any")
      .def(
          "unload", [](const mrpt::obs::CObservation& o) { o.unload(); },
          "Frees externally-stored data from memory, if any")
      .def("__str__", &mrpt::obs::CObservation::asString)
      .def("GetRuntimeClass", &mrpt::obs::CObservation::GetRuntimeClass)
      .def(
          "__repr__", [](const mrpt::obs::CObservation& o)
          { return "CObservation(label='" + o.sensorLabel + "')"; });

  // -------------------------------------------------------------------------
  // CObservation2DRangeScan — 2D laser scan (most important observation type)
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservation2DRangeScan, mrpt::obs::CObservation,
      std::shared_ptr<mrpt::obs::CObservation2DRangeScan>>(m, "CObservation2DRangeScan")
      .def(py::init<>())
      .def_readwrite(
          "aperture", &mrpt::obs::CObservation2DRangeScan::aperture, "Field-of-view in radians")
      .def_readwrite(
          "rightToLeft", &mrpt::obs::CObservation2DRangeScan::rightToLeft,
          "Scan direction: True=CCW, False=CW")
      .def_readwrite(
          "maxRange", &mrpt::obs::CObservation2DRangeScan::maxRange,
          "Maximum sensor range in meters")
      .def_readwrite(
          "sensorPose", &mrpt::obs::CObservation2DRangeScan::sensorPose,
          "Sensor 6D pose relative to robot base")
      .def("resizeScan", &mrpt::obs::CObservation2DRangeScan::resizeScan)
      .def("getScanSize", &mrpt::obs::CObservation2DRangeScan::getScanSize)
      .def(
          "getScanRange",
          [](const mrpt::obs::CObservation2DRangeScan& o, size_t i) { return o.getScanRange(i); })
      .def(
          "setScanRange", [](mrpt::obs::CObservation2DRangeScan& o, size_t i, float val)
          { o.setScanRange(i, val); })
      .def("getScanRangeValidity", &mrpt::obs::CObservation2DRangeScan::getScanRangeValidity)
      .def("setScanRangeValidity", &mrpt::obs::CObservation2DRangeScan::setScanRangeValidity)
      // NumPy helpers
      .def(
          "getScanRangesAsNumpy",
          [](const mrpt::obs::CObservation2DRangeScan& o)
          {
            const size_t n = o.getScanSize();
            std::vector<float> v(n);
            for (size_t i = 0; i < n; i++) v[i] = o.getScanRange(i);
            return py::array_t<float>(
                std::vector<py::ssize_t>{py::ssize_t(n)},
                std::vector<py::ssize_t>{py::ssize_t(sizeof(float))}, v.data());
          },
          "Returns all scan ranges as a 1D float32 numpy array")
      .def(
          "getValidRangesAsNumpy",
          [](const mrpt::obs::CObservation2DRangeScan& o)
          {
            const size_t n = o.getScanSize();
            std::vector<uint8_t> tmp(n);
            for (size_t i = 0; i < n; i++) tmp[i] = o.getScanRangeValidity(i) ? 1 : 0;
            return py::array_t<bool>(
                std::vector<py::ssize_t>{py::ssize_t(n)},
                std::vector<py::ssize_t>{py::ssize_t(sizeof(bool))},
                reinterpret_cast<const bool*>(tmp.data()));
          },
          "Returns validity flags as a 1D bool numpy array")
      .def(
          "__repr__",
          [](const mrpt::obs::CObservation2DRangeScan& o)
          {
            return "CObservation2DRangeScan(label='" + o.sensorLabel +
                   "', nRays=" + std::to_string(o.getScanSize()) + ")";
          });

  // -------------------------------------------------------------------------
  // CObservationImage — single image from a camera
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservationImage, mrpt::obs::CObservation,
      std::shared_ptr<mrpt::obs::CObservationImage>>(m, "CObservationImage")
      .def(py::init<>())
      .def_readwrite("image", &mrpt::obs::CObservationImage::image)
      .def_readwrite("cameraParams", &mrpt::obs::CObservationImage::cameraParams)
      .def_readwrite("cameraPose", &mrpt::obs::CObservationImage::cameraPose)
      .def(
          "__repr__", [](const mrpt::obs::CObservationImage& o)
          { return "CObservationImage(label='" + o.sensorLabel + "')"; });

  // -------------------------------------------------------------------------
  // CObservationIMU — inertial measurement unit data
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservationIMU, mrpt::obs::CObservation,
      std::shared_ptr<mrpt::obs::CObservationIMU>>(m, "CObservationIMU")
      .def(py::init<>())
      .def(
          "getRawMeasurementsAsNumpy",
          [](const mrpt::obs::CObservationIMU& o)
          {
            const size_t n = o.rawMeasurements.size();
            py::array_t<double> arr(std::vector<py::ssize_t>{static_cast<py::ssize_t>(n)});
            auto buf = arr.mutable_unchecked<1>();
            for (size_t i = 0; i < n; i++) buf(i) = o.rawMeasurements[i];
            return arr;
          },
          "Returns raw IMU measurements as 1D float64 numpy array")
      .def(
          "get", [](const mrpt::obs::CObservationIMU& o, mrpt::obs::TIMUDataIndex idx)
          { return o.get(idx); })
      .def(
          "set", [](mrpt::obs::CObservationIMU& o, mrpt::obs::TIMUDataIndex idx, double value)
          { o.set(idx, value); })
      .def(
          "__repr__", [](const mrpt::obs::CObservationIMU& o)
          { return "CObservationIMU(label='" + o.sensorLabel + "')"; });

  // TIMUDataIndex enum
  py::enum_<mrpt::obs::TIMUDataIndex>(m, "TIMUDataIndex")
      .value("IMU_X_ACC", mrpt::obs::IMU_X_ACC)
      .value("IMU_Y_ACC", mrpt::obs::IMU_Y_ACC)
      .value("IMU_Z_ACC", mrpt::obs::IMU_Z_ACC)
      .value("IMU_YAW_VEL", mrpt::obs::IMU_YAW_VEL)
      .value("IMU_WZ", mrpt::obs::IMU_WZ)
      .value("IMU_PITCH_VEL", mrpt::obs::IMU_PITCH_VEL)
      .value("IMU_WY", mrpt::obs::IMU_WY)
      .value("IMU_ROLL_VEL", mrpt::obs::IMU_ROLL_VEL)
      .value("IMU_WX", mrpt::obs::IMU_WX)
      .value("IMU_X_VEL", mrpt::obs::IMU_X_VEL)
      .value("IMU_Y_VEL", mrpt::obs::IMU_Y_VEL)
      .value("IMU_Z_VEL", mrpt::obs::IMU_Z_VEL)
      .value("IMU_YAW", mrpt::obs::IMU_YAW)
      .value("IMU_PITCH", mrpt::obs::IMU_PITCH)
      .value("IMU_ROLL", mrpt::obs::IMU_ROLL)
      .value("IMU_X", mrpt::obs::IMU_X)
      .value("IMU_Y", mrpt::obs::IMU_Y)
      .value("IMU_Z", mrpt::obs::IMU_Z)
      .value("IMU_MAG_X", mrpt::obs::IMU_MAG_X)
      .value("IMU_MAG_Y", mrpt::obs::IMU_MAG_Y)
      .value("IMU_MAG_Z", mrpt::obs::IMU_MAG_Z)
      .value("IMU_PRESSURE", mrpt::obs::IMU_PRESSURE)
      .value("IMU_ALTITUDE", mrpt::obs::IMU_ALTITUDE)
      .value("IMU_TEMPERATURE", mrpt::obs::IMU_TEMPERATURE)
      .value("IMU_ORI_QUAT_X", mrpt::obs::IMU_ORI_QUAT_X)
      .value("IMU_ORI_QUAT_Y", mrpt::obs::IMU_ORI_QUAT_Y)
      .value("IMU_ORI_QUAT_Z", mrpt::obs::IMU_ORI_QUAT_Z)
      .value("IMU_ORI_QUAT_W", mrpt::obs::IMU_ORI_QUAT_W)
      .value("IMU_YAW_VEL_GLOBAL", mrpt::obs::IMU_YAW_VEL_GLOBAL)
      .value("IMU_PITCH_VEL_GLOBAL", mrpt::obs::IMU_PITCH_VEL_GLOBAL)
      .value("IMU_ROLL_VEL_GLOBAL", mrpt::obs::IMU_ROLL_VEL_GLOBAL)
      .value("IMU_X_ACC_GLOBAL", mrpt::obs::IMU_X_ACC_GLOBAL)
      .value("IMU_Y_ACC_GLOBAL", mrpt::obs::IMU_Y_ACC_GLOBAL)
      .value("IMU_Z_ACC_GLOBAL", mrpt::obs::IMU_Z_ACC_GLOBAL)
      .export_values();

  // -------------------------------------------------------------------------
  // CObservationOdometry — raw odometry reading
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservationOdometry, mrpt::obs::CObservation,
      std::shared_ptr<mrpt::obs::CObservationOdometry>>(m, "CObservationOdometry")
      .def(py::init<>())
      .def_readwrite("odometry", &mrpt::obs::CObservationOdometry::odometry)
      .def(
          "__repr__",
          [](const mrpt::obs::CObservationOdometry& o)
          {
            return "CObservationOdometry(label='" + o.sensorLabel +
                   "', pose=" + o.odometry.asString() + ")";
          });

  // -------------------------------------------------------------------------
  // CObservationRobotPose — external robot pose observation
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CObservationRobotPose, mrpt::obs::CObservation,
      std::shared_ptr<mrpt::obs::CObservationRobotPose>>(m, "CObservationRobotPose")
      .def(py::init<>())
      .def_readwrite("pose", &mrpt::obs::CObservationRobotPose::pose)
      .def(
          "__repr__", [](const mrpt::obs::CObservationRobotPose& o)
          { return "CObservationRobotPose(label='" + o.sensorLabel + "')"; });

  // -------------------------------------------------------------------------
  // CAction — abstract base for actions
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CAction, mrpt::serialization::CSerializable, std::shared_ptr<mrpt::obs::CAction>>(
      m, "CAction")
      .def_readwrite("timestamp", &mrpt::obs::CAction::timestamp)
      .def("GetRuntimeClass", &mrpt::obs::CAction::GetRuntimeClass);

  // -------------------------------------------------------------------------
  // CActionRobotMovement2D — 2D odometry action
  // -------------------------------------------------------------------------
  using ARM2D = mrpt::obs::CActionRobotMovement2D;
  py::class_<ARM2D, mrpt::obs::CAction, std::shared_ptr<ARM2D>> arm2d(m, "CActionRobotMovement2D");

  py::enum_<ARM2D::TEstimationMethod>(arm2d, "TEstimationMethod")
      .value("emOdometry", ARM2D::emOdometry)
      .value("emScan2DMatching", ARM2D::emScan2DMatching)
      .export_values();
  py::enum_<ARM2D::TDrawSampleMotionModel>(arm2d, "TDrawSampleMotionModel")
      .value("mmGaussian", ARM2D::mmGaussian)
      .value("mmThrun", ARM2D::mmThrun)
      .export_values();

  using MMO2D = ARM2D::TMotionModelOptions;
  py::class_<MMO2D> mmo2d(arm2d, "TMotionModelOptions");
  py::class_<MMO2D::TOptions_GaussianModel>(mmo2d, "TOptions_GaussianModel")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double, double, double>(), "a1"_a, "a2"_a, "a3"_a,
          "a4"_a, "minStdXY"_a, "minStdPHI"_a)
      .def_readwrite("a1", &MMO2D::TOptions_GaussianModel::a1)
      .def_readwrite("a2", &MMO2D::TOptions_GaussianModel::a2)
      .def_readwrite("a3", &MMO2D::TOptions_GaussianModel::a3)
      .def_readwrite("a4", &MMO2D::TOptions_GaussianModel::a4)
      .def_readwrite("minStdXY", &MMO2D::TOptions_GaussianModel::minStdXY)
      .def_readwrite("minStdPHI", &MMO2D::TOptions_GaussianModel::minStdPHI);
  py::class_<MMO2D::TOptions_ThrunModel>(mmo2d, "TOptions_ThrunModel")
      .def(py::init<>())
      .def_readwrite("nParticlesCount", &MMO2D::TOptions_ThrunModel::nParticlesCount)
      .def_readwrite("alfa1_rot_rot", &MMO2D::TOptions_ThrunModel::alfa1_rot_rot)
      .def_readwrite("alfa2_rot_trans", &MMO2D::TOptions_ThrunModel::alfa2_rot_trans)
      .def_readwrite("alfa3_trans_trans", &MMO2D::TOptions_ThrunModel::alfa3_trans_trans)
      .def_readwrite("alfa4_trans_rot", &MMO2D::TOptions_ThrunModel::alfa4_trans_rot)
      .def_readwrite("additional_std_XY", &MMO2D::TOptions_ThrunModel::additional_std_XY)
      .def_readwrite("additional_std_phi", &MMO2D::TOptions_ThrunModel::additional_std_phi);
  mmo2d.def(py::init<>())
      .def_readwrite("modelSelection", &MMO2D::modelSelection)
      .def_readwrite("gaussianModel", &MMO2D::gaussianModel)
      .def_readwrite("thrunModel", &MMO2D::thrunModel);

  arm2d.def(py::init<>())
      .def_readwrite(
          "rawOdometryIncrementReading", &ARM2D::rawOdometryIncrementReading,
          "Raw odometry reading (increment since last step)")
      .def_readwrite("estimationMethod", &ARM2D::estimationMethod)
      .def_readwrite("hasVelocities", &ARM2D::hasVelocities)
      .def_readwrite("velocityLocal", &ARM2D::velocityLocal)
      .def_readwrite("motionModelConfiguration", &ARM2D::motionModelConfiguration)
      .def_property(
          "poseChange", [](ARM2D& a) { return a.poseChange.get_ptr(); },
          [](ARM2D& a, const mrpt::poses::CPosePDF::Ptr& pdf) { a.poseChange = pdf; },
          "The 2D pose change probabilistic estimation (a CPosePDF)")
      .def(
          "computeFromOdometry", &ARM2D::computeFromOdometry, "odometryIncrement"_a, "options"_a,
          "Computes poseChange from an odometry increment and a motion model")
      .def(
          "drawSingleSample",
          [](const ARM2D& a)
          {
            mrpt::poses::CPose2D p;
            a.drawSingleSample(p);
            return p;
          },
          "Draws a sample from the motion model")
      .def("GetRuntimeClass", &ARM2D::GetRuntimeClass)
      .def(
          "__repr__",
          [](const ARM2D& a) {
            return "CActionRobotMovement2D(odometry=" + a.rawOdometryIncrementReading.asString() +
                   ")";
          });

  // -------------------------------------------------------------------------
  // CActionRobotMovement3D: 3D odometry action
  // -------------------------------------------------------------------------
  using ARM3D = mrpt::obs::CActionRobotMovement3D;
  using MMO3D = ARM3D::TMotionModelOptions;
  py::class_<ARM3D, mrpt::obs::CAction, std::shared_ptr<ARM3D>> arm3d(m, "CActionRobotMovement3D");
  py::class_<MMO3D> mmo3d(arm3d, "TMotionModelOptions");
  py::class_<MMO3D::TOptions_6DOFModel>(mmo3d, "TOptions_6DOFModel")
      .def(py::init<>())
      .def_readwrite("nParticlesCount", &MMO3D::TOptions_6DOFModel::nParticlesCount)
      .def_readwrite("a1", &MMO3D::TOptions_6DOFModel::a1)
      .def_readwrite("a2", &MMO3D::TOptions_6DOFModel::a2)
      .def_readwrite("a3", &MMO3D::TOptions_6DOFModel::a3)
      .def_readwrite("a4", &MMO3D::TOptions_6DOFModel::a4)
      .def_readwrite("a5", &MMO3D::TOptions_6DOFModel::a5)
      .def_readwrite("a6", &MMO3D::TOptions_6DOFModel::a6)
      .def_readwrite("a7", &MMO3D::TOptions_6DOFModel::a7)
      .def_readwrite("a8", &MMO3D::TOptions_6DOFModel::a8)
      .def_readwrite("a9", &MMO3D::TOptions_6DOFModel::a9)
      .def_readwrite("a10", &MMO3D::TOptions_6DOFModel::a10)
      .def_readwrite("additional_std_XYZ", &MMO3D::TOptions_6DOFModel::additional_std_XYZ)
      .def_readwrite("additional_std_angle", &MMO3D::TOptions_6DOFModel::additional_std_angle);
  mmo3d.def(py::init<>()).def_readwrite("mm6DOFModel", &MMO3D::mm6DOFModel);

  arm3d.def(py::init<>())
      .def_readwrite("poseChange", &ARM3D::poseChange, "Pose change as a CPose3DPDFGaussian")
      .def_readwrite("rawOdometryIncrementReading", &ARM3D::rawOdometryIncrementReading)
      .def(
          "computeFromOdometry", &ARM3D::computeFromOdometry, "odometryIncrement"_a, "options"_a,
          "Computes poseChange from an odometry increment and a motion model")
      .def(
          "__repr__",
          [](const ARM3D& a) {
            return "CActionRobotMovement3D(odometry=" + a.rawOdometryIncrementReading.asString() +
                   ")";
          });

  // -------------------------------------------------------------------------
  // CActionCollection — container of actions
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CActionCollection, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::obs::CActionCollection>>(m, "CActionCollection")
      .def(py::init<>())
      .def("size", &mrpt::obs::CActionCollection::size)
      .def("__len__", [](const mrpt::obs::CActionCollection& a) { return a.size(); })
      .def("get", [](const mrpt::obs::CActionCollection& a, size_t i) { return a.get(i); })
      .def(
          "insert", [](mrpt::obs::CActionCollection& a, const mrpt::obs::CAction::Ptr& act)
          { a.insert(*act); })
      .def("clear", &mrpt::obs::CActionCollection::clear)
      .def(
          "getBestMovementEstimation",
          [](mrpt::obs::CActionCollection& a) { return a.getBestMovementEstimation(); },
          "Returns the CActionRobotMovement2D with the best estimation method, or None")
      .def(
          "__iter__",
          [](const mrpt::obs::CActionCollection& a)
          {
            std::vector<mrpt::obs::CAction::ConstPtr> v;
            v.reserve(a.size());
            for (size_t i = 0; i < a.size(); i++)
            {
              v.push_back(a.get(i));
            }
            return py::make_iterator(v.begin(), v.end());
          },
          py::keep_alive<0, 1>());

  // -------------------------------------------------------------------------
  // CSensoryFrame — collection of simultaneous observations
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::obs::CSensoryFrame, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::obs::CSensoryFrame>>(m, "CSensoryFrame")
      .def(py::init<>())
      .def("size", [](const mrpt::obs::CSensoryFrame& sf) { return sf.size(); })
      .def("__len__", [](const mrpt::obs::CSensoryFrame& sf) { return sf.size(); })
      .def(
          "insert", [](mrpt::obs::CSensoryFrame& sf, const mrpt::obs::CObservation::Ptr& obs)
          { sf.insert(obs); })
      .def("clear", [](mrpt::obs::CSensoryFrame& sf) { sf.clear(); })
      .def(
          "__getitem__",
          [](const mrpt::obs::CSensoryFrame& sf, size_t i) { return sf.getObservationByIndex(i); })
      .def(
          "__iter__",
          [](const mrpt::obs::CSensoryFrame& sf)
          { return py::make_iterator(sf.begin(), sf.end()); },
          py::keep_alive<0, 1>())
      .def(
          "__repr__", [](const mrpt::obs::CSensoryFrame& sf)
          { return "CSensoryFrame(" + std::to_string(sf.size()) + " observations)"; });

  // -------------------------------------------------------------------------
  // T3DPointsProjectionParams: options for 3D range image unprojection
  // -------------------------------------------------------------------------
  py::class_<mrpt::obs::T3DPointsProjectionParams>(m, "T3DPointsProjectionParams")
      .def(py::init<>())
      .def_readwrite(
          "takeIntoAccountSensorPoseOnRobot",
          &mrpt::obs::T3DPointsProjectionParams::takeIntoAccountSensorPoseOnRobot)
      .def_readwrite(
          "robotPoseInTheWorld", &mrpt::obs::T3DPointsProjectionParams::robotPoseInTheWorld)
      .def_readwrite("MAKE_ORGANIZED", &mrpt::obs::T3DPointsProjectionParams::MAKE_ORGANIZED)
      .def_readwrite("decimation", &mrpt::obs::T3DPointsProjectionParams::decimation)
      .def_readwrite("layer", &mrpt::obs::T3DPointsProjectionParams::layer);

  // -------------------------------------------------------------------------
  // CObservation3DRangeScan: depth / RGB-D / time-of-flight camera scans
  // -------------------------------------------------------------------------
  using Obs3D = mrpt::obs::CObservation3DRangeScan;
  py::class_<Obs3D, mrpt::obs::CObservation, std::shared_ptr<Obs3D>>(m, "CObservation3DRangeScan")
      .def(py::init<>())
      .def_readwrite("hasRangeImage", &Obs3D::hasRangeImage)
      .def_readwrite("rangeUnits", &Obs3D::rangeUnits, "Meters per unit in the raw range image")
      .def_readwrite(
          "range_is_depth", &Obs3D::range_is_depth,
          "True: ranges are depth (Z); False: distances along each pixel ray")
      .def_readwrite("hasIntensityImage", &Obs3D::hasIntensityImage)
      .def_readwrite("intensityImage", &Obs3D::intensityImage)
      .def_readwrite("hasConfidenceImage", &Obs3D::hasConfidenceImage)
      .def_readwrite("confidenceImage", &Obs3D::confidenceImage)
      .def_readwrite("hasPoints3D", &Obs3D::hasPoints3D)
      .def_readwrite("cameraParams", &Obs3D::cameraParams, "Depth camera intrinsics")
      .def_readwrite(
          "cameraParamsIntensity", &Obs3D::cameraParamsIntensity, "Intensity camera intrinsics")
      .def_readwrite("relativePoseIntensityWRTDepth", &Obs3D::relativePoseIntensityWRTDepth)
      .def_readwrite("maxRange", &Obs3D::maxRange)
      .def_readwrite("sensorPose", &Obs3D::sensorPose)
      .def_readwrite("stdError", &Obs3D::stdError)
      .def("getScanSize", &Obs3D::getScanSize, "Number of 3D points (if hasPoints3D)")
      .def(
          "getRangeImageAsNumpy",
          [](Obs3D& o)
          {
            o.load();
            const auto rows = static_cast<py::ssize_t>(o.rangeImage.rows());
            const auto cols = static_cast<py::ssize_t>(o.rangeImage.cols());
            py::array_t<float> arr(std::vector<py::ssize_t>{rows, cols});
            auto buf = arr.mutable_unchecked<2>();
            for (py::ssize_t r = 0; r < rows; r++)
            {
              for (py::ssize_t c = 0; c < cols; c++)
              {
                buf(r, c) = o.rangeUnits * o.rangeImage(r, c);
              }
            }
            return arr;
          },
          "Returns the range image as an HxW float32 array, in meters (0 = invalid)")
      .def(
          "getRangeImageRawAsNumpy",
          [](Obs3D& o)
          {
            o.load();
            const auto rows = static_cast<py::ssize_t>(o.rangeImage.rows());
            const auto cols = static_cast<py::ssize_t>(o.rangeImage.cols());
            py::array_t<uint16_t> arr(std::vector<py::ssize_t>{rows, cols});
            auto buf = arr.mutable_unchecked<2>();
            for (py::ssize_t r = 0; r < rows; r++)
            {
              for (py::ssize_t c = 0; c < cols; c++)
              {
                buf(r, c) = o.rangeImage(r, c);
              }
            }
            return arr;
          },
          "Returns the raw range image as an HxW uint16 array (multiply by rangeUnits for "
          "meters)")
      .def(
          "setRangeImageFromNumpy",
          [](Obs3D& o, const py::array_t<float, py::array::c_style | py::array::forcecast>& arr)
          {
            if (arr.ndim() != 2)
            {
              throw std::invalid_argument("Expected a 2D (HxW) array of ranges in meters");
            }
            if (!(o.rangeUnits > 0))
            {
              throw std::invalid_argument("rangeUnits must be positive");
            }
            const auto r = arr.unchecked<2>();
            o.rangeImage_setSize(static_cast<int>(r.shape(0)), static_cast<int>(r.shape(1)));
            const float maxRaw = std::numeric_limits<uint16_t>::max();
            for (py::ssize_t row = 0; row < r.shape(0); row++)
            {
              for (py::ssize_t col = 0; col < r.shape(1); col++)
              {
                const float raw = std::round(r(row, col) / o.rangeUnits);
                // NaN or infinite ranges (a common "invalid" mark) are stored as 0:
                o.rangeImage(row, col) =
                    std::isfinite(raw) ? static_cast<uint16_t>(std::clamp(raw, 0.0f, maxRaw)) : 0;
              }
            }
            o.hasRangeImage = true;
          },
          "ranges"_a,
          "Sets the range image from an HxW array of ranges in meters. NaN or infinite values "
          "are stored as 0 (invalid).")
      .def(
          "unprojectInto",
          [](Obs3D& o, const mrpt::obs::T3DPointsProjectionParams& params)
          {
            o.unprojectInto(o, params);
            o.hasPoints3D = true;
          },
          "params"_a = mrpt::obs::T3DPointsProjectionParams(),
          "Computes the 3D points (points3D_*) from the range image and camera intrinsics")
      .def(
          "getPoints3DAsNumpy",
          [](Obs3D& o)
          {
            o.load();
            const auto n = static_cast<py::ssize_t>(o.points3D_x.size());
            py::array_t<float> arr(std::vector<py::ssize_t>{n, 3});
            auto buf = arr.mutable_unchecked<2>();
            for (py::ssize_t i = 0; i < n; i++)
            {
              buf(i, 0) = o.points3D_x[i];
              buf(i, 1) = o.points3D_y[i];
              buf(i, 2) = o.points3D_z[i];
            }
            return arr;
          },
          "Returns the 3D points as an Nx3 float32 array (see unprojectInto())")
      .def(
          "__repr__",
          [](const Obs3D& o)
          {
            return "CObservation3DRangeScan(label='" + o.sensorLabel +
                   "', range=" + std::to_string(o.rangeImage.cols()) + "x" +
                   std::to_string(o.rangeImage.rows()) +
                   ", points=" + std::to_string(o.points3D_x.size()) + ")";
          });

  // -------------------------------------------------------------------------
  // CObservationGPS and the most common GNSS messages
  // -------------------------------------------------------------------------
  auto gnss = m.def_submodule("gnss", "GNSS message types stored in CObservationGPS");

  py::class_<mrpt::obs::gnss::UTC_time>(gnss, "UTC_time")
      .def(py::init<>())
      .def_readwrite("hour", &mrpt::obs::gnss::UTC_time::hour)
      .def_readwrite("minute", &mrpt::obs::gnss::UTC_time::minute)
      .def_readwrite("sec", &mrpt::obs::gnss::UTC_time::sec);

  using GGA = mrpt::obs::gnss::Message_NMEA_GGA;
  py::class_<GGA> gga(gnss, "Message_NMEA_GGA");
  py::class_<GGA::content_t>(gga, "content_t")
      .def(py::init<>())
      .def_readwrite("UTCTime", &GGA::content_t::UTCTime)
      .def_readwrite("latitude_degrees", &GGA::content_t::latitude_degrees)
      .def_readwrite("longitude_degrees", &GGA::content_t::longitude_degrees)
      .def_readwrite("fix_quality", &GGA::content_t::fix_quality)
      .def_readwrite("altitude_meters", &GGA::content_t::altitude_meters)
      .def_readwrite("geoidal_distance", &GGA::content_t::geoidal_distance)
      .def_readwrite("orthometric_altitude", &GGA::content_t::orthometric_altitude)
      .def_readwrite(
          "corrected_orthometric_altitude", &GGA::content_t::corrected_orthometric_altitude)
      .def_readwrite("satellitesUsed", &GGA::content_t::satellitesUsed)
      .def_readwrite("thereis_HDOP", &GGA::content_t::thereis_HDOP)
      .def_readwrite("HDOP", &GGA::content_t::HDOP);
  gga.def(py::init<>()).def_readwrite("fields", &GGA::fields);

  using RMC = mrpt::obs::gnss::Message_NMEA_RMC;
  py::class_<RMC> rmc(gnss, "Message_NMEA_RMC");
  py::class_<RMC::content_t>(rmc, "content_t")
      .def(py::init<>())
      .def_readwrite("UTCTime", &RMC::content_t::UTCTime)
      .def_readwrite("validity_char", &RMC::content_t::validity_char)
      .def_readwrite("latitude_degrees", &RMC::content_t::latitude_degrees)
      .def_readwrite("longitude_degrees", &RMC::content_t::longitude_degrees)
      .def_readwrite("speed_knots", &RMC::content_t::speed_knots)
      .def_readwrite("direction_degrees", &RMC::content_t::direction_degrees)
      .def_readwrite("date_day", &RMC::content_t::date_day)
      .def_readwrite("date_month", &RMC::content_t::date_month)
      .def_readwrite("date_year", &RMC::content_t::date_year)
      .def_readwrite("magnetic_dir", &RMC::content_t::magnetic_dir)
      .def_readwrite("positioning_mode", &RMC::content_t::positioning_mode);
  rmc.def(py::init<>()).def_readwrite("fields", &RMC::fields);

  py::enum_<mrpt::obs::GnssFixType>(m, "GnssFixType")
      .value("UNKNOWN", mrpt::obs::GnssFixType::UNKNOWN)
      .value("NO_FIX", mrpt::obs::GnssFixType::NO_FIX)
      .value("AUTONOMOUS", mrpt::obs::GnssFixType::AUTONOMOUS)
      .value("SBAS", mrpt::obs::GnssFixType::SBAS)
      .value("GBAS", mrpt::obs::GnssFixType::GBAS)
      .value("DGPS", mrpt::obs::GnssFixType::DGPS)
      .value("RTK_FLOAT", mrpt::obs::GnssFixType::RTK_FLOAT)
      .value("RTK_FIXED", mrpt::obs::GnssFixType::RTK_FIXED)
      .value("PPP", mrpt::obs::GnssFixType::PPP)
      .value("DEAD_RECKONING", mrpt::obs::GnssFixType::DEAD_RECKONING)
      .value("SIMULATION", mrpt::obs::GnssFixType::SIMULATION);

  using ObsGPS = mrpt::obs::CObservationGPS;
  py::class_<ObsGPS, mrpt::obs::CObservation, std::shared_ptr<ObsGPS>>(m, "CObservationGPS")
      .def(py::init<>())
      .def_readwrite("sensorPose", &ObsGPS::sensorPose)
      .def_readwrite("originalReceivedTimestamp", &ObsGPS::originalReceivedTimestamp)
      .def_readwrite("has_satellite_timestamp", &ObsGPS::has_satellite_timestamp)
      .def_readwrite(
          "covariance_enu", &ObsGPS::covariance_enu,
          "Optional 3x3 ENU position covariance (m^2), or None")
      .def_readwrite("fix_type", &ObsGPS::fix_type)
      .def("clear", &ObsGPS::clear, "Removes all GNSS messages")
      .def(
          "hasGGA", [](const ObsGPS& o) { return o.hasMsgClass<GGA>(); },
          "True if the observation has an NMEA GGA message")
      .def(
          "getGGA",
          [](const ObsGPS& o) -> std::optional<GGA>
          {
            const auto* msg = o.getMsgByClassPtr<GGA>();
            if (!msg)
            {
              return std::nullopt;
            }
            return *msg;
          },
          "Returns a copy of the NMEA GGA message, or None")
      .def(
          "setGGA", [](ObsGPS& o, const GGA& msg) { o.setMsg(msg); }, "msg"_a,
          "Stores (or replaces) the NMEA GGA message")
      .def(
          "hasRMC", [](const ObsGPS& o) { return o.hasMsgClass<RMC>(); },
          "True if the observation has an NMEA RMC message")
      .def(
          "getRMC",
          [](const ObsGPS& o) -> std::optional<RMC>
          {
            const auto* msg = o.getMsgByClassPtr<RMC>();
            if (!msg)
            {
              return std::nullopt;
            }
            return *msg;
          },
          "Returns a copy of the NMEA RMC message, or None")
      .def(
          "setRMC", [](ObsGPS& o, const RMC& msg) { o.setMsg(msg); }, "msg"_a,
          "Stores (or replaces) the NMEA RMC message")
      .def(
          "__repr__",
          [](const ObsGPS& o) { return "CObservationGPS(label='" + o.sensorLabel + "')"; });

  // -------------------------------------------------------------------------
  // Metric map definitions (map types are registered by mrpt.maps)
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::TMapGenericParams, mrpt::config::CLoadableOptions,
      mrpt::serialization::CSerializable, std::shared_ptr<mrpt::maps::TMapGenericParams>>(
      m, "TMapGenericParams")
      .def(py::init<>())
      .def_readwrite("enableSaveAs3DObject", &mrpt::maps::TMapGenericParams::enableSaveAs3DObject)
      .def_readwrite(
          "enableObservationLikelihood",
          &mrpt::maps::TMapGenericParams::enableObservationLikelihood)
      .def_readwrite(
          "enableObservationInsertion", &mrpt::maps::TMapGenericParams::enableObservationInsertion);

  using MapInit = mrpt::maps::TMetricMapInitializer;
  py::class_<MapInit, mrpt::config::CLoadableOptions, std::shared_ptr<MapInit>>(
      m, "TMetricMapInitializer")
      .def_static(
          "factory", &MapInit::factory, "mapClassName"_a,
          "Creates the definition of a map by its class name, e.g. 'COccupancyGridMap2D'. "
          "Requires importing mrpt.maps first, which registers the map types.")
      .def_readwrite("genericMapParams", &MapInit::genericMapParams)
      .def(
          "getMetricMapClassName",
          [](const MapInit& i) { return std::string(i.getMetricMapClassType()->className); },
          "Returns the C++ class name of the map this definition creates");

  using MapInitSet = mrpt::maps::TSetOfMetricMapInitializers;
  py::class_<MapInitSet, mrpt::config::CLoadableOptions, std::shared_ptr<MapInitSet>>(
      m, "TSetOfMetricMapInitializers")
      .def(py::init<>())
      .def("size", &MapInitSet::size)
      .def("__len__", &MapInitSet::size)
      .def("clear", &MapInitSet::clear)
      .def(
          "push_back", [](MapInitSet& s, const MapInit::Ptr& i) { s.push_back(i); },
          "mapDefinition"_a)
      .def(
          "__getitem__",
          [](MapInitSet& s, size_t i)
          {
            if (i >= s.size())
            {
              throw py::index_error();
            }
            return *(s.begin() + static_cast<std::ptrdiff_t>(i));
          })
      .def(
          "__iter__", [](MapInitSet& s) { return py::make_iterator(s.begin(), s.end()); },
          py::keep_alive<0, 1>())
      .def(
          "__repr__", [](const MapInitSet& s)
          { return "TSetOfMetricMapInitializers(" + std::to_string(s.size()) + " maps)"; });

  // -------------------------------------------------------------------------
  // CSimpleMap: a sequence of keyframes (pose PDF + sensory frame)
  // -------------------------------------------------------------------------
  using SMap = mrpt::maps::CSimpleMap;
  py::class_<SMap, mrpt::serialization::CSerializable, std::shared_ptr<SMap>> smap(m, "CSimpleMap");
  py::class_<SMap::Keyframe>(smap, "Keyframe")
      .def(py::init<>())
      .def(
          py::init<
              const mrpt::poses::CPose3DPDF::Ptr&, const mrpt::obs::CSensoryFrame::Ptr&,
              const std::optional<mrpt::math::TTwist3D>&>(),
          "pose"_a, "sf"_a, "localTwist"_a = std::nullopt)
      .def_readwrite("pose", &SMap::Keyframe::pose)
      .def_readwrite("sf", &SMap::Keyframe::sf)
      .def_readwrite("localTwist", &SMap::Keyframe::localTwist);
  smap.def(py::init<>())
      .def("size", &SMap::size)
      .def("__len__", &SMap::size)
      .def("empty", &SMap::empty)
      .def("clear", &SMap::clear)
      .def("remove", &SMap::remove, "index"_a)
      .def(
          "insert",
          [](SMap& s, const mrpt::poses::CPose3DPDF::Ptr& pose,
             const mrpt::obs::CSensoryFrame::Ptr& sf,
             const std::optional<mrpt::math::TTwist3D>& twist) { s.insert(pose, sf, twist); },
          "pose"_a, "sf"_a, "localTwist"_a = std::nullopt, "Appends a keyframe")
      .def(
          "insert", [](SMap& s, const SMap::Keyframe& kf) { s.insert(kf); }, "keyframe"_a)
      .def(
          "get",
          [](SMap& s, size_t i) -> SMap::Keyframe&
          {
            if (i >= s.size())
            {
              throw py::index_error();
            }
            return s.get(i);
          },
          py::return_value_policy::reference_internal, "index"_a)
      .def(
          "__getitem__",
          [](SMap& s, size_t i) -> SMap::Keyframe&
          {
            if (i >= s.size())
            {
              throw py::index_error();
            }
            return s.get(i);
          },
          py::return_value_policy::reference_internal)
      .def(
          "__iter__", [](SMap& s) { return py::make_iterator(s.begin(), s.end()); },
          py::keep_alive<0, 1>())
      .def(
          "changeCoordinatesOrigin", &SMap::changeCoordinatesOrigin, "newOrigin"_a,
          "Transforms all keyframe poses so the old origin becomes newOrigin")
      .def(
          "loadFromFile", &SMap::loadFromFile, "fileName"_a,
          "Loads a .simplemap file (possibly compressed). Returns False on error.")
      .def(
          "saveToFile", [](const SMap& s, const std::string& f) { return s.saveToFile(f); },
          "fileName"_a, "Saves to a .simplemap file. Returns False on error.")
      .def(
          "__repr__",
          [](const SMap& s) { return "CSimpleMap(" + std::to_string(s.size()) + " keyframes)"; });

  // -------------------------------------------------------------------------
  // CRawlog: a dataset of actions and observations
  // -------------------------------------------------------------------------
  using Rawlog = mrpt::obs::CRawlog;
  py::class_<Rawlog, mrpt::serialization::CSerializable, std::shared_ptr<Rawlog>> rawlog(
      m, "CRawlog");
  py::enum_<Rawlog::TEntryType>(rawlog, "TEntryType")
      .value("etSensoryFrame", Rawlog::TEntryType::etSensoryFrame)
      .value("etActionCollection", Rawlog::TEntryType::etActionCollection)
      .value("etObservation", Rawlog::TEntryType::etObservation)
      .value("etOther", Rawlog::TEntryType::etOther)
      .export_values();

  rawlog.def(py::init<>())
      .def("size", &Rawlog::size)
      .def("__len__", &Rawlog::size)
      .def("empty", &Rawlog::empty)
      .def("clear", &Rawlog::clear)
      .def(
          "loadFromRawLogFile", &Rawlog::loadFromRawLogFile, "fileName"_a,
          "non_obs_objects_are_legal"_a = false,
          "Loads a .rawlog file (possibly compressed). Returns False on error.")
      .def(
          "saveToRawLogFile",
          [](const Rawlog& r, const std::string& f) { return r.saveToRawLogFile(f); }, "fileName"_a,
          "Saves to a .rawlog file. Returns False on error.")
      .def(
          "insert",
          [](Rawlog& r, const mrpt::serialization::CSerializable::Ptr& obj) { r.insert(obj); },
          "obj"_a,
          "Appends an object (CSensoryFrame, CActionCollection, CObservation, ...). The object "
          "is stored by reference, not copied.")
      .def("getType", &Rawlog::getType, "index"_a)
      .def(
          "remove", [](Rawlog& r, size_t i) { r.remove(i); }, "index"_a)
      .def(
          "getAsAction", [](Rawlog& r, size_t i) { return r.getAsAction(i); }, "index"_a,
          "Returns entry i as a CActionCollection. Raises if it has a different type.")
      .def(
          "getAsObservations", [](Rawlog& r, size_t i) { return r.getAsObservations(i); },
          "index"_a, "Returns entry i as a CSensoryFrame. Raises if it has a different type.")
      .def(
          "getAsObservation", [](Rawlog& r, size_t i) { return r.getAsObservation(i); }, "index"_a,
          "Returns entry i as a CObservation. Raises if it has a different type.")
      .def(
          "getAsGeneric", [](Rawlog& r, size_t i) { return r.getAsGeneric(i); }, "index"_a,
          "Returns entry i, whatever its type")
      .def(
          "__getitem__",
          [](Rawlog& r, size_t i)
          {
            if (i >= r.size())
            {
              throw py::index_error();
            }
            return r.getAsGeneric(i);
          })
      .def(
          "__iter__",
          [](Rawlog& r)
          {
            std::vector<mrpt::serialization::CSerializable::Ptr> v;
            v.reserve(r.size());
            for (size_t i = 0; i < r.size(); i++)
            {
              v.push_back(r.getAsGeneric(i));
            }
            return py::iter(py::cast(v));
          },
          "Iterates over all entries")
      .def(
          "getCommentText", [](const Rawlog& r) { return r.getCommentText(); },
          "Returns the embedded comment text, if any")
      .def("setCommentText", &Rawlog::setCommentText, "text"_a)
      .def_static(
          "ReadFromArchive",
          [](mrpt::serialization::CArchive& in, size_t entry)
          {
            size_t retIndex = entry;
            mrpt::obs::CActionCollection::Ptr action;
            mrpt::obs::CSensoryFrame::Ptr sf;
            mrpt::obs::CObservation::Ptr obs;
            const bool ok =
                Rawlog::getActionObservationPairOrObservation(in, action, sf, obs, retIndex);
            return py::make_tuple(ok, retIndex, action, sf, obs);
          },
          "archive"_a, "entry"_a = 0,
          "Reads the next entry from a rawlog stream (see mrpt.io.archiveFrom()). Returns "
          "(readOk, nextEntryIndex, actions, sensoryFrame, observation): either "
          "(actions, sensoryFrame) or observation are None, depending on the rawlog format. "
          "readOk is False at the end of the stream.")
      .def_static(
          "detectImagesDirectory", &Rawlog::detectImagesDirectory, "rawlogFilename"_a,
          "Returns the directory of externally-stored images for a given rawlog file")
      .def(
          "__repr__",
          [](const Rawlog& r) { return "CRawlog(" + std::to_string(r.size()) + " entries)"; });
}
