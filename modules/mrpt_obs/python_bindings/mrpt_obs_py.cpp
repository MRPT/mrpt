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
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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
          "getSensorPose",
          [](const mrpt::obs::CObservation& o)
          {
            mrpt::poses::CPose3D p;
            o.getSensorPose(p);
            return p;
          },
          "Returns the sensor pose (6D) relative to the robot")
      .def("getTimeStamp", &mrpt::obs::CObservation::getTimeStamp)
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
            py::array_t<float> arr(n);
            auto buf = arr.mutable_unchecked<1>();
            for (size_t i = 0; i < n; i++) buf(i) = o.getScanRange(i);
            return arr;
          },
          "Returns all scan ranges as a 1D float32 numpy array")
      .def(
          "getValidRangesAsNumpy",
          [](const mrpt::obs::CObservation2DRangeScan& o)
          {
            const size_t n = o.getScanSize();
            py::array_t<bool> arr(n);
            auto buf = arr.mutable_unchecked<1>();
            for (size_t i = 0; i < n; i++) buf(i) = o.getScanRangeValidity(i);
            return arr;
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
            py::array_t<double> arr(n);
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
      .value("IMU_PITCH_VEL", mrpt::obs::IMU_PITCH_VEL)
      .value("IMU_ROLL_VEL", mrpt::obs::IMU_ROLL_VEL)
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
  py::class_<
      mrpt::obs::CActionRobotMovement2D, mrpt::obs::CAction,
      std::shared_ptr<mrpt::obs::CActionRobotMovement2D>>(m, "CActionRobotMovement2D")
      .def(py::init<>())
      .def_readwrite(
          "rawOdometryIncrementReading",
          &mrpt::obs::CActionRobotMovement2D::rawOdometryIncrementReading,
          "Raw odometry reading (increment since last step)")
      .def("GetRuntimeClass", &mrpt::obs::CActionRobotMovement2D::GetRuntimeClass)
      .def(
          "__repr__",
          [](const mrpt::obs::CActionRobotMovement2D& a) {
            return "CActionRobotMovement2D(odometry=" + a.rawOdometryIncrementReading.asString() +
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
          "__iter__",
          [](const mrpt::obs::CActionCollection& a)
          {
            std::vector<mrpt::obs::CAction::Ptr> v;
            for (size_t i = 0; i < a.size(); i++) v.push_back(a.get(i));
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
}
